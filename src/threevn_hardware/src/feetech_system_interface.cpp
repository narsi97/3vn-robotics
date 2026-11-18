// Copyright 2026 3VN Systems
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "threevn_hardware/feetech_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "threevn_hardware/serial_transport.hpp"
#include "threevn_hardware/tcp_transport.hpp"

namespace threevn
{
namespace hardware
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("FeetechSystemInterface");
}

template<typename Map>
std::string lookup(const Map & map, const std::string & key, const std::string & fallback)
{
  const auto it = map.find(key);
  return it == map.end() ? fallback : it->second;
}

bool hasInterface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo & i) {return i.name == name;});
}

std::string position(const std::string & joint)
{
  return joint + "/" + hardware_interface::HW_IF_POSITION;
}

std::string velocity(const std::string & joint)
{
  return joint + "/" + hardware_interface::HW_IF_VELOCITY;
}

std::string effort(const std::string & joint)
{
  return joint + "/" + hardware_interface::HW_IF_EFFORT;
}

/// Not one of ros2_control's standard names: a servo's own temperature,
/// degrees C. joint_state_broadcaster publishes it on /dynamic_joint_states.
constexpr char kTemperature[] = "temperature";

std::string temperature(const std::string & joint)
{
  return joint + "/" + kTemperature;
}

/// Bytes from Present_Position (56) to Present_Temperature (63), inclusive.
constexpr uint8_t kStatusBlockSize =
  feetech::reg::kPresentTemperature - feetech::reg::kPresentPosition + 1;
}  // namespace

void FeetechSystemInterface::setTransport(std::unique_ptr<Transport> transport)
{
  transport_ = std::move(transport);
}

void FeetechSystemInterface::fail(const std::string & why)
{
  last_fault_ = why;
  RCLCPP_ERROR(logger(), "%s", why.c_str());
}

hardware_interface::CallbackReturn FeetechSystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  last_fault_.clear();

  const auto & info = params.hardware_info;
  transport_kind_ = lookup(info.hardware_parameters, "transport", "serial");
  endpoint_ = lookup(info.hardware_parameters, "endpoint", "/dev/ttyACM0");
  if (transport_kind_ != "serial" && transport_kind_ != "tcp") {
    // An unknown transport is refused here. Accepting it and behaving
    // as serial would be a configuration that silently means something
    // else.
    fail("transport '" + transport_kind_ + "' is not one of: serial, tcp");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    baud_ = std::stoi(lookup(info.hardware_parameters, "baud", "1000000"));
    control_rate_hz_ = std::stod(lookup(info.hardware_parameters, "control_rate_hz", "50"));
    response_timeout_ms_ =
      std::stoi(lookup(info.hardware_parameters, "response_timeout_ms", "10"));
  } catch (const std::exception &) {
    fail("baud, control_rate_hz and response_timeout_ms must be numbers");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Half a second short of answers ends the link, whatever the loop rate.
  stale_limit_ = static_cast<uint32_t>(std::max(5.0, control_rate_hz_ * 0.5));

  joints_.clear();
  ids_.clear();
  std::set<int> seen;
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      fail("joint '" + joint.name + "' must have exactly one position command interface");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!hasInterface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !hasInterface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      fail("joint '" + joint.name + "' must have position and velocity state interfaces");
      return hardware_interface::CallbackReturn::ERROR;
    }

    Joint entry;
    entry.name = joint.name;
    entry.position = std::numeric_limits<double>::quiet_NaN();
    entry.reports_effort = hasInterface(joint.state_interfaces, hardware_interface::HW_IF_EFFORT);
    entry.reports_temperature = hasInterface(joint.state_interfaces, kTemperature);
    int id = -1;
    try {
      // No default for the id. A joint that does not say which servo
      // it is must not be guessed at: two joints sharing an id means
      // two servos answering one packet.
      id = std::stoi(lookup(joint.parameters, "id", ""));
      entry.calibration.zero_count = std::stoi(
        lookup(joint.parameters, "zero_count", std::to_string(feetech::kCentreCount)));
      entry.calibration.direction = std::stoi(lookup(joint.parameters, "direction", "1"));
    } catch (const std::exception &) {
      fail(
        "joint '" + joint.name + "' needs an integer `id` parameter "
        "(and integer zero_count and direction, if given)");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (entry.reports_effort) {
      // No default here either. A load is a percentage; turning it into
      // N.m needs to know what 100 % is, and a guess would put a number
      // on /joint_states that looks measured and is not.
      try {
        entry.full_load_effort = std::stod(lookup(joint.parameters, "full_load_effort", ""));
      } catch (const std::exception &) {
        entry.full_load_effort = 0.0;
      }
      if (!(entry.full_load_effort > 0.0)) {
        fail(
          "joint '" + joint.name + "' has an effort state interface, so it needs a "
          "positive `full_load_effort` parameter: the effort at 100 % servo load");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (id < 0 || id > 253) {
      fail("joint '" + joint.name + "': id " + std::to_string(id) + " is not 0-253");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!seen.insert(id).second) {
      fail(
        "joint '" + joint.name + "' repeats bus id " + std::to_string(id) +
        "; every servo needs its own");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (entry.calibration.direction != 1 && entry.calibration.direction != -1) {
      fail("joint '" + joint.name + "': direction must be 1 or -1");
      return hardware_interface::CallbackReturn::ERROR;
    }
    entry.id = static_cast<uint8_t>(id);
    ids_.push_back(entry.id);
    joints_.push_back(entry);
  }
  if (joints_.empty()) {
    fail("the description names no joints");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool FeetechSystemInterface::transact(
  const std::vector<uint8_t> & request, const std::vector<uint8_t> & ids,
  std::map<uint8_t, feetech::Status> & replies)
{
  replies.clear();

  // Throw away anything still in the pipe. A reply that arrives late
  // belongs to the question before this one, and taking it for this
  // one's answer would report where the arm WAS.
  uint8_t buffer[256];
  for (int drained = 0; drained < 16; ++drained) {
    const int n = transport_->read(buffer, sizeof(buffer));
    if (n < 0) {
      return false;
    }
    if (n == 0) {
      break;
    }
  }
  parser_.reset();

  if (!transport_->write(request.data(), request.size())) {
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(response_timeout_ms_);
  while (replies.size() < ids.size()) {
    const int n = transport_->read(buffer, sizeof(buffer));
    if (n < 0) {
      return false;
    }
    if (n > 0) {
      parser_.push(buffer, static_cast<std::size_t>(n));
      feetech::Status status;
      while (parser_.next(status)) {
        // Only an id that was asked. Anything else on the line is not
        // an answer to this.
        if (std::find(ids.begin(), ids.end(), status.id) != ids.end()) {
          replies[status.id] = status;
        }
      }
      continue;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    // Nothing yet. A reply is ten bytes at a megabaud, so the wait is
    // for the USB adapter, not the servo.
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  return replies.size() == ids.size();
}

bool FeetechSystemInterface::readPositions(std::vector<uint8_t> & silent)
{
  silent.clear();
  // Position, speed, load, voltage and temperature are adjacent, so one
  // request fetches all five. Eight bytes instead of four per servo:
  // at a megabaud that is 32 microseconds more for the whole arm.
  std::map<uint8_t, feetech::Status> replies;
  transact(
    feetech::encodeSyncRead(feetech::reg::kPresentPosition, kStatusBlockSize, ids_), ids_,
    replies);

  for (auto & joint : joints_) {
    const auto it = replies.find(joint.id);
    if (it == replies.end() || it->second.params.size() != kStatusBlockSize) {
      silent.push_back(joint.id);
      continue;
    }
    const auto & data = it->second.params;
    const int count = feetech::fromBytes(data[0], data[1]);
    const int speed = feetech::fromSignMagnitude(feetech::fromBytes(data[2], data[3]), 15);
    joint.position = feetech::toRadians(count, joint.calibration);
    set_state(position(joint.name), joint.position);
    set_state(velocity(joint.name), feetech::toRadiansPerSecond(speed, joint.calibration));
    if (joint.reports_effort) {
      // Tenths of a percent, sign in bit 10. The sign is the servo's
      // direction of push, so it is turned into the joint's direction
      // the same way position and speed are.
      const int load = feetech::fromSignMagnitude(feetech::fromBytes(data[4], data[5]), 10);
      set_state(
        effort(joint.name),
        joint.calibration.direction * (load / 1000.0) * joint.full_load_effort);
    }
    if (joint.reports_temperature) {
      set_state(temperature(joint.name), static_cast<double>(data[7]));
    }

    if (it->second.error != 0) {
      RCLCPP_WARN_THROTTLE(
        logger(), clock_, 2000, "servo %u (%s) reports fault flags 0x%02X",
        joint.id, joint.name.c_str(), it->second.error);
    }
  }
  return silent.empty();
}

bool FeetechSystemInterface::writeTorque(bool enabled)
{
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> values;
  for (const auto id : ids_) {
    values.push_back({id, {static_cast<uint8_t>(enabled ? 1 : 0)}});
  }
  const auto packet = feetech::encodeSyncWrite(feetech::reg::kTorqueEnable, 1, values);
  return transport_->write(packet.data(), packet.size());
}

bool FeetechSystemInterface::writeGoals(const std::vector<double> & radians)
{
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> values;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const int count = feetech::toCount(radians[i], joints_[i].calibration);
    values.push_back({joints_[i].id, feetech::toBytes(static_cast<uint16_t>(count))});
  }
  const auto packet = feetech::encodeSyncWrite(feetech::reg::kGoalPosition, 2, values);
  return transport_->write(packet.data(), packet.size());
}

hardware_interface::CallbackReturn FeetechSystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_fault_.clear();
  if (!transport_) {
    if (transport_kind_ == "tcp") {
      transport_ = std::make_unique<TcpTransport>(endpoint_);
    } else {
      transport_ = std::make_unique<SerialTransport>(endpoint_, baud_);
    }
  }
  if (!transport_->open()) {
    fail("cannot open the servo bus: " + transport_->lastError());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Every servo the description names must be on the bus, and say so,
  // before anything is allowed to come up. Five answering out of six
  // is an arm with one joint nobody is controlling.
  std::string missing;
  for (const auto & joint : joints_) {
    std::map<uint8_t, feetech::Status> reply;
    if (!transact(feetech::encodePing(joint.id), {joint.id}, reply)) {
      missing += (missing.empty() ? "" : ", ") + joint.name +
        " (id " + std::to_string(joint.id) + ")";
    }
  }
  if (!missing.empty()) {
    fail(
      "no answer on " + endpoint_ + " from: " + missing +
      ". Check power, the bus cable, and that each servo was given its id.");
    transport_->close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::vector<uint8_t> silent;
  if (!readPositions(silent)) {
    fail("the servos answered a ping but not a position read");
    transport_->close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger(), "%zu servos answering on %s (%s)", joints_.size(), endpoint_.c_str(),
    transport_kind_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FeetechSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  last_fault_.clear();
  stale_cycles_ = 0;
  link_lost_ = false;

  std::vector<uint8_t> silent;
  if (!readPositions(silent)) {
    fail("cannot read where the arm is, so it will not be powered");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ORDER MATTERS. The goal is set to where each joint already is, and
  // only then is torque enabled. The other way round, every servo
  // drives to whatever its goal register happened to hold the moment
  // it is powered -- at full speed, with the arm wherever it was left.
  std::vector<double> here;
  for (const auto & joint : joints_) {
    here.push_back(joint.position);
    // And the controllers start from here too. Without this the first
    // command they could send is NaN or zero, and zero is a pose.
    set_command(position(joint.name), joint.position);
  }
  if (!writeGoals(here) || !writeTorque(true)) {
    fail("cannot write to the servo bus: " + transport_->lastError());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger(), "torque enabled; holding the pose the arm was found in");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FeetechSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Release torque on the way out, as the 3VN arm's interface does:
  // servos left energised after deactivation are an arm holding a pose
  // that nothing is controlling.
  //
  // On this arm that means it goes LIMP and settles under its own
  // weight. That is why the profile has a `rest` pose, and why the arm
  // is driven there before it is stopped.
  if (transport_ && transport_->isOpen()) {
    writeTorque(false);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FeetechSystemInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (transport_) {
    if (transport_->isOpen()) {
      writeTorque(false);
    }
    transport_->close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FeetechSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!transport_ || !transport_->isOpen()) {
    return hardware_interface::return_type::ERROR;
  }

  std::vector<uint8_t> silent;
  if (readPositions(silent)) {
    stale_cycles_ = 0;
    link_lost_ = false;
    return hardware_interface::return_type::OK;
  }

  // One missed reply is a glitch and the last known state stands. A
  // servo that stays quiet is a robot whose position is no longer
  // known, and that is reported as an error so the controller manager
  // can act -- rather than logged while the controllers carry on
  // against a snapshot.
  ++stale_cycles_;
  if (!link_lost_ && stale_cycles_ >= stale_limit_) {
    link_lost_ = true;
    std::string who;
    for (const auto id : silent) {
      who += (who.empty() ? "" : ", ") + std::to_string(id);
    }
    RCLCPP_ERROR(
      logger(), "no answer from servo id(s) %s for %u cycles; link lost",
      who.c_str(), stale_cycles_);
  }
  return link_lost_ ? hardware_interface::return_type::ERROR :
         hardware_interface::return_type::OK;
}

hardware_interface::return_type FeetechSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!transport_ || !transport_->isOpen()) {
    return hardware_interface::return_type::ERROR;
  }

  std::vector<double> goals;
  for (const auto & joint : joints_) {
    const double command = get_command<double>(position(joint.name));
    // Not finite means nothing has commanded this joint. Hold where it
    // was last seen. Sending zero would drive it to the reference pose.
    goals.push_back(std::isfinite(command) ? command : joint.position);
  }
  for (const double goal : goals) {
    if (!std::isfinite(goal)) {
      // Neither a command nor a position: nothing is known. Say nothing
      // to the servos rather than guess.
      return hardware_interface::return_type::OK;
    }
  }

  if (!writeGoals(goals)) {
    RCLCPP_ERROR_THROTTLE(
      logger(), clock_, 2000, "bus write failed: %s", transport_->lastError().c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace hardware
}  // namespace threevn

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  threevn::hardware::FeetechSystemInterface, hardware_interface::SystemInterface)
