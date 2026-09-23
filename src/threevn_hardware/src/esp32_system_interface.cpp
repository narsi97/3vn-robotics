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

#include "threevn_hardware/esp32_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "threevn_hardware/serial_transport.hpp"

namespace threevn
{
namespace hardware
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("Esp32SystemInterface");
}

/// Read a hardware parameter, falling back when absent.
std::string param(
  const hardware_interface::HardwareInfo & info,
  const std::string & key, const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}
}  // namespace

void Esp32SystemInterface::setTransport(std::unique_ptr<Transport> transport)
{
  transport_ = std::move(transport);
}

hardware_interface::CallbackReturn Esp32SystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & info = params.hardware_info;
  device_ = param(info, "endpoint", "/dev/ttyUSB0");
  baud_ = std::stoi(param(info, "baud", "921600"));
  control_rate_hz_ = std::stod(param(info, "control_rate_hz", "50"));

  const std::string transport_kind = param(info, "transport", "serial");
  if (transport_kind != "serial") {
    // TCP is planned but not implemented. Failing here beats accepting a
    // configuration and then silently behaving as serial.
    RCLCPP_ERROR(
      logger(), "transport '%s' is not implemented; only 'serial' is supported",
      transport_kind.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Half a second of silence ends the link, regardless of loop rate.
  stale_limit_ = static_cast<uint32_t>(std::max(5.0, control_rate_hz_ * 0.5));

  // The joint count is fixed by the wire protocol, which both sides
  // compile. A mismatch means the description and the firmware disagree
  // about what they are moving -- refuse rather than command a subset.
  if (info_.joints.size() != protocol::kJointCount) {
    RCLCPP_ERROR(
      logger(), "description has %zu joints; the protocol carries %u",
      info_.joints.size(), protocol::kJointCount);
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        logger(), "joint '%s' must have exactly one position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto n = info_.joints.size();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  // NaN until the device reports. Zero would be a lie that reads as a
  // robot sitting at its home pose, and a controller would act on it.
  commands_.assign(n, nan);
  positions_.assign(n, nan);
  velocities_.assign(n, 0.0);
  previous_positions_.assign(n, nan);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32SystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!transport_) {
    transport_ = std::make_unique<SerialTransport>(device_, baud_);
  }
  if (!transport_->open()) {
    RCLCPP_ERROR(logger(), "cannot open the device: %s", transport_->lastError().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  parser_.reset();
  RCLCPP_INFO(logger(), "connected to %s at %d baud", device_.c_str(), baud_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32SystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  frames_received_ = 0;
  stale_cycles_ = 0;
  link_lost_ = false;
  status_ = 0;
  activated_ = true;

  // Ask the device to identify itself. Purely informational: the answer
  // arrives on a later read cycle, and activation does not wait for it.
  uint8_t frame[protocol::kMaxFrameSize];
  const size_t n = protocol::encodeFrame(
    protocol::MessageType::kGetInfo, nullptr, 0, frame, sizeof(frame));
  if (n > 0) {transport_->write(frame, n);}

  RCLCPP_INFO(logger(), "activated; waiting for the first state frame");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  activated_ = false;

  // Release torque on the way out. Leaving servos energised after
  // deactivation means an arm that holds a pose nothing is controlling.
  if (transport_ && transport_->isOpen()) {
    protocol::CommandPayload release;
    for (size_t i = 0; i < positions_.size() && i < protocol::kJointCount; ++i) {
      release.position_micro[i] =
        std::isnan(positions_[i]) ? 0 : protocol::toMicro(positions_[i]);
    }
    release.flags = 0;   // enable bit clear
    uint8_t frame[protocol::kMaxFrameSize];
    const size_t n = protocol::encodeCommand(release, frame, sizeof(frame));
    if (n > 0) {transport_->write(frame, n);}
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32SystemInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (transport_) {transport_->close();}
  return hardware_interface::CallbackReturn::SUCCESS;
}

void Esp32SystemInterface::handleFrame(const protocol::FrameParser & parser)
{
  switch (parser.type()) {
    case protocol::MessageType::kState: {
        protocol::StatePayload state;
        if (!protocol::decodeState(parser.payload(), parser.payloadSize(), state)) {
          return;
        }
        for (size_t i = 0; i < positions_.size() && i < protocol::kJointCount; ++i) {
          positions_[i] = protocol::fromMicro(state.position_micro[i]);
        }
        status_ = state.status;
        ++frames_received_;
        stale_cycles_ = 0;
        link_lost_ = false;
        break;
      }

    case protocol::MessageType::kInfo: {
        protocol::InfoPayload info;
        if (protocol::decodeInfo(parser.payload(), parser.payloadSize(), info)) {
          RCLCPP_INFO(
          logger(), "device firmware %u.%u.%u, protocol %u, heartbeat %u ms",
          info.firmware_major, info.firmware_minor, info.firmware_patch,
          info.protocol_version, info.heartbeat_timeout_ms);
        }
        break;
      }

    case protocol::MessageType::kFault: {
        protocol::FaultPayload fault;
        if (protocol::decodeFault(parser.payload(), parser.payloadSize(), fault)) {
          RCLCPP_ERROR(
          logger(), "device fault %u (detail %u)",
          static_cast<unsigned>(fault.code), fault.detail);
          status_ |= protocol::kStatusFault;
        }
        break;
      }

    default:
      break;   // pongs and anything unexpected are ignored here
  }
}

void Esp32SystemInterface::pumpIncoming()
{
  uint8_t buffer[512];
  for (;; ) {
    const int n = transport_->read(buffer, sizeof(buffer));
    if (n < 0) {
      RCLCPP_ERROR_THROTTLE(
        logger(), clock_, 2000,
        "transport read failed: %s", transport_->lastError().c_str());
      return;
    }
    if (n == 0) {return;}
    parser_.pushBuffer(
      buffer, static_cast<size_t>(n),
      [this](const protocol::FrameParser & p) {this->handleFrame(p);});
    if (static_cast<size_t>(n) < sizeof(buffer)) {return;}
  }
}

hardware_interface::return_type Esp32SystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!transport_ || !transport_->isOpen()) {
    return hardware_interface::return_type::ERROR;
  }

  const auto before = frames_received_;
  pumpIncoming();

  if (frames_received_ == before) {
    ++stale_cycles_;
    if (!link_lost_ && stale_cycles_ >= stale_limit_) {
      link_lost_ = true;
      RCLCPP_ERROR(
        logger(), "no state from the device for %u cycles; link lost",
        stale_cycles_);
    }
  }

  // Differentiate position for velocity. The device reports position
  // only: hobby servos have no velocity feedback, and inventing one from
  // the commanded trajectory would report what we asked for rather than
  // what happened.
  const double dt = period.seconds();
  if (dt > 0.0) {
    for (size_t i = 0; i < positions_.size(); ++i) {
      if (!std::isnan(positions_[i]) && !std::isnan(previous_positions_[i])) {
        velocities_[i] = (positions_[i] - previous_positions_[i]) / dt;
      }
      previous_positions_[i] = positions_[i];
    }
  }

  // A lost link is reported as an error so controller_manager can act on
  // it, rather than being logged while the controllers carry on against
  // stale state.
  return link_lost_ ? hardware_interface::return_type::ERROR :
         hardware_interface::return_type::OK;
}

hardware_interface::return_type Esp32SystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!transport_ || !transport_->isOpen()) {
    return hardware_interface::return_type::ERROR;
  }

  protocol::CommandPayload command;
  bool any_valid = false;
  for (size_t i = 0; i < commands_.size() && i < protocol::kJointCount; ++i) {
    if (std::isnan(commands_[i])) {
      // Before the first command arrives, hold the measured position.
      // Sending zero would drive every joint to its reference pose the
      // instant the hardware activated.
      command.position_micro[i] =
        std::isnan(positions_[i]) ? 0 : protocol::toMicro(positions_[i]);
    } else {
      command.position_micro[i] = protocol::toMicro(commands_[i]);
      any_valid = true;
    }
  }
  command.flags = protocol::kFlagEnable;

  // Sent every cycle even when nothing changed: the frame doubles as the
  // heartbeat, and the device holds position if it stops arriving.
  (void)any_valid;

  uint8_t frame[protocol::kMaxFrameSize];
  const size_t n = protocol::encodeCommand(command, frame, sizeof(frame));
  if (n == 0) {return hardware_interface::return_type::ERROR;}

  if (!transport_->write(frame, n)) {
    RCLCPP_ERROR_THROTTLE(
      logger(), clock_, 2000,
      "transport write failed: %s", transport_->lastError().c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace hardware
}  // namespace threevn

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  threevn::hardware::Esp32SystemInterface, hardware_interface::SystemInterface)
