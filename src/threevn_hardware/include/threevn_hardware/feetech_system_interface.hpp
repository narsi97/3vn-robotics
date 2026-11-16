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

/// \file
/// The Feetech hardware interface: the SO-101's third target.
///
/// `target:=feetech`. Everything above this file -- controllers, client,
/// scenarios, tests -- is what already runs against `mock` and `gz`. This
/// is the only code that is different when the arm is real.
///
/// Compare it with Esp32SystemInterface, because the contrast is the
/// lesson. The 3VN arm's hobby servos are open-loop, so that interface
/// needs a microcontroller, a protocol of our own, and a heartbeat to
/// decide what "the device went quiet" means. A bus servo is already a
/// closed-loop controller with a serial port. There is no firmware of
/// ours on the far side: every control cycle the interface asks the
/// servos where they are and tells them where to go.
///
/// What it is responsible for, and nothing else:
///
///   * refusing to start if a servo the description names is not there
///   * never moving the arm on activation: the first goal is wherever
///     the arm already is
///   * converting between a joint's radians and a servo's counts, using
///     the calibration in the robot profile
///   * saying so, as an error the controller manager can act on, when
///     the bus stops answering
///
/// It talks to a Transport and not to a file descriptor, so all of that
/// is tested against a simulated bus, and the same class runs unchanged
/// over a serial port or a TCP socket.

#ifndef THREEVN_HARDWARE__FEETECH_SYSTEM_INTERFACE_HPP_
#define THREEVN_HARDWARE__FEETECH_SYSTEM_INTERFACE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "threevn_hardware/feetech_protocol.hpp"
#include "threevn_hardware/transport.hpp"

namespace threevn
{
namespace hardware
{

class FeetechSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FeetechSystemInterface)

  /// Inject a transport, for tests. Ownership is taken.
  void setTransport(std::unique_ptr<Transport> transport);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // -- exposed for tests -------------------------------------------

  /// Consecutive read cycles in which not every servo answered.
  uint32_t staleCycles() const {return stale_cycles_;}

  /// True once the bus has been short of answers for longer than the limit.
  bool linkLost() const {return link_lost_;}

  /// Why the last lifecycle transition failed, in words. Empty if it did not.
  const std::string & lastFault() const {return last_fault_;}

private:
  struct Joint
  {
    std::string name;
    uint8_t id = 0;
    feetech::Calibration calibration;
    /// Last position the servo reported, radians. NaN until it has.
    double position = 0.0;
  };

  /// Send `request` and collect one status packet from each id listed.
  ///
  /// Returns true only if every one of them answered inside the timeout.
  /// `replies` holds whichever did, so a caller can name the ones that
  /// did not.
  bool transact(
    const std::vector<uint8_t> & request, const std::vector<uint8_t> & ids,
    std::map<uint8_t, feetech::Status> & replies);

  bool readPositions(std::vector<uint8_t> & silent);
  bool writeTorque(bool enabled);
  bool writeGoals(const std::vector<double> & radians);
  void fail(const std::string & why);

  std::unique_ptr<Transport> transport_;
  feetech::StatusParser parser_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  std::vector<Joint> joints_;
  std::vector<uint8_t> ids_;

  std::string transport_kind_ = "serial";
  std::string endpoint_;
  int baud_ = 1000000;
  double control_rate_hz_ = 50.0;
  /// How long a servo has to answer, milliseconds.
  int response_timeout_ms_ = 10;

  /// Read cycles short of answers before the link counts as lost.
  uint32_t stale_limit_ = 25;      // 0.5 s at 50 Hz
  uint32_t stale_cycles_ = 0;
  bool link_lost_ = false;
  std::string last_fault_;
};

}  // namespace hardware
}  // namespace threevn

#endif  // THREEVN_HARDWARE__FEETECH_SYSTEM_INTERFACE_HPP_
