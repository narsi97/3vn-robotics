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
/// The ESP32 hardware interface: the third leg of the hardware seam.
///
/// `mock` and `gz` have been working since Phase 1 and 2. This is the
/// one we write. Because the seam is a pluginlib class selected by a
/// xacro argument, adding it is purely additive: no launch file, no
/// controller, no scenario and no test above this line changes.
///
/// It is a THIN adapter, on purpose. The control loop runs on the host
/// at 50 Hz; the device is a servo driver. Kinematics, trajectory
/// generation and planning stay in ROS, or the "not coupled to the
/// ESP32" property is lost and debugging moves onto a microcontroller.

#ifndef THREEVN_HARDWARE__ESP32_SYSTEM_INTERFACE_HPP_
#define THREEVN_HARDWARE__ESP32_SYSTEM_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "threevn_hardware/protocol.hpp"
#include "threevn_hardware/transport.hpp"

namespace threevn
{
namespace hardware
{

class Esp32SystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Esp32SystemInterface)

  /// Inject a transport, for tests. Ownership is taken.
  ///
  /// Without this the class could only be exercised against a real
  /// serial device, which would mean its disconnection and corruption
  /// behaviour -- the behaviour that actually matters -- could never be
  /// tested deliberately.
  void setTransport(std::unique_ptr<Transport> transport);

  /// Current signature. The older on_init(const HardwareInfo&) is
  /// deprecated in hardware_interface 4.x; -Werror caught it rather than
  /// letting the package build on an API scheduled for removal.
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

  /// Status flags from the most recent state frame.
  uint8_t deviceStatus() const {return status_;}

  /// State frames received since activation.
  uint32_t framesReceived() const {return frames_received_;}

  /// Consecutive read cycles with no state frame.
  uint32_t staleCycles() const {return stale_cycles_;}

  /// True once the device has gone quiet for longer than the timeout.
  bool linkLost() const {return link_lost_;}

private:
  void pumpIncoming();
  void handleFrame(const protocol::FrameParser & parser);

  std::unique_ptr<Transport> transport_;
  protocol::FrameParser parser_;

  /// Owned by the interface, not created per log call.
  ///
  /// RCLCPP_*_THROTTLE takes the clock BY REFERENCE and keeps per-call-site
  /// state against it. Passing `*rclcpp::Clock::make_shared()` dereferences
  /// a temporary shared_ptr that dies at the end of the statement, leaving
  /// the macro holding a dangling reference - and even if it survived, a
  /// clock created fresh on every call restarts the throttle window each
  /// time, so nothing is ever actually throttled.
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  std::vector<double> commands_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  /// Positions from the previous cycle, for differentiating velocity.
  std::vector<double> previous_positions_;

  std::string device_;
  int baud_ = 921600;
  double control_rate_hz_ = 50.0;

  /// Read cycles without a state frame before the link counts as lost.
  uint32_t stale_limit_ = 25;      // 0.5 s at 50 Hz

  uint8_t status_ = 0;
  uint32_t frames_received_ = 0;
  uint32_t stale_cycles_ = 0;
  bool link_lost_ = false;
  bool activated_ = false;
};

}  // namespace hardware
}  // namespace threevn

#endif  // THREEVN_HARDWARE__ESP32_SYSTEM_INTERFACE_HPP_
