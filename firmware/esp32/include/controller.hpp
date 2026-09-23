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
/// The firmware's decision logic, with no Arduino in sight.
///
/// Everything that decides what a servo is told lives here: limit
/// clamping, the heartbeat timeout, the emergency stop latch. main.cpp
/// is reduced to moving bytes and writing PWM.
///
/// That split is the point. This header compiles on a laptop, so the
/// behaviour that matters - what happens when the host goes silent, what
/// happens on an out-of-range command, whether an e-stop can be cleared
/// by accident - is tested deliberately instead of discovered on a bench
/// with a powered arm.
///
/// No Arduino, no allocation, no floating point: time arrives as a
/// parameter rather than being read, which is what makes the timeout
/// testable without waiting.

#ifndef THREEVN_FIRMWARE__CONTROLLER_HPP_
#define THREEVN_FIRMWARE__CONTROLLER_HPP_

#include <stdint.h>

#include "joint_limits.hpp"
#include "threevn_hardware/protocol.hpp"

namespace threevn
{
namespace firmware
{

/// Silence after which the device stops trusting the host.
///
/// 250 ms is 12 missed frames at 50 Hz: long enough to ride out a
/// scheduling hiccup, short enough that a crashed host cannot leave the
/// arm driving toward a stale target for a noticeable time.
constexpr uint32_t kHeartbeatTimeoutMs = 250;

constexpr uint8_t kFirmwareMajor = 0;
constexpr uint8_t kFirmwareMinor = 1;
constexpr uint8_t kFirmwarePatch = 0;

/// What the servos should be doing right now.
struct Output
{
  int32_t position_micro[protocol::kJointCount] = {0, 0, 0, 0, 0};
  /// False means cut torque: release the servos rather than hold.
  bool torque_enabled = false;
};

class Controller
{
public:
  /// Accept a command from the host. `now_ms` is the device uptime.
  void onCommand(const protocol::CommandPayload & command, uint32_t now_ms)
  {
    last_command_ms_ = now_ms;
    heartbeat_lost_ = false;

    if (command.flags & protocol::kFlagEmergencyStop) {
      // Latched on purpose. An e-stop that clears itself as soon as the
      // triggering command stops arriving is not an e-stop.
      estop_latched_ = true;
    }
    if (command.flags & protocol::kFlagClearFault) {
      // Only an EXPLICIT clear releases the latch, and only when the
      // stop bit is no longer being asserted.
      if (!(command.flags & protocol::kFlagEmergencyStop)) {
        estop_latched_ = false;
        fault_ = protocol::FaultCode::kNone;
      }
    }

    protocol::CommandPayload clamped = command;
    limit_clamped_ = clampCommand(clamped);

    for (uint8_t i = 0; i < protocol::kJointCount; ++i) {
      target_micro_[i] = clamped.position_micro[i];
    }
    host_wants_enable_ = (command.flags & protocol::kFlagEnable) != 0;
    has_target_ = true;
  }

  /// Advance time. Call every loop, command or not.
  void tick(uint32_t now_ms)
  {
    if (!has_target_) { return; }
    if (now_ms - last_command_ms_ > kHeartbeatTimeoutMs) {
      if (!heartbeat_lost_) {
        heartbeat_lost_ = true;
        fault_ = protocol::FaultCode::kHeartbeatTimeout;
      }
    }
  }

  /// What to drive the servos with.
  Output output() const
  {
    Output out;
    for (uint8_t i = 0; i < protocol::kJointCount; ++i) {
      out.position_micro[i] = target_micro_[i];
    }
    // HOLD POSITION rather than release on a lost heartbeat. Cutting
    // torque would drop the arm under gravity, which on a desk means the
    // gripper swinging into whatever is in front of it. Holding is the
    // safer failure, and the operator still has the power switch.
    out.torque_enabled =
      has_target_ && host_wants_enable_ && !estop_latched_;
    return out;
  }

  protocol::StatePayload state(const int32_t * measured_micro, uint16_t sequence) const
  {
    protocol::StatePayload payload;
    for (uint8_t i = 0; i < protocol::kJointCount; ++i) {
      payload.position_micro[i] = measured_micro[i];
    }
    uint8_t status = 0;
    if (output().torque_enabled) { status |= protocol::kStatusEnabled; }
    if (estop_latched_) { status |= protocol::kStatusEmergencyStop; }
    if (heartbeat_lost_) { status |= protocol::kStatusHeartbeatLost; }
    if (limit_clamped_) { status |= protocol::kStatusLimitClamped; }
    if (fault_ != protocol::FaultCode::kNone) { status |= protocol::kStatusFault; }
    payload.status = status;
    payload.sequence = sequence;
    return payload;
  }

  protocol::InfoPayload info() const
  {
    protocol::InfoPayload out;
    out.firmware_major = kFirmwareMajor;
    out.firmware_minor = kFirmwareMinor;
    out.firmware_patch = kFirmwarePatch;
    out.heartbeat_timeout_ms = kHeartbeatTimeoutMs;
    return out;
  }

  bool heartbeatLost() const {return heartbeat_lost_;}
  bool emergencyStopLatched() const {return estop_latched_;}
  bool lastCommandWasClamped() const {return limit_clamped_;}
  protocol::FaultCode fault() const {return fault_;}

  void setFault(protocol::FaultCode code) {fault_ = code;}

private:
  int32_t target_micro_[protocol::kJointCount] = {0, 0, 0, 0, 0};
  uint32_t last_command_ms_ = 0;
  bool has_target_ = false;
  bool host_wants_enable_ = false;
  bool heartbeat_lost_ = false;
  bool estop_latched_ = false;
  bool limit_clamped_ = false;
  protocol::FaultCode fault_ = protocol::FaultCode::kNone;
};

}  // namespace firmware
}  // namespace threevn

#endif  // THREEVN_FIRMWARE__CONTROLLER_HPP_
