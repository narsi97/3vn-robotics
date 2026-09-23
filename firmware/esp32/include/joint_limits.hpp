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
/// The firmware's own joint limits. Compiled in, not configured.
///
/// This is the LAST line of defence, and the only one that survives a
/// host crash, a wedged control loop or a malicious command. Phase 2
/// measured that nothing below the hardware seam enforced the limits the
/// URDF declared; Phase 4 fixed that in ros2_control. Both of those run
/// on the host. If the host stops, neither helps.
///
/// So these values are COMPILED IN rather than sent over the wire.
/// A limit the host can set is a limit the host can get wrong.
///
/// They must match config/threevn_arm_v1.yaml. They are duplicated on
/// purpose - the duplication is the safety property - and
/// test_firmware_logic.cpp asserts the two agree, so the copy cannot
/// silently drift.

#ifndef THREEVN_FIRMWARE__JOINT_LIMITS_HPP_
#define THREEVN_FIRMWARE__JOINT_LIMITS_HPP_

#include <stdint.h>

#include "threevn_hardware/protocol.hpp"

namespace threevn
{
namespace firmware
{

/// Per-joint travel, in the micro-units the protocol carries.
/// Revolute joints are microradians; the gripper is micrometres.
struct JointLimit
{
  int32_t min_micro;
  int32_t max_micro;
};

/// Indexed by protocol::JointIndex. Mirrors threevn_arm_v1.yaml.
///
///   shoulder_pan   -90 .. +90 deg
///   shoulder_lift  -15 .. +120 deg
///   elbow         -120 .. +10 deg
///   wrist          -90 .. +90 deg
///   gripper          0 .. 18 mm
constexpr JointLimit kJointLimits[protocol::kJointCount] = {
  {-1570796, 1570796},    // shoulder_pan
  {-261799, 2094395},     // shoulder_lift
  {-2094395, 174533},     // elbow
  {-1570796, 1570796},    // wrist
  {0, 18000},             // gripper (metres -> micrometres)
};

/// Clamp one joint. Sets `clamped` when the value was outside its range.
inline int32_t clampJoint(uint8_t index, int32_t value_micro, bool & clamped)
{
  if (index >= protocol::kJointCount) {
    clamped = true;
    return 0;
  }
  const JointLimit & limit = kJointLimits[index];
  if (value_micro < limit.min_micro) {
    clamped = true;
    return limit.min_micro;
  }
  if (value_micro > limit.max_micro) {
    clamped = true;
    return limit.max_micro;
  }
  return value_micro;
}

/// Clamp a whole command in place. Returns true if anything was clamped.
inline bool clampCommand(protocol::CommandPayload & command)
{
  bool clamped = false;
  for (uint8_t i = 0; i < protocol::kJointCount; ++i) {
    command.position_micro[i] = clampJoint(i, command.position_micro[i], clamped);
  }
  return clamped;
}

}  // namespace firmware
}  // namespace threevn

#endif  // THREEVN_FIRMWARE__JOINT_LIMITS_HPP_
