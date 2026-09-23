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
/// The ESP32 firmware's safety logic, tested on a laptop.
///
/// It lives in this package because this is where the repository runs
/// C++ tests, and because the alternative - testing firmware only by
/// flashing it - means discovering an e-stop bug with a powered arm on
/// the desk. The include path reaches into firmware/esp32/include; the
/// dependency is test-only and deliberate.
///
/// These assertions cover the layer that survives a host crash. Nothing
/// above it does.

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "controller.hpp"
#include "joint_limits.hpp"
#include "threevn_hardware/protocol.hpp"

namespace proto = threevn::protocol;
using threevn::firmware::Controller;
using threevn::firmware::clampCommand;
using threevn::firmware::kHeartbeatTimeoutMs;
using threevn::firmware::kJointLimits;

namespace
{
proto::CommandPayload enabledCommand()
{
  proto::CommandPayload cmd;
  cmd.flags = proto::kFlagEnable;
  return cmd;
}
}  // namespace

// ------------------------------------------------------------ limits --

TEST(FirmwareLimits, ClampsAboveTheMaximum)
{
  auto cmd = enabledCommand();
  cmd.position_micro[proto::kShoulderPan] = proto::toMicro(3.14159);   // 180 deg
  EXPECT_TRUE(clampCommand(cmd));
  EXPECT_EQ(
    cmd.position_micro[proto::kShoulderPan], kJointLimits[proto::kShoulderPan].max_micro);
}

TEST(FirmwareLimits, ClampsBelowTheMinimum)
{
  auto cmd = enabledCommand();
  cmd.position_micro[proto::kElbow] = proto::toMicro(-3.0);
  EXPECT_TRUE(clampCommand(cmd));
  EXPECT_EQ(cmd.position_micro[proto::kElbow], kJointLimits[proto::kElbow].min_micro);
}

TEST(FirmwareLimits, LeavesALegalCommandAlone)
{
  // A limiter that clamped everything would pass the tests above while
  // making the arm useless.
  auto cmd = enabledCommand();
  cmd.position_micro[proto::kWrist] = proto::toMicro(0.5);
  EXPECT_FALSE(clampCommand(cmd));
  EXPECT_EQ(cmd.position_micro[proto::kWrist], proto::toMicro(0.5));
}

TEST(FirmwareLimits, TheGripperCannotBeDrivenNegative)
{
  auto cmd = enabledCommand();
  cmd.position_micro[proto::kGripper] = -5000;
  EXPECT_TRUE(clampCommand(cmd));
  EXPECT_EQ(cmd.position_micro[proto::kGripper], 0);
}

TEST(FirmwareLimits, EveryLimitIsOrderedAndNonEmpty)
{
  for (uint8_t i = 0; i < proto::kJointCount; ++i) {
    EXPECT_LT(kJointLimits[i].min_micro, kJointLimits[i].max_micro) << "joint " << int(i);
  }
}

TEST(FirmwareLimits, MatchTheRobotProfileTheyDuplicate)
{
  // The firmware's limits are compiled in ON PURPOSE - a limit the host
  // can set is a limit the host can get wrong, and this is the layer that
  // has to survive the host crashing.
  //
  // The cost of that duplication is drift, so it is asserted here against
  // the YAML rather than trusted to a comment.
  const char * path =
    "/ws/install/threevn_robot_description/share/threevn_robot_description"
    "/config/threevn_arm_v1.yaml";
  std::ifstream file(path);
  if (!file) {
    GTEST_SKIP() << "profile not installed at " << path;
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  const std::string yaml = buffer.str();

  // Minimal scrape: find "lower_deg: X, upper_deg: Y" per joint block.
  struct Expect
  {
    const char * joint;
    uint8_t index;
  };
  const Expect joints[] = {
    {"shoulder_pan_joint", proto::kShoulderPan},
    {"shoulder_lift_joint", proto::kShoulderLift},
    {"elbow_joint", proto::kElbow},
    {"wrist_joint", proto::kWrist},
  };

  for (const auto & entry : joints) {
    const auto block = yaml.find(std::string("  ") + entry.joint + ":");
    ASSERT_NE(block, std::string::npos) << entry.joint << " not found in the profile";
    const auto lower_at = yaml.find("lower_deg:", block);
    const auto upper_at = yaml.find("upper_deg:", block);
    ASSERT_NE(lower_at, std::string::npos);
    ASSERT_NE(upper_at, std::string::npos);

    const double lower_deg = std::stod(yaml.substr(lower_at + 10, 24));
    const double upper_deg = std::stod(yaml.substr(upper_at + 10, 24));
    const int32_t lower = proto::toMicro(lower_deg * M_PI / 180.0);
    const int32_t upper = proto::toMicro(upper_deg * M_PI / 180.0);

    // One microradian of slack for the degree->radian rounding.
    EXPECT_NEAR(kJointLimits[entry.index].min_micro, lower, 2)
      << entry.joint << ": firmware lower limit has drifted from the profile";
    EXPECT_NEAR(kJointLimits[entry.index].max_micro, upper, 2)
      << entry.joint << ": firmware upper limit has drifted from the profile";
  }
}

// -------------------------------------------------------- heartbeat --

TEST(FirmwareHeartbeat, StaysHealthyWhileCommandsArrive)
{
  Controller controller;
  uint32_t now = 1000;
  for (int i = 0; i < 50; ++i) {
    controller.onCommand(enabledCommand(), now);
    now += 20;               // 50 Hz
    controller.tick(now);
  }
  EXPECT_FALSE(controller.heartbeatLost());
  EXPECT_TRUE(controller.output().torque_enabled);
}

TEST(FirmwareHeartbeat, TripsAfterTheHostGoesSilent)
{
  // The failure this layer exists for. Nothing on the host can help once
  // the host is the thing that stopped.
  Controller controller;
  controller.onCommand(enabledCommand(), 1000);
  ASSERT_FALSE(controller.heartbeatLost());

  controller.tick(1000 + kHeartbeatTimeoutMs + 1);

  EXPECT_TRUE(controller.heartbeatLost());
  EXPECT_EQ(controller.fault(), proto::FaultCode::kHeartbeatTimeout);
}

TEST(FirmwareHeartbeat, DoesNotTripEarly)
{
  // Tripping on an ordinary scheduling hiccup would make the arm stutter
  // constantly, and an alarm that cries wolf gets ignored.
  Controller controller;
  controller.onCommand(enabledCommand(), 1000);
  controller.tick(1000 + kHeartbeatTimeoutMs);
  EXPECT_FALSE(controller.heartbeatLost());
}

TEST(FirmwareHeartbeat, HoldsPositionRatherThanDroppingTheArm)
{
  // Cutting torque on a lost heartbeat would drop the arm under gravity.
  // On a desk that means the gripper swinging into whatever is in front
  // of it, so holding is the safer failure.
  Controller controller;
  auto cmd = enabledCommand();
  cmd.position_micro[proto::kShoulderLift] = proto::toMicro(1.0);
  controller.onCommand(cmd, 1000);

  const auto before = controller.output();
  controller.tick(1000 + kHeartbeatTimeoutMs + 1);
  const auto after = controller.output();

  EXPECT_EQ(after.position_micro[proto::kShoulderLift],
    before.position_micro[proto::kShoulderLift])
    << "the held target should not move when the heartbeat is lost";
}

TEST(FirmwareHeartbeat, RecoversWhenTheHostReturns)
{
  Controller controller;
  controller.onCommand(enabledCommand(), 1000);
  controller.tick(1000 + kHeartbeatTimeoutMs + 1);
  ASSERT_TRUE(controller.heartbeatLost());

  controller.onCommand(enabledCommand(), 2000);
  EXPECT_FALSE(controller.heartbeatLost());
}

// ------------------------------------------------------------ e-stop --

TEST(FirmwareEstop, CutsTorqueImmediately)
{
  Controller controller;
  auto cmd = enabledCommand();
  cmd.flags |= proto::kFlagEmergencyStop;
  controller.onCommand(cmd, 1000);

  EXPECT_TRUE(controller.emergencyStopLatched());
  EXPECT_FALSE(controller.output().torque_enabled);
}

TEST(FirmwareEstop, StaysLatchedWhenTheStopCommandStops)
{
  // An e-stop that clears itself the moment the triggering command stops
  // arriving is not an e-stop. Losing the link must not re-energise an
  // arm someone stopped on purpose.
  Controller controller;
  auto stop = enabledCommand();
  stop.flags |= proto::kFlagEmergencyStop;
  controller.onCommand(stop, 1000);

  controller.onCommand(enabledCommand(), 1020);

  EXPECT_TRUE(controller.emergencyStopLatched());
  EXPECT_FALSE(controller.output().torque_enabled);
}

TEST(FirmwareEstop, NeedsAnExplicitClear)
{
  Controller controller;
  auto stop = enabledCommand();
  stop.flags |= proto::kFlagEmergencyStop;
  controller.onCommand(stop, 1000);

  auto clear = enabledCommand();
  clear.flags |= proto::kFlagClearFault;
  controller.onCommand(clear, 1100);

  EXPECT_FALSE(controller.emergencyStopLatched());
  EXPECT_TRUE(controller.output().torque_enabled);
}

TEST(FirmwareEstop, CannotBeClearedWhileStillBeingAsserted)
{
  // Setting both bits at once must not produce a stop that instantly
  // releases; the stop wins.
  Controller controller;
  auto both = enabledCommand();
  both.flags |= proto::kFlagEmergencyStop | proto::kFlagClearFault;
  controller.onCommand(both, 1000);

  EXPECT_TRUE(controller.emergencyStopLatched());
  EXPECT_FALSE(controller.output().torque_enabled);
}

// ------------------------------------------------------------ status --

TEST(FirmwareStatus, ReportsClampingToTheHost)
{
  // The host cannot see that the firmware overrode it unless the device
  // says so, and silent disagreement between commanded and actual is
  // exactly what makes a robot baffling to debug.
  Controller controller;
  auto cmd = enabledCommand();
  cmd.position_micro[proto::kShoulderPan] = proto::toMicro(3.14159);
  controller.onCommand(cmd, 1000);

  EXPECT_TRUE(controller.lastCommandWasClamped());

  const int32_t measured[proto::kJointCount] = {0, 0, 0, 0, 0};
  const auto state = controller.state(measured, 1);
  EXPECT_TRUE(state.status & proto::kStatusLimitClamped);
}

TEST(FirmwareStatus, ReportsTheHeartbeatAndStopFlags)
{
  Controller controller;
  auto stop = enabledCommand();
  stop.flags |= proto::kFlagEmergencyStop;
  controller.onCommand(stop, 1000);
  controller.tick(1000 + kHeartbeatTimeoutMs + 1);

  const int32_t measured[proto::kJointCount] = {0, 0, 0, 0, 0};
  const auto state = controller.state(measured, 7);
  EXPECT_TRUE(state.status & proto::kStatusEmergencyStop);
  EXPECT_TRUE(state.status & proto::kStatusHeartbeatLost);
  EXPECT_FALSE(state.status & proto::kStatusEnabled);
  EXPECT_EQ(state.sequence, 7);
}

TEST(FirmwareStatus, StartsWithTorqueOffBeforeAnyCommand)
{
  // A device that energises at boot would move the arm before anything
  // had told it where to go.
  Controller controller;
  EXPECT_FALSE(controller.output().torque_enabled);
}

TEST(FirmwareInfo, ReportsItsOwnVersionAndTimeout)
{
  Controller controller;
  const auto info = controller.info();
  EXPECT_EQ(info.protocol_version, proto::kProtocolVersion);
  EXPECT_EQ(info.joint_count, proto::kJointCount);
  EXPECT_EQ(info.heartbeat_timeout_ms, kHeartbeatTimeoutMs);
}
