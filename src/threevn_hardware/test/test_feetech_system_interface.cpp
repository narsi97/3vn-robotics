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
/// The Feetech hardware interface, driven against a simulated bus.
///
/// Everything here would otherwise be learned with a powered arm on the
/// bench: a servo missing from the bus, an arm that leaps when torque
/// comes on, a joint assembled the other way round, a cable that works
/// loose mid-motion. Each is provoked deliberately, with nothing
/// plugged in.
///
/// States and commands are read and written through the SAME handles
/// the controller manager uses, not through the class's own members.
/// A hardware interface can do everything right on the wire and still
/// never tell the framework: the controllers then see a robot that
/// never moves, and command one that never listens.

#include <gmock/gmock.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "support/fake_feetech_bus.hpp"
#include "threevn_hardware/feetech_protocol.hpp"
#include "threevn_hardware/feetech_system_interface.hpp"

using threevn::hardware::FeetechSystemInterface;
using threevn::hardware::Transport;
namespace ft = threevn::feetech;
using ::testing::HasSubstr;

namespace
{

/// The six joints of the SO-101, with the bus ids the profile gives them.
const std::vector<std::pair<std::string, int>> kJoints = {
  {"shoulder_pan", 1}, {"shoulder_lift", 2}, {"elbow_flex", 3},
  {"wrist_flex", 4}, {"wrist_roll", 5}, {"gripper", 6},
};

/// A Transport whose far end is the simulated bus, in this process.
class BusTransport : public Transport
{
public:
  bool open() override {open_ = open_ok_; return open_;}
  void close() override {open_ = false;}
  bool isOpen() const override {return open_;}

  int read(uint8_t * buffer, std::size_t size) override
  {
    std::size_t n = 0;
    while (n < size && !to_host_.empty()) {
      buffer[n++] = to_host_.front();
      to_host_.erase(to_host_.begin());
    }
    return static_cast<int>(n);
  }

  bool write(const uint8_t * buffer, std::size_t size) override
  {
    if (write_fails_) {
      return false;
    }
    const auto reply = bus.exchange(buffer, size);
    if (!deaf_) {
      to_host_.insert(to_host_.end(), reply.begin(), reply.end());
    }
    return true;
  }

  std::string lastError() const override {return open_ok_ ? "" : "no such device";}

  // -- test controls --
  void failOpen() {open_ok_ = false;}
  void failWrites(bool value) {write_fails_ = value;}
  /// The cable from the bus back to the host comes out: servos still
  /// hear and obey, and nothing they say arrives.
  void dropReplies(bool value) {deaf_ = value;}

  ft::FakeBus bus;

private:
  std::vector<uint8_t> to_host_;
  bool open_ = false;
  bool open_ok_ = true;
  bool write_fails_ = false;
  bool deaf_ = false;
};

/// Effort at 100 % load: the SO-101's derived joint limit, N.m.
constexpr double kFullLoad = 1.3002;

/// A HardwareInfo shaped like the <ros2_control> block describe() emits.
///
/// `self_reporting` adds what the SO-101's servos measure about
/// themselves: an effort and a temperature state interface per joint.
/// Without it, the shape the interface had before it read either.
hardware_interface::HardwareInfo makeInfo(bool self_reporting = true)
{
  hardware_interface::HardwareInfo info;
  info.name = "so101_follower";
  info.type = "system";
  info.hardware_parameters = {
    {"transport", "tcp"},
    {"endpoint", "127.0.0.1:6560"},
    {"baud", "1000000"},
    {"control_rate_hz", "50"},
    // Short, so a test about silence does not spend its time waiting.
    {"response_timeout_ms", "1"},
  };
  auto make_interface = [](const std::string & interface_name) {
      hardware_interface::InterfaceInfo iface;
      iface.name = interface_name;
      iface.data_type = "double";
      return iface;
    };
  for (const auto & entry : kJoints) {
    hardware_interface::ComponentInfo joint;
    joint.name = entry.first;
    joint.type = "joint";
    joint.parameters = {{"id", std::to_string(entry.second)}};
    joint.command_interfaces.push_back(make_interface("position"));
    joint.state_interfaces.push_back(make_interface("position"));
    joint.state_interfaces.push_back(make_interface("velocity"));
    if (self_reporting) {
      joint.parameters["full_load_effort"] = std::to_string(kFullLoad);
      joint.state_interfaces.push_back(make_interface("effort"));
      joint.state_interfaces.push_back(make_interface("temperature"));
    }
    info.joints.push_back(joint);
  }
  return info;
}

struct Rig
{
  FeetechSystemInterface interface;
  BusTransport * link = nullptr;

  explicit Rig(hardware_interface::HardwareInfo info = makeInfo())
  {
    auto transport = std::make_unique<BusTransport>();
    link = transport.get();
    for (const auto & entry : kJoints) {
      link->bus.addServo(static_cast<uint8_t>(entry.second));
    }
    interface.setTransport(std::move(transport));

    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = std::move(info);
    init_ok = interface.on_init(params) == hardware_interface::CallbackReturn::SUCCESS;
    if (init_ok) {
      // What the resource manager does next: create the handles the
      // controllers will read and write through.
      states = interface.on_export_state_interfaces();
      commands = interface.on_export_command_interfaces();
    }
  }

  ft::FakeBus & bus() {return link->bus;}

  bool configure()
  {
    return interface.on_configure(rclcpp_lifecycle::State()) ==
           hardware_interface::CallbackReturn::SUCCESS;
  }

  bool activate()
  {
    return interface.on_activate(rclcpp_lifecycle::State()) ==
           hardware_interface::CallbackReturn::SUCCESS;
  }

  bool configureAndActivate() {return configure() && activate();}

  /// One control cycle, with 20 ms passing for the servos.
  hardware_interface::return_type cycle()
  {
    const rclcpp::Time now(0, 0, RCL_ROS_TIME);
    const rclcpp::Duration period(0, 20000000);
    const auto result = interface.read(now, period);
    interface.write(now, period);
    link->bus.step(0.02);
    return result;
  }

  void cycles(int count)
  {
    for (int i = 0; i < count; ++i) {
      cycle();
    }
  }

  double state(const std::string & joint, const std::string & what = "position")
  {
    return interface.get_state<double>(joint + "/" + what);
  }

  void command(const std::string & joint, double radians)
  {
    interface.set_command(joint + "/position", radians);
  }

  bool init_ok = false;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> states;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
};

}  // namespace

// ------------------------------------------------------------- init --

TEST(Init, AcceptsTheRealDescription)
{
  Rig rig;
  EXPECT_TRUE(rig.init_ok) << rig.interface.lastFault();
}

TEST(Init, RefusesAJointThatDoesNotSayWhichServoItIs)
{
  // No default id. Guessing one risks two joints on the same servo.
  auto info = makeInfo();
  info.joints[2].parameters.clear();
  Rig rig(info);
  EXPECT_FALSE(rig.init_ok);
  EXPECT_THAT(rig.interface.lastFault(), HasSubstr("elbow_flex"));
}

TEST(Init, RefusesTwoJointsOnOneBusId)
{
  // Two servos with one id both answer every packet for it, and the
  // replies collide on the wire.
  auto info = makeInfo();
  info.joints[4].parameters["id"] = "2";
  Rig rig(info);
  EXPECT_FALSE(rig.init_ok);
  EXPECT_THAT(rig.interface.lastFault(), HasSubstr("wrist_roll"));
  EXPECT_THAT(rig.interface.lastFault(), HasSubstr("id 2"));
}

TEST(Init, RefusesATransportItDoesNotHave)
{
  auto info = makeInfo();
  info.hardware_parameters["transport"] = "bluetooth";
  Rig rig(info);
  EXPECT_FALSE(rig.init_ok);
}

TEST(Init, RefusesADirectionThatIsNotASign)
{
  auto info = makeInfo();
  info.joints[0].parameters["direction"] = "0";
  Rig rig(info);
  EXPECT_FALSE(rig.init_ok);
}

TEST(Init, RefusesAJointWithNoVelocityState)
{
  auto info = makeInfo();
  auto & states = info.joints[1].state_interfaces;
  states.erase(
    std::remove_if(
      states.begin(), states.end(),
      [](const hardware_interface::InterfaceInfo & i) {return i.name == "velocity";}),
    states.end());
  Rig rig(info);
  EXPECT_FALSE(rig.init_ok);
}

TEST(Init, RefusesAnEffortStateWithNoFullLoadEffort)
{
  // The servo reports a percentage. Without knowing what 100 % is, any
  // N.m put on /joint_states would look measured and be invented.
  auto info = makeInfo();
  info.joints[3].parameters.erase("full_load_effort");
  Rig rig(info);
  EXPECT_FALSE(rig.init_ok);
  EXPECT_THAT(rig.interface.lastFault(), HasSubstr("wrist_flex"));
  EXPECT_THAT(rig.interface.lastFault(), HasSubstr("full_load_effort"));
}

TEST(Init, LoadAndTemperatureAreOptional)
{
  // A description that asks for neither gets the interface as it was.
  Rig rig(makeInfo(false));
  ASSERT_TRUE(rig.init_ok) << rig.interface.lastFault();
  ASSERT_TRUE(rig.configureAndActivate());
  rig.command("shoulder_pan", 0.5);
  rig.cycles(100);
  EXPECT_NEAR(rig.state("shoulder_pan"), 0.5, ft::kRadiansPerCount);
}

// ---------------------------------------------------- self-reporting --
//
// What the 3VN arm cannot tell you: that it has been stopped. A bus
// servo held still keeps trying, and says how hard.

TEST(Load, AJointAtRestPushesOnNothing)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.cycles(5);
  EXPECT_NEAR(rig.state("shoulder_lift", "effort"), 0.0, 1e-9);
}

TEST(Load, AJointHeldStillReportsItsFullEffort)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.bus().servo(2).blocked = true;
  rig.command("shoulder_lift", 0.5);
  rig.cycles(25);   // half a second of pushing

  EXPECT_NEAR(rig.state("shoulder_lift"), 0.0, 1e-9) << "it has not moved";
  EXPECT_NEAR(rig.state("shoulder_lift", "effort"), kFullLoad, 1e-6)
    << "and it is pushing as hard as it can, toward the goal";
  EXPECT_NEAR(rig.state("elbow_flex", "effort"), 0.0, 1e-9) << "nobody else is";
}

TEST(Load, PushingTheOtherWayIsANegativeEffort)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.bus().servo(2).blocked = true;
  rig.command("shoulder_lift", -0.5);
  rig.cycles(5);
  EXPECT_NEAR(rig.state("shoulder_lift", "effort"), -kFullLoad, 1e-6);
}

TEST(Load, EffortIsInTheJointsDirectionNotTheServos)
{
  // A servo mounted the other way round pushes toward LOWER counts to
  // drive the joint positive. The effort reported must still be
  // positive, or a strain check would read a push as a pull.
  auto info = makeInfo();
  info.joints[1].parameters["direction"] = "-1";
  Rig rig(info);
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.bus().servo(2).blocked = true;
  rig.command("shoulder_lift", 0.5);
  rig.cycles(5);
  EXPECT_LT(rig.bus().goal(2), ft::kCentreCount) << "the servo is driven down";
  EXPECT_NEAR(rig.state("shoulder_lift", "effort"), kFullLoad, 1e-6);
}

TEST(Load, ReleasedTheJointMovesAndTheEffortFalls)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.bus().servo(2).blocked = true;
  rig.command("shoulder_lift", 0.5);
  rig.cycles(10);
  ASSERT_GT(rig.state("shoulder_lift", "effort"), 1.0);
  rig.bus().servo(2).blocked = false;
  rig.cycles(100);
  EXPECT_NEAR(rig.state("shoulder_lift"), 0.5, ft::kRadiansPerCount);
  EXPECT_NEAR(rig.state("shoulder_lift", "effort"), 0.0, 1e-9);
}

TEST(Temperature, IsTheServosOwnReading)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  rig.bus().servo(4).table[ft::reg::kPresentTemperature] = 58;
  ASSERT_TRUE(rig.configureAndActivate());
  rig.cycles(2);
  EXPECT_DOUBLE_EQ(rig.state("wrist_flex", "temperature"), 58.0);
  EXPECT_DOUBLE_EQ(rig.state("shoulder_pan", "temperature"), 25.0);
}

// -------------------------------------------------------- configure --

TEST(Configure, FailsWithTheReasonWhenTheBusCannotBeOpened)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  rig.link->failOpen();
  EXPECT_FALSE(rig.configure());
  EXPECT_THAT(rig.interface.lastFault(), HasSubstr("no such device"));
}

TEST(Configure, NamesTheServoThatIsNotAnswering)
{
  // Five of six answering is an arm with one joint nobody controls.
  // The message has to say WHICH, by the joint's name and its id: on a
  // bench that is the difference between reseating one connector and
  // checking six.
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  rig.bus().servo(4).answering = false;
  EXPECT_FALSE(rig.configure());
  EXPECT_THAT(rig.interface.lastFault(), HasSubstr("wrist_flex (id 4)"));
  EXPECT_THAT(rig.interface.lastFault(), ::testing::Not(HasSubstr("elbow_flex")));
}

TEST(Configure, ReportsWhereTheArmIsBeforeAnythingIsPowered)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  rig.bus().servo(2).position = 3072;       // a quarter turn from centre
  ASSERT_TRUE(rig.configure());
  EXPECT_NEAR(rig.state("shoulder_lift"), ft::kPi / 2.0, 1e-9);
  EXPECT_FALSE(rig.bus().torqueEnabled(2)) << "configured is not powered";
}

// --------------------------------------------------------- activate --

TEST(Activate, DoesNotMoveTheArm)
{
  // THE test for a real arm. It is found wherever it was left -- here,
  // every joint somewhere different and none at centre -- and turning
  // the torque on must not move it at all.
  //
  // The simulated servos hold a goal of centre until told otherwise, so
  // an interface that enabled torque before writing goals would send
  // all six there at full speed.
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  const std::vector<int> found = {1500, 2600, 900, 3100, 2000, 2300};
  for (std::size_t i = 0; i < found.size(); ++i) {
    rig.bus().servo(static_cast<uint8_t>(i + 1)).position = found[i];
  }
  ASSERT_TRUE(rig.configureAndActivate()) << rig.interface.lastFault();

  auto expect_unmoved = [&rig, &found](const char * when) {
      for (std::size_t i = 0; i < found.size(); ++i) {
        const auto id = static_cast<uint8_t>(i + 1);
        EXPECT_NEAR(rig.bus().servo(id).position, found[i], 0.5)
          << "servo " << static_cast<int>(id) << " moved " << when;
        EXPECT_EQ(rig.bus().goal(id), found[i]) << when;
      }
    };

  // Time passes BEFORE the first control cycle, as it does for real:
  // activation and the next write() are separate ticks. And the check
  // is made HERE, not only at the end.
  //
  // Both halves were learned by breaking the code under test. Written
  // without this step, the test could not fail: the first write()
  // repaired a wrong goal before the servos had any time to act on it.
  // Written with the step but checking only afterwards, it still could
  // not: the arm lurched toward centre and was then commanded back, and
  // arrived where it started. A jerk that ends where it began is still
  // a jerk.
  rig.bus().step(0.05);
  expect_unmoved("between activation and the first cycle");

  rig.cycles(50);    // then a second with no controller commanding anything
  expect_unmoved("with nothing commanding it");
  for (const auto & entry : kJoints) {
    EXPECT_TRUE(rig.bus().torqueEnabled(static_cast<uint8_t>(entry.second)));
  }
}

TEST(Activate, HandsTheControllersTheirStartingPoint)
{
  // A controller starts from the command interface's value. If that is
  // zero, its first trajectory begins at a pose the arm is not in.
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  rig.bus().servo(3).position = 1024;       // elbow a quarter turn back
  ASSERT_TRUE(rig.configureAndActivate());
  EXPECT_NEAR(
    rig.interface.get_command<double>("elbow_flex/position"), -ft::kPi / 2.0, 1e-9);
}

TEST(Activate, RefusesToPowerAnArmItCannotLocate)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configure());
  rig.link->dropReplies(true);
  EXPECT_FALSE(rig.activate());
  for (const auto & entry : kJoints) {
    EXPECT_FALSE(rig.bus().torqueEnabled(static_cast<uint8_t>(entry.second)));
  }
}

// ------------------------------------------------------ read / write --

TEST(Write, ACommandReachesTheServoAndTheStateComesBack)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());

  rig.command("shoulder_pan", 0.5);
  rig.cycles(100);   // two seconds: far longer than the move takes

  // 0.5 rad is 325.9 counts from centre.
  EXPECT_EQ(rig.bus().goal(1), 2048 + 326);
  EXPECT_NEAR(rig.state("shoulder_pan"), 0.5, ft::kRadiansPerCount);
  EXPECT_NEAR(rig.state("shoulder_pan", "velocity"), 0.0, 1e-9) << "it has arrived";
  // And nothing else went anywhere.
  EXPECT_NEAR(rig.state("elbow_flex"), 0.0, 1e-9);
}

TEST(Write, VelocityIsReportedWhileItMoves)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.command("wrist_flex", -1.0);
  rig.cycles(3);
  EXPECT_LT(rig.state("wrist_flex", "velocity"), -0.1);
}

TEST(Write, AJointAssembledTheOtherWayRoundStillTurnsTheRightWay)
{
  // direction = -1: positive radians are DECREASING counts on this
  // servo. The joint must still read and move in the description's
  // sense, or every pose is mirrored on the real arm.
  auto info = makeInfo();
  info.joints[1].parameters["direction"] = "-1";
  info.joints[1].parameters["zero_count"] = "2100";
  Rig rig(info);
  ASSERT_TRUE(rig.init_ok);
  rig.bus().servo(2).position = 2100;
  ASSERT_TRUE(rig.configureAndActivate());
  EXPECT_NEAR(rig.state("shoulder_lift"), 0.0, 1e-9);

  rig.command("shoulder_lift", 0.5);
  rig.cycles(100);
  EXPECT_EQ(rig.bus().goal(2), 2100 - 326);
  EXPECT_NEAR(rig.state("shoulder_lift"), 0.5, ft::kRadiansPerCount);
}

TEST(Write, ACommandBeyondTheEncoderIsClampedNotWrapped)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.command("wrist_roll", 10.0);
  rig.cycle();
  EXPECT_EQ(rig.bus().goal(5), 4095);
}

TEST(Write, AnUncommandedJointHoldsWhereItWasSeen)
{
  // A NaN command means no controller has claimed the joint. It holds
  // its measured position; it does not go to zero.
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  rig.bus().servo(6).position = 2500;
  ASSERT_TRUE(rig.configureAndActivate());
  rig.command("gripper", std::nan(""));
  rig.cycles(25);
  EXPECT_EQ(rig.bus().goal(6), 2500);
}

TEST(Write, ReportsAFailedWrite)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.link->failWrites(true);
  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period(0, 20000000);
  EXPECT_EQ(rig.interface.write(now, period), hardware_interface::return_type::ERROR);
}

// --------------------------------------------------------------- link --

TEST(Link, OneMissedReplyIsAGlitchAndTheStateStands)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  rig.bus().servo(1).position = 2400;
  ASSERT_TRUE(rig.configureAndActivate());
  const double before = rig.state("shoulder_pan");

  rig.bus().corruptNextReply();
  EXPECT_EQ(rig.cycle(), hardware_interface::return_type::OK);
  EXPECT_EQ(rig.interface.staleCycles(), 1u);
  EXPECT_DOUBLE_EQ(rig.state("shoulder_pan"), before) << "a bad packet is not data";

  EXPECT_EQ(rig.cycle(), hardware_interface::return_type::OK);
  EXPECT_EQ(rig.interface.staleCycles(), 0u);
}

TEST(Link, AServoThatStaysQuietIsAnErrorNotALogLine)
{
  // The failure that matters most: one connector works loose while the
  // rest of the bus carries on. Without this the controllers would run
  // against a snapshot of a joint that could be anywhere.
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.bus().servo(3).answering = false;

  hardware_interface::return_type result = hardware_interface::return_type::OK;
  int cycles = 0;
  while (result == hardware_interface::return_type::OK && cycles < 200) {
    result = rig.cycle();
    ++cycles;
  }
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
  EXPECT_TRUE(rig.interface.linkLost());
  // Half a second at 50 Hz: long enough to ride out a glitch, short
  // enough that the arm has not gone far.
  EXPECT_EQ(cycles, 25);
}

TEST(Link, RecoversWhenTheServoComesBack)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.bus().servo(3).answering = false;
  rig.cycles(30);
  ASSERT_TRUE(rig.interface.linkLost());

  rig.bus().servo(3).answering = true;
  EXPECT_EQ(rig.cycle(), hardware_interface::return_type::OK);
  EXPECT_FALSE(rig.interface.linkLost());
}

// ----------------------------------------------------------- shutdown --

TEST(Shutdown, DeactivationReleasesTheServos)
{
  // Energised servos with nothing commanding them are an arm holding a
  // pose nobody chose. On this arm, released means limp: it is driven
  // to its rest pose first.
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  ASSERT_TRUE(rig.bus().torqueEnabled(1));
  rig.interface.on_deactivate(rclcpp_lifecycle::State());
  for (const auto & entry : kJoints) {
    EXPECT_FALSE(rig.bus().torqueEnabled(static_cast<uint8_t>(entry.second)));
  }
}

TEST(Shutdown, CleanupClosesTheBus)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.interface.on_cleanup(rclcpp_lifecycle::State());
  EXPECT_FALSE(rig.link->isOpen());
  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period(0, 20000000);
  EXPECT_EQ(rig.interface.read(now, period), hardware_interface::return_type::ERROR);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleMock(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
