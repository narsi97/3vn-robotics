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
/// The hardware interface, driven against a fake device.
///
/// This is the argument for the transport seam, made concrete. Every
/// behaviour that matters on a physical robot - the device going quiet,
/// a corrupted line, the first cycle before any command exists,
/// releasing torque on shutdown - is provoked here deliberately, with no
/// serial port and no ESP32. On a bench these are the failures you wait
/// for and cannot reproduce.

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "threevn_hardware/esp32_system_interface.hpp"
#include "threevn_hardware/protocol.hpp"

using threevn::hardware::Esp32SystemInterface;
using threevn::hardware::Transport;
namespace proto = threevn::protocol;

namespace
{

/// A Transport backed by two byte queues.
class FakeTransport : public Transport
{
public:
  bool open() override {open_ = true; return open_ok_;}
  void close() override {open_ = false;}
  bool isOpen() const override {return open_;}

  int read(uint8_t * buffer, std::size_t size) override
  {
    if (read_fails_) {return -1;}
    std::size_t n = 0;
    while (n < size && !to_host_.empty()) {
      buffer[n++] = to_host_.front();
      to_host_.erase(to_host_.begin());
    }
    return static_cast<int>(n);
  }

  bool write(const uint8_t * buffer, std::size_t size) override
  {
    if (write_fails_) {last_error_ = "device not draining"; return false;}
    to_device_.insert(to_device_.end(), buffer, buffer + size);
    return true;
  }

  std::string lastError() const override {return last_error_;}

  // -- test controls --

  void queueFromDevice(const std::vector<uint8_t> & bytes)
  {
    to_host_.insert(to_host_.end(), bytes.begin(), bytes.end());
  }

  void queueState(const proto::StatePayload & state)
  {
    uint8_t frame[proto::kMaxFrameSize];
    const size_t n = proto::encodeState(state, frame, sizeof(frame));
    queueFromDevice({frame, frame + n});
  }

  /// Every command frame the interface has sent.
  std::vector<proto::CommandPayload> sentCommands()
  {
    std::vector<proto::CommandPayload> out;
    proto::FrameParser parser;
    parser.pushBuffer(
      to_device_.data(), to_device_.size(), [&out](const proto::FrameParser & p) {
        if (p.type() == proto::MessageType::kCommand) {
          proto::CommandPayload cmd;
          if (proto::decodeCommand(p.payload(), p.payloadSize(), cmd)) {
            out.push_back(cmd);
          }
        }
      });
    return out;
  }

  void failReads(bool value) {read_fails_ = value;}
  void failWrites(bool value) {write_fails_ = value;}
  void failOpen() {open_ok_ = false;}

private:
  std::vector<uint8_t> to_host_;
  std::vector<uint8_t> to_device_;
  bool open_ = false;
  bool open_ok_ = true;
  bool read_fails_ = false;
  bool write_fails_ = false;
  std::string last_error_;
};

/// A minimal HardwareInfo matching the real ros2_control block.
hardware_interface::HardwareInfo makeInfo()
{
  hardware_interface::HardwareInfo info;
  info.name = "threevn_arm";
  info.type = "system";
  info.hardware_parameters = {
    {"transport", "serial"},
    {"endpoint", "/dev/null"},
    {"baud", "921600"},
    {"control_rate_hz", "50"},
  };
  for (const auto & name : {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_joint", "gripper_left_finger_joint"})
  {
    // Fields set by name rather than brace-initialised: InterfaceInfo
    // gains members between ros2_control releases, and an aggregate
    // initialiser breaks on every one of them.
    auto make_interface = [](const std::string & interface_name) {
        hardware_interface::InterfaceInfo iface;
        iface.name = interface_name;
        iface.data_type = "double";
        return iface;
      };

    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.command_interfaces.push_back(make_interface("position"));
    joint.state_interfaces.push_back(make_interface("position"));
    joint.state_interfaces.push_back(make_interface("velocity"));
    info.joints.push_back(joint);
  }
  return info;
}

/// Bring an interface up with a fake device attached.
struct Rig
{
  Esp32SystemInterface interface;
  FakeTransport * fake = nullptr;

  explicit Rig(hardware_interface::HardwareInfo info = makeInfo())
  {
    auto transport = std::make_unique<FakeTransport>();
    fake = transport.get();
    interface.setTransport(std::move(transport));

    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = std::move(info);
    init_ok = interface.on_init(params) ==
      hardware_interface::CallbackReturn::SUCCESS;
  }

  bool configureAndActivate()
  {
    const rclcpp_lifecycle::State unconfigured;
    return interface.on_configure(unconfigured) ==
           hardware_interface::CallbackReturn::SUCCESS &&
           interface.on_activate(unconfigured) ==
           hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type cycle()
  {
    const rclcpp::Time now(0, 0, RCL_ROS_TIME);
    const rclcpp::Duration period(0, 20000000);   // 20 ms == 50 Hz
    const auto r = interface.read(now, period);
    interface.write(now, period);
    return r;
  }

  bool init_ok = false;
};

}  // namespace

// ------------------------------------------------------------- init --

TEST(Init, AcceptsTheRealDescription)
{
  Rig rig;
  EXPECT_TRUE(rig.init_ok);
}

TEST(Init, RefusesAJointCountTheProtocolCannotCarry)
{
  // The wire protocol carries exactly five joints. A description with a
  // different number means it and the firmware disagree about what they
  // are moving, so commanding a subset would be worse than refusing.
  auto info = makeInfo();
  info.joints.pop_back();
  Rig rig(info);
  EXPECT_FALSE(rig.init_ok);
}

TEST(Init, RefusesAnUnimplementedTransport)
{
  // TCP is planned, not written. Accepting the parameter and behaving as
  // serial would be a configuration that silently does something else.
  auto info = makeInfo();
  info.hardware_parameters["transport"] = "tcp";
  Rig rig(info);
  EXPECT_FALSE(rig.init_ok);
}

TEST(Configure, FailsLoudlyWhenTheDeviceCannotBeOpened)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  rig.fake->failOpen();
  const rclcpp_lifecycle::State state;
  EXPECT_EQ(
    rig.interface.on_configure(state), hardware_interface::CallbackReturn::ERROR);
}

// -------------------------------------------------------------- read --

TEST(Read, AppliesPositionsFromAStateFrame)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());

  proto::StatePayload state;
  state.position_micro[proto::kShoulderPan] = proto::toMicro(0.4);
  state.position_micro[proto::kElbow] = proto::toMicro(-1.2);
  state.status = proto::kStatusEnabled;
  rig.fake->queueState(state);

  EXPECT_EQ(rig.cycle(), hardware_interface::return_type::OK);
  EXPECT_EQ(rig.interface.framesReceived(), 1u);
  EXPECT_EQ(rig.interface.deviceStatus(), proto::kStatusEnabled);
}

TEST(Read, DeclaresTheLinkLostAfterSustainedSilence)
{
  // The failure that matters most: a device that stops talking while the
  // port stays open. Without this the controllers would keep running
  // against a frozen snapshot of a robot that may be doing anything.
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());

  proto::StatePayload state;
  rig.fake->queueState(state);
  EXPECT_EQ(rig.cycle(), hardware_interface::return_type::OK);
  EXPECT_FALSE(rig.interface.linkLost());

  // 0.5 s of silence at 50 Hz.
  hardware_interface::return_type last = hardware_interface::return_type::OK;
  for (int i = 0; i < 30; ++i) {
    last = rig.cycle();
                                                     }

  EXPECT_TRUE(rig.interface.linkLost());
  EXPECT_EQ(last, hardware_interface::return_type::ERROR)
    << "a lost link must be reported to controller_manager, not just logged";
}

TEST(Read, RecoversWhenTheDeviceComesBack)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());

  for (int i = 0; i < 30; ++i) {
    rig.cycle();
                                              }
  ASSERT_TRUE(rig.interface.linkLost());

  proto::StatePayload state;
  state.position_micro[proto::kWrist] = proto::toMicro(0.2);
  rig.fake->queueState(state);

  EXPECT_EQ(rig.cycle(), hardware_interface::return_type::OK);
  EXPECT_FALSE(rig.interface.linkLost());
}

TEST(Read, IgnoresGarbageWithoutLosingTheLink)
{
  // Line noise must not be mistaken for a disconnection, or every
  // electrical glitch would fault the arm.
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());

  rig.fake->queueFromDevice({0x00, 0xFF, 0x12, 0x33, 0x99, 0x56, 0x01});
  proto::StatePayload state;
  state.position_micro[proto::kElbow] = proto::toMicro(0.33);
  rig.fake->queueState(state);

  EXPECT_EQ(rig.cycle(), hardware_interface::return_type::OK);
  EXPECT_EQ(rig.interface.framesReceived(), 1u);
  EXPECT_FALSE(rig.interface.linkLost());
}

TEST(Read, ReportsAnErrorWhenTheTransportItselfFails)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.fake->close();
  EXPECT_EQ(rig.cycle(), hardware_interface::return_type::ERROR);
}

// ------------------------------------------------------------- write --

TEST(Write, HoldsMeasuredPositionBeforeAnyCommandExists)
{
  // On the first cycles the command interfaces are NaN. Sending zero
  // would drive every joint to its reference pose the instant the
  // hardware activated - a real arm would slam to a new pose on startup.
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());

  proto::StatePayload state;
  state.position_micro[proto::kShoulderLift] = proto::toMicro(0.85);
  rig.fake->queueState(state);
  rig.cycle();

  const auto commands = rig.fake->sentCommands();
  ASSERT_FALSE(commands.empty());
  EXPECT_EQ(
    commands.back().position_micro[proto::kShoulderLift], proto::toMicro(0.85))
    << "the first command should hold where the arm already is";
}

TEST(Write, SendsACommandEveryCycleAsTheHeartbeat)
{
  // The command frame doubles as the heartbeat, so it goes out even when
  // nothing changed. Sending only on change would let a stationary arm
  // look like a dead host and trip the device's timeout.
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());

  for (int i = 0; i < 5; ++i) {
    rig.cycle();
                                             }
  EXPECT_GE(rig.fake->sentCommands().size(), 5u);
}

TEST(Write, SetsTheEnableFlagWhileActive)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.cycle();

  const auto commands = rig.fake->sentCommands();
  ASSERT_FALSE(commands.empty());
  EXPECT_TRUE(commands.back().flags & proto::kFlagEnable);
}

TEST(Write, ReleasesTorqueOnDeactivate)
{
  // Leaving servos energised after deactivation means an arm holding a
  // pose that nothing is controlling any more.
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.cycle();

  const rclcpp_lifecycle::State state;
  ASSERT_EQ(
    rig.interface.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);

  const auto commands = rig.fake->sentCommands();
  ASSERT_FALSE(commands.empty());
  EXPECT_FALSE(commands.back().flags & proto::kFlagEnable)
    << "the last frame out should clear the enable bit";
}

TEST(Write, ReportsAnErrorWhenTheDeviceStopsDraining)
{
  Rig rig;
  ASSERT_TRUE(rig.init_ok);
  ASSERT_TRUE(rig.configureAndActivate());
  rig.fake->failWrites(true);

  const rclcpp::Time now(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period(0, 20000000);
  EXPECT_EQ(
    rig.interface.write(now, period), hardware_interface::return_type::ERROR);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
