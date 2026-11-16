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
/// The simulated servo bus, tested as the instrument it is.
///
/// Every test of the hardware interface leans on this fake. If it
/// answered a servo that is not there, or let a limp servo move, those
/// tests would pass for a reason that has nothing to do with the code
/// they are about. So the fake is held to its own claims first.

#include <gmock/gmock.h>

#include <cmath>
#include <vector>

#include "support/fake_feetech_bus.hpp"
#include "threevn_hardware/feetech_protocol.hpp"

namespace ft = threevn::feetech;
using Bytes = std::vector<uint8_t>;

namespace
{

Bytes send(ft::FakeBus & bus, const Bytes & packet)
{
  return bus.exchange(packet.data(), packet.size());
}

std::vector<ft::Status> parse(const Bytes & bytes)
{
  ft::StatusParser parser;
  parser.push(bytes.data(), bytes.size());
  std::vector<ft::Status> out;
  ft::Status status;
  while (parser.next(status)) {
    out.push_back(status);
  }
  return out;
}

int presentPosition(ft::FakeBus & bus, uint8_t id)
{
  const auto replies = parse(send(bus, ft::encodeRead(id, ft::reg::kPresentPosition, 2)));
  EXPECT_EQ(replies.size(), 1u);
  return replies.empty() ? -1 : ft::fromBytes(replies[0].params[0], replies[0].params[1]);
}

void enableTorque(ft::FakeBus & bus, uint8_t id)
{
  send(bus, ft::encodeWrite(id, ft::reg::kTorqueEnable, {1}));
}

void setGoal(ft::FakeBus & bus, uint8_t id, int count)
{
  send(
    bus, ft::encodeWrite(id, ft::reg::kGoalPosition, ft::toBytes(static_cast<uint16_t>(count))));
}

}  // namespace

// -------------------------------------------------------- who answers --

TEST(Presence, AServoAnswersItsOwnPing)
{
  ft::FakeBus bus;
  bus.addServo(3);
  const auto replies = parse(send(bus, ft::encodePing(3)));
  ASSERT_EQ(replies.size(), 1u);
  EXPECT_EQ(replies[0].id, 3);
  EXPECT_EQ(replies[0].error, 0);
}

TEST(Presence, NobodyAnswersForAServoThatIsNotThere)
{
  ft::FakeBus bus;
  bus.addServo(3);
  EXPECT_TRUE(send(bus, ft::encodePing(4)).empty());
}

TEST(Presence, AnUnpluggedServoIsSilent)
{
  ft::FakeBus bus;
  bus.addServo(3);
  bus.servo(3).answering = false;
  EXPECT_TRUE(send(bus, ft::encodePing(3)).empty());
  EXPECT_TRUE(send(bus, ft::encodeRead(3, ft::reg::kPresentPosition, 2)).empty());
}

TEST(Presence, ABroadcastGetsNoReply)
{
  // Six servos answering one packet at once would be six drivers on one
  // wire. Real servos stay quiet on the broadcast id, and so does this.
  ft::FakeBus bus;
  bus.addServo(1);
  bus.addServo(2);
  EXPECT_TRUE(send(bus, ft::encodePing(ft::kBroadcastId)).empty());
}

TEST(Presence, ACorruptInstructionIsIgnored)
{
  ft::FakeBus bus;
  bus.addServo(1);
  Bytes ping = ft::encodePing(1);
  ping.back() ^= 0xFF;
  EXPECT_TRUE(send(bus, ping).empty());
  EXPECT_EQ(bus.instructionsSeen(), 0u);
}

TEST(Presence, AnInstructionSplitAcrossTwoDeliveriesIsStillOne)
{
  ft::FakeBus bus;
  bus.addServo(1);
  const Bytes ping = ft::encodePing(1);
  EXPECT_TRUE(bus.exchange(ping.data(), 3).empty());
  EXPECT_EQ(parse(bus.exchange(ping.data() + 3, ping.size() - 3)).size(), 1u);
}

TEST(Presence, AServoGivenANewIdAnswersThereAndNowhereElse)
{
  // How an arm is commissioned: every servo ships as id 1, and is
  // renumbered while it is alone on the bus.
  ft::FakeBus bus;
  bus.addServo(1);
  send(bus, ft::encodeWrite(1, ft::reg::kId, {5}));
  EXPECT_TRUE(send(bus, ft::encodePing(1)).empty());
  EXPECT_EQ(parse(send(bus, ft::encodePing(5))).size(), 1u);
}

// ------------------------------------------------------------- motion --

TEST(Motion, ALimpServoDoesNotMove)
{
  ft::FakeBus bus;
  bus.addServo(1, 1000);
  setGoal(bus, 1, 3000);
  bus.step(5.0);
  EXPECT_EQ(presentPosition(bus, 1), 1000);
}

TEST(Motion, TheGoalDoesNotFollowTheShaftWhileLimp)
{
  // The strict half of a question the real servo has not answered yet.
  // A servo added at 1000 still has its goal at centre, so enabling
  // torque without first writing a goal moves it. An interface that
  // gets the order wrong fails against this fake.
  ft::FakeBus bus;
  bus.addServo(1, 1000);
  EXPECT_EQ(bus.goal(1), ft::kCentreCount);
  enableTorque(bus, 1);
  bus.step(5.0);
  EXPECT_EQ(presentPosition(bus, 1), ft::kCentreCount);
}

TEST(Motion, AServoApproachesItsGoalAtAFiniteSpeed)
{
  ft::FakeBus bus;
  bus.addServo(1, 1000);
  setGoal(bus, 1, 3000);
  enableTorque(bus, 1);

  bus.step(0.1);
  const int after_a_tenth = presentPosition(bus, 1);
  EXPECT_GT(after_a_tenth, 1000);
  EXPECT_LE(after_a_tenth, 1000 + static_cast<int>(ft::FakeBus::kFreeSpeed * 0.1) + 1);

  bus.step(5.0);
  EXPECT_EQ(presentPosition(bus, 1), 3000) << "it must arrive, and stop exactly there";
}

TEST(Motion, AGoalSpeedSlowsItDown)
{
  ft::FakeBus bus;
  bus.addServo(1, 1000);
  send(bus, ft::encodeWrite(1, ft::reg::kGoalSpeed, ft::toBytes(100)));
  setGoal(bus, 1, 3000);
  enableTorque(bus, 1);
  bus.step(1.0);
  EXPECT_EQ(presentPosition(bus, 1), 1100);
}

TEST(Motion, GoingBackwardsReportsANegativeSpeed)
{
  ft::FakeBus bus;
  bus.addServo(1, 3000);
  setGoal(bus, 1, 1000);
  enableTorque(bus, 1);
  bus.step(0.05);
  const auto replies = parse(send(bus, ft::encodeRead(1, ft::reg::kPresentSpeed, 2)));
  ASSERT_EQ(replies.size(), 1u);
  const int speed = ft::fromSignMagnitude(
    ft::fromBytes(replies[0].params[0], replies[0].params[1]), 15);
  EXPECT_LT(speed, 0);
}

TEST(Motion, ABlockedServoPushesAndReportsItWithoutMoving)
{
  // What the 3VN arm cannot tell you: that it has been stopped.
  ft::FakeBus bus;
  bus.addServo(1, 2048);
  enableTorque(bus, 1);
  bus.servo(1).blocked = true;
  setGoal(bus, 1, 1000);
  bus.step(1.0);

  EXPECT_EQ(presentPosition(bus, 1), 2048);
  const auto replies = parse(send(bus, ft::encodeRead(1, ft::reg::kPresentLoad, 2)));
  ASSERT_EQ(replies.size(), 1u);
  const int load = ft::fromSignMagnitude(
    ft::fromBytes(replies[0].params[0], replies[0].params[1]), 10);
  EXPECT_LT(load, 0) << "pushing toward a lower count";
}

// ---------------------------------------------------------- many at once --

TEST(Sync, ReadAnswersInTheOrderAskedAndSkipsTheAbsent)
{
  ft::FakeBus bus;
  bus.addServo(1, 100);
  bus.addServo(2, 200);
  bus.addServo(3, 300);
  bus.servo(2).answering = false;

  const auto replies = parse(
    send(bus, ft::encodeSyncRead(ft::reg::kPresentPosition, 2, {3, 2, 1})));
  ASSERT_EQ(replies.size(), 2u);
  EXPECT_EQ(replies[0].id, 3);
  EXPECT_EQ(ft::fromBytes(replies[0].params[0], replies[0].params[1]), 300);
  EXPECT_EQ(replies[1].id, 1);
  EXPECT_EQ(ft::fromBytes(replies[1].params[0], replies[1].params[1]), 100);
}

TEST(Sync, WriteReachesEveryServoNamedAndGetsNoReply)
{
  ft::FakeBus bus;
  bus.addServo(1);
  bus.addServo(2);
  const Bytes reply = send(
    bus, ft::encodeSyncWrite(
      ft::reg::kGoalPosition, 2, {{1, ft::toBytes(1111)}, {2, ft::toBytes(2222)}}));
  EXPECT_TRUE(reply.empty());
  EXPECT_EQ(bus.goal(1), 1111);
  EXPECT_EQ(bus.goal(2), 2222);
}

TEST(Faults, ACorruptedReplyFailsItsChecksum)
{
  ft::FakeBus bus;
  bus.addServo(1);
  bus.corruptNextReply();
  EXPECT_TRUE(parse(send(bus, ft::encodePing(1))).empty());
  EXPECT_EQ(parse(send(bus, ft::encodePing(1))).size(), 1u) << "only the next one";
}
