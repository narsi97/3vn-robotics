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
/// The wire protocol, tested with no hardware attached.
///
/// This is the payoff for putting the hardware seam at a plugin
/// boundary: the code that decides what a servo is told is pure, so
/// every failure mode a serial line actually produces - truncation,
/// corruption, garbage, a device resetting mid-frame - can be provoked
/// deliberately here instead of discovered on a bench.

#include <gtest/gtest.h>

#include <vector>

#include "threevn_hardware/protocol.hpp"

using namespace threevn::protocol;  // NOLINT(build/namespaces)

namespace
{

/// Feed a buffer to a parser, collecting every frame it completes.
struct Collected
{
  MessageType type;
  std::vector<uint8_t> payload;
};

std::vector<Collected> parseAll(const std::vector<uint8_t> & bytes, FrameParser & parser)
{
  std::vector<Collected> frames;
  parser.pushBuffer(
    bytes.data(), bytes.size(), [&frames](const FrameParser & p) {
      frames.push_back(
        {p.type(), std::vector<uint8_t>(p.payload(), p.payload() + p.payloadSize())});
    });
  return frames;
}

std::vector<uint8_t> encodeCommandBytes(const CommandPayload & cmd)
{
  uint8_t buffer[kMaxFrameSize];
  const size_t n = encodeCommand(cmd, buffer, sizeof(buffer));
  return std::vector<uint8_t>(buffer, buffer + n);
}

}  // namespace

// ------------------------------------------------------------- units --

TEST(Units, RoundTripsThroughMicroUnits)
{
  for (double value : {0.0, 1.5707963, -1.5707963, 0.018, -0.0001, 2.0943951}) {
    EXPECT_NEAR(fromMicro(toMicro(value)), value, 1e-6) << "value " << value;
  }
}

TEST(Units, SaturatesInsteadOfWrapping)
{
  // A wrapped int32 turns a large positive angle into a large negative
  // one, which is the worst thing that can happen to a servo command.
  EXPECT_EQ(toMicro(1e9), 2147483647);
  EXPECT_EQ(toMicro(-1e9), -2147483647 - 1);
}

TEST(Units, ResolutionExceedsAnyHobbyServo)
{
  // 1 microradian is ~0.00006 degrees; an MG996R resolves ~0.3 degrees.
  EXPECT_NE(toMicro(0.000001), toMicro(0.0));
}

// --------------------------------------------------------------- crc --

TEST(Crc, MatchesTheKnownCcittFalseVector)
{
  // "123456789" -> 0x29B1 for CRC-16/CCITT-FALSE. Pinning a published
  // vector means the firmware and the host cannot merely agree with each
  // other while both being wrong.
  const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(crc16(data, sizeof(data)), 0x29B1);
}

TEST(Crc, DetectsASingleFlippedBit)
{
  const uint8_t original[] = {0x01, 0x02, 0x03, 0x04};
  uint8_t mutated[] = {0x01, 0x02, 0x03, 0x04};
  mutated[2] ^= 0x08;
  EXPECT_NE(crc16(original, 4), crc16(mutated, 4));
}

// ---------------------------------------------------------- encoding --

TEST(Encode, CommandRoundTrips)
{
  CommandPayload sent;
  sent.position_micro[kShoulderPan] = toMicro(0.4);
  sent.position_micro[kShoulderLift] = toMicro(-0.3);
  sent.position_micro[kElbow] = toMicro(1.2);
  sent.position_micro[kWrist] = toMicro(-0.05);
  sent.position_micro[kGripper] = toMicro(0.018);
  sent.flags = kFlagEnable;

  FrameParser parser;
  const auto frames = parseAll(encodeCommandBytes(sent), parser);

  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].type, MessageType::kCommand);

  CommandPayload received;
  ASSERT_TRUE(
    decodeCommand(frames[0].payload.data(), frames[0].payload.size(), received));
  for (uint8_t i = 0; i < kJointCount; ++i) {
    EXPECT_EQ(received.position_micro[i], sent.position_micro[i]) << "joint " << int(i);
  }
  EXPECT_EQ(received.flags, sent.flags);
}

TEST(Encode, StateRoundTrips)
{
  StatePayload sent;
  sent.position_micro[kElbow] = toMicro(-1.9);
  sent.status = kStatusEnabled | kStatusLimitClamped;
  sent.sequence = 65535;

  uint8_t buffer[kMaxFrameSize];
  const size_t n = encodeState(sent, buffer, sizeof(buffer));
  ASSERT_GT(n, 0u);

  FrameParser parser;
  const auto frames = parseAll({buffer, buffer + n}, parser);
  ASSERT_EQ(frames.size(), 1u);

  StatePayload received;
  ASSERT_TRUE(decodeState(frames[0].payload.data(), frames[0].payload.size(), received));
  EXPECT_EQ(received.position_micro[kElbow], sent.position_micro[kElbow]);
  EXPECT_EQ(received.status, sent.status);
  EXPECT_EQ(received.sequence, sent.sequence);
}

TEST(Encode, InfoAndFaultRoundTrip)
{
  InfoPayload info;
  info.firmware_major = 1;
  info.firmware_minor = 2;
  info.firmware_patch = 3;
  info.heartbeat_timeout_ms = 250;

  uint8_t buffer[kMaxFrameSize];
  size_t n = encodeInfo(info, buffer, sizeof(buffer));
  FrameParser parser;
  auto frames = parseAll({buffer, buffer + n}, parser);
  ASSERT_EQ(frames.size(), 1u);
  InfoPayload got_info;
  ASSERT_TRUE(decodeInfo(frames[0].payload.data(), frames[0].payload.size(), got_info));
  EXPECT_EQ(got_info.firmware_minor, 2);
  EXPECT_EQ(got_info.heartbeat_timeout_ms, 250);

  FaultPayload fault{FaultCode::kHeartbeatTimeout, 1234};
  n = encodeFault(fault, buffer, sizeof(buffer));
  FrameParser parser2;
  frames = parseAll({buffer, buffer + n}, parser2);
  ASSERT_EQ(frames.size(), 1u);
  FaultPayload got_fault;
  ASSERT_TRUE(decodeFault(frames[0].payload.data(), frames[0].payload.size(), got_fault));
  EXPECT_EQ(got_fault.code, FaultCode::kHeartbeatTimeout);
  EXPECT_EQ(got_fault.detail, 1234);
}

TEST(Encode, RefusesABufferThatIsTooSmall)
{
  CommandPayload cmd;
  uint8_t tiny[4];
  EXPECT_EQ(encodeCommand(cmd, tiny, sizeof(tiny)), 0u);
}

// ------------------------------------------------------------ parsing --

TEST(Parser, RejectsACorruptedPayload)
{
  CommandPayload cmd;
  cmd.position_micro[kShoulderPan] = toMicro(0.5);
  auto bytes = encodeCommandBytes(cmd);
  bytes[kHeaderSize + 1] ^= 0xFF;   // flip a byte inside the payload

  FrameParser parser;
  EXPECT_TRUE(parseAll(bytes, parser).empty());
  EXPECT_EQ(parser.lastError(), ParseError::kBadCrc);
  EXPECT_EQ(parser.errorCount(), 1u);
}

TEST(Parser, RejectsACorruptedLengthRatherThanActingOnIt)
{
  // The CRC deliberately covers the header, so a length mangled in
  // transit is caught. Trusting it would mean reading past the frame.
  CommandPayload cmd;
  auto bytes = encodeCommandBytes(cmd);
  bytes[4] = 8;

  FrameParser parser;
  EXPECT_TRUE(parseAll(bytes, parser).empty());
}

TEST(Parser, RejectsAnUnknownProtocolVersion)
{
  // Guessing at an unrecognised layout means moving servos on misread
  // numbers, so an unknown version is refused outright.
  CommandPayload cmd;
  auto bytes = encodeCommandBytes(cmd);
  bytes[2] = kProtocolVersion + 7;

  FrameParser parser;
  EXPECT_TRUE(parseAll(bytes, parser).empty());
  EXPECT_EQ(parser.lastError(), ParseError::kBadVersion);
}

TEST(Parser, RejectsAnOverlongLength)
{
  FrameParser parser;
  const std::vector<uint8_t> bytes = {
    kMagic0, kMagic1, kProtocolVersion,
    static_cast<uint8_t>(MessageType::kCommand),
    static_cast<uint8_t>(kMaxPayloadSize + 1)};
  EXPECT_TRUE(parseAll(bytes, parser).empty());
  EXPECT_EQ(parser.lastError(), ParseError::kBadLength);
}

TEST(Parser, ReassemblesAFrameSplitAcrossReads)
{
  // The normal case on a serial port, not an edge case.
  CommandPayload cmd;
  cmd.position_micro[kWrist] = toMicro(-0.77);
  const auto bytes = encodeCommandBytes(cmd);

  FrameParser parser;
  size_t completed = 0;
  for (uint8_t byte : bytes) {
    if (parser.push(byte)) {++completed;}
  }
  EXPECT_EQ(completed, 1u);
}

TEST(Parser, ResyncsAfterLeadingGarbage)
{
  CommandPayload cmd;
  cmd.position_micro[kElbow] = toMicro(0.9);
  std::vector<uint8_t> bytes = {0x00, 0xFF, 0x33, 0x11, 0xAB, 0x56, 0x33};
  const auto frame = encodeCommandBytes(cmd);
  bytes.insert(bytes.end(), frame.begin(), frame.end());

  FrameParser parser;
  const auto frames = parseAll(bytes, parser);
  ASSERT_EQ(frames.size(), 1u);

  CommandPayload received;
  ASSERT_TRUE(
    decodeCommand(frames[0].payload.data(), frames[0].payload.size(), received));
  EXPECT_EQ(received.position_micro[kElbow], toMicro(0.9));
}

TEST(Parser, RecoversAfterATruncatedFrame_LosingExactlyOneFrame)
{
  // A device resetting mid-transmission truncates a frame. The parser is
  // then stranded mid-payload and will consume the NEXT frame's bytes as
  // payload before its CRC fails and it resyncs.
  //
  // That cost is inherent, not a bug to fix: payload bytes can legitimately
  // contain 0x33 0x56, so scanning for the magic inside a payload would
  // resync on data and corrupt good frames. Bounded loss is the better
  // trade.
  //
  // What matters is that the loss is BOUNDED. At 50 Hz one lost frame is
  // 20 ms of staleness, well inside the heartbeat window - whereas an
  // unbounded stall would wedge the link until someone power-cycled it.
  CommandPayload cmd;
  cmd.position_micro[kShoulderLift] = toMicro(1.1);
  const auto good = encodeCommandBytes(cmd);

  std::vector<uint8_t> bytes(good.begin(), good.begin() + 8);  // cut short
  bytes.insert(bytes.end(), good.begin(), good.end());          // sacrificed
  bytes.insert(bytes.end(), good.begin(), good.end());          // must arrive

  FrameParser parser;
  const auto frames = parseAll(bytes, parser);

  ASSERT_EQ(frames.size(), 1u) << "expected exactly one frame to survive";
  CommandPayload received;
  ASSERT_TRUE(
    decodeCommand(frames.back().payload.data(), frames.back().payload.size(), received));
  EXPECT_EQ(received.position_micro[kShoulderLift], toMicro(1.1));
  EXPECT_EQ(parser.errorCount(), 1u) << "the truncation should be reported once";
}

TEST(Parser, ATruncationDoesNotWedgeTheLinkPermanently)
{
  // The property that actually matters operationally: after any single
  // disruption, a steady stream returns to being parsed correctly.
  CommandPayload cmd;
  cmd.position_micro[kElbow] = toMicro(-0.42);
  const auto good = encodeCommandBytes(cmd);

  std::vector<uint8_t> bytes(good.begin(), good.begin() + 11);
  for (int i = 0; i < 10; ++i) {
    bytes.insert(bytes.end(), good.begin(), good.end());
  }

  FrameParser parser;
  const auto frames = parseAll(bytes, parser);
  EXPECT_GE(frames.size(), 8u) << "the link should recover, not stall";
}

TEST(Parser, ReadsBackToBackFramesFromOneRead)
{
  // At 50 Hz a single read often returns several frames.
  std::vector<uint8_t> bytes;
  for (int i = 0; i < 4; ++i) {
    CommandPayload cmd;
    cmd.position_micro[kShoulderPan] = toMicro(0.1 * i);
    const auto frame = encodeCommandBytes(cmd);
    bytes.insert(bytes.end(), frame.begin(), frame.end());
  }

  FrameParser parser;
  const auto frames = parseAll(bytes, parser);
  ASSERT_EQ(frames.size(), 4u);
  EXPECT_EQ(parser.frameCount(), 4u);
  EXPECT_EQ(parser.errorCount(), 0u);
}

TEST(Parser, SurvivesAStreamOfPureNoise)
{
  // Must not crash, hang or read out of bounds. A deterministic pseudo
  // random stream so a failure is reproducible.
  FrameParser parser;
  uint32_t seed = 12345;
  for (int i = 0; i < 200000; ++i) {
    seed = seed * 1103515245u + 12345u;
    parser.push(static_cast<uint8_t>((seed >> 16) & 0xFF));
  }
  SUCCEED();
}

TEST(Parser, ResetDropsAPartialFrame)
{
  CommandPayload cmd;
  const auto bytes = encodeCommandBytes(cmd);

  FrameParser parser;
  for (size_t i = 0; i < 7; ++i) {
    parser.push(bytes[i]);
                                                          }
  parser.reset();

  // The remainder alone must not be mistaken for a frame.
  size_t completed = 0;
  for (size_t i = 7; i < bytes.size(); ++i) {
    if (parser.push(bytes[i])) {++completed;}
  }
  EXPECT_EQ(completed, 0u);
}
