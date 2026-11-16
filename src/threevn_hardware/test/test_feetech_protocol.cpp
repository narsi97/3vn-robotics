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
/// The Feetech packet layout, checked against packets written by hand.
///
/// The expected bytes below were worked out on paper from the published
/// layout -- header, id, length, instruction, parameters, complement of
/// the sum -- and typed in. They were not produced by running the code
/// under test and pasting what it printed: a vector made that way agrees
/// with the encoder by construction and proves nothing.
///
/// Each one shows its arithmetic, so a reader can redo it.
///
/// What this cannot show is that the layout itself was read correctly.
/// Only a servo can do that.

#include <gmock/gmock.h>

#include <cmath>
#include <utility>
#include <vector>

#include "threevn_hardware/feetech_protocol.hpp"

namespace ft = threevn::feetech;
using Bytes = std::vector<uint8_t>;
using ::testing::ElementsAreArray;

// ------------------------------------------------------ instructions --

TEST(Encode, Ping)
{
  // FF FF | id 01 | len 02 | ping 01 | chk
  // sum = 01 + 02 + 01 = 04, complement = FB
  EXPECT_THAT(ft::encodePing(1), ElementsAreArray(Bytes{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
}

TEST(Encode, ReadPresentPosition)
{
  // FF FF | id 01 | len 04 | read 02 | addr 38 (56) | count 02 | chk
  // sum = 01 + 04 + 02 + 38 + 02 = 41, complement = BE
  EXPECT_THAT(
    ft::encodeRead(1, ft::reg::kPresentPosition, 2),
    ElementsAreArray(Bytes{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE}));
}

TEST(Encode, WriteGoalPositionIsLowByteFirst)
{
  // Centre, 2048 = 0x0800, goes on the wire as 00 08.
  // FF FF | id 01 | len 05 | write 03 | addr 2A (42) | 00 08 | chk
  // sum = 01 + 05 + 03 + 2A + 00 + 08 = 3B, complement = C4
  EXPECT_THAT(
    ft::encodeWrite(1, ft::reg::kGoalPosition, ft::toBytes(2048)),
    ElementsAreArray(Bytes{0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x00, 0x08, 0xC4}));
}

TEST(Encode, SyncWriteCarriesOneRecordPerServo)
{
  // Servo 1 to 2048 (00 08), servo 2 to 1000 = 0x03E8 (E8 03).
  // FF FF | FE | len 0A | 83 | addr 2A | size 02 | 01 00 08 | 02 E8 03 | chk
  // len = 2 header params + 2 records of 3 bytes + 2 = 10
  // sum = FE + 0A + 83 + 2A + 02 + 01 + 00 + 08 + 02 + E8 + 03 = 2AD -> AD
  // complement = 52
  EXPECT_THAT(
    ft::encodeSyncWrite(
      ft::reg::kGoalPosition, 2, {{1, ft::toBytes(2048)}, {2, ft::toBytes(1000)}}),
    ElementsAreArray(
      Bytes{0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x2A, 0x02,
        0x01, 0x00, 0x08, 0x02, 0xE8, 0x03, 0x52}));
}

TEST(Encode, SyncWriteDropsARecordOfTheWrongWidth)
{
  // A one-byte record in a two-byte write would shift every later
  // servo's data onto its neighbour. It is left out, and the packet
  // that remains is still well formed for the servos it does name.
  const Bytes with_bad = ft::encodeSyncWrite(
    ft::reg::kGoalPosition, 2, {{1, ft::toBytes(2048)}, {2, {0x01}}});
  const Bytes without = ft::encodeSyncWrite(
    ft::reg::kGoalPosition, 2, {{1, ft::toBytes(2048)}});
  EXPECT_EQ(with_bad, without);
}

TEST(Encode, SyncReadNamesEveryServoOnce)
{
  // Position and speed, four bytes from 0x38, from servos 1 to 6.
  // FF FF | FE | len 0A | 82 | addr 38 | size 04 | 01 02 03 04 05 06 | chk
  // sum = FE + 0A + 82 + 38 + 04 + (1+2+3+4+5+6 = 15) = 1DB -> DB
  // complement = 24
  EXPECT_THAT(
    ft::encodeSyncRead(ft::reg::kPresentPosition, 4, {1, 2, 3, 4, 5, 6}),
    ElementsAreArray(
      Bytes{0xFF, 0xFF, 0xFE, 0x0A, 0x82, 0x38, 0x04,
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x24}));
}

// ------------------------------------------------------------ replies --

namespace
{
/// A servo's reply to a ping: FF FF 01 02 00 FC (sum 03, complement FC).
const Bytes kPingReply = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
/// Servo 1 reporting position 2048: FF FF 01 04 00 | 00 08 | F2
/// (sum = 01 + 04 + 00 + 00 + 08 = 0D, complement F2).
const Bytes kPositionReply = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2};

std::vector<ft::Status> parseAll(ft::StatusParser & parser)
{
  std::vector<ft::Status> out;
  ft::Status status;
  while (parser.next(status)) {
    out.push_back(status);
  }
  return out;
}
}  // namespace

TEST(Parse, AHandWrittenReply)
{
  ft::StatusParser parser;
  parser.push(kPositionReply.data(), kPositionReply.size());
  const auto packets = parseAll(parser);
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].id, 1);
  EXPECT_EQ(packets[0].error, 0);
  EXPECT_THAT(packets[0].params, ElementsAreArray(Bytes{0x00, 0x08}));
}

TEST(Parse, AReplyWithNoParameters)
{
  ft::StatusParser parser;
  parser.push(kPingReply.data(), kPingReply.size());
  const auto packets = parseAll(parser);
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_TRUE(packets[0].params.empty());
}

TEST(Parse, AReplyArrivingOneByteAtATime)
{
  // A read() returns whatever has arrived. Nothing promises that is a
  // whole packet.
  ft::StatusParser parser;
  ft::Status status;
  for (std::size_t i = 0; i + 1 < kPositionReply.size(); ++i) {
    parser.push(&kPositionReply[i], 1);
    EXPECT_FALSE(parser.next(status)) << "complete after only " << i + 1 << " bytes";
  }
  parser.push(&kPositionReply.back(), 1);
  EXPECT_TRUE(parser.next(status));
}

TEST(Parse, SixRepliesInOneRead)
{
  // What a sync read looks like from the host: one request, and the
  // replies back to back.
  ft::StatusParser parser;
  for (int i = 0; i < 6; ++i) {
    parser.push(kPingReply.data(), kPingReply.size());
  }
  EXPECT_EQ(parseAll(parser).size(), 6u);
}

TEST(Parse, NoiseBeforeTheHeaderIsSkipped)
{
  ft::StatusParser parser;
  const Bytes noise = {0x00, 0x13, 0xFF, 0x37};
  parser.push(noise.data(), noise.size());
  parser.push(kPingReply.data(), kPingReply.size());
  EXPECT_EQ(parseAll(parser).size(), 1u);
  EXPECT_EQ(parser.discarded(), noise.size());
}

TEST(Parse, AnExtraHeaderByteDoesNotBecomeAnId)
{
  // FF FF FF 01 ...: the id cannot be FF, so the real header starts one
  // byte later than it first appears.
  ft::StatusParser parser;
  const uint8_t extra = 0xFF;
  parser.push(&extra, 1);
  parser.push(kPingReply.data(), kPingReply.size());
  const auto packets = parseAll(parser);
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].id, 1);
}

TEST(Parse, ACorruptReplyIsDroppedNotDelivered)
{
  Bytes corrupt = kPositionReply;
  corrupt[5] ^= 0x01;   // one bit of the position
  ft::StatusParser parser;
  parser.push(corrupt.data(), corrupt.size());
  EXPECT_TRUE(parseAll(parser).empty());
  EXPECT_EQ(parser.checksumFailures(), 1u);
}

TEST(Parse, AGoodReplyBehindACorruptLengthIsStillFound)
{
  // A corrupt packet's length field cannot be believed. This one claims
  // 0x40 bytes follow. Skipping by that would swallow the good reply
  // behind it -- and then wait for sixty more bytes that never come.
  Bytes stream = {0xFF, 0xFF, 0x01, 0x40, 0x00, 0x00};
  stream.insert(stream.end(), kPositionReply.begin(), kPositionReply.end());
  // Pad to the bogus length so the parser has enough to judge it by.
  stream.resize(4 + 0x40 + kPositionReply.size(), 0x00);
  stream.insert(stream.end(), kPingReply.begin(), kPingReply.end());

  ft::StatusParser parser;
  parser.push(stream.data(), stream.size());
  const auto packets = parseAll(parser);
  ASSERT_GE(packets.size(), 1u);
  EXPECT_THAT(packets[0].params, ElementsAreArray(Bytes{0x00, 0x08}));
}

TEST(Parse, TheErrorByteIsCarriedThrough)
{
  // FF FF 02 02 | error 20 (overload) | chk: sum = 02 + 02 + 20 = 24, DB
  const Bytes faulted = {0xFF, 0xFF, 0x02, 0x02, 0x20, 0xDB};
  ft::StatusParser parser;
  parser.push(faulted.data(), faulted.size());
  const auto packets = parseAll(parser);
  ASSERT_EQ(packets.size(), 1u);
  EXPECT_EQ(packets[0].id, 2);
  EXPECT_EQ(packets[0].error, 0x20);
}

// ------------------------------------------------------------- values --

TEST(Values, SpeedSignIsBitFifteenNotTwosComplement)
{
  // 0x8032 is "backwards at 50", not 32818.
  EXPECT_EQ(ft::fromSignMagnitude(0x8032, 15), -50);
  EXPECT_EQ(ft::fromSignMagnitude(0x0032, 15), 50);
  EXPECT_EQ(ft::toSignMagnitude(-50, 15), 0x8032);
}

TEST(Values, LoadSignIsBitTen)
{
  // 0x0464 = bit 10 set, magnitude 0x64 = 100 -> -10.0 % of full load.
  EXPECT_EQ(ft::fromSignMagnitude(0x0464, 10), -100);
  EXPECT_EQ(ft::fromSignMagnitude(0x0064, 10), 100);
  EXPECT_EQ(ft::toSignMagnitude(-100, 10), 0x0464);
}

TEST(Values, SignMagnitudeSaturatesInsteadOfSpillingIntoTheSignBit)
{
  // 1500 does not fit in ten bits. It must not come back negative.
  EXPECT_EQ(ft::fromSignMagnitude(ft::toSignMagnitude(1500, 10), 10), 1023);
}

TEST(Values, CentreIsZeroRadians)
{
  EXPECT_DOUBLE_EQ(ft::toRadians(2048, ft::Calibration{}), 0.0);
  EXPECT_EQ(ft::toCount(0.0, ft::Calibration{}), 2048);
}

TEST(Values, AQuarterTurnIsAThousandAndTwentyFourCounts)
{
  EXPECT_NEAR(ft::toRadians(3072, ft::Calibration{}), ft::kPi / 2.0, 1e-12);
  EXPECT_EQ(ft::toCount(-ft::kPi / 2.0, ft::Calibration{}), 1024);
}

TEST(Values, DirectionReversesTheJointNotTheServo)
{
  ft::Calibration reversed;
  reversed.direction = -1;
  EXPECT_NEAR(ft::toRadians(3072, reversed), -ft::kPi / 2.0, 1e-12);
  EXPECT_EQ(ft::toCount(ft::kPi / 2.0, reversed), 1024);
  EXPECT_NEAR(ft::toRadiansPerSecond(100, reversed), -100 * ft::kRadiansPerCount, 1e-12);
}

TEST(Values, ZeroCountMovesWhereZeroRadiansIs)
{
  // An arm assembled with this horn a few teeth round reads 2100 when
  // the joint is at the description's zero.
  ft::Calibration offset;
  offset.zero_count = 2100;
  EXPECT_DOUBLE_EQ(ft::toRadians(2100, offset), 0.0);
  EXPECT_EQ(ft::toCount(0.0, offset), 2100);
}

TEST(Values, AGoalBeyondTheEncoderIsClampedNotWrapped)
{
  // +10 rad is past 4095. Wrapped, it would be a small count, and the
  // servo would take the long way round to reach it.
  EXPECT_EQ(ft::toCount(10.0, ft::Calibration{}), 4095);
  EXPECT_EQ(ft::toCount(-10.0, ft::Calibration{}), 0);
}

TEST(Values, RadiansSurviveARoundTripToWithinHalfACount)
{
  ft::Calibration calibration;
  calibration.zero_count = 1987;
  calibration.direction = -1;
  for (double radians = -1.5; radians <= 1.5; radians += 0.0371) {
    const double back = ft::toRadians(ft::toCount(radians, calibration), calibration);
    EXPECT_NEAR(back, radians, ft::kRadiansPerCount / 2.0 + 1e-12);
  }
}
