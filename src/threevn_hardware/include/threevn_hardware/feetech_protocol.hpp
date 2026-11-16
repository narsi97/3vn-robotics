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
/// The Feetech serial-bus servo protocol, as the STS3215 speaks it.
///
/// THIS ONE IS NOT OURS. protocol.hpp is a wire format this project
/// designed and compiles into both ends, so the two cannot disagree.
/// Here the other end is a servo's firmware. We cannot compile into it,
/// and the only way to be wrong is to have misread the manual.
///
/// So this header is written to be CHECKED rather than trusted:
///
///   * no dependencies but <cstdint>, <vector> and <cmath>, so every
///     byte of it runs in a unit test with no bus attached
///   * the tests compare its output against packets written out by
///     hand from the published packet layout, not against itself
///   * every register address is named once, here, with its width
///
/// What that does NOT establish is that a real servo agrees. Nothing in
/// this repository has yet been run against one. The first session with
/// the physical arm exists to find out, and it starts with PING.
///
/// The wire format (half-duplex TTL, 1 Mbaud by default, one byte order
/// throughout: little-endian on the STS series):
///
///     instruction   FF FF  ID  LEN  INSTR  PARAM...  CHK
///     status        FF FF  ID  LEN  ERROR  PARAM...  CHK
///
///     LEN = number of PARAM bytes + 2
///     CHK = ~(ID + LEN + INSTR-or-ERROR + PARAM...) & 0xFF
///
/// An eight-bit complement of a sum is a weak check: it misses two
/// errors that cancel. That is the servo's design, not a choice made
/// here, and it is why a reply is also matched against the id that was
/// asked and the length that was expected.

#ifndef THREEVN_HARDWARE__FEETECH_PROTOCOL_HPP_
#define THREEVN_HARDWARE__FEETECH_PROTOCOL_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace threevn
{
namespace feetech
{

constexpr uint8_t kHeader = 0xFF;
/// Every servo acts on a packet sent here, and none of them replies.
constexpr uint8_t kBroadcastId = 0xFE;

enum class Instruction : uint8_t
{
  kPing = 0x01,
  kRead = 0x02,
  kWrite = 0x03,
  kSyncRead = 0x82,
  kSyncWrite = 0x83,
};

/// STS3215 control table: the registers this project touches.
///
/// Addresses are from the SMS/STS series memory table. Two-byte values
/// are low byte first.
namespace reg
{
constexpr uint8_t kId = 5;                   // 1 byte, EEPROM
constexpr uint8_t kTorqueEnable = 40;        // 1 byte: 0 limp, 1 holding
constexpr uint8_t kGoalPosition = 42;        // 2 bytes, counts
constexpr uint8_t kGoalSpeed = 46;           // 2 bytes, counts/s, 0 = fastest
constexpr uint8_t kPresentPosition = 56;     // 2 bytes, counts
constexpr uint8_t kPresentSpeed = 58;        // 2 bytes, sign in bit 15
constexpr uint8_t kPresentLoad = 60;         // 2 bytes, 0.1 %, sign in bit 10
constexpr uint8_t kPresentVoltage = 62;      // 1 byte, 0.1 V
constexpr uint8_t kPresentTemperature = 63;  // 1 byte, degrees C
}  // namespace reg

/// One turn of the output shaft, in encoder counts.
constexpr int kCountsPerTurn = 4096;
/// Mid-travel. A freshly centred servo reads this.
constexpr int kCentreCount = 2048;
constexpr int kMaxCount = kCountsPerTurn - 1;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansPerCount = 2.0 * kPi / kCountsPerTurn;

// -- building instructions -------------------------------------------

inline uint8_t checksum(const std::vector<uint8_t> & packet)
{
  // Everything after the two header bytes and before the checksum slot.
  unsigned sum = 0;
  for (std::size_t i = 2; i < packet.size(); ++i) {
    sum += packet[i];
  }
  return static_cast<uint8_t>(~sum & 0xFF);
}

inline std::vector<uint8_t> encode(
  uint8_t id, Instruction instruction, const std::vector<uint8_t> & params)
{
  std::vector<uint8_t> packet = {kHeader, kHeader, id,
    static_cast<uint8_t>(params.size() + 2), static_cast<uint8_t>(instruction)};
  packet.insert(packet.end(), params.begin(), params.end());
  packet.push_back(checksum(packet));
  return packet;
}

inline std::vector<uint8_t> encodePing(uint8_t id)
{
  return encode(id, Instruction::kPing, {});
}

inline std::vector<uint8_t> encodeRead(uint8_t id, uint8_t address, uint8_t length)
{
  return encode(id, Instruction::kRead, {address, length});
}

inline std::vector<uint8_t> encodeWrite(
  uint8_t id, uint8_t address, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> params = {address};
  params.insert(params.end(), data.begin(), data.end());
  return encode(id, Instruction::kWrite, params);
}

/// Ask several servos for the same registers in one packet.
///
/// Each replies with its own status packet, in the order asked. One
/// request instead of six is the difference between a control cycle
/// that fits in 20 ms over USB and one that does not.
inline std::vector<uint8_t> encodeSyncRead(
  uint8_t address, uint8_t length, const std::vector<uint8_t> & ids)
{
  std::vector<uint8_t> params = {address, length};
  params.insert(params.end(), ids.begin(), ids.end());
  return encode(kBroadcastId, Instruction::kSyncRead, params);
}

/// Write the same registers on several servos in one packet. No reply.
///
/// `values` pairs each id with exactly `length` bytes. A pair of the
/// wrong length is skipped rather than sent short: a short record would
/// shift every later servo's bytes onto its neighbour.
inline std::vector<uint8_t> encodeSyncWrite(
  uint8_t address, uint8_t length,
  const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & values)
{
  std::vector<uint8_t> params = {address, length};
  for (const auto & value : values) {
    if (value.second.size() != length) {
      continue;
    }
    params.push_back(value.first);
    params.insert(params.end(), value.second.begin(), value.second.end());
  }
  return encode(kBroadcastId, Instruction::kSyncWrite, params);
}

// -- values ----------------------------------------------------------

inline std::vector<uint8_t> toBytes(uint16_t value)
{
  return {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>(value >> 8)};
}

inline uint16_t fromBytes(uint8_t low, uint8_t high)
{
  return static_cast<uint16_t>(low | (high << 8));
}

/// Decode a sign-and-magnitude value whose sign lives in `sign_bit`.
///
/// The servo does not use two's complement. Present speed keeps its
/// sign in bit 15 and present load in bit 10, with the magnitude in the
/// bits below. Read as an ordinary integer, a servo turning slowly
/// backwards reports turning forwards at an enormous speed.
inline int fromSignMagnitude(uint16_t raw, unsigned sign_bit)
{
  const uint16_t mask = static_cast<uint16_t>(1u << sign_bit);
  const int magnitude = raw & (mask - 1);
  return (raw & mask) ? -magnitude : magnitude;
}

inline uint16_t toSignMagnitude(int value, unsigned sign_bit)
{
  const uint16_t mask = static_cast<uint16_t>(1u << sign_bit);
  const int limit = mask - 1;
  int magnitude = value < 0 ? -value : value;
  if (magnitude > limit) {
    magnitude = limit;
  }
  return static_cast<uint16_t>(magnitude | (value < 0 ? mask : 0));
}

/// How one servo's counts map onto one joint's radians.
///
/// Two numbers per joint, and they are facts about an ASSEMBLED arm:
/// which count the encoder reads when the joint is at the URDF's zero,
/// and whether the servo turns the same way the joint's axis does. They
/// come from calibration and live in the robot profile.
struct Calibration
{
  int zero_count = kCentreCount;
  int direction = 1;    // +1 or -1
};

inline double toRadians(int count, const Calibration & calibration)
{
  return calibration.direction * (count - calibration.zero_count) * kRadiansPerCount;
}

/// Convert radians to the count to command, held inside the servo's range.
///
/// Clamped, not wrapped. A count of 4096 is not "one past the end": the
/// goal register would take it as 0, and the servo would go the long
/// way round through everything in between.
inline int toCount(double radians, const Calibration & calibration)
{
  const double exact =
    calibration.zero_count + calibration.direction * radians / kRadiansPerCount;
  const auto rounded = std::lround(exact);
  if (rounded < 0) {
    return 0;
  }
  if (rounded > kMaxCount) {
    return kMaxCount;
  }
  return static_cast<int>(rounded);
}

/// Counts per second to radians per second, with the joint's direction.
inline double toRadiansPerSecond(int counts_per_second, const Calibration & calibration)
{
  return calibration.direction * counts_per_second * kRadiansPerCount;
}

// -- reading replies -------------------------------------------------

struct Status
{
  uint8_t id = 0;
  /// The servo's own fault flags. Zero is healthy.
  uint8_t error = 0;
  std::vector<uint8_t> params;
};

/// Reassemble status packets from a byte stream.
///
/// A stream, because that is what arrives: a reply may come in two
/// reads or six replies in one, and after a glitch the next header may
/// be anywhere. Bytes are pushed as they come; complete, checksummed
/// packets come out.
///
/// A packet that fails its checksum is dropped and the search for a
/// header resumes ONE BYTE on, not past the bad packet. The length
/// field of a corrupt packet cannot be believed, and skipping by it
/// can swallow the good packet behind.
class StatusParser
{
public:
  void push(const uint8_t * data, std::size_t size)
  {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  /// Extract the next complete status packet. False when there is none yet.
  bool next(Status & out)
  {
    for (;; ) {
      // Find FF FF.
      std::size_t start = 0;
      while (start + 1 < buffer_.size() &&
        !(buffer_[start] == kHeader && buffer_[start + 1] == kHeader))
      {
        ++start;
      }
      if (start > 0) {
        discarded_ += start;
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(start));
      }
      if (buffer_.size() < 4) {
        return false;
      }
      // A third FF is not an id: 0xFF is never a valid one, so the
      // header starts a byte later than it looked.
      if (buffer_[2] == kHeader) {
        buffer_.erase(buffer_.begin());
        ++discarded_;
        continue;
      }
      const std::size_t length = buffer_[3];
      if (length < 2) {
        buffer_.erase(buffer_.begin());
        ++discarded_;
        continue;
      }
      const std::size_t total = 4 + length;
      if (buffer_.size() < total) {
        return false;
      }
      const std::vector<uint8_t> packet(
        buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total - 1));
      if (checksum(packet) != buffer_[total - 1]) {
        ++checksum_failures_;
        buffer_.erase(buffer_.begin());
        ++discarded_;
        continue;
      }
      out.id = buffer_[2];
      out.error = buffer_[4];
      out.params.assign(
        buffer_.begin() + 5, buffer_.begin() + static_cast<std::ptrdiff_t>(total - 1));
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total));
      return true;
    }
  }

  void reset()
  {
    buffer_.clear();
  }

  /// Bytes thrown away while hunting for a header.
  std::size_t discarded() const {return discarded_;}
  /// Packets that had a header and a length and failed their checksum.
  std::size_t checksumFailures() const {return checksum_failures_;}

private:
  std::vector<uint8_t> buffer_;
  std::size_t discarded_ = 0;
  std::size_t checksum_failures_ = 0;
};

}  // namespace feetech
}  // namespace threevn

#endif  // THREEVN_HARDWARE__FEETECH_PROTOCOL_HPP_
