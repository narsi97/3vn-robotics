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
/// The 3VN host <-> ESP32 wire protocol.
///
/// COMPILED INTO BOTH SIDES. The ROS 2 hardware interface and the ESP32
/// firmware include this same header, so the two cannot disagree about
/// framing, field order or units. A protocol defined twice is a protocol
/// that drifts, and the drift shows up as a robot moving to the wrong
/// place rather than as a compile error.
///
/// Therefore this header has NO dependencies: no ROS, no Arduino, no
/// STL containers, no dynamic allocation, no exceptions. Fixed-width
/// integers only. That also means the whole thing is testable natively
/// on a laptop with no hardware attached, which is the point.
///
/// Frame layout, little-endian throughout:
///
///     +--------+--------+------+------+---------+--------+
///     | MAGIC0 | MAGIC1 | VER  | TYPE | LEN     | ...    |
///     | 0x33   | 0x56   | u8   | u8   | u8      | payload|
///     +--------+--------+------+------+---------+--------+
///     | CRC16 (little-endian, over VER..payload)         |
///     +--------------------------------------------------+
///
/// The CRC deliberately covers the header fields as well as the payload,
/// so a corrupted length is caught rather than acted upon.

#ifndef THREEVN_HARDWARE__PROTOCOL_HPP_
#define THREEVN_HARDWARE__PROTOCOL_HPP_

#include <stddef.h>
#include <stdint.h>

namespace threevn
{
namespace protocol
{

/// Frame delimiter: '3' 'V'.
constexpr uint8_t kMagic0 = 0x33;
constexpr uint8_t kMagic1 = 0x56;

/// Bumped on any incompatible change. The firmware refuses a frame whose
/// version it does not implement rather than guessing at the layout,
/// because guessing means moving servos on misread numbers.
constexpr uint8_t kProtocolVersion = 1;

/// Joints, in the order they appear on the wire. Fixed here so both
/// sides index the same array. Matches the ros2_control block in
/// arm.ros2_control.xacro.
constexpr uint8_t kJointCount = 5;

enum JointIndex : uint8_t
{
  kShoulderPan = 0,
  kShoulderLift = 1,
  kElbow = 2,
  kWrist = 3,
  kGripper = 4,
};

/// Message types. High bit set means device-to-host, which makes the
/// direction of a captured byte obvious when reading a trace.
enum class MessageType : uint8_t
{
  kCommand = 0x01,   ///< host -> device: target positions + flags
  kPing = 0x02,      ///< host -> device: liveness probe carrying a nonce
  kGetInfo = 0x03,   ///< host -> device: request identity

  kState = 0x81,     ///< device -> host: measured positions + status
  kPong = 0x82,      ///< device -> host: echoes the ping nonce
  kInfo = 0x83,      ///< device -> host: firmware identity
  kFault = 0x84,     ///< device -> host: something is wrong
};

/// Command flags, host -> device.
enum CommandFlag : uint8_t
{
  kFlagEnable = 1 << 0,      ///< torque on. Clear to release the servos.
  kFlagEmergencyStop = 1 << 1,  ///< latch a stop; needs an explicit clear
  kFlagClearFault = 1 << 2,  ///< acknowledge and clear a latched fault
};

/// Status flags, device -> host.
enum StatusFlag : uint8_t
{
  kStatusEnabled = 1 << 0,
  kStatusEmergencyStop = 1 << 1,
  kStatusHeartbeatLost = 1 << 2,  ///< no command within the timeout
  kStatusLimitClamped = 1 << 3,   ///< last command hit a firmware limit
  kStatusFault = 1 << 4,
};

/// Fault codes, device -> host.
enum class FaultCode : uint8_t
{
  kNone = 0,
  kHeartbeatTimeout = 1,
  kBadCrc = 2,
  kBadVersion = 3,
  kBadLength = 4,
  kUnknownType = 5,
  kServoFault = 6,
  kBrownout = 7,
};

/// Positions travel as signed micro-units of the joint's SI unit:
/// microradians for a revolute joint, micrometres for a prismatic one.
///
/// This is deliberately the SAME conversion for every joint, because
/// ros2_control already carries radians or metres per joint type. A
/// per-joint scale factor would be one more table to keep in sync, and
/// the wrong entry would move a servo to a plausible-looking wrong angle.
///
/// int32 spans +/-2147 rad, and 1e-6 rad is ~0.00006 degrees - four
/// orders of magnitude finer than any hobby servo resolves.
constexpr double kMicroPerUnit = 1e6;

inline int32_t toMicro(double si_units)
{
  const double scaled = si_units * kMicroPerUnit;
  // Saturate rather than wrap. A wrapped int32 turns a large positive
  // angle into a large negative one, which is the worst possible failure
  // for a servo command.
  if (scaled > 2147483647.0) {return 2147483647;}
  if (scaled < -2147483648.0) {return -2147483647 - 1;}
  return static_cast<int32_t>(scaled >= 0 ? scaled + 0.5 : scaled - 0.5);
}

inline double fromMicro(int32_t micro)
{
  return static_cast<double>(micro) / kMicroPerUnit;
}

// -------------------------------------------------------------- CRC --

/// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final
/// XOR. Chosen because it is trivial to implement identically on both
/// sides and catches every burst error up to 16 bits, which covers the
/// realistic serial failure modes.
inline uint16_t crc16(const uint8_t * data, size_t length, uint16_t seed = 0xFFFF)
{
  uint16_t crc = seed;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// ---------------------------------------------------------- payloads --

/// Host -> device. 5 positions plus flags.
struct CommandPayload
{
  int32_t position_micro[kJointCount] = {0, 0, 0, 0, 0};
  uint8_t flags = 0;
};

/// Device -> host.
struct StatePayload
{
  int32_t position_micro[kJointCount] = {0, 0, 0, 0, 0};
  uint8_t status = 0;
  /// Wraps. Used only to detect gaps, never as a clock.
  uint16_t sequence = 0;
};

/// Device -> host.
struct InfoPayload
{
  uint8_t protocol_version = kProtocolVersion;
  uint8_t firmware_major = 0;
  uint8_t firmware_minor = 0;
  uint8_t firmware_patch = 0;
  uint8_t joint_count = kJointCount;
  /// Milliseconds without a command before the device holds position.
  uint16_t heartbeat_timeout_ms = 0;
};

/// Device -> host.
struct FaultPayload
{
  FaultCode code = FaultCode::kNone;
  /// Free-form, meaningful per code. Zero when unused.
  uint16_t detail = 0;
};

constexpr size_t kHeaderSize = 5;   ///< magic0 magic1 version type length
constexpr size_t kCrcSize = 2;
constexpr size_t kMaxPayloadSize = 64;
constexpr size_t kMaxFrameSize = kHeaderSize + kMaxPayloadSize + kCrcSize;

constexpr size_t kCommandPayloadSize = kJointCount * 4 + 1;  // 21
constexpr size_t kStatePayloadSize = kJointCount * 4 + 1 + 2;  // 23
constexpr size_t kInfoPayloadSize = 7;
constexpr size_t kFaultPayloadSize = 3;
constexpr size_t kPingPayloadSize = 4;

// -------------------------------------------------- little-endian io --

inline void putU16(uint8_t * out, uint16_t value)
{
  out[0] = static_cast<uint8_t>(value & 0xFF);
  out[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline uint16_t getU16(const uint8_t * in)
{
  return static_cast<uint16_t>(in[0]) | static_cast<uint16_t>(in[1] << 8);
}

inline void putI32(uint8_t * out, int32_t value)
{
  const uint32_t raw = static_cast<uint32_t>(value);
  out[0] = static_cast<uint8_t>(raw & 0xFF);
  out[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>((raw >> 16) & 0xFF);
  out[3] = static_cast<uint8_t>((raw >> 24) & 0xFF);
}

inline int32_t getI32(const uint8_t * in)
{
  const uint32_t raw = static_cast<uint32_t>(in[0]) |
    (static_cast<uint32_t>(in[1]) << 8) |
    (static_cast<uint32_t>(in[2]) << 16) |
    (static_cast<uint32_t>(in[3]) << 24);
  return static_cast<int32_t>(raw);
}

// -------------------------------------------------------------- encode --

/// Wrap a payload in a frame. Returns the total bytes written, or 0 if
/// the buffer is too small.
inline size_t encodeFrame(
  MessageType type, const uint8_t * payload, size_t payload_size,
  uint8_t * out, size_t out_capacity)
{
  if (payload_size > kMaxPayloadSize) {return 0;}
  const size_t total = kHeaderSize + payload_size + kCrcSize;
  if (out_capacity < total) {return 0;}

  out[0] = kMagic0;
  out[1] = kMagic1;
  out[2] = kProtocolVersion;
  out[3] = static_cast<uint8_t>(type);
  out[4] = static_cast<uint8_t>(payload_size);
  for (size_t i = 0; i < payload_size; ++i) {
    out[kHeaderSize + i] = payload[i];
  }
  // CRC covers version, type, length and payload - but not the magic,
  // which is only a resync marker.
  const uint16_t crc = crc16(out + 2, payload_size + 3);
  putU16(out + kHeaderSize + payload_size, crc);
  return total;
}

inline size_t encodeCommand(const CommandPayload & cmd, uint8_t * out, size_t capacity)
{
  uint8_t payload[kCommandPayloadSize];
  for (uint8_t i = 0; i < kJointCount; ++i) {
    putI32(payload + i * 4, cmd.position_micro[i]);
  }
  payload[kJointCount * 4] = cmd.flags;
  return encodeFrame(MessageType::kCommand, payload, sizeof(payload), out, capacity);
}

inline size_t encodeState(const StatePayload & state, uint8_t * out, size_t capacity)
{
  uint8_t payload[kStatePayloadSize];
  for (uint8_t i = 0; i < kJointCount; ++i) {
    putI32(payload + i * 4, state.position_micro[i]);
  }
  payload[kJointCount * 4] = state.status;
  putU16(payload + kJointCount * 4 + 1, state.sequence);
  return encodeFrame(MessageType::kState, payload, sizeof(payload), out, capacity);
}

inline size_t encodeInfo(const InfoPayload & info, uint8_t * out, size_t capacity)
{
  uint8_t payload[kInfoPayloadSize];
  payload[0] = info.protocol_version;
  payload[1] = info.firmware_major;
  payload[2] = info.firmware_minor;
  payload[3] = info.firmware_patch;
  payload[4] = info.joint_count;
  putU16(payload + 5, info.heartbeat_timeout_ms);
  return encodeFrame(MessageType::kInfo, payload, sizeof(payload), out, capacity);
}

inline size_t encodeFault(const FaultPayload & fault, uint8_t * out, size_t capacity)
{
  uint8_t payload[kFaultPayloadSize];
  payload[0] = static_cast<uint8_t>(fault.code);
  putU16(payload + 1, fault.detail);
  return encodeFrame(MessageType::kFault, payload, sizeof(payload), out, capacity);
}

inline size_t encodePing(uint32_t nonce, uint8_t * out, size_t capacity)
{
  uint8_t payload[kPingPayloadSize];
  putI32(payload, static_cast<int32_t>(nonce));
  return encodeFrame(MessageType::kPing, payload, sizeof(payload), out, capacity);
}

inline size_t encodePong(uint32_t nonce, uint8_t * out, size_t capacity)
{
  uint8_t payload[kPingPayloadSize];
  putI32(payload, static_cast<int32_t>(nonce));
  return encodeFrame(MessageType::kPong, payload, sizeof(payload), out, capacity);
}

// -------------------------------------------------------------- decode --

inline bool decodeCommand(const uint8_t * payload, size_t size, CommandPayload & out)
{
  if (size != kCommandPayloadSize) {return false;}
  for (uint8_t i = 0; i < kJointCount; ++i) {
    out.position_micro[i] = getI32(payload + i * 4);
  }
  out.flags = payload[kJointCount * 4];
  return true;
}

inline bool decodeState(const uint8_t * payload, size_t size, StatePayload & out)
{
  if (size != kStatePayloadSize) {return false;}
  for (uint8_t i = 0; i < kJointCount; ++i) {
    out.position_micro[i] = getI32(payload + i * 4);
  }
  out.status = payload[kJointCount * 4];
  out.sequence = getU16(payload + kJointCount * 4 + 1);
  return true;
}

inline bool decodeInfo(const uint8_t * payload, size_t size, InfoPayload & out)
{
  if (size != kInfoPayloadSize) {return false;}
  out.protocol_version = payload[0];
  out.firmware_major = payload[1];
  out.firmware_minor = payload[2];
  out.firmware_patch = payload[3];
  out.joint_count = payload[4];
  out.heartbeat_timeout_ms = getU16(payload + 5);
  return true;
}

inline bool decodeFault(const uint8_t * payload, size_t size, FaultPayload & out)
{
  if (size != kFaultPayloadSize) {return false;}
  out.code = static_cast<FaultCode>(payload[0]);
  out.detail = getU16(payload + 1);
  return true;
}

inline bool decodePing(const uint8_t * payload, size_t size, uint32_t & nonce)
{
  if (size != kPingPayloadSize) {return false;}
  nonce = static_cast<uint32_t>(getI32(payload));
  return true;
}


// -------------------------------------------------------------- parser --

/// Why a frame was discarded. Surfaced so a fault can name the cause
/// rather than reporting a generic "bad data".
enum class ParseError : uint8_t
{
  kNone = 0,
  kBadVersion,
  kBadLength,
  kBadCrc,
};

/// Streaming frame parser.
///
/// Serial is a byte stream, not a message stream: frames arrive split
/// across reads, and a device reset or a noisy line drops bytes in the
/// middle of one. A parser that assumed whole frames would work
/// perfectly on a desk and fail intermittently on a robot.
///
/// So this consumes bytes one at a time, holds partial frames across
/// calls, and RESYNCS on the magic after any corruption. Fixed buffer,
/// no allocation, safe on a microcontroller.
class FrameParser
{
public:
  /// Feed one byte. Returns true when `this` now holds a complete,
  /// CRC-valid frame, readable via type(), payload() and payloadSize().
  bool push(uint8_t byte)
  {
    switch (state_) {
      case State::kMagic0:
        if (byte == kMagic0) {state_ = State::kMagic1;}
        return false;

      case State::kMagic1:
        // A repeated magic0 is not an error: it may be the real start of
        // a frame whose predecessor was truncated.
        if (byte == kMagic1) {state_ = State::kVersion;} else if (byte == kMagic0) {
          state_ = State::kMagic1;
        } else {state_ = State::kMagic0;}
        return false;

      case State::kVersion:
        if (byte != kProtocolVersion) {
          fail(ParseError::kBadVersion);
          return false;
        }
        version_ = byte;
        state_ = State::kType;
        return false;

      case State::kType:
        type_ = byte;
        state_ = State::kLength;
        return false;

      case State::kLength:
        if (byte > kMaxPayloadSize) {
          fail(ParseError::kBadLength);
          return false;
        }
        length_ = byte;
        received_ = 0;
        state_ = (length_ == 0) ? State::kCrc0 : State::kPayload;
        return false;

      case State::kPayload:
        payload_[received_++] = byte;
        if (received_ == length_) {state_ = State::kCrc0;}
        return false;

      case State::kCrc0:
        crc_low_ = byte;
        state_ = State::kCrc1;
        return false;

      case State::kCrc1: {
          const uint16_t received_crc =
            static_cast<uint16_t>(crc_low_) | static_cast<uint16_t>(byte << 8);
          uint8_t header[3] = {version_, type_, length_};
          uint16_t crc = crc16(header, 3);
          crc = crc16(payload_, length_, crc);
          state_ = State::kMagic0;
          if (crc != received_crc) {
            error_ = ParseError::kBadCrc;
            ++error_count_;
            return false;
          }
          error_ = ParseError::kNone;
          ++frame_count_;
          return true;
        }
    }
    return false;
  }

  /// Feed a buffer, invoking `on_frame(parser)` for each complete frame.
  template<typename Callback>
  void pushBuffer(const uint8_t * data, size_t size, Callback on_frame)
  {
    for (size_t i = 0; i < size; ++i) {
      if (push(data[i])) {on_frame(*this);}
    }
  }

  MessageType type() const {return static_cast<MessageType>(type_);}
  const uint8_t * payload() const {return payload_;}
  size_t payloadSize() const {return length_;}

  ParseError lastError() const {return error_;}
  uint32_t frameCount() const {return frame_count_;}
  uint32_t errorCount() const {return error_count_;}

  /// Drop any partial frame. Used after a reconnect, where leftover
  /// bytes belong to a previous session.
  void reset()
  {
    state_ = State::kMagic0;
    received_ = 0;
    length_ = 0;
  }

private:
  enum class State : uint8_t
  {
    kMagic0, kMagic1, kVersion, kType, kLength, kPayload, kCrc0, kCrc1,
  };

  void fail(ParseError why)
  {
    error_ = why;
    ++error_count_;
    state_ = State::kMagic0;
  }

  State state_ = State::kMagic0;
  uint8_t version_ = 0;
  uint8_t type_ = 0;
  uint8_t length_ = 0;
  uint8_t received_ = 0;
  uint8_t crc_low_ = 0;
  uint8_t payload_[kMaxPayloadSize] = {};
  ParseError error_ = ParseError::kNone;
  uint32_t frame_count_ = 0;
  uint32_t error_count_ = 0;
};

}  // namespace protocol
}  // namespace threevn

#endif  // THREEVN_HARDWARE__PROTOCOL_HPP_
