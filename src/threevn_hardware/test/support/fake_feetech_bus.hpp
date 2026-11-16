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
/// A servo bus with no servos on it.
///
/// Six STS3215s, simulated: enough of the control table and enough of
/// the protocol that the real hardware interface cannot tell, byte for
/// byte, that nothing is plugged in. It is the SO-101 one layer below
/// the simulator. Gazebo stands in for the arm's physics; this stands
/// in for its wiring.
///
/// TEST EQUIPMENT. It is built only with the tests and is not in the
/// image that ships to a robot.
///
/// It is used two ways, and they are the same object:
///
///   * in-process, behind a Transport, by the unit tests
///   * behind a TCP socket (fake_feetech_bus_main.cpp), so the whole
///     stack can be launched with target:=feetech and nothing attached
///
/// WHAT IT IS NOT is evidence about a real servo. It was written from
/// the same reading of the protocol as the interface it tests, so the
/// two agree by construction. Where the real servo's behaviour is not
/// known, this is deliberately the STRICTER of the possibilities: the
/// goal register does not follow the shaft while torque is off, so an
/// interface that enables torque without first setting the goal jumps
/// here, visibly, whether or not it would on the bench.

#ifndef SUPPORT__FAKE_FEETECH_BUS_HPP_
#define SUPPORT__FAKE_FEETECH_BUS_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "threevn_hardware/feetech_protocol.hpp"

namespace threevn
{
namespace feetech
{

class FakeBus
{
public:
  /// Counts per second with no speed limit set: about 0.2 s per 60 degrees.
  static constexpr double kFreeSpeed = 3400.0;

  struct Servo
  {
    std::array<uint8_t, 128> table{};
    /// The shaft, in counts. Fractional so slow motion accumulates.
    double position = kCentreCount;
    double velocity = 0.0;
    /// Unplugged servos stay on the bus object and say nothing.
    bool answering = true;
    /// Held still from outside. It pushes, reports the load, and does not move.
    bool blocked = false;
  };

  void addServo(uint8_t id, int position = kCentreCount)
  {
    Servo servo;
    servo.position = position;
    servo.table[reg::kId] = id;
    // The goal does NOT start at the shaft. See the file comment.
    writeWord(servo, reg::kGoalPosition, kCentreCount);
    servo.table[reg::kPresentVoltage] = 50;       // 5.0 V, the bill of materials' supply
    servo.table[reg::kPresentTemperature] = 25;
    servos_[id] = servo;
  }

  bool has(uint8_t id) const {return servos_.count(id) != 0;}
  Servo & servo(uint8_t id) {return servos_.at(id);}

  int goal(uint8_t id) {return readWord(servos_.at(id), reg::kGoalPosition);}
  bool torqueEnabled(uint8_t id) {return servos_.at(id).table[reg::kTorqueEnable] != 0;}

  /// Corrupt the checksum of the next reply the bus sends.
  void corruptNextReply() {corrupt_next_ = true;}

  std::size_t instructionsSeen() const {return instructions_seen_;}

  /// Hand the bus bytes from the host; get back whatever it replies.
  std::vector<uint8_t> exchange(const uint8_t * data, std::size_t size)
  {
    // An instruction has the same framing as a status packet, with the
    // instruction where the error byte would be.
    parser_.push(data, size);
    std::vector<uint8_t> reply;
    Status packet;
    while (parser_.next(packet)) {
      ++instructions_seen_;
      handle(packet.id, static_cast<Instruction>(packet.error), packet.params, reply);
    }
    return reply;
  }

  /// Let time pass.
  void step(double seconds)
  {
    if (seconds <= 0.0) {
      return;
    }
    for (auto & entry : servos_) {
      Servo & servo = entry.second;
      const double goal = readWord(servo, reg::kGoalPosition);
      const double error = goal - servo.position;
      int load = 0;
      servo.velocity = 0.0;
      if (servo.table[reg::kTorqueEnable] != 0 && std::fabs(error) > 1e-9) {
        if (servo.blocked) {
          // Stalled against something: full effort, no motion.
          load = error > 0 ? 1000 : -1000;
        } else {
          const int limit = readWord(servo, reg::kGoalSpeed);
          const double speed = limit > 0 ? limit : kFreeSpeed;
          const double travel = std::fmin(std::fabs(error), speed * seconds);
          const double moved = error > 0 ? travel : -travel;
          servo.position += moved;
          servo.velocity = moved / seconds;
        }
      }
      writeWord(servo, reg::kPresentLoad, toSignMagnitude(load, 10));
    }
  }

private:
  static int readWord(const Servo & servo, uint8_t address)
  {
    return fromBytes(servo.table[address], servo.table[address + 1]);
  }

  static void writeWord(Servo & servo, uint8_t address, int value)
  {
    servo.table[address] = static_cast<uint8_t>(value & 0xFF);
    servo.table[address + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  }

  /// Copy the simulated shaft into the registers a host reads.
  static void refresh(Servo & servo)
  {
    const auto count = std::lround(servo.position);
    writeWord(servo, reg::kPresentPosition, static_cast<int>(count));
    writeWord(
      servo, reg::kPresentSpeed,
      toSignMagnitude(static_cast<int>(std::lround(servo.velocity)), 15));
  }

  void appendStatus(
    uint8_t id, const std::vector<uint8_t> & params, std::vector<uint8_t> & reply)
  {
    // A status packet carries the error byte where an instruction
    // carries its opcode. Zero: this servo has nothing to complain of.
    std::vector<uint8_t> packet = encode(id, static_cast<Instruction>(0), params);
    if (corrupt_next_) {
      packet.back() = static_cast<uint8_t>(packet.back() ^ 0x5A);
      corrupt_next_ = false;
    }
    reply.insert(reply.end(), packet.begin(), packet.end());
  }

  void store(Servo & servo, uint8_t address, const uint8_t * data, std::size_t size)
  {
    for (std::size_t i = 0; i < size && address + i < servo.table.size(); ++i) {
      servo.table[address + i] = data[i];
    }
  }

  void handle(
    uint8_t id, Instruction instruction, const std::vector<uint8_t> & params,
    std::vector<uint8_t> & reply)
  {
    switch (instruction) {
      case Instruction::kPing: {
          // A broadcast ping would have every servo answer at once.
          const auto it = servos_.find(id);
          if (it != servos_.end() && it->second.answering) {
            appendStatus(id, {}, reply);
          }
          break;
        }

      case Instruction::kRead: {
          const auto it = servos_.find(id);
          if (it == servos_.end() || !it->second.answering || params.size() != 2) {
            break;
          }
          refresh(it->second);
          const std::size_t address = params[0];
          const std::size_t length = params[1];
          if (address + length > it->second.table.size()) {
            break;
          }
          appendStatus(
            id, std::vector<uint8_t>(
              it->second.table.begin() + static_cast<std::ptrdiff_t>(address),
              it->second.table.begin() + static_cast<std::ptrdiff_t>(address + length)), reply);
          break;
        }

      case Instruction::kWrite: {
          if (params.empty()) {
            break;
          }
          for (auto & entry : servos_) {
            if (id != kBroadcastId && entry.first != id) {
              continue;
            }
            if (!entry.second.answering) {
              continue;
            }
            store(entry.second, params[0], params.data() + 1, params.size() - 1);
            if (id != kBroadcastId) {
              appendStatus(id, {}, reply);
            }
          }
          rekey();
          break;
        }

      case Instruction::kSyncRead: {
          if (params.size() < 2) {
            break;
          }
          const std::size_t address = params[0];
          const std::size_t length = params[1];
          // One reply per servo asked, in the order asked. A servo that
          // is not there leaves a gap, not a placeholder.
          for (std::size_t i = 2; i < params.size(); ++i) {
            const auto it = servos_.find(params[i]);
            if (it == servos_.end() || !it->second.answering) {
              continue;
            }
            if (address + length > it->second.table.size()) {
              continue;
            }
            refresh(it->second);
            appendStatus(
              params[i], std::vector<uint8_t>(
                it->second.table.begin() + static_cast<std::ptrdiff_t>(address),
                it->second.table.begin() + static_cast<std::ptrdiff_t>(address + length)), reply);
          }
          break;
        }

      case Instruction::kSyncWrite: {
          if (params.size() < 2) {
            break;
          }
          const uint8_t address = params[0];
          const std::size_t length = params[1];
          for (std::size_t i = 2; i + length < params.size(); i += length + 1) {
            const auto it = servos_.find(params[i]);
            if (it != servos_.end() && it->second.answering) {
              store(it->second, address, params.data() + i + 1, length);
            }
          }
          break;
        }

      default:
        break;
    }
  }

  /// Follow a servo whose id register was just rewritten.
  ///
  /// How a real arm is commissioned: each servo ships as id 1 and is
  /// renumbered while it is alone on the bus.
  void rekey()
  {
    std::map<uint8_t, Servo> renamed;
    for (auto & entry : servos_) {
      renamed[entry.second.table[reg::kId]] = entry.second;
    }
    servos_.swap(renamed);
  }

  std::map<uint8_t, Servo> servos_;
  StatusParser parser_;
  bool corrupt_next_ = false;
  std::size_t instructions_seen_ = 0;
};

}  // namespace feetech
}  // namespace threevn

#endif  // SUPPORT__FAKE_FEETECH_BUS_HPP_
