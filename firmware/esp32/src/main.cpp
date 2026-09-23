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
/// ESP32 firmware for the 3VN arm.
///
/// Deliberately thin. Every decision - limit clamping, the heartbeat
/// timeout, the emergency stop latch - lives in controller.hpp, which
/// compiles on a laptop and is covered by test_firmware_logic.cpp. This
/// file moves bytes and writes PWM.
///
/// That split is not tidiness. Debugging a 4 MB microcontroller with no
/// debugger, by flashing and watching an arm, is enormously slower than
/// running a test. Anything that can be decided off the device should be.
///
/// The device is a SERVO DRIVER, not a robot controller. Kinematics,
/// trajectories and planning stay on the host, or the project's central
/// claim - that the application is not coupled to the ESP32 - stops
/// being true.

#include <Arduino.h>
#include <ESP32Servo.h>

#include "controller.hpp"
#include "joint_limits.hpp"
#include "threevn_hardware/protocol.hpp"

namespace proto = threevn::protocol;
namespace fw = threevn::firmware;

namespace
{

/// Servo signal pins, in protocol joint order.
constexpr int kServoPins[proto::kJointCount] = {13, 12, 14, 27, 26};

/// Pulse widths. 500-2500 us is the usable span of an MG996R; the SG90
/// gripper shares it. Narrower than a datasheet's absolute range on
/// purpose - commanding a servo to its mechanical end stop makes it buzz
/// and cook itself.
constexpr int kMinPulseUs = 500;
constexpr int kMaxPulseUs = 2500;

/// Diagnostics go to Serial1, never Serial. Serial carries binary frames,
/// and a stray printf there corrupts the protocol.
constexpr int kDiagTxPin = 17;

constexpr uint32_t kStateIntervalMs = 20;   // 50 Hz, matching the host

Servo servos[proto::kJointCount];
fw::Controller controller;
proto::FrameParser parser;

uint16_t sequence = 0;
uint32_t last_state_ms = 0;

/// Map a joint's micro-units onto a pulse width.
///
/// Linear across the joint's own configured travel, so the mapping stays
/// correct if a limit changes. Note this uses the FIRMWARE's limits, not
/// anything the host sent.
int toPulseUs(uint8_t index, int32_t value_micro)
{
  const fw::JointLimit & limit = fw::kJointLimits[index];
  const int32_t span = limit.max_micro - limit.min_micro;
  if (span <= 0) { return (kMinPulseUs + kMaxPulseUs) / 2; }

  int32_t clamped = value_micro;
  if (clamped < limit.min_micro) { clamped = limit.min_micro; }
  if (clamped > limit.max_micro) { clamped = limit.max_micro; }

  const int64_t offset = static_cast<int64_t>(clamped - limit.min_micro);
  return static_cast<int>(
    kMinPulseUs + (offset * (kMaxPulseUs - kMinPulseUs)) / span);
}

void sendFrame(const uint8_t * frame, size_t length)
{
  Serial.write(frame, length);
}

void sendState()
{
  int32_t measured[proto::kJointCount];
  for (uint8_t i = 0; i < proto::kJointCount; ++i) {
    // Open-loop: hobby servos have no position feedback, so the best
    // available "measurement" is the target we are driving toward.
    //
    // This is REPORTED AS MEASURED and it is a real limitation: a
    // stalled or stripped servo looks perfectly healthy from here. It is
    // why the host differentiates position for velocity rather than
    // trusting a number the device invented, and why the BOM lists
    // feedback servos as the upgrade that matters most.
    measured[i] = controller.output().position_micro[i];
  }

  uint8_t frame[proto::kMaxFrameSize];
  const size_t n = proto::encodeState(
    controller.state(measured, sequence++), frame, sizeof(frame));
  if (n > 0) { sendFrame(frame, n); }
}

void sendInfo()
{
  uint8_t frame[proto::kMaxFrameSize];
  const size_t n = proto::encodeInfo(controller.info(), frame, sizeof(frame));
  if (n > 0) { sendFrame(frame, n); }
}

void sendFault(proto::FaultCode code, uint16_t detail)
{
  uint8_t frame[proto::kMaxFrameSize];
  proto::FaultPayload fault{code, detail};
  const size_t n = proto::encodeFault(fault, frame, sizeof(frame));
  if (n > 0) { sendFrame(frame, n); }
}

void applyOutput()
{
  const fw::Output out = controller.output();
  for (uint8_t i = 0; i < proto::kJointCount; ++i) {
    if (out.torque_enabled) {
      servos[i].writeMicroseconds(toPulseUs(i, out.position_micro[i]));
    } else {
      // Detaching stops the pulse train, which releases the servo.
      // Writing a "neutral" pulse instead would drive the arm to the
      // middle of its range, which is a movement nobody asked for.
      if (servos[i].attached()) { servos[i].detach(); }
    }
  }
  if (out.torque_enabled) {
    for (uint8_t i = 0; i < proto::kJointCount; ++i) {
      if (!servos[i].attached()) {
        servos[i].attach(kServoPins[i], kMinPulseUs, kMaxPulseUs);
      }
    }
  }
}

void onFrame(const proto::FrameParser & p)
{
  switch (p.type()) {
    case proto::MessageType::kCommand: {
      proto::CommandPayload command;
      if (!proto::decodeCommand(p.payload(), p.payloadSize(), command)) {
        sendFault(proto::FaultCode::kBadLength, p.payloadSize());
        return;
      }
      // The host clamps too, and ros2_control clamps as well. This clamp
      // is the one that still works when the host has crashed.
      controller.onCommand(command, millis());
      break;
    }

    case proto::MessageType::kPing: {
      uint32_t nonce = 0;
      if (proto::decodePing(p.payload(), p.payloadSize(), nonce)) {
        uint8_t frame[proto::kMaxFrameSize];
        const size_t n = proto::encodePong(nonce, frame, sizeof(frame));
        if (n > 0) { sendFrame(frame, n); }
      }
      break;
    }

    case proto::MessageType::kGetInfo:
      sendInfo();
      break;

    default:
      sendFault(proto::FaultCode::kUnknownType, static_cast<uint16_t>(p.type()));
      break;
  }
}

}  // namespace

void setup()
{
  // Binary frames only. Nothing else may ever write to Serial.
  Serial.begin(921600);

  Serial1.begin(115200, SERIAL_8N1, -1, kDiagTxPin);
  Serial1.printf(
    "3VN firmware %u.%u.%u, protocol %u, heartbeat %u ms\n",
    fw::kFirmwareMajor, fw::kFirmwareMinor, fw::kFirmwarePatch,
    proto::kProtocolVersion, fw::kHeartbeatTimeoutMs);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (uint8_t i = 0; i < proto::kJointCount; ++i) {
    servos[i].setPeriodHertz(50);
  }

  // Servos are NOT attached here. The arm must not move until the host
  // has said where it should be - a device that energises at boot moves
  // before anything has told it where to go.
  last_state_ms = millis();
}

void loop()
{
  const uint32_t now = millis();

  while (Serial.available() > 0) {
    const int byte = Serial.read();
    if (byte < 0) { break; }
    if (parser.push(static_cast<uint8_t>(byte))) {
      onFrame(parser);
    }
  }

  const bool was_lost = controller.heartbeatLost();
  controller.tick(now);
  if (!was_lost && controller.heartbeatLost()) {
    Serial1.println("heartbeat lost: holding position");
    sendFault(proto::FaultCode::kHeartbeatTimeout, 0);
  }

  applyOutput();

  if (now - last_state_ms >= kStateIntervalMs) {
    last_state_ms = now;
    sendState();
  }
}
