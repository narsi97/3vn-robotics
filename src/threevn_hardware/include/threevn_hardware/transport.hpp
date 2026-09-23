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
/// A byte pipe to the device, and nothing more.
///
/// The seam inside the seam. Esp32SystemInterface talks to this, never
/// to a file descriptor, so the whole hardware interface can be tested
/// against a fake with no serial port, no device and no timing - which
/// is what makes its behaviour under disconnection and garbage provable
/// rather than hoped for.
///
/// It is deliberately narrow: open, close, read, write. No framing, no
/// retries, no protocol knowledge. Anything richer would have to be
/// reimplemented by every transport, and TCP in Phase 12 would inherit
/// assumptions that only hold for a UART.

#ifndef THREEVN_HARDWARE__TRANSPORT_HPP_
#define THREEVN_HARDWARE__TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace threevn
{
namespace hardware
{

class Transport
{
public:
  virtual ~Transport() = default;

  /// Open the device. Returns false and sets lastError() on failure.
  virtual bool open() = 0;

  virtual void close() = 0;

  virtual bool isOpen() const = 0;

  /// Read up to `size` bytes without blocking.
  ///
  /// Returns the count read, 0 when nothing is available, or -1 on a
  /// transport error. Non-blocking is not a preference: this is called
  /// from the ros2_control read() cycle at 50 Hz, and a blocking read
  /// would stall the entire control loop behind a silent device.
  virtual int read(uint8_t * buffer, std::size_t size) = 0;

  /// Write all `size` bytes. Returns false on a short or failed write.
  virtual bool write(const uint8_t * buffer, std::size_t size) = 0;

  /// Why the last call failed. Empty when it did not.
  virtual std::string lastError() const = 0;
};

}  // namespace hardware
}  // namespace threevn

#endif  // THREEVN_HARDWARE__TRANSPORT_HPP_
