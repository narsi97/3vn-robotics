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
/// A TCP Transport: a serial bus that is somewhere else.
///
/// Docker Desktop on macOS cannot pass a USB device into a container.
/// The adapter is plugged into the Mac, the control stack runs in the
/// container, and nothing but a network sits between them. So the bus
/// is served from the host as a TCP port, and this is the container's
/// end of it.
///
/// The same Transport contract as SerialTransport: non-blocking reads,
/// whole writes, no framing. Which means the hardware interface above
/// does not know which one it has, and what is proven against one
/// holds for the other.

#ifndef THREEVN_HARDWARE__TCP_TRANSPORT_HPP_
#define THREEVN_HARDWARE__TCP_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "threevn_hardware/transport.hpp"

namespace threevn
{
namespace hardware
{

class TcpTransport : public Transport
{
public:
  /// `endpoint` is "host:port". `connect_timeout_ms` bounds open().
  explicit TcpTransport(std::string endpoint, int connect_timeout_ms = 2000);
  ~TcpTransport() override;

  bool open() override;
  void close() override;
  bool isOpen() const override {return fd_ >= 0;}
  int read(uint8_t * buffer, std::size_t size) override;
  bool write(const uint8_t * buffer, std::size_t size) override;
  std::string lastError() const override {return last_error_;}

private:
  std::string endpoint_;
  int connect_timeout_ms_;
  int fd_ = -1;
  std::string last_error_;
};

}  // namespace hardware
}  // namespace threevn

#endif  // THREEVN_HARDWARE__TCP_TRANSPORT_HPP_
