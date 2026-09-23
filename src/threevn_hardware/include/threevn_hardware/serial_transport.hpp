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
/// A POSIX serial Transport for the ESP32's USB CDC port.

#ifndef THREEVN_HARDWARE__SERIAL_TRANSPORT_HPP_
#define THREEVN_HARDWARE__SERIAL_TRANSPORT_HPP_

#include <cstdint>
#include <string>

#include "threevn_hardware/transport.hpp"

namespace threevn
{
namespace hardware
{

class SerialTransport : public Transport
{
public:
  SerialTransport(std::string device, int baud);
  ~SerialTransport() override;

  bool open() override;
  void close() override;
  bool isOpen() const override {return fd_ >= 0;}
  int read(uint8_t * buffer, std::size_t size) override;
  bool write(const uint8_t * buffer, std::size_t size) override;
  std::string lastError() const override {return last_error_;}

private:
  std::string device_;
  int baud_;
  int fd_ = -1;
  std::string last_error_;
};

}  // namespace hardware
}  // namespace threevn

#endif  // THREEVN_HARDWARE__SERIAL_TRANSPORT_HPP_
