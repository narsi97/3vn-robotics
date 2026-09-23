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

#include "threevn_hardware/serial_transport.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <string>
#include <utility>

namespace threevn
{
namespace hardware
{
namespace
{

/// Map a numeric baud to its termios constant. Only rates the ESP32's
/// USB CDC actually uses; an unknown value is refused rather than
/// silently falling back, because a wrong baud produces garbage that
/// looks like line noise.
speed_t toSpeed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
  }
}

}  // namespace

SerialTransport::SerialTransport(std::string device, int baud)
: device_(std::move(device)), baud_(baud) {}

SerialTransport::~SerialTransport()
{
  close();
}

bool SerialTransport::open()
{
  close();

  const speed_t speed = toSpeed(baud_);
  if (speed == 0) {
    last_error_ = "unsupported baud rate " + std::to_string(baud_);
    return false;
  }

  // O_NONBLOCK so neither open() nor read() can stall the control loop.
  // O_NOCTTY so a disconnect cannot deliver a signal to this process.
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    last_error_ = "cannot open " + device_ + ": " + strerror(errno);
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    last_error_ = std::string("tcgetattr: ") + strerror(errno);
    close();
    return false;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // Raw mode. Any line discipline processing would mangle binary frames:
  // ICRNL would rewrite 0x0D, and ISIG would let a payload byte send a
  // signal.
  cfmakeraw(&tty);

  tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem lines, enable receive
  tty.c_cflag &= ~CSTOPB;            // one stop bit
  tty.c_cflag &= ~PARENB;            // no parity; the CRC covers us
  tty.c_cflag &= ~CRTSCTS;           // no hardware flow control

  // Non-blocking: return whatever is available immediately.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    last_error_ = std::string("tcsetattr: ") + strerror(errno);
    close();
    return false;
  }

  // Discard anything buffered from a previous session; those bytes
  // belong to a different conversation.
  tcflush(fd_, TCIOFLUSH);

  last_error_.clear();
  return true;
}

void SerialTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int SerialTransport::read(uint8_t * buffer, std::size_t size)
{
  if (fd_ < 0) {
    last_error_ = "read on a closed port";
    return -1;
  }
  const ssize_t n = ::read(fd_, buffer, size);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;        // nothing available; not an error
    }
    last_error_ = std::string("read: ") + strerror(errno);
    return -1;
  }
  return static_cast<int>(n);
}

bool SerialTransport::write(const uint8_t * buffer, std::size_t size)
{
  if (fd_ < 0) {
    last_error_ = "write on a closed port";
    return false;
  }
  std::size_t written = 0;
  while (written < size) {
    const ssize_t n = ::write(fd_, buffer + written, size - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // The kernel buffer is full. A command frame is ~28 bytes and
        // the buffer is kilobytes, so this means the device has stopped
        // draining -- report it rather than spinning.
        last_error_ = "write would block; device not draining";
        return false;
      }
      last_error_ = std::string("write: ") + strerror(errno);
      return false;
    }
    written += static_cast<std::size_t>(n);
  }
  return true;
}

}  // namespace hardware
}  // namespace threevn
