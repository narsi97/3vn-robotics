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

#include "threevn_hardware/tcp_transport.hpp"

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <utility>

namespace threevn
{
namespace hardware
{

TcpTransport::TcpTransport(std::string endpoint, int connect_timeout_ms)
: endpoint_(std::move(endpoint)), connect_timeout_ms_(connect_timeout_ms) {}

TcpTransport::~TcpTransport()
{
  close();
}

bool TcpTransport::open()
{
  close();

  // The LAST colon, so an IPv6 literal's own colons are not mistaken
  // for the separator.
  const std::size_t colon = endpoint_.rfind(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 == endpoint_.size()) {
    last_error_ = "endpoint '" + endpoint_ + "' is not host:port";
    return false;
  }
  const std::string host = endpoint_.substr(0, colon);
  const std::string port = endpoint_.substr(colon + 1);

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo * found = nullptr;
  const int lookup = ::getaddrinfo(host.c_str(), port.c_str(), &hints, &found);
  if (lookup != 0) {
    last_error_ = "cannot resolve " + endpoint_ + ": " + gai_strerror(lookup);
    return false;
  }

  last_error_ = "cannot connect to " + endpoint_ + ": no address worked";
  for (addrinfo * candidate = found; candidate != nullptr; candidate = candidate->ai_next) {
    const int fd = ::socket(
      candidate->ai_family, candidate->ai_socktype, candidate->ai_protocol);
    if (fd < 0) {
      continue;
    }
    // Non-blocking from the first call: a connect() to a host that is
    // not answering otherwise blocks for the kernel's own timeout,
    // which is minutes, inside a lifecycle transition.
    ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    int result = ::connect(fd, candidate->ai_addr, candidate->ai_addrlen);
    if (result != 0 && errno == EINPROGRESS) {
      pollfd waiting{};
      waiting.fd = fd;
      waiting.events = POLLOUT;
      if (::poll(&waiting, 1, connect_timeout_ms_) == 1) {
        int error = 0;
        socklen_t length = sizeof(error);
        ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &length);
        result = error == 0 ? 0 : -1;
        errno = error;
      } else {
        result = -1;
        errno = ETIMEDOUT;
      }
    }
    if (result != 0) {
      last_error_ = "cannot connect to " + endpoint_ + ": " + std::strerror(errno);
      ::close(fd);
      continue;
    }

    // Packets here are ten bytes and each one is a question waiting for
    // its answer. Nagle's algorithm would hold them back to batch them.
    const int yes = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    fd_ = fd;
    last_error_.clear();
    break;
  }
  ::freeaddrinfo(found);
  return fd_ >= 0;
}

void TcpTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int TcpTransport::read(uint8_t * buffer, std::size_t size)
{
  if (fd_ < 0) {
    last_error_ = "not connected";
    return -1;
  }
  const ssize_t n = ::recv(fd_, buffer, size, MSG_DONTWAIT);
  if (n > 0) {
    return static_cast<int>(n);
  }
  if (n == 0) {
    // An orderly close from the far end. On a serial line the
    // equivalent is the cable coming out, and it is reported the same
    // way: as an error, not as "nothing to read yet".
    last_error_ = endpoint_ + " closed the connection";
    close();
    return -1;
  }
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
    return 0;
  }
  last_error_ = std::string("recv: ") + std::strerror(errno);
  return -1;
}

bool TcpTransport::write(const uint8_t * buffer, std::size_t size)
{
  if (fd_ < 0) {
    last_error_ = "not connected";
    return false;
  }
  std::size_t sent = 0;
  while (sent < size) {
    // MSG_NOSIGNAL: a peer that has gone must produce an error return,
    // not a SIGPIPE that kills the controller manager.
    const ssize_t n = ::send(fd_, buffer + sent, size - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd waiting{};
      waiting.fd = fd_;
      waiting.events = POLLOUT;
      if (::poll(&waiting, 1, 20) == 1) {
        continue;
      }
      last_error_ = "send: the far end is not draining";
      return false;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    last_error_ = std::string("send: ") + std::strerror(errno);
    return false;
  }
  return true;
}

}  // namespace hardware
}  // namespace threevn
