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
/// The fake servo bus, behind a TCP socket.
///
///     ros2 run threevn_hardware fake_feetech_bus --port 6560
///
/// So that the real hardware interface, in the real controller manager,
/// under the real launch file, can be started with target:=feetech and
/// nothing plugged in. What it talks to is test/support/fake_feetech_bus.hpp:
/// the same object the unit tests drive in-process.
///
/// TCP rather than a pseudo-terminal because TCP is also how a Mac
/// reaches the real bus: Docker Desktop cannot pass USB into a
/// container, so the adapter is served from the host. The transport
/// this exercises is the one the physical arm will use there.
///
/// Bound to 127.0.0.1. It simulates motors; it has no business being
/// reachable from anywhere else.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "support/fake_feetech_bus.hpp"

namespace
{

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int signal_number)
{
  (void)signal_number;
  g_stop = 1;
}

std::vector<int> parseIds(const std::string & text)
{
  std::vector<int> ids;
  std::size_t start = 0;
  while (start < text.size()) {
    const std::size_t comma = text.find(',', start);
    const std::string item = text.substr(start, comma - start);
    if (!item.empty()) {
      ids.push_back(std::atoi(item.c_str()));
    }
    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }
  return ids;
}

}  // namespace

int main(int argc, char ** argv)
{
  int port = 6560;
  std::string ids_text = "1,2,3,4,5,6";
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string flag = argv[i];
    if (flag == "--port") {
      port = std::atoi(argv[i + 1]);
    } else if (flag == "--ids") {
      ids_text = argv[i + 1];
    } else {
      std::fprintf(stderr, "usage: fake_feetech_bus [--port N] [--ids 1,2,...]\n");
      return 2;
    }
  }

  threevn::feetech::FakeBus bus;
  const std::vector<int> ids = parseIds(ids_text);
  for (const int id : ids) {
    if (id < 0 || id > 253) {
      std::fprintf(stderr, "fake_feetech_bus: %d is not a servo id (0-253)\n", id);
      return 2;
    }
    bus.addServo(static_cast<uint8_t>(id));
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  // A host that disconnects mid-write must not kill the bus.
  std::signal(SIGPIPE, SIG_IGN);

  const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listener < 0) {
    std::perror("socket");
    return 1;
  }
  const int yes = 1;
  ::setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(port));
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
    ::listen(listener, 1) != 0)
  {
    std::fprintf(stderr, "fake_feetech_bus: cannot listen on 127.0.0.1:%d: %s\n",
      port, std::strerror(errno));
    ::close(listener);
    return 1;
  }

  std::printf(
    "fake Feetech bus: %zu servos (ids %s) on 127.0.0.1:%d\n",
    ids.size(), ids_text.c_str(), port);
  std::fflush(stdout);

  using clock = std::chrono::steady_clock;
  auto last = clock::now();
  int client = -1;

  while (!g_stop) {
    // Time passes for the servos whether or not anyone is talking to
    // them, exactly as it does for real ones.
    const auto now = clock::now();
    bus.step(std::chrono::duration<double>(now - last).count());
    last = now;

    pollfd watched{};
    watched.fd = client >= 0 ? client : listener;
    watched.events = POLLIN;
    if (::poll(&watched, 1, 1) <= 0) {
      continue;
    }

    if (client < 0) {
      client = ::accept(listener, nullptr, nullptr);
      if (client >= 0) {
        ::setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        std::printf("fake Feetech bus: host connected\n");
        std::fflush(stdout);
      }
      continue;
    }

    uint8_t buffer[512];
    const ssize_t received = ::recv(client, buffer, sizeof(buffer), 0);
    if (received <= 0) {
      ::close(client);
      client = -1;
      std::printf("fake Feetech bus: host disconnected\n");
      std::fflush(stdout);
      continue;
    }
    const std::vector<uint8_t> reply =
      bus.exchange(buffer, static_cast<std::size_t>(received));
    if (!reply.empty()) {
      ::send(client, reply.data(), reply.size(), MSG_NOSIGNAL);
    }
  }

  if (client >= 0) {
    ::close(client);
  }
  ::close(listener);
  return 0;
}
