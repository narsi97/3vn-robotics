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
/// The TCP transport, over a real socket on the loopback interface.
///
/// No fake here: the thing under test IS the socket handling, so the
/// far end is a real listener in a thread. What matters is what the
/// hardware interface relies on -- that a read never blocks, that a
/// far end which goes away is an error and not silence, and that a
/// host which is not there fails promptly with a reason.

#include <gmock/gmock.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "threevn_hardware/tcp_transport.hpp"

using threevn::hardware::TcpTransport;
using ::testing::HasSubstr;

namespace
{

/// A listener on a port the kernel chooses.
struct Listener
{
  Listener()
  {
    fd = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    ::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address));
    ::listen(fd, 1);
    socklen_t length = sizeof(address);
    ::getsockname(fd, reinterpret_cast<sockaddr *>(&address), &length);
    port = ntohs(address.sin_port);
  }

  ~Listener()
  {
    if (peer >= 0) {::close(peer);}
    if (fd >= 0) {::close(fd);}
  }

  void accept() {peer = ::accept(fd, nullptr, nullptr);}
  std::string endpoint() const {return "127.0.0.1:" + std::to_string(port);}

  int fd = -1;
  int peer = -1;
  int port = 0;
};

/// Poll a non-blocking read until `want` bytes arrive or time runs out.
std::vector<uint8_t> readFor(TcpTransport & transport, std::size_t want)
{
  std::vector<uint8_t> got;
  uint8_t buffer[64];
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (got.size() < want && std::chrono::steady_clock::now() < deadline) {
    const int n = transport.read(buffer, sizeof(buffer));
    if (n < 0) {
      break;
    }
    got.insert(got.end(), buffer, buffer + n);
    if (n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return got;
}

}  // namespace

TEST(Tcp, CarriesBytesBothWays)
{
  Listener listener;
  TcpTransport transport(listener.endpoint());
  ASSERT_TRUE(transport.open()) << transport.lastError();
  listener.accept();

  const std::vector<uint8_t> out = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  ASSERT_TRUE(transport.write(out.data(), out.size()));
  uint8_t received[16];
  ASSERT_EQ(::recv(listener.peer, received, sizeof(received), 0), 6);
  EXPECT_EQ(std::vector<uint8_t>(received, received + 6), out);

  const std::vector<uint8_t> back = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
  ASSERT_EQ(::send(listener.peer, back.data(), back.size(), 0), 6);
  EXPECT_EQ(readFor(transport, 6), back);
}

TEST(Tcp, AReadWithNothingWaitingReturnsAtOnce)
{
  // Called from the control loop. A read that waited for data would
  // stall every joint behind one quiet servo.
  Listener listener;
  TcpTransport transport(listener.endpoint());
  ASSERT_TRUE(transport.open());
  listener.accept();

  uint8_t buffer[16];
  const auto started = std::chrono::steady_clock::now();
  EXPECT_EQ(transport.read(buffer, sizeof(buffer)), 0);
  EXPECT_LT(std::chrono::steady_clock::now() - started, std::chrono::milliseconds(50));
}

TEST(Tcp, TheFarEndGoingAwayIsAnErrorNotSilence)
{
  // On a serial line this is the cable coming out. "Nothing to read"
  // would look exactly like a servo that is slow to answer.
  Listener listener;
  TcpTransport transport(listener.endpoint());
  ASSERT_TRUE(transport.open());
  listener.accept();
  ::close(listener.peer);
  listener.peer = -1;

  uint8_t buffer[16];
  int result = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (result == 0 && std::chrono::steady_clock::now() < deadline) {
    result = transport.read(buffer, sizeof(buffer));
  }
  EXPECT_EQ(result, -1);
  EXPECT_FALSE(transport.isOpen());
  EXPECT_THAT(transport.lastError(), HasSubstr("closed"));
}

TEST(Tcp, NobodyListeningFailsPromptlyWithAReason)
{
  int port = 0;
  {
    Listener listener;          // take a port, then give it back
    port = listener.port;
  }
  TcpTransport transport("127.0.0.1:" + std::to_string(port), 500);
  const auto started = std::chrono::steady_clock::now();
  EXPECT_FALSE(transport.open());
  EXPECT_LT(std::chrono::steady_clock::now() - started, std::chrono::seconds(2));
  EXPECT_THAT(transport.lastError(), HasSubstr("cannot connect"));
}

TEST(Tcp, AnEndpointThatIsNotHostAndPortIsRefused)
{
  for (const std::string bad : {"/dev/ttyACM0", "127.0.0.1", ":6560", "host:"}) {
    TcpTransport transport(bad);
    EXPECT_FALSE(transport.open()) << bad;
    EXPECT_THAT(transport.lastError(), HasSubstr("host:port")) << bad;
  }
}

TEST(Tcp, WritingBeforeOpenIsAnError)
{
  TcpTransport transport("127.0.0.1:1");
  const uint8_t byte = 0;
  EXPECT_FALSE(transport.write(&byte, 1));
  uint8_t buffer[4];
  EXPECT_EQ(transport.read(buffer, sizeof(buffer)), -1);
}
