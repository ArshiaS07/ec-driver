// TcpSocket against a real server on 127.0.0.1 (support/loopback_server.hpp).
// Not covered here: a connect that times out (needs a network that silently
// drops packets) and waits interrupted by signals.

#include "ec/net/tcp_socket.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <expected>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "support/loopback_server.hpp"

#if !defined(_WIN32)
#include <fcntl.h>
#include <sys/resource.h>
#include <sys/select.h>
#include <unistd.h>
#endif

namespace {

using namespace std::chrono_literals;
using ec::net::NetError;
using ec::net::TcpSocket;
using ec::test::LoopbackServer;
using Clock = std::chrono::steady_clock;

std::span<const std::uint8_t> bytes(std::string_view s) {
    return {reinterpret_cast<const std::uint8_t*>(s.data()), s.size()};
}

// Receives until `count` bytes have arrived (TCP may deliver them in pieces).
std::string receive_exactly(TcpSocket& sock, std::size_t count) {
    std::string out;
    std::array<std::uint8_t, 256> buf{};
    while (out.size() < count) {
        const auto n = sock.receive(buf, 1s);
        if (!n) break;
        out.append(reinterpret_cast<const char*>(buf.data()), *n);
    }
    return out;
}

TcpSocket connect_to(const LoopbackServer& server) {
    auto sock = TcpSocket::connect("127.0.0.1", server.port(), 2s);
    if (!sock) throw std::runtime_error("connect failed: " + sock.error().message());
    return std::move(*sock);
}

TEST(TcpSocket, SendsAndReceives) {
    LoopbackServer server;
    TcpSocket sock = connect_to(server);
    auto peer = server.accept();

    ASSERT_TRUE(sock.send_all(bytes("ping\n"), 1s));
    const auto got = peer.receive(5);
    EXPECT_EQ(std::string(got.begin(), got.end()), "ping\n");

    peer.send("pong\n");
    EXPECT_EQ(receive_exactly(sock, 5), "pong\n");
}

TEST(TcpSocket, ConnectsByHostName) {
    // Where "localhost" also resolves to ::1 (as on most systems), ::1 comes
    // first and is refused, since the server listens on IPv4 only: connecting
    // then succeeds only because the next address is tried.
    LoopbackServer server;
    const auto sock = TcpSocket::connect("localhost", server.port(), 10s);
    ASSERT_TRUE(sock) << sock.error().message();
}

TEST(TcpSocket, RefusedConnectionIsAConnectError) {
    std::uint16_t port = 0;
    {
        const LoopbackServer server;
        port = server.port();
    }  // nothing listens on `port` any more
    const auto sock = TcpSocket::connect("127.0.0.1", port, 5s);
    ASSERT_FALSE(sock);
    EXPECT_EQ(sock.error().kind, NetError::Kind::Connect);
}

TEST(TcpSocket, ReceiveTimesOutAfterTheFullTimeoutAndStaysUsable) {
    LoopbackServer server;
    TcpSocket sock = connect_to(server);
    auto peer = server.accept();

    std::array<std::uint8_t, 16> buf{};
    const auto start = Clock::now();
    const auto n = sock.receive(buf, 100ms);
    const auto elapsed = Clock::now() - start;
    ASSERT_FALSE(n);
    EXPECT_EQ(n.error().kind, NetError::Kind::Timeout);
    EXPECT_GE(elapsed, 100ms);
    EXPECT_LT(elapsed, 2s);  // generous: shared CI machines can be slow

    peer.send("x");
    EXPECT_EQ(receive_exactly(sock, 1), "x");
}

TEST(TcpSocket, OrderlyCloseByPeerIsClosed) {
    LoopbackServer server;
    TcpSocket sock = connect_to(server);
    auto peer = server.accept();
    peer.close();

    std::array<std::uint8_t, 16> buf{};
    const auto n = sock.receive(buf, 1s);
    ASSERT_FALSE(n);
    EXPECT_EQ(n.error().kind, NetError::Kind::Closed);
}

TEST(TcpSocket, ResetByPeerIsClosed) {
    LoopbackServer server;
    TcpSocket sock = connect_to(server);
    auto peer = server.accept();
    peer.reset();

    std::array<std::uint8_t, 16> buf{};
    const auto n = sock.receive(buf, 1s);
    ASSERT_FALSE(n);
    EXPECT_EQ(n.error().kind, NetError::Kind::Closed);
}

TEST(TcpSocket, SendingToAResetPeerIsClosed) {
    LoopbackServer server;
    TcpSocket sock = connect_to(server);
    auto peer = server.accept();
    peer.reset();

    // The reset arrives asynchronously: the first sends may still succeed.
    std::expected<void, NetError> sent;
    for (int i = 0; i < 100 && sent; ++i) {
        sent = sock.send_all(bytes("x"), 1s);
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_FALSE(sent);
    EXPECT_EQ(sent.error().kind, NetError::Kind::Closed);
}

TEST(TcpSocket, LargeSendArrivesIntact) {
    LoopbackServer server;
    TcpSocket sock = connect_to(server);
    auto peer = server.accept();

    std::vector<std::uint8_t> data(std::size_t{8} << 20);
    for (std::size_t i = 0; i < data.size(); ++i) data[i] = static_cast<std::uint8_t>(i * 7);

    std::vector<std::uint8_t> received;
    std::thread reader([&] { received = peer.receive(data.size()); });
    const auto sent = sock.send_all(data, 10s);
    sock.close();  // unblocks the reader if the send failed
    reader.join();

    ASSERT_TRUE(sent) << sent.error().message();
    EXPECT_EQ(received.size(), data.size());
    EXPECT_TRUE(received == data);
}

TEST(TcpSocket, SendTimesOutWhenThePeerDoesNotRead) {
    LoopbackServer server;
    TcpSocket sock = connect_to(server);
    auto peer = server.accept();

    const std::vector<std::uint8_t> data(std::size_t{64} << 20);  // more than both socket buffers hold
    const auto sent = sock.send_all(data, 200ms);
    ASSERT_FALSE(sent);
    EXPECT_EQ(sent.error().kind, NetError::Kind::Timeout);
}

TEST(TcpSocket, ClosedSocketReportsClosed) {
    LoopbackServer server;
    TcpSocket sock = connect_to(server);
    sock.close();
    EXPECT_FALSE(sock.is_open());

    std::array<std::uint8_t, 16> buf{};
    const auto n = sock.receive(buf, 0ms);
    ASSERT_FALSE(n);
    EXPECT_EQ(n.error().kind, NetError::Kind::Closed);
    const auto sent = sock.send_all(bytes("x"), 0ms);
    ASSERT_FALSE(sent);
    EXPECT_EQ(sent.error().kind, NetError::Kind::Closed);
}

TEST(TcpSocket, EmptyReceiveBufferIsAnError) {
    LoopbackServer server;
    TcpSocket sock = connect_to(server);
    const auto n = sock.receive({}, 0ms);
    ASSERT_FALSE(n);
    EXPECT_EQ(n.error().kind, NetError::Kind::Io);
}

#if !defined(_WIN32)
TEST(TcpSocket, WorksOnDescriptorsAboveFdSetsize) {
    // select() cannot handle descriptors >= FD_SETSIZE (1024); waiting on one
    // must still work. Occupy the low descriptors so the socket gets a high one.
    rlimit limit{};
    ASSERT_EQ(::getrlimit(RLIMIT_NOFILE, &limit), 0);
    constexpr rlim_t kNeeded = FD_SETSIZE + 64;
    if (limit.rlim_cur < kNeeded) {
        if (limit.rlim_max < kNeeded) GTEST_SKIP() << "file descriptor limit too low";
        limit.rlim_cur = kNeeded;
        ASSERT_EQ(::setrlimit(RLIMIT_NOFILE, &limit), 0);
    }
    std::vector<int> filler;
    while (filler.empty() || filler.back() < FD_SETSIZE) {
        const int fd = ::open("/dev/null", O_RDONLY);
        ASSERT_GE(fd, 0);
        filler.push_back(fd);
    }

    LoopbackServer server;
    TcpSocket sock = connect_to(server);
    auto peer = server.accept();
    peer.send("x");
    EXPECT_EQ(receive_exactly(sock, 1), "x");

    for (const int fd : filler) ::close(fd);
}
#endif

TEST(NetError, MessageNamesTheKindAndTheOsError) {
    EXPECT_EQ(NetError{NetError::Kind::Timeout}.message(), "timeout");
    const auto with_code = NetError{NetError::Kind::Connect, 1}.message();
    EXPECT_TRUE(with_code.starts_with("connect: ")) << with_code;
    EXPECT_GT(with_code.size(), std::string_view("connect: ").size());
}

}  // namespace
