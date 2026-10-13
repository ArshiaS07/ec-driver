#pragma once
// Test helper: a TCP server on 127.0.0.1 with a port chosen by the OS, for
// testing clients against real sockets without the robot. Blocking and
// minimal: each test scripts the server side step by step. Failures throw;
// GoogleTest reports an exception as a failed test.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace ec::test {

namespace detail {
#if defined(_WIN32)
using native_socket = SOCKET;
using io_size = int;
inline constexpr native_socket kNoSocket = INVALID_SOCKET;
inline constexpr int kNoSigpipe = 0;
inline constexpr int kShutBoth = SD_BOTH;
inline void close_native(native_socket s) { ::closesocket(s); }
struct NetworkInit {
    NetworkInit() {
        WSADATA data;
        if (::WSAStartup(MAKEWORD(2, 2), &data) != 0) throw std::runtime_error("WSAStartup failed");
    }
    ~NetworkInit() { ::WSACleanup(); }
    NetworkInit(const NetworkInit&) = delete;
    NetworkInit& operator=(const NetworkInit&) = delete;
};
#else
using native_socket = int;
using io_size = std::size_t;
inline constexpr native_socket kNoSocket = -1;
inline constexpr int kNoSigpipe = MSG_NOSIGNAL;
inline constexpr int kShutBoth = SHUT_RDWR;
inline void close_native(native_socket s) { ::close(s); }
struct NetworkInit {};
#endif
}  // namespace detail

// The server side of one accepted connection.
class Peer {
public:
    explicit Peer(detail::native_socket s) noexcept : s_(s) {}
    Peer(Peer&& other) noexcept : s_(std::exchange(other.s_, detail::kNoSocket)) {}
    Peer& operator=(Peer&&) = delete;
    Peer(const Peer&) = delete;
    Peer& operator=(const Peer&) = delete;
    ~Peer() { close(); }

    void send(std::string_view bytes) {
        while (!bytes.empty()) {
            const auto n = ::send(s_, bytes.data(), static_cast<detail::io_size>(bytes.size()), detail::kNoSigpipe);
            if (n <= 0) throw std::runtime_error("peer: send failed");
            bytes.remove_prefix(static_cast<std::size_t>(n));
        }
    }

    // Receives `count` bytes, or fewer if the client closes first.
    std::vector<std::uint8_t> receive(std::size_t count) {
        std::vector<std::uint8_t> out(count);
        std::size_t got = 0;
        while (got < count) {
            const auto chunk = std::min<std::size_t>(count - got, std::size_t{1} << 20);
            const auto n =
                ::recv(s_, reinterpret_cast<char*>(out.data() + got), static_cast<detail::io_size>(chunk), 0);
            if (n <= 0) break;
            got += static_cast<std::size_t>(n);
        }
        out.resize(got);
        return out;
    }

    // Receives one line, without its '\n' (what arrived so far if the client closes first).
    std::string receive_line() {
        std::string line;
        char c = 0;
        while (::recv(s_, &c, 1, 0) == 1 && c != '\n') line.push_back(c);
        return line;
    }

    // Orderly close: the client reads end of stream.
    void close() noexcept {
        if (s_ != detail::kNoSocket) detail::close_native(std::exchange(s_, detail::kNoSocket));
    }

    // Ends both directions, waking a receive blocked on this connection in
    // another thread; the socket stays open until close().
    void shutdown() noexcept {
        if (s_ != detail::kNoSocket) ::shutdown(s_, detail::kShutBoth);
    }

    // Abortive close: the client sees the connection reset.
    void reset() noexcept {
        linger l{};
        l.l_onoff = 1;
        l.l_linger = 0;
        ::setsockopt(s_, SOL_SOCKET, SO_LINGER, reinterpret_cast<const char*>(&l), sizeof l);
        close();
    }

private:
    detail::native_socket s_;
};

class LoopbackServer {
public:
    LoopbackServer() {
        s_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (s_ == detail::kNoSocket) throw std::runtime_error("server: socket failed");
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = 0;  // any free port
        ::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        socklen_t len = sizeof addr;
        if (::bind(s_, reinterpret_cast<const sockaddr*>(&addr), len) != 0 || ::listen(s_, 8) != 0 ||
            ::getsockname(s_, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
            detail::close_native(s_);
            throw std::runtime_error("server: bind/listen failed");
        }
        port_ = ntohs(addr.sin_port);
    }
    ~LoopbackServer() { detail::close_native(s_); }
    LoopbackServer(const LoopbackServer&) = delete;
    LoopbackServer& operator=(const LoopbackServer&) = delete;

    [[nodiscard]] std::uint16_t port() const noexcept { return port_; }

    // Accepts the next connection. Call it after the client's connect() has
    // succeeded: the connection is then already queued, so this does not block.
    Peer accept() {
        const auto s = ::accept(s_, nullptr, nullptr);
        if (s == detail::kNoSocket) throw std::runtime_error("server: accept failed");
        return Peer(s);
    }

    // Accepts the next connection once one arrives, for clients that connect
    // on a thread of their own. Throws if none arrives within `timeout`.
    Peer accept(std::chrono::milliseconds timeout) {
        const int ms = static_cast<int>(timeout.count());
#if defined(_WIN32)
        fd_set ready;
        FD_ZERO(&ready);
        FD_SET(s_, &ready);
        timeval tv{};
        tv.tv_sec = ms / 1000;
        tv.tv_usec = (ms % 1000) * 1000;
        const int n = ::select(0, &ready, nullptr, nullptr, &tv);
#else
        pollfd p{};
        p.fd = s_;
        p.events = POLLIN;
        const int n = ::poll(&p, 1, ms);
#endif
        if (n <= 0) throw std::runtime_error("server: no connection within the timeout");
        return accept();
    }

private:
    detail::NetworkInit network_;
    detail::native_socket s_ = detail::kNoSocket;
    std::uint16_t port_ = 0;
};

}  // namespace ec::test
