#pragma once
// Blocking-with-timeout TCP client socket, Linux and Windows.
//
// Both controller ports (8055 commands, 8056 feedback) are plain TCP, so this
// one class serves both. Every call that can wait takes a timeout: nothing in
// the driver may block forever on the network. Platform differences (Winsock
// versus BSD sockets) stay inside tcp_socket.cpp; this header includes no
// system headers.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <string_view>

namespace ec::net {

struct NetError {
    enum class Kind {
        Resolve,  // host name could not be resolved
        Connect,  // connection refused, unreachable, or timed out
        Timeout,  // no data within the timeout (receive / send)
        Closed,   // peer closed the connection
        Io,       // any other socket error
    };
    Kind kind;
    int code = 0;  // OS error code (errno / WSAGetLastError / getaddrinfo), 0 if none

    [[nodiscard]] std::string message() const;
};
[[nodiscard]] std::string_view to_string(NetError::Kind k) noexcept;

class TcpSocket {
public:
    using Timeout = std::chrono::milliseconds;

    // Resolves `host`, connects within `timeout`, and enables TCP_NODELAY
    // (small command messages must not wait in Nagle's buffer).
    [[nodiscard]] static std::expected<TcpSocket, NetError> connect(const std::string& host, std::uint16_t port,
                                                                    Timeout timeout);

    TcpSocket(TcpSocket&& other) noexcept;
    TcpSocket& operator=(TcpSocket&& other) noexcept;
    TcpSocket(const TcpSocket&) = delete;
    TcpSocket& operator=(const TcpSocket&) = delete;
    ~TcpSocket();

    // Waits up to `timeout` for data, then returns 1..buffer.size() bytes.
    // Never returns 0: a closed peer is NetError::Closed, no data is
    // NetError::Timeout.
    [[nodiscard]] std::expected<std::size_t, NetError> receive(std::span<std::uint8_t> buffer, Timeout timeout);

    // Sends all bytes or fails; `timeout` bounds the whole call.
    [[nodiscard]] std::expected<void, NetError> send_all(std::span<const std::uint8_t> data, Timeout timeout);

    [[nodiscard]] bool is_open() const noexcept;
    void close() noexcept;

private:
    using Handle = std::uintptr_t;  // int fd on POSIX, SOCKET on Windows
    explicit TcpSocket(Handle h) noexcept : handle_(h) {}

    Handle handle_;
};

}  // namespace ec::net
