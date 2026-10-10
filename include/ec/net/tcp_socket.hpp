#pragma once
/// @file
/// TCP client socket for Linux and Windows. Generic: it knows nothing about the
/// robot or its protocols, and serves both controller ports.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <string_view>

namespace ec::net {

/// Why a socket operation failed.
struct NetError {
    enum class Kind {
        Resolve,  ///< the host name could not be resolved
        Connect,  ///< refused, unreachable, or not established within the timeout
        Timeout,  ///< receive: no data; send: no buffer space; within the timeout
        Closed,   ///< the peer closed or reset the connection, or this socket is closed
        Io,       ///< any other socket error
    };
    Kind kind;
    int code = 0;  ///< OS error code (errno, WSAGetLastError() or a getaddrinfo code); 0 if none

    /// Readable description, e.g. "connect: Connection refused".
    [[nodiscard]] std::string message() const;
};

/// Short name of an error kind, e.g. "timeout".
[[nodiscard]] std::string_view to_string(NetError::Kind k) noexcept;

/// A connected TCP client socket.
///
/// Every call that can wait takes a timeout, blocks the calling thread for at
/// most that long, and reports Kind::Timeout only once the timeout has fully
/// elapsed. The socket starts no threads; a caller that needs concurrency runs
/// it on a thread of its own.
///
/// Move-only; closed on destruction. Not thread-safe: use each object from one
/// thread at a time.
class TcpSocket {
public:
    using Timeout = std::chrono::milliseconds;

    /// Resolves `host` (a name or an IPv4/IPv6 address) and connects to the first
    /// resolved address that accepts, within `timeout` in total. Name resolution
    /// itself is not bounded by the timeout; pass an address to avoid DNS.
    /// TCP_NODELAY is set, so small messages are sent without delay.
    [[nodiscard]] static std::expected<TcpSocket, NetError> connect(const std::string& host, std::uint16_t port,
                                                                    Timeout timeout);

    TcpSocket(TcpSocket&& other) noexcept;
    TcpSocket& operator=(TcpSocket&& other) noexcept;
    TcpSocket(const TcpSocket&) = delete;
    TcpSocket& operator=(const TcpSocket&) = delete;
    ~TcpSocket();

    /// Waits up to `timeout` for data, then returns between 1 and buffer.size()
    /// bytes. Never returns 0: a closed peer is Kind::Closed. `buffer` must not be
    /// empty (Kind::Io otherwise).
    [[nodiscard]] std::expected<std::size_t, NetError> receive(std::span<std::uint8_t> buffer, Timeout timeout);

    /// Sends all of `data` within `timeout`, or fails. After a failure, an unknown
    /// prefix of `data` may have been sent.
    [[nodiscard]] std::expected<void, NetError> send_all(std::span<const std::uint8_t> data, Timeout timeout);

    /// True while this object owns a socket. It stays true after the peer closes;
    /// call close() once an error is not going to be recovered from.
    [[nodiscard]] bool is_open() const noexcept;

    /// Closes the socket. Does nothing if it is already closed.
    void close() noexcept;

private:
    using Handle = std::uintptr_t;  // int fd on POSIX, SOCKET on Windows; keeps system headers out of here
    explicit TcpSocket(Handle h) noexcept : handle_(h) {}

    Handle handle_;
};

}  // namespace ec::net
