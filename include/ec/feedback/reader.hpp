#pragma once
/// @file
/// Reads frames from the controller's monitoring port (8056).

#include <array>
#include <chrono>
#include <cstdint>
#include <expected>
#include <string>
#include <utility>

#include "ec/feedback/framer.hpp"
#include "ec/feedback/sample.hpp"
#include "ec/net/tcp_socket.hpp"

namespace ec::feedback {

/// A connection to the monitoring port that hands out frames as samples:
/// socket, framer, arrival time. It does not decode them. Synchronous: next()
/// blocks the calling thread for at most its timeout.
///
/// Move-only. Not thread-safe: use it from one thread. To reconnect, create a
/// new Reader.
class Reader {
public:
    using Timeout = net::TcpSocket::Timeout;

    /// Connects to the monitoring port of the controller at `host`.
    [[nodiscard]] static std::expected<Reader, net::NetError> connect(const std::string& host, std::uint16_t port,
                                                                      Timeout timeout);

    /// The next frame. Waits up to `timeout` for the bytes still missing, then
    /// fails with the socket's error: Kind::Timeout if no frame was completed in
    /// time (the connection stays usable), Kind::Closed if the controller closed it.
    [[nodiscard]] std::expected<Sample, net::NetError> next(Timeout timeout);

    /// Framing statistics of this connection.
    [[nodiscard]] const Framer::Stats& stats() const noexcept { return framer_.stats(); }

private:
    explicit Reader(net::TcpSocket socket) noexcept : socket_(std::move(socket)) {}

    net::TcpSocket socket_;
    Framer framer_;
    std::chrono::steady_clock::time_point arrival_{};  // when the buffered bytes last grew
    std::array<std::uint8_t, 4096> chunk_{};
};

}  // namespace ec::feedback
