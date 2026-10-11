#include "ec/feedback/reader.hpp"

#include <span>

#include "detail/deadline.hpp"

namespace ec::feedback {

std::expected<Reader, net::NetError> Reader::connect(const std::string& host, std::uint16_t port, Timeout timeout) {
    auto socket = net::TcpSocket::connect(host, port, timeout);
    if (!socket) return std::unexpected(socket.error());
    return Reader(std::move(*socket));
}

std::expected<Sample, net::NetError> Reader::next(Timeout timeout) {
    const auto deadline = detail::deadline_after(timeout);
    for (;;) {
        if (const auto raw = framer_.pop()) return Sample{arrival_, *raw};
        const auto n = socket_.receive(chunk_, detail::time_left(deadline));
        if (!n) return std::unexpected(n.error());
        arrival_ = std::chrono::steady_clock::now();
        framer_.push(std::span(chunk_.data(), *n));
    }
}

}  // namespace ec::feedback
