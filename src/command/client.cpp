#include "ec/command/client.hpp"

#include <array>
#include <span>
#include <utility>

#include "detail/deadline.hpp"
#include "ec/command/codec.hpp"

namespace ec::command {
namespace {

Error disconnected(const net::NetError& e) { return Error{Error::Kind::Disconnected, e.code, e.message()}; }

std::unexpected<Error> timed_out() {
    return std::unexpected(Error{Error::Kind::Timeout, 0, "no reply within the timeout"});
}

std::span<const std::uint8_t> as_bytes(const std::string& s) {
    return {reinterpret_cast<const std::uint8_t*>(s.data()), s.size()};
}

}  // namespace

Client::Client(net::TcpSocket socket, Timeout reply_timeout) noexcept
    : socket_(std::move(socket)), reply_timeout_(reply_timeout) {}

std::expected<Client, Error> Client::connect(const std::string& host) { return connect(host, Options{}); }

std::expected<Client, Error> Client::connect(const std::string& host, const Options& options) {
    auto socket = net::TcpSocket::connect(host, options.port, options.connect_timeout);
    if (!socket) return std::unexpected(disconnected(socket.error()));
    return Client(std::move(*socket), options.reply_timeout);
}

Client::Stats Client::stats() const noexcept {
    Stats s = stats_;
    s.oversized_lines = framer_.oversized_lines();
    return s;
}

void Client::disconnect() noexcept {
    socket_.close();
    framer_.reset();
}

std::expected<Json, Error> Client::call(const Request& request) { return call(request, reply_timeout_); }

std::expected<void, Error> Client::send_only(const Request& request) {
    if (!socket_.is_open()) return std::unexpected(Error{Error::Kind::Disconnected, 0, "not connected"});
    ++stats_.calls;
    const std::string line = encode(request, next_id_++);
    if (const auto sent = socket_.send_all(as_bytes(line), reply_timeout_); !sent) {
        disconnect();
        return std::unexpected(disconnected(sent.error()));
    }
    return {};
}

std::expected<Json, Error> Client::call(const Request& request, Timeout timeout) {
    if (!socket_.is_open()) return std::unexpected(Error{Error::Kind::Disconnected, 0, "not connected"});

    const std::uint64_t id = next_id_++;
    ++stats_.calls;
    const auto deadline = detail::deadline_after(timeout);

    const std::string line = encode(request, id);
    if (const auto sent = socket_.send_all(as_bytes(line), timeout); !sent) {
        // Part of the line may be on the wire already, and the next request
        // would be appended to it: the connection cannot be used any more.
        disconnect();
        if (sent.error().kind == net::NetError::Kind::Timeout)
            return std::unexpected(
                Error{Error::Kind::Disconnected, 0, "request not sent within the timeout; connection closed"});
        return std::unexpected(disconnected(sent.error()));
    }

    std::array<std::uint8_t, 4096> chunk{};
    for (;;) {
        while (const auto received = framer_.pop()) {
            auto response = parse_response(*received);
            if (!response) {
                ++stats_.bad_lines;
                continue;
            }
            if (response->id && *response->id != id) {
                ++stats_.stale_replies;
                continue;
            }
            return std::move(response->result);  // ours, or an error with "id": null
        }

        const auto left = detail::time_left(deadline);
        if (left == Timeout::zero()) {
            ++stats_.timeouts;
            return timed_out();
        }
        const auto n = socket_.receive(chunk, left);
        if (!n) {
            if (n.error().kind == net::NetError::Kind::Timeout) {
                ++stats_.timeouts;
                return timed_out();
            }
            disconnect();
            return std::unexpected(disconnected(n.error()));
        }
        framer_.push(std::span(chunk.data(), *n));
    }
}

}  // namespace ec::command
