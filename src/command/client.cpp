#include "ec/command/client.hpp"

#include <array>
#include <span>
#include <utility>

#include "ec/command/codec.hpp"

namespace ec::command {
namespace {

using Clock = std::chrono::steady_clock;

Error from_net(const net::NetError& e) {
    if (e.kind == net::NetError::Kind::Timeout) return Error{Error::Kind::Timeout, 0, "no reply within the timeout"};
    return Error{Error::Kind::Disconnected, e.code, e.message()};
}

}  // namespace

std::expected<Client, Error> Client::connect(const Options& options) {
    auto socket = net::TcpSocket::connect(options.host, options.port, options.connect_timeout);
    if (!socket) return std::unexpected(from_net(socket.error()));
    return Client(std::move(*socket), options.reply_timeout);
}

std::expected<Json, Error> Client::call(const Request& request) {
    if (!socket_.is_open()) return std::unexpected(Error{Error::Kind::Disconnected, 0, "not connected"});

    const std::uint64_t id = next_id_++;
    ++stats_.calls;
    const auto deadline = Clock::now() + reply_timeout_;

    const std::string line = encode(request, id);
    const auto bytes = std::span(reinterpret_cast<const std::uint8_t*>(line.data()), line.size());
    if (auto sent = socket_.send_all(bytes, reply_timeout_); !sent) {
        const Error e = from_net(sent.error());
        if (e.kind == Error::Kind::Disconnected) socket_.close();
        return std::unexpected(e);
    }

    std::array<std::uint8_t, 4096> chunk{};
    for (;;) {
        // Lines already received: the reply may have come with an earlier chunk.
        while (auto reply_line = framer_.pop()) {
            const auto reply = parse_reply(*reply_line);
            if (!reply) return std::unexpected(reply.error());  // not a JSON-RPC reply at all
            if (reply->id != id) {
                ++stats_.stale_replies;  // answer to an earlier call that timed out
                continue;
            }
            return reply->result;
        }

        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
        if (left.count() <= 0) {
            ++stats_.timeouts;
            return std::unexpected(Error{Error::Kind::Timeout, 0, "no reply within the timeout"});
        }
        const auto n = socket_.receive(chunk, left);
        if (!n) {
            const Error e = from_net(n.error());
            if (e.kind == Error::Kind::Timeout) {
                ++stats_.timeouts;
            } else {
                socket_.close();
                framer_.reset();
            }
            return std::unexpected(e);
        }
        framer_.push(std::span(chunk.data(), *n));
    }
}

}  // namespace ec::command
