#pragma once
// Port-8055 client: sends one command, waits for the reply with the same id.
//
//   auto client = ec::command::Client::connect({.host = "192.168.0.200"});
//   if (!client) { /* client.error().message */ }
//   auto mode = client->call(ec::command::GetRobotMode{});   // expected<RobotMode, Error>
//
// One request in flight at a time: the simplest correct client. A call that
// times out leaves the connection usable; if its reply arrives later, the next
// call recognises it by its old id, discards it, and counts it. A transport
// failure closes the connection; reconnect by creating a new Client.
//
// Not thread-safe: one owner (later, the supervisor thread) makes all calls.

#include <chrono>
#include <cstdint>
#include <expected>
#include <string>

#include "ec/command/line_framer.hpp"
#include "ec/command/request.hpp"
#include "ec/net/tcp_socket.hpp"

namespace ec::command {

class Client {
public:
    struct Options {
        std::string host = "192.168.0.200";
        std::uint16_t port = 8055;
        std::chrono::milliseconds connect_timeout{2000};
        std::chrono::milliseconds reply_timeout{1000};  // default for every call
    };

    struct Stats {
        std::uint64_t calls = 0;
        std::uint64_t timeouts = 0;
        std::uint64_t stale_replies = 0;  // replies to earlier, timed-out calls
    };

    [[nodiscard]] static std::expected<Client, Error> connect(const Options& options);

    // Typed call: encodes the command, waits for its reply, decodes it.
    template <Command C>
    [[nodiscard]] std::expected<typename C::Reply, Error> call(const C& command) {
        const auto result = call(to_request(command));
        if (!result) return std::unexpected(result.error());
        return C::decode(*result);
    }

    // Untyped call: the unwrapped result as JSON. For tools and for commands
    // that do not have a type yet.
    [[nodiscard]] std::expected<Json, Error> call(const Request& request);

    [[nodiscard]] bool is_connected() const noexcept { return socket_.is_open(); }
    [[nodiscard]] const Stats& stats() const noexcept { return stats_; }
    void set_reply_timeout(std::chrono::milliseconds t) noexcept { reply_timeout_ = t; }

private:
    Client(net::TcpSocket socket, std::chrono::milliseconds reply_timeout)
        : socket_(std::move(socket)), reply_timeout_(reply_timeout) {}

    net::TcpSocket socket_;
    LineFramer framer_;
    std::chrono::milliseconds reply_timeout_;
    std::uint64_t next_id_ = 1;
    Stats stats_;
};

}  // namespace ec::command
