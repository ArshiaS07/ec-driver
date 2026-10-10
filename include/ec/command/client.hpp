#pragma once
/// @file
/// Client for the controller's command port (8055).

#include <chrono>
#include <cstdint>
#include <expected>
#include <string>

#include "ec/command/error.hpp"
#include "ec/command/json.hpp"
#include "ec/command/line_framer.hpp"
#include "ec/command/request.hpp"
#include "ec/net/tcp_socket.hpp"

namespace ec::command {

/// A connection to the command port that makes one call at a time:
///
///     auto client = ec::command::Client::connect("192.168.0.200");
///     if (!client) { /* client.error().message */ }
///     auto mode = client->call(ec::command::GetRobotMode{});     // expected<RobotMode, Error>
///     auto q = client->call(ec::command::GetJointPos{}, 200ms);  // with its own timeout
///
/// - A call that times out leaves the connection usable. If its response comes
///   later, a later call recognises it by its id and discards it (Stats::stale_replies).
/// - Lines that are not JSON-RPC responses are discarded (Stats::bad_lines).
/// - An error response with "id": null is the error of the call in flight.
/// - A failed send or a lost connection closes the connection: every later call
///   fails with Kind::Disconnected. To reconnect, create a new Client.
///
/// Not thread-safe: make all calls from one thread.
class Client {
public:
    using Timeout = std::chrono::milliseconds;

    struct Options {
        std::uint16_t port = 8055;
        Timeout connect_timeout{2000};
        Timeout reply_timeout{1000};  ///< for calls made without a timeout argument
    };

    struct Stats {
        std::uint64_t calls = 0;
        std::uint64_t timeouts = 0;
        std::uint64_t stale_replies = 0;    ///< responses to earlier calls that had timed out
        std::uint64_t bad_lines = 0;        ///< lines that were not JSON-RPC responses
        std::uint64_t oversized_lines = 0;  ///< lines longer than LineFramer::kMaxLine
    };

    /// Connects to the command port of the controller at `host` (a name or an
    /// address; there is no default). Fails with Kind::Disconnected.
    [[nodiscard]] static std::expected<Client, Error> connect(const std::string& host);
    [[nodiscard]] static std::expected<Client, Error> connect(const std::string& host, const Options& options);

    /// Sends `command` and waits up to Options::reply_timeout for its reply.
    template <Command C>
    [[nodiscard]] std::expected<typename C::Reply, Error> call(const C& command) {
        return call(command, reply_timeout_);
    }

    /// Sends `command` and waits up to `timeout` for its reply.
    template <Command C>
    [[nodiscard]] std::expected<typename C::Reply, Error> call(const C& command, Timeout timeout) {
        const auto result = call(to_request(command), timeout);
        if (!result) return std::unexpected(result.error());
        return decode_reply<C>(*result);
    }

    /// Untyped call, returning the unwrapped result as JSON: for tools, and for
    /// calls without a type yet.
    [[nodiscard]] std::expected<Json, Error> call(const Request& request);
    [[nodiscard]] std::expected<Json, Error> call(const Request& request, Timeout timeout);

    /// False once the connection has been closed after an error.
    [[nodiscard]] bool is_connected() const noexcept { return socket_.is_open(); }

    [[nodiscard]] Stats stats() const noexcept;

private:
    Client(net::TcpSocket socket, Timeout reply_timeout) noexcept;
    void disconnect() noexcept;

    net::TcpSocket socket_;
    LineFramer framer_;
    Timeout reply_timeout_;
    std::uint64_t next_id_ = 1;
    Stats stats_;
};

}  // namespace ec::command
