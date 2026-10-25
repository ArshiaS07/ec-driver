#pragma once
// What a Robot, its parts and its motions share (the session): the command
// connection, on which calls go one at a time; the open motion, of which there
// is at most one; the monitor; where the log port is. Private to the library.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <expected>
#include <format>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "ec/error.hpp"
#include "ec/log/reader.hpp"
#include "ec/monitor.hpp"
#include "ec/robot.hpp"
#include "ec/sdk/client.hpp"
#include "ec/sdk/request.hpp"
#include "ec/stamped.hpp"

namespace ec::detail {

struct Connection {
    Connection(sdk::Client c, std::string h) : client(std::move(c)), host(std::move(h)) {}

    /// Sends one call and waits for its answer; other threads wait their turn.
    [[nodiscard]] std::expected<sdk::Json, Error> call(const sdk::Request& request);

    /// The same, typed: the decoded reply, or the failure as "method: why".
    template <sdk::Command C>
    [[nodiscard]] std::expected<typename C::Reply, Error> send(const C& c) {
        const auto json = call(sdk::to_request(c));
        if (!json) return std::unexpected(json.error());
        auto reply = sdk::decode_reply<C>(*json);
        if (!reply) return std::unexpected(from_call(C::method, reply.error()));
        return std::move(*reply);
    }

    /// For the robot's users (Robot::call, the parts): the same, but while a
    /// motion is open only queries and computations; anything else is Busy.
    [[nodiscard]] std::expected<sdk::Json, Error> call_checked(const sdk::Request& request, sdk::Kind kind);

    template <sdk::Command C>
    [[nodiscard]] std::expected<typename C::Reply, Error> send_checked(const C& c) {
        const auto json = call_checked(sdk::to_request(c), C::kind);
        if (!json) return std::unexpected(json.error());
        auto reply = sdk::decode_reply<C>(*json);
        if (!reply) return std::unexpected(from_call(C::method, reply.error()));
        return std::move(*reply);
    }

    /// Registers a motion being opened, with its stop (sent by stop()), and
    /// clears an earlier stop request; Busy while another motion is open.
    [[nodiscard]] std::expected<std::uint64_t, Error> open(std::function<void()> stop);
    /// Unregisters it; nothing if `id` is not the open motion.
    void close(std::uint64_t id) noexcept;
    [[nodiscard]] bool motion_open() const;

    /// The motion controller's stop, from any thread: marks the stop (the open
    /// motion's waits end with Stopped), sends the open motion's own stop, then
    /// `stop` [S 2.2.3.9]. Each waits for a call in flight to be answered.
    void stop() noexcept;

    /// Waits up to `timeout` for the arm to have moved and stopped. It reads the
    /// monitoring path while it streams (a state at most 100 ms old), and
    /// otherwise asks getRobotState every 10 ms, as the manual's wait_stop.
    /// `started` is when the motion was started, `ran` whether it was seen
    /// running (kept by the caller across calls): until it is, a stop counts
    /// only a second after the start, since a motion just started may not show
    /// yet. nullopt: still running when `timeout` ran out; Stopped once stop()
    /// was called, if `stop_ends_wait`; Fault on an emergency stop, an alarm or
    /// a collision.
    [[nodiscard]] std::optional<std::expected<void, Error>> wait_stopped(Clock::time_point started, bool& ran,
                                                                         std::chrono::nanoseconds timeout,
                                                                         bool stop_ends_wait = true);

    std::mutex calls;
    sdk::Client client;
    const std::string host;
    log::Options log{};  // the log port (8058) and its timeouts

    mutable std::mutex motions;
    std::uint64_t open_motion = 0;  // 0: none
    std::uint64_t next_motion = 1;
    std::function<void()> stop_open;
    std::atomic<bool> stop_requested{false};

    std::unique_ptr<Monitor> monitor = std::make_unique<Monitor>();  // last: stopped first; never null
};

/// The session behind a part's or a motion's handle; Disconnected once its robot is gone.
[[nodiscard]] inline std::expected<std::shared_ptr<Connection>, Error> reach(const std::weak_ptr<Connection>& session,
                                                                             std::string_view what) {
    if (auto connection = session.lock()) return connection;
    return std::unexpected(Error{Error::Kind::Disconnected, std::format("{}: the robot is gone", what)});
}

/// What a part holds: its robot's session, without keeping it. The typed call
/// through it (Connection::send_checked); Disconnected once the robot is gone.
template <sdk::Command C>
[[nodiscard]] std::expected<typename C::Reply, Error> send(const std::weak_ptr<Connection>& session, const C& c) {
    const auto connection = session.lock();
    if (!connection)
        return std::unexpected(Error{Error::Kind::Disconnected, std::format("{}: the robot is gone", C::method)});
    return connection->send_checked(c);
}

}  // namespace ec::detail
