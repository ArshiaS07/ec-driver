#pragma once
// What a Robot and its motions share: the command connection, on
// which calls go one at a time; the open motion, of which there is at most one;
// and the monitor. Private to the library.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>
#include <utility>

#include "ec/error.hpp"
#include "ec/monitor.hpp"
#include "ec/robot.hpp"
#include "ec/sdk/client.hpp"
#include "ec/sdk/request.hpp"
#include "ec/stamped.hpp"

namespace ec::detail {

struct Connection {
    explicit Connection(sdk::Client c) : client(std::move(c)) {}

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

    /// Registers a motion being opened, with its stop (sent by Robot::stop()),
    /// and clears an earlier stop request; Busy while another motion is open.
    [[nodiscard]] std::expected<std::uint64_t, Error> open(std::function<void()> stop);
    /// Unregisters it; nothing if `id` is not the open motion.
    void close(std::uint64_t id) noexcept;
    [[nodiscard]] bool motion_open() const;

    /// Waits up to `timeout` for the arm to have moved and stopped, asking
    /// getRobotState every 10 ms, as the manual's wait_stop. `started` is when
    /// the motion was started, `ran` whether it was seen running (kept by the
    /// caller across calls): until it is, a stop counts only a second after the
    /// start, since a motion just started may not show yet. nullopt: still
    /// running when `timeout` ran out; Stopped once Robot::stop() was called;
    /// Fault on an emergency stop, an alarm or a collision.
    [[nodiscard]] std::optional<std::expected<void, Error>> wait_stopped(Clock::time_point started, bool& ran,
                                                                         std::chrono::nanoseconds timeout);

    std::mutex calls;
    sdk::Client client;

    mutable std::mutex motions;
    std::uint64_t open_motion = 0;  // 0: none
    std::uint64_t next_motion = 1;
    std::function<void()> stop_open;
    std::atomic<bool> stop_requested{false};

    std::unique_ptr<Monitor> monitor = std::make_unique<Monitor>();  // last: stopped first; never null
};

}  // namespace ec::detail
