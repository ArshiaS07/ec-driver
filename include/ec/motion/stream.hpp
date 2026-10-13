#pragma once
/// @file
/// Streamed motion (docs/motion.md 5.5): the application pushes references into
/// a channel, from any thread at any rate; the stream activity takes the newest
/// once per cycle, guards its age (the watchdog: hold, then Starved), shapes it,
/// and feeds the controller at the controller's pace.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <deque>
#include <expected>
#include <memory>
#include <optional>
#include <string_view>

#include "ec/latest.hpp"
#include "ec/motion/activity.hpp"
#include "ec/motion/shaper.hpp"
#include "ec/units.hpp"

namespace ec::motion {

/// A reference and when the application pushed it.
template <typename T>
struct Stamped {
    T value{};
    Clock::time_point at{};
};

/// What a stream's handle and its activity share: the newest reference (a
/// newest-wins mailbox) and a request to close. Thread-safe.
template <typename T>
class Channel {
public:
    /// Makes `value` the newest reference, stamped now.
    void push(const T& value) { push(value, Clock::now()); }
    /// Makes `value` the newest reference, stamped `at` (for tests and replays).
    void push(const T& value, Clock::time_point at) { latest_.publish(Stamped<T>{value, at}); }

    /// The newest reference, if any was pushed.
    [[nodiscard]] std::optional<Stamped<T>> newest() const {
        if (auto e = latest_.get()) return e->value;
        return std::nullopt;
    }

    /// Asks the stream to end gracefully: decelerate, settle, close the family.
    void request_close() noexcept { close_.store(true, std::memory_order_release); }
    [[nodiscard]] bool close_requested() const noexcept { return close_.load(std::memory_order_acquire); }

    /// Set by the stream activity once the controller accepted the stream and
    /// targets are taken; the application's handle waits for it.
    void mark_open() noexcept { open_.store(true, std::memory_order_release); }
    [[nodiscard]] bool is_open() const noexcept { return open_.load(std::memory_order_acquire); }

private:
    Latest<Stamped<T>> latest_;
    std::atomic<bool> close_{false};
    std::atomic<bool> open_{false};
};

/// The watchdog of every stream: how old the newest reference may get.
struct Watchdog {
    std::chrono::milliseconds target_timeout{100};  ///< older than this: hold (a smooth stop)
    std::chrono::milliseconds hold_for{500};        ///< holding longer than this: fail with Starved
};

/// Options of a position stream (transparent transmission).
struct ServoOptions {
    std::chrono::milliseconds lookahead{40};  ///< the controller's queue delay, [10, 1000] ms, > the period
    double smoothness = 0.1;                  ///< the controller's smoothing gain, [0, 1]
    Watchdog watchdog{};
    /// The shaper's limits [rad/s]; default 10 % of the EC66's joint speeds (docs/motion.md 7.2).
    Joints max_velocity = scale(kEc66MaxJointSpeed, 0.1);
    std::chrono::milliseconds accelerate_in{250};  ///< the acceleration limit reaches max_velocity in this time
    double tracking_tolerance = deg_to_rad(5.0);   ///< [rad] beyond the setpoints of the last lookahead
    double joint_limit = kEc66JointLimit;          ///< [rad]; a target beyond it ends the stream (Invalid)
    bool replies = true;                           ///< response_enable: each point confirmed
};

/// Streams joint positions through transparent transmission (SDK manual
/// 2.2.7). Opens at standstill and starts from the measured position, so
/// nothing moves before the first target; until then the watchdog does not
/// run. Sends one point per cycle, the cycle being the controller's sampling
/// period. Closes gracefully (decelerate, let the queue drain, clear it) or,
/// after a stop or a fault, at once (stop, clear).
class ServoStream {
public:
    ServoStream(const ServoOptions& options, std::shared_ptr<Channel<Joints>> channel);

    [[nodiscard]] Needs needs() const noexcept { return {kCanStartMotion, kCanMove}; }
    [[nodiscard]] std::string_view name() const noexcept { return "servo_stream"; }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out);
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why);

private:
    enum class Stage { Open, AwaitOpen, Streaming };
    enum class Closing { Begin, Settle, Clear, AwaitClear };

    [[nodiscard]] std::expected<void, Error> check_replies(const Cycle& c) const;
    [[nodiscard]] std::expected<void, Error> check_tracking(const Cycle& c) const;
    void put(const Cycle& c, Outbox& out, const Joints& setpoint);

    ServoOptions options_;
    std::shared_ptr<Channel<Joints>> channel_;
    std::optional<RateLimiter> shaper_;
    Stage stage_ = Stage::Open;
    Closing closing_ = Closing::Begin;
    Clock::time_point sent_at_{};
    bool pushed_ = false;        // a target has been pushed: the watchdog runs
    std::deque<Joints> recent_;  // the setpoints of the last lookahead and then some, for the tracking check
    std::size_t recent_capacity_ = 1;
    Clock::time_point settled_since_{};
};

/// Options of a joint velocity stream (moveBySpeedj).
struct VelocityOptions {
    double acceleration = 1.0;                       ///< [rad/s^2], the controller ramps speeds with it
    std::chrono::milliseconds command_duration{40};  ///< each command's t: the controller stops by itself after it
    Watchdog watchdog{};
    Joints max_velocity = scale(kEc66MaxJointSpeed, 0.1);  ///< [rad/s]; faster targets are scaled down
};

/// Streams joint velocities through moveBySpeedj, re-sent every cycle with a
/// short duration, so that the controller itself stops the arm if the PC goes
/// silent. A stale target becomes zero velocity; then Starved. Closes with stopj.
class VelocityStream {
public:
    VelocityStream(const VelocityOptions& options, std::shared_ptr<Channel<Joints>> channel)
        : options_(options), channel_(std::move(channel)) {}

    [[nodiscard]] Needs needs() const noexcept { return {kCanStartMotion, kCanMove}; }
    [[nodiscard]] std::string_view name() const noexcept { return "velocity_stream"; }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out);
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why);

private:
    VelocityOptions options_;
    std::shared_ptr<Channel<Joints>> channel_;
    bool pushed_ = false;
    std::optional<Clock::time_point> stop_sent_at_;
};

/// A jog direction: an axis of the selected coordinate system, minus or plus.
struct JogDirection {
    int axis = 0;          ///< 0..5: joints J1..J6, or x, y, z, rx, ry, rz in a Cartesian system
    bool positive = true;  ///< false: the minus direction
};

/// Options of a jog stream.
struct JogOptions {
    double speed = 0.1;                     ///< fraction of the manual speed, (0, 1]
    std::chrono::milliseconds repeat{200};  ///< jog is re-sent this often; the controller stops 1 s after the last
    Watchdog watchdog{};                    ///< hold_for is not used: a stale direction stops the jog
};

/// Jogs while the application keeps pushing a direction; an empty direction
/// (nullopt) or a stale one stops the arm. In the coordinate system selected
/// on the controller (SetCurrentCoord).
class JogStream {
public:
    JogStream(const JogOptions& options, std::shared_ptr<Channel<std::optional<JogDirection>>> channel)
        : options_(options), channel_(std::move(channel)) {}

    [[nodiscard]] Needs needs() const noexcept { return {kCanStartMotion, kCanMove}; }
    [[nodiscard]] std::string_view name() const noexcept { return "jog_stream"; }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out);
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why);

private:
    JogOptions options_;
    std::shared_ptr<Channel<std::optional<JogDirection>>> channel_;
    std::optional<int> jogging_;  // the index being jogged
    Clock::time_point last_sent_{};
    std::optional<Clock::time_point> stop_sent_at_;
};

}  // namespace ec::motion
