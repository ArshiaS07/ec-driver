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
#include "ec/pose.hpp"
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

/// The transparent-transmission protocol that position streams share (SDK
/// manual 2.2.7, docs/motion.md 2.4): opens the controller's queue, puts one
/// joint setpoint per cycle, checks the replies and how the arm follows, and
/// closes, gracefully (decelerating from the last setpoints, letting the queue
/// drain, then clearing it) or at once (stop, clear). ServoStream,
/// PoseServoStream and FollowTrajectory are built on it; what to put each
/// cycle is theirs to decide. Pure, like every activity.
class ServoLink {
public:
    /// `name` prefixes its error messages (the owning activity's name).
    ServoLink(std::string_view name, const ServoOptions& options) : name_(name), options_(options) {}

    /// Opens the stream; call every cycle until it returns true. The first call
    /// checks the options against the cycle's period (Kind::Invalid), starts the
    /// shaper at rest at the measured position, and sends the init. True in the
    /// cycle the controller accepts it: put the first setpoint in that cycle.
    [[nodiscard]] std::expected<bool, Error> open(const Cycle& c, Outbox& out);
    [[nodiscard]] bool is_open() const noexcept { return stage_ == Stage::Streaming; }

    /// The shaper, holding the last setpoint put and its speed. Meaningful only
    /// after the first open().
    [[nodiscard]] RateLimiter& shaper() noexcept { return shaper_; }

    /// Whether the controller accepted the previous cycle's points (only checked
    /// when they are confirmed, ServoOptions::replies).
    [[nodiscard]] std::expected<void, Error> check_replies(const Cycle& c) const;
    /// Kind::Invalid for a target that is not finite or beyond the joint limit.
    [[nodiscard]] std::expected<void, Error> check_target(const Joints& target) const;
    /// Kind::Tracking when a joint is further than the tracking tolerance from
    /// the range of the setpoints of the last lookahead (and 200 ms more).
    [[nodiscard]] std::expected<void, Error> check_tracking(const Cycle& c) const;
    /// Puts one setpoint into the controller's queue.
    void put(Outbox& out, const Joints& setpoint);

    /// Closes the stream (an activity's close()): after Done or Cancelled
    /// gracefully, after Stopped or Faulted at once; true once the arm rests and
    /// the queue is cleared.
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why);

private:
    enum class Stage { Open, AwaitOpen, Streaming };
    enum class Closing { Begin, Settle, Clear, AwaitClear };

    std::string_view name_;
    ServoOptions options_;
    RateLimiter shaper_{RateLimiter::Limits{}, 0.008};  // replaced by open() with the options' limits
    Stage stage_ = Stage::Open;
    Closing closing_ = Closing::Begin;
    Clock::time_point sent_at_{};
    std::deque<Joints> recent_;  // the setpoints of the last lookahead and then some, for the tracking check
    std::size_t recent_capacity_ = 1;
    Clock::time_point settled_since_{};
};

/// Streams joint positions through transparent transmission (SDK manual
/// 2.2.7). Opens at standstill and starts from the measured position, so
/// nothing moves before the first target; until then the watchdog does not
/// run. Sends one point per cycle, the cycle being the controller's sampling
/// period, each target shaped by the rate limiter. Closes gracefully
/// (decelerate, let the queue drain, clear it) or, after a stop or a fault, at
/// once (stop, clear).
class ServoStream {
public:
    ServoStream(const ServoOptions& options, std::shared_ptr<Channel<Joints>> channel);

    [[nodiscard]] Needs needs() const noexcept { return {kCanStartMotion, kCanMove}; }
    [[nodiscard]] std::string_view name() const noexcept { return "servo_stream"; }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out);
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why) { return link_.close(c, out, why); }

private:
    ServoOptions options_;
    std::shared_ptr<Channel<Joints>> channel_;
    ServoLink link_;
};

/// Options of a Cartesian position stream.
struct PoseServoOptions {
    ServoOptions servo{};  ///< the joint stream the solutions go into
    /// [rad]: a solution further than this from the one before on any joint ends
    /// the stream (Kind::Invalid): the arm would change configuration, or pass a
    /// singularity; or the targets jump.
    double max_step = deg_to_rad(10.0);
};

/// Streams TCP poses (the current tool, in the base frame): each new pose is
/// solved by the controller's inverse kinematics (inverseKinematic, near the
/// solution before it), and the joint positions go into a position stream the
/// way ServoStream's targets do: shaped, guarded by the watchdog, one point per
/// cycle. A solution arrives one cycle after its pose, so the stream lags a
/// joint stream by one period and sends two commands per cycle. A pose without
/// a solution ends the stream (Kind::Invalid), gracefully. The first pose should
/// be the TCP's current one.
class PoseServoStream {
public:
    PoseServoStream(const PoseServoOptions& options, std::shared_ptr<Channel<Pose>> channel);

    [[nodiscard]] Needs needs() const noexcept { return {kCanStartMotion, kCanMove}; }
    [[nodiscard]] std::string_view name() const noexcept { return "pose_stream"; }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out);
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why) { return link_.close(c, out, why); }

private:
    [[nodiscard]] std::expected<void, Error> take_solution(const Cycle& c);

    PoseServoOptions options_;
    std::shared_ptr<Channel<Pose>> channel_;
    ServoLink link_;
    Joints reference_{};                     // the newest solution; at first the measured position
    std::optional<Stamped<Joints>> target_;  // the newest solution, stamped as its pose was
    std::optional<Stamped<Pose>> asked_;     // the pose whose solution is awaited
    Clock::time_point last_asked_{};         // the stamp of the newest pose sent to be solved
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
