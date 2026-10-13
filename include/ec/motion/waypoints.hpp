#pragma once
/// @file
/// Waypoint paths that the controller runs as one motion (SDK manual 2.2.3.5 to
/// 2.2.3.7, docs/motion.md 2.3): the waypoints are uploaded one by one, then
/// run with their corners blended, so the arm need not stop at each.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <string_view>
#include <vector>

#include "ec/motion/activity.hpp"
#include "ec/units.hpp"

namespace ec::motion {

/// How a waypoint is reached from the point before it.
enum class PathMove : std::uint8_t {
    Joint,  ///< joint interpolation
    Line,   ///< the TCP in a straight line
};

/// One waypoint of a path.
struct Waypoint {
    Joints position{};  ///< [rad]; for Line, the TCP goes straight to these joints' pose
    PathMove move = PathMove::Line;
    /// Line: TCP speed [m/s] in [0.001, 3], absolute (AV) as in MoveLine. Joint:
    /// a fraction of the joints' maximum speed in [0.01, 1], which the
    /// controller's speed setting scales.
    double speed = 0.05;
    /// [m] in [0, 2.608]: how far from the waypoint the path may round the corner
    /// toward the next one; 0 stops there. The last waypoint's must be 0.
    double blend_radius = 0.0;
};

/// Runs `points` as one motion: clearPathPoint, one addPathPoint per point
/// (`points_per_cycle` per cycle, each confirmed), then moveByPath, from
/// standstill. Done when the arm rests within `tolerance` of the last point.
/// Fails if, after moving, it rests elsewhere for `stopped_for`, or after
/// `timeout`. Its close() stops the arm unless it arrived.
struct MovePath {
    std::vector<Waypoint> points{};
    double tolerance = deg_to_rad(0.2);  ///< [rad] on every joint, at the last point
    std::chrono::milliseconds timeout{120'000};
    std::chrono::milliseconds stopped_for{250};
    int points_per_cycle = 5;
};

class MovePathActivity {
public:
    explicit MovePathActivity(MovePath spec) : spec_(std::move(spec)) {}

    [[nodiscard]] Needs needs() const noexcept { return {kCanStartMotion, kCanMove}; }
    [[nodiscard]] std::string_view name() const noexcept { return "move_path"; }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out);
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why);

private:
    enum class Stage { Check, AwaitClear, Upload, AwaitRun, Moving };

    // The reply to `method` in this cycle: nullopt while none has come, else ok or the error.
    [[nodiscard]] std::optional<std::expected<void, Error>> await_ok(const Cycle& c, std::string_view method) const;
    [[nodiscard]] std::expected<Phase, Error> upload(const Cycle& c, Outbox& out);
    [[nodiscard]] std::expected<Phase, Error> watch(const Cycle& c);

    MovePath spec_;
    Stage stage_ = Stage::Check;
    Clock::time_point sent_at_{};
    std::size_t uploaded_ = 0;
    std::size_t awaited_ = 0;  // addPathPoint replies due this cycle
    bool run_sent_ = false;    // moveByPath went out: the arm may move
    bool moved_ = false;
    std::optional<Clock::time_point> resting_since_{};
    std::optional<Clock::time_point> stop_sent_at_{};
};

[[nodiscard]] inline MovePathActivity make_activity(const MovePath& spec) { return MovePathActivity(spec); }

static_assert(Spec<MovePath>);

}  // namespace ec::motion
