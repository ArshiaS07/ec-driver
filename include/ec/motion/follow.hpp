#pragma once
/// @file
/// Following a timed joint trajectory through transparent transmission: the
/// streamed counterpart of PlayTrajectory (docs/motion.md 5.11). The PC sends
/// the trajectory point by point, so it runs on the PC's cycle and can be
/// stopped or followed by a live stream without the controller's trajectory
/// service; PlayTrajectory hands the whole trajectory to the controller instead.

#include <chrono>
#include <cstdint>
#include <expected>
#include <optional>
#include <string_view>

#include "ec/motion/activity.hpp"
#include "ec/motion/stream.hpp"
#include "ec/motion/trajectory.hpp"
#include "ec/units.hpp"

namespace ec::motion {

/// Plays `path` through a position stream (ServoLink): in the n-th cycle after
/// the stream opens, the setpoint is the trajectory at n periods, interpolated
/// linearly between its points (so its times need not match the period). The
/// setpoints go out as they are, without the live streams' shaping, which
/// would lag behind a moving target; instead the trajectory is checked before
/// anything moves, against the stream's limits: validate() with
/// servo.max_velocity and servo.joint_limit, and an acceleration of at most
/// servo.max_velocity / servo.accelerate_in on every joint (Kind::Invalid).
/// The arm must rest at the first point. Done once the last point was sent and
/// the arm rests within `tolerance` of it; a stop or a fault closes the stream
/// at once, a cancel gracefully.
struct FollowTrajectory {
    JointTrajectory path{};
    ServoOptions servo{};                      ///< lookahead, smoothness, limits, tracking tolerance
    double start_tolerance = deg_to_rad(0.5);  ///< [rad]: how close to the first point the arm must be
    double tolerance = deg_to_rad(0.2);        ///< [rad]: arrival at the last point
    /// After the last point: how long the arm may take to arrive, beyond the lookahead.
    std::chrono::milliseconds settle{2'000};
};

class FollowTrajectoryActivity {
public:
    explicit FollowTrajectoryActivity(FollowTrajectory spec);

    [[nodiscard]] Needs needs() const noexcept { return {kCanStartMotion, kCanMove}; }
    [[nodiscard]] std::string_view name() const noexcept { return "follow_trajectory"; }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out);
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why) { return link_.close(c, out, why); }

private:
    [[nodiscard]] std::expected<void, Error> check(const Cycle& c) const;

    FollowTrajectory spec_;
    ServoLink link_;
    bool checked_ = false;
    std::uint64_t sent_ = 0;                       // setpoints put since the stream opened
    std::optional<Clock::time_point> finished_{};  // when the last point was first put
};

[[nodiscard]] inline FollowTrajectoryActivity make_activity(const FollowTrajectory& spec) {
    return FollowTrajectoryActivity(spec);
}

static_assert(Spec<FollowTrajectory>);

}  // namespace ec::motion
