#pragma once
/// @file
/// Trajectories with timing, and playing one on the controller's own clock
/// through the timestamp service (SDK manual 2.2.9; docs/motion.md 2.3).

#include <chrono>
#include <cstddef>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "ec/motion/activity.hpp"
#include "ec/units.hpp"

namespace ec::motion {

/// Joint positions at increasing times: a path with its timing, in SI.
struct JointTrajectory {
    std::vector<double> time{};      ///< [s]; the first is 0, then strictly increasing
    std::vector<Joints> position{};  ///< [rad]; one per time

    [[nodiscard]] std::size_t size() const noexcept { return time.size(); }
    [[nodiscard]] double duration() const noexcept { return time.empty() ? 0.0 : time.back(); }
};

/// The bounds a trajectory is checked against before it runs.
struct TrajectoryLimits {
    Joints max_velocity = kEc66MaxJointSpeed;  ///< [rad/s], between neighbouring points
    /// [rad/s^2], peak_acceleration(): the datasheet speeds reached in 0.5 s. The
    /// datasheet gives no accelerations; to tune on the robot (docs/motion.md 7.2).
    Joints max_acceleration = scale(kEc66MaxJointSpeed, 2.0);
    double joint_limit = kEc66JointLimit;  ///< [rad], on |q|
};

/// Why `t` cannot run, or nothing: it needs at least 3 points, as many times as
/// positions, times from 0 strictly increasing, finite positions within the
/// joint limit, no segment faster than the speed limit on any joint, and no
/// change of speed (from rest at the start, to rest at the end) above the
/// acceleration limit.
[[nodiscard]] std::expected<void, std::string> validate(const JointTrajectory& t, const TrajectoryLimits& limits = {});

/// Each joint's largest speed [rad/s] between neighbouring points of a valid
/// trajectory.
[[nodiscard]] Joints peak_velocity(const JointTrajectory& t) noexcept;

/// Each joint's largest acceleration [rad/s^2] of a valid trajectory, from the
/// change of speed between neighbouring segments, and from rest into the first
/// segment and out of the last (a trajectory is run from rest to rest).
[[nodiscard]] Joints peak_acceleration(const JointTrajectory& t) noexcept;

/// The position at time `s` [s] of a valid trajectory, interpolated linearly
/// between its points; the first or last point outside its times.
[[nodiscard]] Joints position_at(const JointTrajectory& t, double s) noexcept;

/// Uploads a trajectory, has the controller check it, and runs it, timed by the
/// controller: the PC's timing does not matter once it runs. The arm must
/// already rest at the first point (e.g. MoveJoint first, in a Sequence). Done
/// when the arm rests at the last point. Its time limit is the duration scaled
/// by `speed` and by the controller's speed setting (in case that scales the
/// trajectory too; docs/motion.md 7), plus `margin`.
struct PlayTrajectory {
    JointTrajectory path{};
    double speed = 1.0;  ///< time scale, (0, 1]: 0.5 plays at half speed
    TrajectoryLimits limits{};
    double start_tolerance = deg_to_rad(0.5);  ///< [rad]: how close to the first point the arm must be
    double tolerance = deg_to_rad(0.2);        ///< [rad]: arrival at the last point
    int points_per_cycle = 5;                  ///< uploaded per cycle, each confirmed (ret_flag 1)
    std::chrono::milliseconds margin{5'000};
};

class PlayTrajectoryActivity {
public:
    explicit PlayTrajectoryActivity(PlayTrajectory spec) : spec_(std::move(spec)) {}

    [[nodiscard]] Needs needs() const noexcept { return {kCanStartMotion, kCanMove}; }
    [[nodiscard]] std::string_view name() const noexcept { return "play_trajectory"; }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out);
    [[nodiscard]] bool close(const Cycle& c, Outbox& out, End why);

private:
    enum class Stage { Check, AwaitSpeed, AwaitBegin, Upload, AwaitSeal, AwaitCheck, AwaitStart, Running };
    enum class Closing { Begin, AwaitRest, Flush, AwaitFlush };

    // The reply to `method` in this cycle: nullopt while none has come, else ok or the error.
    std::optional<std::expected<void, Error>> await(const Cycle& c, std::string_view method) const;

    PlayTrajectory spec_;
    Stage stage_ = Stage::Check;
    Closing closing_ = Closing::Begin;
    Clock::time_point sent_at_{};
    Clock::time_point started_at_{};
    std::chrono::nanoseconds shortest_{};  // the trajectory's own time, at the speed asked for
    std::chrono::nanoseconds limit_{};
    double setting_ = 1.0;  // the controller's speed setting, as a fraction
    std::size_t uploaded_ = 0;
    std::size_t awaited_ = 0;  // PushPos replies due this cycle
    bool uploading_ = false;   // start_push_pos was accepted: the buffer needs flushing
    bool moved_ = false;
    int short_cycles_ = 0;
    Clock::time_point stop_sent_at_{};
};

[[nodiscard]] inline PlayTrajectoryActivity make_activity(const PlayTrajectory& spec) {
    return PlayTrajectoryActivity(spec);
}

static_assert(Spec<PlayTrajectory>);

}  // namespace ec::motion
