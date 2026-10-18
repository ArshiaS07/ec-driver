#pragma once
/// @file
/// Timestamp service (TrajectoryService), SDK manual section 2.2.9: a
/// trajectory of timestamped points is uploaded, checked, then run by the
/// controller on its own clock. The order: StartPushPos, PushPos for every
/// point, StopPushPos, CheckTrajectory, StartTrajectory; FlushTrajectory clears
/// it. Until it is cleared or replaced, it can be run again without uploading.

#include <array>
#include <cstdint>
#include <string_view>

#include "ec/sdk/json.hpp"
#include "ec/sdk/reply.hpp"
#include "ec/sdk/request.hpp"

namespace ec::sdk {

/// What a trajectory's points are (pos_type).
enum class PointType : std::uint8_t {
    Joint = 0,  ///< joint positions [deg]
    Pose = 1,   ///< poses: x, y, z [mm], rx, ry, rz (radians assumed; the manual does not say)
};

/// The result of checking an uploaded trajectory (§2.2.9.4).
enum class TrajectoryCheck : std::int8_t {
    Valid = 0,           ///< points and timestamps are correct
    CountMismatch = -1,  ///< the number of points differs from StartPushPos's
    FormatError = -2,    ///< a point is malformed
    BadTimestamps = -3,  ///< the timestamps do not start at 0 or do not increase
};
[[nodiscard]] constexpr bool is_valid(TrajectoryCheck c) noexcept {
    return static_cast<int>(c) <= 0 && static_cast<int>(c) >= -3;
}

/// §2.2.9.1 Starts an upload of `point_count` points.
struct StartPushPos {
    static constexpr std::string_view method = "start_push_pos";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    int point_count = 0;                      ///< path_lenth (sic), >= 3
    PointType point_type = PointType::Joint;  ///< pos_type
    JointsDeg reference{};  ///< ref_joint_pos: for poses, the inverse-kinematics reference of the first
    PoseMmRad frame{};      ///< ref_frame: the frame of pose points; all zero for the base frame
    bool replies = true;    ///< ret_flag: whether PushPos replies

    [[nodiscard]] Json params() const {
        return Json{{"path_lenth", point_count},
                    {"pos_type", static_cast<int>(point_type)},
                    {"ref_joint_pos", reference},
                    {"ref_frame", frame},
                    {"ret_flag", replies ? 1 : 0}};
    }
};

/// §2.2.9.2 Adds one point. Timestamps start at 0 and increase.
struct PushPos {
    static constexpr std::string_view method = "push_pos";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    double time_s = 0.0;            ///< timestamp
    std::array<double, 6> point{};  ///< pos: joints [deg] or a pose, as StartPushPos's point_type says

    [[nodiscard]] Json params() const { return Json{{"timestamp", time_s}, {"pos", point}}; }
};

/// §2.2.9.3 Ends the upload; true only if every point was correct.
struct StopPushPos {
    static constexpr std::string_view method = "stop_push_pos";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

/// §2.2.9.4 Checks the uploaded trajectory.
struct CheckTrajectory {
    static constexpr std::string_view method = "check_trajectory";
    static constexpr Kind kind = Kind::Query;
    using Reply = TrajectoryCheck;
};

/// §2.2.9.5 Runs the uploaded trajectory with its times scaled by `speed_pct`.
struct StartTrajectory {
    static constexpr std::string_view method = "start_trajectory";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
    double speed_pct = 100.0;  ///< speed_percent, >= 0.1

    [[nodiscard]] Json params() const { return Json{{"speed_percent", speed_pct}}; }
};

/// §2.2.9.6 Pauses the running trajectory.
struct PauseTrajectory {
    static constexpr std::string_view method = "pause_trajectory";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

/// §2.2.9.7 Resumes a paused trajectory.
struct ResumeTrajectory {
    static constexpr std::string_view method = "resume_trajectory";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

/// §2.2.9.8 Stops the running trajectory.
struct StopTrajectory {
    static constexpr std::string_view method = "stop_trajectory";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

/// §2.2.9.9 Clears the uploaded trajectory.
struct FlushTrajectory {
    static constexpr std::string_view method = "flush_trajectory";
    static constexpr Kind kind = Kind::Motion;
    using Reply = Ok;
};

static_assert(Command<StartPushPos>);
static_assert(Command<PushPos>);
static_assert(Command<StopPushPos>);
static_assert(Command<CheckTrajectory>);
static_assert(Command<StartTrajectory>);
static_assert(Command<PauseTrajectory>);
static_assert(Command<ResumeTrajectory>);
static_assert(Command<StopTrajectory>);
static_assert(Command<FlushTrajectory>);

}  // namespace ec::sdk
