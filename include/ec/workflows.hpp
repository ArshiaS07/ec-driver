#pragma once
/// @file
/// Workflows: the operating manuals' sequences, as free functions over a
/// robot's parts (docs/architecture.md §6). Each carries out one task, step
/// by step, and returns a Report: what it saw and did, why it stopped short,
/// and what the operator must do. An application may use them or compose the
/// parts itself; nothing in the parts depends on them.
///
///     const auto report = ec::bring_up(robot);
///     std::print("{}", describe(report));
///     if (!report.ok()) return;

#include <chrono>
#include <expected>
#include <optional>
#include <string>
#include <vector>

#include "ec/error.hpp"
#include "ec/readiness.hpp"
#include "ec/robot.hpp"

namespace ec {

/// One step of a workflow.
struct Step {
    std::string what;                    ///< what was read or done, e.g. "clearAlarm"
    std::string outcome;                 ///< what came of it, for people
    bool ok = true;                      ///< false: it failed (the workflow may go on)
    std::vector<std::string> details{};  ///< e.g. the log's lines
};

/// What a workflow did, and how it ended.
struct Report {
    std::vector<Step> steps;
    std::optional<Error> error{};      ///< why it stopped short; none: the task is done
    std::optional<Blocker> blocker{};  ///< what stood in the way, when that is why it stopped
    [[nodiscard]] bool ok() const noexcept { return !error; }
    /// The step needs a person: the key switch, the emergency stop button, or
    /// the pendant's step to make the encoders precise.
    [[nodiscard]] bool operator_needed() const noexcept {
        return blocker && (needs_operator(*blocker) || *blocker == Blocker::NotPrecise);
    }
};

/// The report, one line per step and the ending, for people.
[[nodiscard]] std::string describe(const Report& report);

/// The conditions readiness() judges, asked over the command connection, one
/// call each: getRobotMode, get_estop_status, getCollisionState,
/// getRobotState, getMotorStatus, getServoStatus, and
/// get_servo_precise_position_status if `with_precise`.
[[nodiscard]] std::expected<Conditions, Error> read_conditions(const Robot& robot, bool with_precise = false);

struct BringUpOptions {
    /// Also require precise encoders (get_servo_precise_position_status).
    /// Off by default: what that flag reports on V3.15.3 is not yet checked
    /// (docs/ec66.md §8).
    bool require_precise = false;
    /// How long the servos may take to report on after set_servo_status.
    std::chrono::milliseconds servo_within{3'000};
};

/// Starts a session, as the manual's examples do it (docs/ec66.md §5.10):
/// reads the conditions over the command connection, and takes the first
/// missing step (readiness()) until nothing is missing: a collision reset
/// then the alarm cleared, the alarm cleared (0.5 s), the encoders
/// synchronised (0.5 s), the servos switched on (after 0.5 s, then until they
/// report on). Ends with the speed slider in the report. Stops short, naming
/// the step, on what only the operator can do (the key at REMOTE, the
/// emergency stop button; Command or Fault), on a motion still running
/// (Busy), when a step does not take (still the same blocker afterwards), or
/// when the servos do not come on in time (Timeout). Sends only what the
/// conditions call for.
[[nodiscard]] Report bring_up(Robot& robot, const BringUpOptions& options = {});

struct ShutDownOptions {
    std::chrono::milliseconds stop_within{5'000};   ///< for the arm to come to rest
    std::chrono::milliseconds servo_within{3'000};  ///< for the servos to report off
};

/// Ends a session (built from documented calls; the manuals give no
/// sequence): the motion controller's stop, the arm at rest, the servos off
/// (the brakes engage), until they report off. The key switch and the power
/// stay with the operator.
[[nodiscard]] Report shut_down(Robot& robot, const ShutDownOptions& options = {});

/// Recovers after an alarm, a collision or an emergency stop (once the
/// operator has released the button): first explains, with the last alarms
/// (getAlarmNum) and the last error lines of the controller's log (port
/// 8058), then brings the robot up. The explanation is best effort: a step
/// that cannot be read is reported and the recovery goes on.
[[nodiscard]] Report recover(Robot& robot, const BringUpOptions& options = {});

/// A session with the servos on: brought up when it starts, shut down when it
/// ends, whichever way its scope is left (the end, an error, an exception,
/// Ctrl+C), so that a powered arm does not outlive the program supervising
/// it. The robot must outlive the session: declare the session after it.
///
///     auto session = ec::Session::start(robot);
///     if (!session) return fail(describe(session.error()));
///     ...                                         // motions
///     std::print("{}", describe(session->end())); // or leave it to the destructor
class Session {
public:
    /// bring_up. On failure, the report of it, and the servos are left as
    /// found, unless the bring-up switched them on before it failed: then they
    /// are switched off again (its report says so).
    [[nodiscard]] static std::expected<Session, Report> start(Robot& robot, const BringUpOptions& up = {},
                                                              const ShutDownOptions& down = {});

    /// shut_down, once: the stop, the arm at rest, the servos off. Later calls
    /// send nothing and return an empty report.
    Report end();
    /// The bring-up's report.
    [[nodiscard]] const Report& started() const noexcept { return started_; }
    [[nodiscard]] bool active() const noexcept { return robot_ != nullptr; }

    Session(Session&& other) noexcept;
    Session& operator=(Session&& other) noexcept;
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    /// end(), if it is still active; its report is lost (call end() to see it).
    ~Session();

private:
    Session(Robot& robot, Report started, const ShutDownOptions& down) noexcept
        : robot_(&robot), started_(std::move(started)), down_(down) {}

    Robot* robot_;  // null once ended (or moved from)
    Report started_;
    ShutDownOptions down_;
};

}  // namespace ec
