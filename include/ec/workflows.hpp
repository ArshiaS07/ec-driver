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

}  // namespace ec
