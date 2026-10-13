#pragma once
/// @file
/// Bring-up: from a stopped controller to one that can move, in the vendor's
/// order (SDK manual 2.2.7.6, 2.2.9.10), each step confirmed in 8056 before the
/// next. Used at start-up and, after a person acknowledged a fault, as recovery.

#include <chrono>
#include <expected>
#include <optional>
#include <string_view>

#include "ec/motion/activity.hpp"

namespace ec::motion {

/// Clears an alarm, resets a collision, synchronises the encoders and switches
/// the servos on: only the steps the state shows are needed. Done once
/// everything a motion needs holds (kCanMove). Needs remote mode, no emergency
/// stop and a fresh state.
struct BringUp {
    std::chrono::milliseconds step_timeout{5'000};  ///< per step, from its command to its effect in 8056
};

class BringUpActivity {
public:
    explicit BringUpActivity(const BringUp& spec) : spec_(spec) {}

    [[nodiscard]] Needs needs() const noexcept;
    [[nodiscard]] std::string_view name() const noexcept { return "bring_up"; }
    [[nodiscard]] std::expected<Phase, Error> step(const Cycle& c, Outbox& out);
    /// Nothing to undo: the arm does not move.
    [[nodiscard]] bool close(const Cycle& /*c*/, Outbox& /*out*/, End /*why*/) noexcept { return true; }

private:
    enum class Step { ClearAlarm, ResetCollision, Sync, ServoOn };

    BringUp spec_;
    std::optional<Step> current_;  // the step whose effect is awaited
    Clock::time_point sent_at_{};
    bool replied_ = false;
};

[[nodiscard]] inline BringUpActivity make_activity(const BringUp& spec) { return BringUpActivity(spec); }

static_assert(Spec<BringUp>);

}  // namespace ec::motion
