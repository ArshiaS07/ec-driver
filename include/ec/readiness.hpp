#pragma once
/// @file
/// Readiness: what stands between the robot and a motion command, read from
/// an observed State. Pure: it sends nothing and decides nothing for the
/// controller, which checks every command itself; it names the next step and
/// its remedy, for a person or a workflow (docs/ec66.md §5.2, §5.10).
///
///     const auto blocker = ec::readiness(robot.monitor().latest().value());
///     if (blocker != ec::Blocker::None) std::println("{}: {}", blocker, ec::remedy(blocker));

#include <cstdint>
#include <format>
#include <optional>
#include <string_view>

#include "ec/state.hpp"

namespace ec {

/// The first step missing before a motion command, in the order they are to
/// be taken.
enum class Blocker : std::uint8_t {
    None,           ///< ready: REMOTE, nothing to clear, synchronised, servo on, state stop
    NotRemote,      ///< the key switch is not at REMOTE (the operator's)
    EmergencyStop,  ///< an emergency stop is pressed (the operator's)
    Collision,      ///< a collision was detected
    Alarm,          ///< an alarm (state alarm, or the short-lived emergency-stop state)
    NotSynced,      ///< the encoders are not synchronised
    ServoOff,       ///< the servos are off
    NotPrecise,     ///< the encoders are not precise (only when told: 8056 does not carry it)
    NotStopped,     ///< a motion is running or paused (or the state is one the manual does not define)
};

/// The first blocker in `state`. `precise` is get_servo_precise_position_status
/// if the caller has read it; without it, precise is not judged. The order:
/// the operator's steps first (the key; the button); then a collision, reset
/// before its alarm is cleared [S 2.2.2.35]; the alarm [S 2.2.1.4]; the
/// synchronisation [S 2.2.1.3]; the servo, which calibrating needs on
/// [S 2.2.3.21]; precise; and the state back at stop.
[[nodiscard]] Blocker readiness(const State& state, std::optional<bool> precise = std::nullopt) noexcept;

/// What removes it, in words, naming the SDK calls or the pendant step.
[[nodiscard]] std::string_view remedy(Blocker b) noexcept;

/// True for the steps only the operator can take: the key switch and the
/// emergency stop button.
[[nodiscard]] constexpr bool needs_operator(Blocker b) noexcept {
    return b == Blocker::NotRemote || b == Blocker::EmergencyStop;
}

/// Short name, e.g. "servo off".
[[nodiscard]] std::string_view to_string(Blocker b) noexcept;

}  // namespace ec

template <>
struct std::formatter<ec::Blocker> : std::formatter<std::string_view> {
    auto format(ec::Blocker b, std::format_context& ctx) const {
        return std::formatter<std::string_view>::format(ec::to_string(b), ctx);
    }
};
