#pragma once
/// @file
/// Why an activity did not finish, and which of those reasons latch a fault.

#include <string>
#include <string_view>

#include "ec/motion/conditions.hpp"

namespace ec::motion {

/// Why an activity was refused or ended early. The kinds that latch a fault
/// (latches()) need acknowledge() before anything else runs; the others only
/// end that one activity.
struct Error {
    enum class Kind {
        // Do not latch:
        Refused,    ///< not started: another activity runs, a fault is latched, or a condition is missing
        Invalid,    ///< the activity's arguments are impossible: out of range, empty, inconsistent
        Cancelled,  ///< ended early, gracefully, on request
        Stopped,    ///< ended by stop()
        // Latch a fault:
        ConditionLost,  ///< a condition the activity needs stopped holding while it ran
        Command,        ///< the controller refused or failed a command, or the arm did not do what it was told
        Starved,        ///< a stream's newest target got older than its timeout
        Tracking,       ///< the arm is too far from where it was commanded to be
        Timeout,        ///< an activity ran past its time limit, or its close did not finish in time
        Overrun,        ///< the control loop fell behind its period
        Disconnected,   ///< the command connection was lost
        Internal,       ///< a bug: an exception escaped an activity; the loop stopped the arm and ended
    };
    Kind kind = Kind::Refused;
    std::string message{};  ///< what was observed, for people
    Conditions missing{};   ///< for Refused and ConditionLost: the conditions that did not hold
};

/// Whether an error of this kind latches a fault.
[[nodiscard]] constexpr bool latches(Error::Kind k) noexcept {
    switch (k) {
        case Error::Kind::Refused:
        case Error::Kind::Invalid:
        case Error::Kind::Cancelled:
        case Error::Kind::Stopped: return false;
        case Error::Kind::ConditionLost:
        case Error::Kind::Command:
        case Error::Kind::Starved:
        case Error::Kind::Tracking:
        case Error::Kind::Timeout:
        case Error::Kind::Overrun:
        case Error::Kind::Disconnected:
        case Error::Kind::Internal: return true;
    }
    return true;
}

/// Lower-case name, e.g. "condition lost".
[[nodiscard]] constexpr std::string_view to_string(Error::Kind k) noexcept {
    switch (k) {
        case Error::Kind::Refused: return "refused";
        case Error::Kind::Invalid: return "invalid";
        case Error::Kind::Cancelled: return "cancelled";
        case Error::Kind::Stopped: return "stopped";
        case Error::Kind::ConditionLost: return "condition lost";
        case Error::Kind::Command: return "command failed";
        case Error::Kind::Starved: return "starved";
        case Error::Kind::Tracking: return "tracking error";
        case Error::Kind::Timeout: return "timeout";
        case Error::Kind::Overrun: return "overrun";
        case Error::Kind::Disconnected: return "disconnected";
        case Error::Kind::Internal: return "internal error";
    }
    return "?";
}

/// "kind: message", for people.
[[nodiscard]] inline std::string describe(const Error& e) {
    std::string out(to_string(e.kind));
    if (!e.message.empty()) out += ": " + e.message;
    return out;
}

}  // namespace ec::motion
