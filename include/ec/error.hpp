#pragma once
/// @file
/// The driver's error (ADR 0018): what failed, and what was observed. Every
/// call of the API returns its result or one of these. The command layer
/// keeps its own error below it (ec/command/error.hpp).

#include <format>
#include <string>
#include <string_view>

#include "ec/command/error.hpp"

namespace ec {

struct Error {
    enum class Kind {
        Invalid,       ///< impossible arguments (out of range, empty, inconsistent); nothing was sent
        Busy,          ///< refused: another motion is open
        Command,       ///< the controller refused or failed a call, or the arm did not do what it was told
        Fault,         ///< the controller stopped the arm: emergency stop, alarm, collision
        Stopped,       ///< ended by Robot::stop()
        Timeout,       ///< no answer in time
        Disconnected,  ///< the connection is lost
    };
    Kind kind = Kind::Command;
    std::string message{};  ///< what was observed, for people
};

/// Lower-case name, e.g. "command failed".
[[nodiscard]] constexpr std::string_view to_string(Error::Kind k) noexcept {
    switch (k) {
        case Error::Kind::Invalid: return "invalid";
        case Error::Kind::Busy: return "busy";
        case Error::Kind::Command: return "command failed";
        case Error::Kind::Fault: return "fault";
        case Error::Kind::Stopped: return "stopped";
        case Error::Kind::Timeout: return "timeout";
        case Error::Kind::Disconnected: return "disconnected";
    }
    return "?";
}

/// "kind: message", for people.
[[nodiscard]] inline std::string describe(const Error& e) {
    std::string out(to_string(e.kind));
    if (!e.message.empty()) out += ": " + e.message;
    return out;
}

/// A failed call of the command layer as the driver's error, its message
/// "method: why".
[[nodiscard]] inline Error from_command(std::string_view method, const command::Error& e) {
    const Error::Kind kind = e.kind == command::Error::Kind::Timeout        ? Error::Kind::Timeout
                             : e.kind == command::Error::Kind::Disconnected ? Error::Kind::Disconnected
                                                                            : Error::Kind::Command;
    std::string message = std::format("{}: {}", method, command::to_string(e.kind));
    if (!e.message.empty()) message += ": " + e.message;
    return Error{kind, std::move(message)};
}

}  // namespace ec
