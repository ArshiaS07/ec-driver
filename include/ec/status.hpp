#pragma once
/// @file
/// Controller status enums, shared by both ports: the 8056 frame carries them,
/// and 8055 queries (getRobotState, getRobotMode, getCycleMode) return the same
/// values. Values and meanings are the manual's; the underlying types are the
/// 8056 field types.
///
/// The values arrive as plain integers, so a variable of these types can hold a
/// value the manual does not define. is_valid() tells; printing one with
/// std::format gives "unknown (N)".

#include <cstdint>
#include <format>
#include <string>
#include <string_view>
#include <utility>

namespace ec {

/// Robot state. Collision (5) is documented for getRobotState; the 8056 table lists 0..4.
enum class RobotState : std::int32_t { Stop = 0, Pause = 1, EmergencyStop = 2, Running = 3, Alarm = 4, Collision = 5 };

/// Robot mode.
enum class RobotMode : std::int32_t { Teach = 0, Auto = 1, Remote = 2 };

/// Program cycle mode (8056: autorun_cycelMode).
enum class CycleMode : std::uint8_t { Step = 0, Cycle = 1, Continuous = 2 };

/// Name of a value, e.g. "running"; "unknown" for a value the manual does not define.
[[nodiscard]] std::string_view to_string(RobotState s) noexcept;
[[nodiscard]] std::string_view to_string(RobotMode m) noexcept;
[[nodiscard]] std::string_view to_string(CycleMode c) noexcept;

/// True if the manual defines the value.
[[nodiscard]] constexpr bool is_valid(RobotState s) noexcept {
    switch (s) {  // no default: a new enumerator without a case here is a compiler warning (an error in CI)
        case RobotState::Stop:
        case RobotState::Pause:
        case RobotState::EmergencyStop:
        case RobotState::Running:
        case RobotState::Alarm:
        case RobotState::Collision: return true;
    }
    return false;
}

[[nodiscard]] constexpr bool is_valid(RobotMode m) noexcept {
    switch (m) {
        case RobotMode::Teach:
        case RobotMode::Auto:
        case RobotMode::Remote: return true;
    }
    return false;
}

[[nodiscard]] constexpr bool is_valid(CycleMode c) noexcept {
    switch (c) {
        case CycleMode::Step:
        case CycleMode::Cycle:
        case CycleMode::Continuous: return true;
    }
    return false;
}

namespace detail {
// Shared body of the std::formatter specializations below.
template <typename E>
struct StatusFormatter : std::formatter<std::string_view> {
    auto format(E value, std::format_context& ctx) const {
        if (is_valid(value)) return std::formatter<std::string_view>::format(to_string(value), ctx);
        const std::string text = std::format("unknown ({})", +std::to_underlying(value));
        return std::formatter<std::string_view>::format(text, ctx);
    }
};
}  // namespace detail

}  // namespace ec

/// std::format prints a status value by name ("running"), or as "unknown (N)" for
/// a value the manual does not define. String format specs apply, e.g. "{:>10}".
template <>
struct std::formatter<ec::RobotState> : ec::detail::StatusFormatter<ec::RobotState> {};
template <>
struct std::formatter<ec::RobotMode> : ec::detail::StatusFormatter<ec::RobotMode> {};
template <>
struct std::formatter<ec::CycleMode> : ec::detail::StatusFormatter<ec::CycleMode> {};
