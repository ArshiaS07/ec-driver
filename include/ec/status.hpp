#pragma once
// Controller status enums, shared by both ports: the 8056 frame carries them,
// and 8055 queries (getRobotMode, getRobotState, getCycleMode) return the same
// values. Values are the manual's.

#include <cstdint>
#include <string_view>

namespace ec {

enum class RobotState : std::int32_t { Stop = 0, Pause = 1, EmergencyStop = 2, Running = 3, Alarm = 4, Collision = 5 };
enum class RobotMode : std::int32_t { Teach = 0, Auto = 1, Remote = 2 };
enum class CycleMode : std::uint8_t { Step = 0, Cycle = 1, Continuous = 2 };

[[nodiscard]] std::string_view to_string(RobotState s) noexcept;
[[nodiscard]] std::string_view to_string(RobotMode m) noexcept;
[[nodiscard]] std::string_view to_string(CycleMode c) noexcept;

// Range checks for values that arrive as plain integers from the controller.
[[nodiscard]] constexpr bool is_valid(RobotState s) noexcept {
    return static_cast<std::int32_t>(s) >= 0 && static_cast<std::int32_t>(s) <= 5;
}
[[nodiscard]] constexpr bool is_valid(RobotMode m) noexcept {
    return static_cast<std::int32_t>(m) >= 0 && static_cast<std::int32_t>(m) <= 2;
}
[[nodiscard]] constexpr bool is_valid(CycleMode c) noexcept { return static_cast<std::uint8_t>(c) <= 2; }

}  // namespace ec
