#include "ec/status.hpp"

namespace ec {

std::string_view to_string(RobotState s) noexcept {
    switch (s) {
        case RobotState::Stop: return "stop";
        case RobotState::Pause: return "pause";
        case RobotState::EmergencyStop: return "e-stop";
        case RobotState::Running: return "running";
        case RobotState::Alarm: return "alarm";
        case RobotState::Collision: return "collision";
    }
    return "unknown";
}

std::string_view to_string(RobotMode m) noexcept {
    switch (m) {
        case RobotMode::Teach: return "teach";
        case RobotMode::Auto: return "auto";
        case RobotMode::Remote: return "remote";
    }
    return "unknown";
}

std::string_view to_string(CycleMode c) noexcept {
    switch (c) {
        case CycleMode::Step: return "single step";
        case CycleMode::Cycle: return "single cycle";
        case CycleMode::Continuous: return "continuous";
    }
    return "unknown";
}

}  // namespace ec
