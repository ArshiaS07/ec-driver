#include "ec/motion/conditions.hpp"

#include <cmath>

namespace ec::motion {

std::string_view to_string(Condition c) noexcept {
    switch (c) {
        case Condition::Fresh: return "fresh state";
        case Condition::Remote: return "remote mode";
        case Condition::NoEmergencyStop: return "no emergency stop";
        case Condition::NoAlarm: return "no alarm";
        case Condition::NoCollision: return "no collision";
        case Condition::ServoOn: return "servo on";
        case Condition::Synced: return "encoders synchronised";
        case Condition::AtRest: return "at rest";
        case Condition::Standstill: return "standstill";
    }
    return "?";
}

std::string to_string(Conditions conditions) {
    std::string out;
    for (int i = 0; i < kConditionCount; ++i) {
        const auto c = static_cast<Condition>(i);
        if (!conditions.has(c)) continue;
        if (!out.empty()) out += ", ";
        out += to_string(c);
    }
    return out.empty() ? "none" : out;
}

Conditions observe(const feedback::State& state, std::chrono::nanoseconds age, const Thresholds& thresholds) noexcept {
    Conditions held;
    if (age >= std::chrono::nanoseconds::zero() && age < thresholds.fresh_within) held.insert(Condition::Fresh);
    if (state.robot_mode == RobotMode::Remote) held.insert(Condition::Remote);

    const bool state_known = is_valid(state.robot_state);
    if (state_known && !state.emergency_stop && state.robot_state != RobotState::EmergencyStop)
        held.insert(Condition::NoEmergencyStop);
    if (state_known && state.robot_state != RobotState::Alarm) held.insert(Condition::NoAlarm);
    if (state_known && !state.collision && state.robot_state != RobotState::Collision)
        held.insert(Condition::NoCollision);
    if (state.brakes_released) held.insert(Condition::ServoOn);
    if (state.motors_synced) held.insert(Condition::Synced);

    bool at_rest = true;
    for (double v : state.joint_velocity) at_rest = at_rest && std::abs(v) < thresholds.rest_speed;
    if (at_rest) held.insert(Condition::AtRest);
    if (at_rest && state.robot_state == RobotState::Stop) held.insert(Condition::Standstill);
    return held;
}

}  // namespace ec::motion
