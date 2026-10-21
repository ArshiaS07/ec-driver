#include "ec/readiness.hpp"

namespace ec {

Blocker readiness(const State& s, std::optional<bool> precise) noexcept {
    if (s.system.mode != RobotMode::Remote) return Blocker::NotRemote;
    if (s.safety.emergency_stop) return Blocker::EmergencyStop;
    if (s.safety.collision || s.motion.state == RobotState::Collision) return Blocker::Collision;
    // The emergency-stop state "will only exist for a short time and will be
    // covered by the alarm soon" [S 2.2.2.1]; with the button released, it is the alarm.
    if (s.motion.state == RobotState::Alarm || s.motion.state == RobotState::EmergencyStop) return Blocker::Alarm;
    if (!s.drives.synced) return Blocker::NotSynced;
    if (!s.drives.servo_on) return Blocker::ServoOff;
    if (precise.has_value() && !*precise) return Blocker::NotPrecise;
    if (s.motion.state != RobotState::Stop) return Blocker::NotStopped;
    return Blocker::None;
}

std::string_view remedy(Blocker b) noexcept {
    switch (b) {
        case Blocker::None: return "nothing: a motion command can be sent";
        case Blocker::NotRemote: return "turn the key switch to REMOTE (the operator)";
        case Blocker::EmergencyStop: return "remove the cause and release the emergency stop button (the operator)";
        case Blocker::Collision: return "resetCollisionState, then clearAlarm";
        case Blocker::Alarm: return "read the alarm (getAlarmNum, the pendant), then clearAlarm";
        case Blocker::NotSynced: return "syncMotorStatus";
        case Blocker::ServoOff: return "set_servo_status 1 (the brakes release with a small motion)";
        case Blocker::NotPrecise:
            return "calibrate_encoder_zero_position (the arm moves a little), or move each axis on the pendant";
        case Blocker::NotStopped: return "wait for the motion to end, or stop it";
    }
    return "unknown";
}

std::string_view to_string(Blocker b) noexcept {
    switch (b) {
        case Blocker::None: return "ready";
        case Blocker::NotRemote: return "not in remote mode";
        case Blocker::EmergencyStop: return "emergency stop";
        case Blocker::Collision: return "collision";
        case Blocker::Alarm: return "alarm";
        case Blocker::NotSynced: return "not synchronised";
        case Blocker::ServoOff: return "servo off";
        case Blocker::NotPrecise: return "not precise";
        case Blocker::NotStopped: return "not stopped";
    }
    return "unknown";
}

}  // namespace ec
