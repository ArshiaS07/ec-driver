#pragma once
/// @file
/// Servo service (ServoService), SDK manual section 2.2.1: servo power, encoder
/// synchronisation, alarms. These types describe the calls; nothing here sends anything.

#include <string_view>

#include "ec/command/json.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/request.hpp"

namespace ec::command {

/// §2.2.1.1 Whether the arm's servos are on (8056: servoReady).
struct GetServoStatus {
    static constexpr std::string_view method = "getServoStatus";
    static constexpr Kind kind = Kind::Query;
    using Reply = bool;
};

/// §2.2.1.2 Switches the arm's servos on or off. Remote mode only. The reply
/// means accepted: the servos are on once 8056 shows servoReady.
struct SetServoStatus {
    static constexpr std::string_view method = "set_servo_status";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
    bool on = true;  ///< status: 1 on, 0 off

    [[nodiscard]] Json params() const { return Json{{"status", on ? 1 : 0}}; }
};

/// §2.2.1.3 Synchronises the servo encoder data, needed after power-up before
/// any motion (8056: can_motor_run). Remote mode only.
struct SyncMotorStatus {
    static constexpr std::string_view method = "syncMotorStatus";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
};

/// §2.2.1.4 Clears the alarm. Remote mode only.
struct ClearAlarm {
    static constexpr std::string_view method = "clearAlarm";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
};

/// §2.2.1.5 Whether the encoders are synchronised (8056: can_motor_run).
struct GetMotorStatus {
    static constexpr std::string_view method = "getMotorStatus";
    static constexpr Kind kind = Kind::Query;
    using Reply = bool;
};

static_assert(Command<GetServoStatus>);
static_assert(Command<SetServoStatus>);
static_assert(Command<SyncMotorStatus>);
static_assert(Command<ClearAlarm>);
static_assert(Command<GetMotorStatus>);

}  // namespace ec::command
