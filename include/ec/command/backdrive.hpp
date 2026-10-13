#pragma once
/// @file
/// Backdrive service (BackdriveService), SDK manual section 2.2.11.

#include <string_view>
#include <vector>

#include "ec/command/reply.hpp"
#include "ec/command/request.hpp"

namespace ec::command {

/// §2.2.11.1 Whether each joint's brake is released; shape unverified.
struct GetServoBrakeOffStatus {
    static constexpr std::string_view method = "get_servo_brake_off_status";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::vector<int>;
};

/// §2.2.11.2 Whether the robot is in backdrive mode.
struct GetBackdriveStatus {
    static constexpr std::string_view method = "get_backdrive_status";
    static constexpr Kind kind = Kind::Query;
    using Reply = Flag;
};

/// §2.2.11.3 Enters backdrive mode; only from the reset state. Remote mode only.
struct EnterBackdrive {
    static constexpr std::string_view method = "enter_backdrive";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
};

/// §2.2.11.4 Leaves backdrive mode. Remote mode only.
struct ExitBackdrive {
    static constexpr std::string_view method = "exit_backdrive";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
};

/// §2.2.11.5 Resets the controller's state; only when stopped or in an error
/// other than an emergency stop. Remote mode only.
struct ResetRobotStatus {
    static constexpr std::string_view method = "reset_robot_status";
    static constexpr Kind kind = Kind::Action;
    using Reply = Ok;
};

static_assert(Command<GetServoBrakeOffStatus>);
static_assert(Command<GetBackdriveStatus>);
static_assert(Command<EnterBackdrive>);
static_assert(Command<ExitBackdrive>);
static_assert(Command<ResetRobotStatus>);

}  // namespace ec::command
