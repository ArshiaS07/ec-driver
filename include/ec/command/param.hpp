#pragma once
/// @file
/// Parameter service (ParamService), SDK manual section 2.2.2. Values are in the
/// controller's units, as in the manual.

#include <string_view>

#include "ec/command/json.hpp"
#include "ec/command/request.hpp"
#include "ec/status.hpp"

namespace ec::command {

/// §2.2.2.1 Robot state: stopped, paused, emergency stop, running, alarm or collision.
struct GetRobotState {
    static constexpr std::string_view method = "getRobotState";
    using Reply = RobotState;
};

/// §2.2.2.2 Robot mode: teach, auto or remote.
struct GetRobotMode {
    static constexpr std::string_view method = "getRobotMode";
    using Reply = RobotMode;
};

/// §2.2.2.3 Joint positions on the output side (after the gearbox) [deg].
struct GetJointPos {
    static constexpr std::string_view method = "get_joint_pos";
    using Reply = JointsDeg;
};

/// §2.2.2.29 One DH link parameter. The manual gives no unit; the EA66 returns
/// 95.431 for index 0, which reads as millimetres.
struct GetDH {
    static constexpr std::string_view method = "getDH";
    using Reply = double;
    int index = 0;  ///< 0..11 for d1..d12; the controller rejects others (Kind::Rejected)

    [[nodiscard]] Json params() const { return Json{{"index", index}}; }
};

static_assert(Command<GetRobotState>);
static_assert(Command<GetRobotMode>);
static_assert(Command<GetJointPos>);
static_assert(Command<GetDH>);

}  // namespace ec::command
