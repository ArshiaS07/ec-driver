#pragma once
// Parameter service (ParamService), SDK manual section 2.2.2.
// Replies are in the controller's units (deg, mm), as in the manual.

#include <expected>
#include <string_view>

#include "ec/command/request.hpp"
#include "ec/status.hpp"

namespace ec::command {

// §2.2.2.1 Get robot state.
struct GetRobotState {
    static constexpr std::string_view method = "getRobotState";
    using Reply = RobotState;
    [[nodiscard]] Json params() const { return nullptr; }
    [[nodiscard]] static std::expected<Reply, Error> decode(const Json& result);
};

// §2.2.2.2 Get robot mode (teach / auto / remote).
struct GetRobotMode {
    static constexpr std::string_view method = "getRobotMode";
    using Reply = RobotMode;
    [[nodiscard]] Json params() const { return nullptr; }
    [[nodiscard]] static std::expected<Reply, Error> decode(const Json& result);
};

// §2.2.2.3 Get output-side joint positions (after the gearbox).
struct GetJointPos {
    static constexpr std::string_view method = "get_joint_pos";
    using Reply = JointsDeg;
    [[nodiscard]] Json params() const { return nullptr; }
    [[nodiscard]] static std::expected<Reply, Error> decode(const Json& result);
};

// §2.2.2.29 Get one DH link parameter. The manual documents index 0..11 as
// d1..d12 and gives no unit; the EA66 returns 95.431 for index 0, which reads
// as mm. Out-of-range indices are left to the controller to reject (-32693).
struct GetDH {
    static constexpr std::string_view method = "getDH";
    using Reply = double;
    int index = 0;
    [[nodiscard]] Json params() const { return Json{{"index", index}}; }
    [[nodiscard]] static std::expected<Reply, Error> decode(const Json& result);
};

static_assert(Command<GetRobotState>);
static_assert(Command<GetRobotMode>);
static_assert(Command<GetJointPos>);
static_assert(Command<GetDH>);

}  // namespace ec::command
