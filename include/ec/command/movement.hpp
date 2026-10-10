#pragma once
// Movement service (MovementService), SDK manual section 2.2.3.
//
// These types only describe the calls. Nothing here sends anything; motion
// requests will reach the robot only through the supervisor, which checks the
// controller state first.

#include <expected>
#include <optional>
#include <string_view>

#include "ec/command/request.hpp"

namespace ec::command {

// §2.2.3.1 Joint motion to a target position. Remote mode only; the robot must
// be stopped. The optional stop-condition parameters (cond_*) are not wrapped
// yet.
struct MoveByJoint {
    static constexpr std::string_view method = "moveByJoint";
    using Reply = Ok;
    JointsDeg target{};                // targetPos, each in [-360, 360] deg
    double speed_pct = 0.0;            // speed, [0.01, 100] %
    std::optional<int> acc_pct;        // acc, [1, 100] %, controller default 80
    std::optional<int> dec_pct;        // dec, [1, 100] %, controller default = acc

    [[nodiscard]] Json params() const;
    [[nodiscard]] static std::expected<Reply, Error> decode(const Json& result);
};

// §2.2.3.9 Stop the robot. Remote mode only.
struct Stop {
    static constexpr std::string_view method = "stop";
    using Reply = Ok;
    [[nodiscard]] Json params() const { return nullptr; }
    [[nodiscard]] static std::expected<Reply, Error> decode(const Json& result);
};

static_assert(Command<MoveByJoint>);
static_assert(Command<Stop>);

}  // namespace ec::command
