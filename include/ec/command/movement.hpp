#pragma once
/// @file
/// Movement service (MovementService), SDK manual section 2.2.3. These types
/// describe the calls; nothing here sends anything.

#include <optional>
#include <string_view>

#include "ec/command/json.hpp"
#include "ec/command/reply.hpp"
#include "ec/command/request.hpp"

namespace ec::command {

/// §2.2.3.1 Joint motion to a target position. Remote mode only; the robot must
/// be stopped. The optional stop conditions (cond_*) are not wrapped yet.
struct MoveByJoint {
    static constexpr std::string_view method = "moveByJoint";
    using Reply = Ok;
    JointsDeg target{};          ///< targetPos, each in [-360, 360] deg
    double speed_pct = 0.0;      ///< speed, [0.01, 100] %
    std::optional<int> acc_pct;  ///< acc, [1, 100] %; controller default 80
    std::optional<int> dec_pct;  ///< dec, [1, 100] %; controller default: acc

    [[nodiscard]] Json params() const {
        Json p;  // members in the manual's order
        p["targetPos"] = target;
        p["speed"] = speed_pct;
        if (acc_pct) p["acc"] = *acc_pct;
        if (dec_pct) p["dec"] = *dec_pct;
        return p;
    }
};

/// §2.2.3.9 Stops the robot. Remote mode only.
struct Stop {
    static constexpr std::string_view method = "stop";
    using Reply = Ok;
};

static_assert(Command<MoveByJoint>);
static_assert(Command<Stop>);

}  // namespace ec::command
