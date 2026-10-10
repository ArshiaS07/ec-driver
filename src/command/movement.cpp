#include "ec/command/movement.hpp"

#include "decode_helpers.hpp"

namespace ec::command {

Json MoveByJoint::params() const {
    Json p;  // field order as in the manual
    p["targetPos"] = target;
    p["speed"] = speed_pct;
    if (acc_pct) p["acc"] = *acc_pct;
    if (dec_pct) p["dec"] = *dec_pct;
    return p;
}

std::expected<MoveByJoint::Reply, Error> MoveByJoint::decode(const Json& result) { return detail::as_ok(result); }

std::expected<Stop::Reply, Error> Stop::decode(const Json& result) { return detail::as_ok(result); }

}  // namespace ec::command
