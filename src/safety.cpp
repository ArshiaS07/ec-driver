#include "ec/safety.hpp"

#include <array>

#include "detail/connection.hpp"
#include "detail/part.hpp"
#include "ec/sdk/param.hpp"

namespace ec {
namespace {

using detail::done;
using detail::on;

std::expected<SafetyLimit, Error> limit(const std::expected<std::array<double, 2>, Error>& pair, double scale = 1.0) {
    if (!pair) return std::unexpected(pair.error());
    return SafetyLimit{.normal = (*pair)[0] * scale, .reduced = (*pair)[1] * scale};
}

}  // namespace

std::expected<bool, Error> Safety::emergency_stop() const { return on(detail::send(session_, sdk::GetEstopStatus{})); }

std::expected<bool, Error> Safety::collision() const { return on(detail::send(session_, sdk::GetCollisionState{})); }

std::expected<void, Error> Safety::reset_collision() const {
    return done(detail::send(session_, sdk::ResetCollisionState{}));
}

std::expected<bool, Error> Safety::collision_detection() const {
    return on(detail::send(session_, sdk::GetCollisionEnableStatus{}));
}

std::expected<double, Error> Safety::collision_sensitivity() const {
    const auto pct = detail::send(session_, sdk::GetCollisionSensitivity{});
    if (!pct) return std::unexpected(pct.error());
    return *pct / 100.0;
}

std::expected<bool, Error> Safety::limits_enabled() const {
    return on(detail::send(session_, sdk::GetRobotSafetyParamsEnabled{}));
}

std::expected<SafetyLimit, Error> Safety::power_limit() const {
    return limit(detail::send(session_, sdk::GetRobotSafetyPower{}));
}

std::expected<SafetyLimit, Error> Safety::momentum_limit() const {
    return limit(detail::send(session_, sdk::GetRobotSafetyMomentum{}));
}

std::expected<SafetyLimit, Error> Safety::tool_force_limit() const {
    return limit(detail::send(session_, sdk::GetRobotSafetyToolForce{}));
}

std::expected<SafetyLimit, Error> Safety::elbow_force_limit() const {
    return limit(detail::send(session_, sdk::GetRobotSafetyElbowForce{}));
}

std::expected<SafetyLimit, Error> Safety::speed_limit() const {
    return limit(detail::send(session_, sdk::GetRobotSpeedPercentage{}), 1.0 / 100.0);
}

}  // namespace ec
