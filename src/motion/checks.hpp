#pragma once
// The manual's ranges for the motion families' arguments, and their
// conversion to the controller's units. Private to the library.

#include <cmath>
#include <cstddef>
#include <expected>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "ec/error.hpp"
#include "ec/sdk/movement.hpp"
#include "ec/units.hpp"

namespace ec::motion::checks {

inline std::unexpected<Error> invalid(std::string message) {
    return std::unexpected(Error{Error::Kind::Invalid, std::move(message)});
}

/// Every joint within the manual's [-360, 360] deg.
inline std::expected<void, Error> check_target(std::string_view name, const Joints& q) {
    for (std::size_t j = 0; j < kJointCount; ++j)
        if (!(std::abs(q[j]) <= kEc66JointLimit))
            return invalid(std::format("{}: joint {} at {} rad is outside [-360, 360] deg", name, j + 1, q[j]));
    return {};
}

/// A fraction as the manual's percentage, int [1, 100]; none stays the controller's default.
inline std::expected<std::optional<int>, Error> percent(std::string_view name, const std::optional<double>& fraction) {
    if (!fraction) return std::optional<int>{};
    if (!(*fraction >= 0.01 && *fraction <= 1.0))
        return invalid(std::format("{} must be in [0.01, 1], not {}", name, *fraction));
    return std::optional<int>(static_cast<int>(std::lround(*fraction * 100.0)));
}

/// A TCP speed in the controller's units, as speed_type says: Linear [0.001, 3]
/// m/s, Rotational [1, 300] deg/s; the absolute types lie within the
/// controller's own speed parameters, unchecked here.
inline std::expected<double, Error> tcp_speed(sdk::SpeedType type, double speed) {
    switch (type) {
        case sdk::SpeedType::Linear:
            if (!(speed >= 0.001 && speed <= 3.0))
                return invalid(std::format("speed must be in [0.001, 3] m/s, not {} m/s", speed));
            return speed * 1000.0;
        case sdk::SpeedType::Rotational:
            if (!(speed >= deg_to_rad(1.0) && speed <= deg_to_rad(300.0)))
                return invalid(std::format("speed must be in [1, 300] deg/s, not {} deg/s", rad_to_deg(speed)));
            return rad_to_deg(speed);
        case sdk::SpeedType::AbsoluteLinear: return speed * 1000.0;
        case sdk::SpeedType::AbsoluteRotational: return rad_to_deg(speed);
    }
    return invalid("unknown speed type");
}

/// A joint speed as a fraction of the joints' maximum, as the manual's [1, 100] %
/// (moveByJoint takes down to 0.01 %, addPathPoint from 1 %).
inline std::expected<double, Error> joint_speed(double fraction, double low) {
    if (!(fraction >= low && fraction <= 1.0))
        return invalid(std::format("speed must be in [{}, 1], not {}", low, fraction));
    return fraction * 100.0;
}

}  // namespace ec::motion::checks
