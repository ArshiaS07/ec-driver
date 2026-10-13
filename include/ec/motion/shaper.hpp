#pragma once
/// @file
/// Shapes a stream's targets into setpoints the arm can follow: jumps become
/// ramps, and a hold becomes a smooth stop (docs/motion.md 5.5). Pure.

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "ec/units.hpp"

namespace ec::motion {

/// Moves a setpoint toward a target one period at a time, each joint within its
/// velocity and acceleration limits, and lands on the target without
/// overshooting it. The joints are limited independently: the path between far
/// targets is not straight in joint space, which is why applications should
/// push targets close to each other and leave the limiter to absorb the steps.
class RateLimiter {
public:
    struct Limits {
        Joints max_velocity{};      ///< [rad/s], each > 0
        Joints max_acceleration{};  ///< [rad/s^2], each > 0
    };

    RateLimiter(const Limits& limits, double period_s) : limits_(limits), dt_(period_s) {}

    /// Starts at rest at `position`.
    void reset(const Joints& position) noexcept {
        position_ = position;
        velocity_ = {};
    }

    /// The setpoint one period closer to `target`. Per joint, the speed is the
    /// fastest from which the joint can still stop on the target by
    /// decelerating at its limit, so it lands on it exactly.
    const Joints& next(const Joints& target) noexcept {
        for (std::size_t j = 0; j < kJointCount; ++j) {
            const double error = target[j] - position_[j];
            // Work toward the target: distance >= 0, speed positive toward it.
            const double sign = error > 0.0 || (error == 0.0 && velocity_[j] >= 0.0) ? 1.0 : -1.0;
            const double distance = std::abs(error);
            const double v0 = sign * velocity_[j];
            const double dv = limits_.max_acceleration[j] * dt_;
            const double lo = v0 - dv;
            const double hi = std::max(lo, std::min(limits_.max_velocity[j], v0 + dv));
            double v = lo;  // if even the hardest braking overshoots, brake hardest
            if (travel(hi, dv) <= distance) {
                v = hi;
            } else if (travel(lo, dv) <= distance) {
                double a = lo;  // travel(a) <= distance < travel(b)
                double b = hi;
                for (int k = 0; k < 60; ++k) {
                    const double m = (a + b) / 2.0;
                    (travel(m, dv) <= distance ? a : b) = m;
                }
                v = a;
            }
            if (std::abs(v) < 1e-12) v = 0.0;  // the bisection's leftover at rest
            const double step = v * dt_;
            if (std::abs(distance - step) < 1e-12) {
                position_[j] = target[j];  // landed: the rounding error is dropped
            } else {
                position_[j] += sign * step;
            }
            velocity_[j] = sign * v;
        }
        return position_;
    }

    /// Takes `setpoint` as the next one as it is, without limiting it, and keeps
    /// the speed it implies, so that brake() stops from there: for setpoints
    /// planned within the limits elsewhere (FollowTrajectory).
    const Joints& follow(const Joints& setpoint) noexcept {
        for (std::size_t j = 0; j < kJointCount; ++j) {
            velocity_[j] = (setpoint[j] - position_[j]) / dt_;
            position_[j] = setpoint[j];
        }
        return position_;
    }

    /// The setpoint one period further along the fastest stop: every joint
    /// decelerates at its limit until it rests.
    const Joints& brake() noexcept {
        for (std::size_t j = 0; j < kJointCount; ++j) {
            const double dv = limits_.max_acceleration[j] * dt_;
            const double v = std::abs(velocity_[j]) <= dv ? 0.0 : velocity_[j] - std::copysign(dv, velocity_[j]);
            position_[j] += v * dt_;
            velocity_[j] = v;
        }
        return position_;
    }

    [[nodiscard]] const Joints& position() const noexcept { return position_; }
    [[nodiscard]] const Joints& velocity() const noexcept { return velocity_; }
    [[nodiscard]] bool at_rest() const noexcept { return max_abs(velocity_) == 0.0; }

private:
    // How far a joint moves toward the target if it moves at `v` this period
    // and then brakes by `dv` per period until it rests: v dt plus the
    // stopping distance (negative when v points away from the target).
    [[nodiscard]] double travel(double v, double dv) const noexcept {
        const double speed = std::abs(v);
        const double n = std::floor(speed / dv);  // further periods with speed left
        const double stopping = dt_ * (n * speed - dv * n * (n + 1.0) / 2.0);
        return v * dt_ + std::copysign(stopping, v);
    }

    Limits limits_;
    double dt_;
    Joints position_{};
    Joints velocity_{};
};

}  // namespace ec::motion
