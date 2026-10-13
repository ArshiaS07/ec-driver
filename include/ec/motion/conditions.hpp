#pragma once
/// @file
/// The conditions the controller's state must meet for an activity to start or
/// to keep running, recomputed from the newest monitoring frame every cycle
/// (docs/motion.md 5.6). Pure: no I/O, no clocks.

#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <string_view>

#include "ec/feedback/state.hpp"

namespace ec::motion {

/// One condition on the controller's state.
enum class Condition : std::uint8_t {
    Fresh,            ///< the newest state is younger than Thresholds::fresh_within
    Remote,           ///< the robot is in remote mode (the pendant's key switch)
    NoEmergencyStop,  ///< the E-stop flag is clear and the state is not emergency stop
    NoAlarm,          ///< the state is not alarm
    NoCollision,      ///< the collision flag is clear and the state is not collision
    ServoOn,          ///< the servos are on (brakes released)
    Synced,           ///< the encoders are synchronised
    AtRest,           ///< every joint is slower than Thresholds::rest_speed, whatever the state
    Standstill,       ///< AtRest, and the state is stop: ready for the next motion
};

inline constexpr int kConditionCount = 9;

/// Lower-case name, e.g. "servo on".
[[nodiscard]] std::string_view to_string(Condition c) noexcept;

/// A set of conditions.
class Conditions {
public:
    constexpr Conditions() = default;
    constexpr Conditions(std::initializer_list<Condition> conditions) noexcept {
        for (Condition c : conditions) bits_ |= bit(c);
    }

    [[nodiscard]] constexpr bool has(Condition c) const noexcept { return (bits_ & bit(c)) != 0; }
    [[nodiscard]] constexpr bool empty() const noexcept { return bits_ == 0; }
    /// Every condition of `other` is in this set.
    [[nodiscard]] constexpr bool contains(Conditions other) const noexcept {
        return (bits_ & other.bits_) == other.bits_;
    }
    /// The conditions of this set that `held` lacks.
    [[nodiscard]] constexpr Conditions missing_from(Conditions held) const noexcept {
        return from_bits(bits_ & ~held.bits_);
    }
    constexpr Conditions& insert(Condition c) noexcept {
        bits_ |= bit(c);
        return *this;
    }

    friend constexpr Conditions operator|(Conditions a, Conditions b) noexcept { return from_bits(a.bits_ | b.bits_); }
    friend constexpr Conditions operator&(Conditions a, Conditions b) noexcept { return from_bits(a.bits_ & b.bits_); }
    constexpr bool operator==(const Conditions&) const noexcept = default;

private:
    static constexpr std::uint32_t bit(Condition c) noexcept { return std::uint32_t{1} << static_cast<unsigned>(c); }
    static constexpr Conditions from_bits(std::uint32_t bits) noexcept {
        Conditions c;
        c.bits_ = bits;
        return c;
    }
    std::uint32_t bits_ = 0;
};

/// The names of a set's conditions, comma-separated; "none" for an empty set.
[[nodiscard]] std::string to_string(Conditions conditions);

/// What the arm needs to move at all.
inline constexpr Conditions kCanMove{Condition::Fresh,   Condition::Remote,      Condition::NoEmergencyStop,
                                     Condition::NoAlarm, Condition::NoCollision, Condition::ServoOn,
                                     Condition::Synced};

/// What a new motion needs: kCanMove, from standstill.
inline constexpr Conditions kCanStartMotion = kCanMove | Conditions{Condition::Standstill};

/// The thresholds the conditions are judged by (docs/motion.md 7.2).
struct Thresholds {
    std::chrono::nanoseconds fresh_within{std::chrono::milliseconds{50}};  ///< about 6 frames
    double rest_speed = 0.0087;                                            ///< [rad/s], 0.5 deg/s on every joint
};

/// The conditions that hold for `state`, `age` after it arrived. A status value
/// the manual does not define (ADR 0007) satisfies none of the conditions that
/// depend on it.
[[nodiscard]] Conditions observe(const feedback::State& state, std::chrono::nanoseconds age,
                                 const Thresholds& thresholds) noexcept;

}  // namespace ec::motion
