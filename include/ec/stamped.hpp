#pragma once
/// @file
/// Values with the time they are for, on the driver's one clock.

#include <chrono>

namespace ec {

/// The clock of every time the driver keeps: frame arrivals, the control loop's
/// deadlines, stamps. Monotonic; its epoch is unspecified, so its time points
/// mean something only within one process.
using Clock = std::chrono::steady_clock;

/// A value and the time it is for. `Time` is a Clock::time_point (an instant),
/// or a Clock::duration (the time since the start of a trajectory). An
/// aggregate: `Stamped<Joints>{Clock::now(), q}`.
template <typename T, typename Time = Clock::time_point>
struct Stamped {
    Time stamp{};
    T value{};
};

}  // namespace ec
