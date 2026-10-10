#pragma once
// Deadlines for calls that take a timeout (private to the library).

#include <chrono>

namespace ec::detail {

using Clock = std::chrono::steady_clock;

// now + timeout. Saturates at the clock's maximum instead of overflowing for
// very long timeouts; zero or negative timeouts give `now`.
inline Clock::time_point deadline_after(std::chrono::milliseconds timeout) {
    const auto now = Clock::now();
    if (timeout <= std::chrono::milliseconds::zero()) return now;
    const auto room = std::chrono::floor<std::chrono::milliseconds>(Clock::time_point::max() - now);
    return timeout >= room ? Clock::time_point::max() : now + timeout;
}

// Time left until `deadline`, rounded up to whole milliseconds so that waiting
// that long reaches the deadline; zero once it has passed.
inline std::chrono::milliseconds time_left(Clock::time_point deadline) {
    const auto left = std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now());
    return left < std::chrono::milliseconds::zero() ? std::chrono::milliseconds::zero() : left;
}

}  // namespace ec::detail
