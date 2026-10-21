#pragma once
/// @file
/// Time as recordings store it.

#include <chrono>
#include <cstdint>

namespace ec::record {

/// A steady-clock time point as nanoseconds since that clock's epoch: the time
/// base of every record (arrival times).
[[nodiscard]] inline std::int64_t to_ns(std::chrono::steady_clock::time_point t) noexcept {
    return static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count());
}

/// The inverse of to_ns().
[[nodiscard]] inline std::chrono::steady_clock::time_point from_ns(std::int64_t ns) noexcept {
    return std::chrono::steady_clock::time_point(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::nanoseconds(ns)));
}

/// The steady clock and the system clock, read together when a recording
/// starts. The steady clock's epoch is arbitrary (often the last boot); with
/// this pair, record times can be placed in calendar time later.
struct ClockPair {
    std::int64_t steady_ns = 0;  ///< steady clock, ns since its epoch
    std::int64_t system_ns = 0;  ///< system clock, ns since 1970-01-01 UTC

    [[nodiscard]] static ClockPair now() noexcept {
        const auto steady = std::chrono::steady_clock::now();
        const auto system = std::chrono::system_clock::now();
        return {to_ns(steady),
                static_cast<std::int64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(system.time_since_epoch()).count())};
    }
};

}  // namespace ec::record
