#pragma once
/// @file
/// The record the monitoring stream produces: one frame as received.

#include <chrono>

#include "ec/feedback/frame.hpp"

namespace ec::feedback {

/// One frame as received: its 1024 bytes, untouched, and when they arrived.
/// Lossless on purpose: decoding and unit conversion are up to whoever consumes it.
struct Sample {
    /// When the bytes that completed the frame were received (this PC's steady clock).
    std::chrono::steady_clock::time_point arrival{};
    RawFrame raw{};
};

}  // namespace ec::feedback
