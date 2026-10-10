#pragma once
/// @file
/// Splits the port-8056 byte stream into whole, aligned 1024-byte frames.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "ec/feedback/frame.hpp"

namespace ec::feedback {

/// Collects received bytes and hands out whole frames, however TCP split or
/// joined them. Pure: no sockets, no clocks, no threads; usable for live data and
/// for replaying recorded bytes.
///
/// One rule finds frames: a frame is the 1024 bytes that end with the first
/// matching word at offset 1020 or later. While aligned, that word is at 1020
/// exactly. After joining a stream mid-frame or after corrupted data, the bytes
/// before the frame are dropped and counted as a resync. A payload that happens
/// to contain the matching word can cause one wrong cut, which the next frame's
/// matching word corrects.
class Framer {
public:
    struct Stats {
        std::uint64_t frames = 0;           ///< frames returned by pop()
        std::uint64_t resyncs = 0;          ///< times alignment was lost and found again
        std::uint64_t bytes_discarded = 0;  ///< bytes dropped because no frame contains them
    };

    /// Appends received bytes.
    void push(std::span<const std::uint8_t> bytes);

    /// The next whole frame, or nullopt if more bytes are needed.
    [[nodiscard]] std::optional<RawFrame> pop();

    /// Drops all buffered bytes, e.g. after a reconnect. Keeps the statistics.
    void reset() noexcept { buffer_.clear(); }

    [[nodiscard]] const Stats& stats() const noexcept { return stats_; }

    /// Bytes waiting for the rest of their frame; at most a frame's worth after pop().
    [[nodiscard]] std::size_t buffered() const noexcept { return buffer_.size(); }

private:
    std::vector<std::uint8_t> buffer_;
    Stats stats_;
};

}  // namespace ec::feedback
