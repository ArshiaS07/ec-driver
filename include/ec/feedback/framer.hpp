#pragma once
// Turns the 8056 byte stream into whole, aligned 1024-byte frames.
//
// TCP has no message boundaries: one receive can return half a frame or
// several. Bytes are buffered and cut into frames by size; each candidate is
// accepted only if it ends with the matching word. If it does not (connected
// mid-frame, corrupted data), the framer discards up to and including the next
// matching word it finds and counts a resync. A payload that happens to contain
// the matching-word bytes can cause one bad cut; the next check fails and the
// framer re-aligns again, so it heals by itself.
//
// Pure: no sockets, no clocks, no threads. Usable for live data and for
// replaying recorded raw bytes.

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "ec/feedback/frame.hpp"

namespace ec::feedback {

class Framer {
public:
    struct Stats {
        std::uint64_t frames = 0;           // aligned frames returned by pop()
        std::uint64_t resyncs = 0;          // times alignment was lost and recovered
        std::uint64_t bytes_discarded = 0;  // bytes dropped while re-aligning
    };

    // Append received bytes.
    void push(std::span<const std::uint8_t> bytes);

    // Next complete frame, or nullopt if more bytes are needed.
    [[nodiscard]] std::optional<RawFrame> pop();

    // Drop buffered bytes, e.g. after a reconnect. Keeps the statistics.
    void reset() noexcept { buffer_.clear(); }

    [[nodiscard]] const Stats& stats() const noexcept { return stats_; }
    [[nodiscard]] std::size_t buffered() const noexcept { return buffer_.size(); }

private:
    // A vector with erase-from-front is O(buffered) per frame. The buffer holds
    // one to a few frames (a few KB), so that is cheaper than a ring buffer's
    // extra code would be worth.
    std::vector<std::uint8_t> buffer_;
    Stats stats_;
};

}  // namespace ec::feedback
