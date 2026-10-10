#pragma once
// Turns the 8055 byte stream into newline-terminated lines.
//
// The command protocol is one JSON message per line. TCP may deliver a reply
// in pieces, or several replies at once; this collects bytes and hands out
// complete lines. Pure: no sockets, no clocks.
//
// A line longer than kMaxLine is not a reply this driver can use (the largest
// documented replies are a few hundred bytes). Instead of growing without
// bound, the framer drops it, skips to the next newline, and counts it.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace ec::command {

class LineFramer {
public:
    static constexpr std::size_t kMaxLine = 64 * 1024;

    void push(std::span<const std::uint8_t> bytes);

    // Next complete line without its '\n' (and without a trailing '\r'),
    // or nullopt if no complete line is buffered.
    [[nodiscard]] std::optional<std::string> pop();

    // Drop buffered bytes, e.g. after a reconnect. Keeps the counter.
    void reset() noexcept;

    [[nodiscard]] std::uint64_t oversized_lines() const noexcept { return oversized_; }

private:
    std::string buffer_;
    bool discarding_ = false;  // inside an oversized line, waiting for its '\n'
    std::uint64_t oversized_ = 0;
};

}  // namespace ec::command
