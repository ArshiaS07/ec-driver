#pragma once
/// @file
/// Splits the port-8055 byte stream into newline-terminated lines.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace ec::command {

/// Collects received bytes and hands out complete lines, however TCP split or
/// joined them. Pure: no sockets, no clocks, no threads.
///
/// A line longer than kMaxLine cannot be a reply this driver uses (the largest
/// documented replies are a few hundred bytes): instead of growing without
/// bound, the framer drops it up to its newline and counts it.
class LineFramer {
public:
    static constexpr std::size_t kMaxLine = std::size_t{64} * 1024;  ///< bytes, without the newline

    /// Appends received bytes.
    void push(std::span<const std::uint8_t> bytes);

    /// The next complete line without its '\n' (and without a trailing '\r'), or
    /// nullopt if no complete line is buffered.
    [[nodiscard]] std::optional<std::string> pop();

    /// Drops all buffered bytes, e.g. after a reconnect. Keeps the counter.
    void reset() noexcept;

    /// Number of lines dropped for exceeding kMaxLine.
    [[nodiscard]] std::uint64_t oversized_lines() const noexcept { return oversized_; }

private:
    std::string buffer_;       // complete lines, then the current unterminated line
    std::size_t partial_ = 0;  // length of the current unterminated line at the end of buffer_
    bool discarding_ = false;  // inside an oversized line, waiting for its '\n'
    std::uint64_t oversized_ = 0;
};

}  // namespace ec::command
