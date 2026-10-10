#include "ec/command/line_framer.hpp"

namespace ec::command {

void LineFramer::push(std::span<const std::uint8_t> bytes) {
    for (const std::uint8_t b : bytes) {
        const char c = static_cast<char>(b);
        if (discarding_) {
            if (c == '\n') discarding_ = false;  // oversized line ends here; resume
            continue;
        }
        buffer_.push_back(c);
        // A line without its newline yet that is already too long: drop it.
        if (buffer_.size() > kMaxLine) {
            const auto last_newline = buffer_.rfind('\n');
            if (last_newline == std::string::npos || buffer_.size() - last_newline - 1 > kMaxLine) {
                buffer_.erase(last_newline == std::string::npos ? 0 : last_newline + 1);
                discarding_ = true;
                ++oversized_;
            }
        }
    }
}

std::optional<std::string> LineFramer::pop() {
    const auto end = buffer_.find('\n');
    if (end == std::string::npos) return std::nullopt;
    std::string line = buffer_.substr(0, end);
    buffer_.erase(0, end + 1);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    return line;
}

void LineFramer::reset() noexcept {
    buffer_.clear();
    discarding_ = false;
}

}  // namespace ec::command
