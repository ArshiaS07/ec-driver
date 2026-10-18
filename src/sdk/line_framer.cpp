#include "ec/sdk/line_framer.hpp"

namespace ec::sdk {

void LineFramer::push(std::span<const std::uint8_t> bytes) {
    for (const std::uint8_t b : bytes) {
        const char c = static_cast<char>(b);
        if (discarding_) {
            if (c == '\n') discarding_ = false;  // the oversized line ends here
            continue;
        }
        buffer_.push_back(c);
        if (c == '\n') {
            partial_ = 0;
        } else if (++partial_ > kMaxLine) {
            buffer_.resize(buffer_.size() - partial_);  // drop the line so far, keep complete lines before it
            partial_ = 0;
            discarding_ = true;
            ++oversized_;
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
    partial_ = 0;
    discarding_ = false;
}

}  // namespace ec::sdk
