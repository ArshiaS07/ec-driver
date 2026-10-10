#include "ec/feedback/framer.hpp"

#include <algorithm>

namespace ec::feedback {
namespace {

bool matching_word_at(const std::vector<std::uint8_t>& buf, std::size_t offset) {
    return is_matching_word(std::span<const std::uint8_t, 4>(buf.data() + offset, 4));
}

}  // namespace

void Framer::push(std::span<const std::uint8_t> bytes) {
    buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
}

std::optional<RawFrame> Framer::pop() {
    while (buffer_.size() >= kFrameSize) {
        if (matching_word_at(buffer_, kMatchOffset)) {
            RawFrame frame{};
            std::copy_n(buffer_.begin(), kFrameSize, frame.begin());
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(kFrameSize));
            ++stats_.frames;
            return frame;
        }

        // Misaligned. A matching word marks the end of some frame, so the next
        // frame starts right after the first one found.
        std::size_t drop = buffer_.size() - 3;  // none found: keep a possible partial word
        for (std::size_t i = 0; i + 4 <= buffer_.size(); ++i) {
            if (matching_word_at(buffer_, i)) {
                drop = i + 4;
                ++stats_.resyncs;
                break;
            }
        }
        stats_.bytes_discarded += drop;
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(drop));
    }
    return std::nullopt;
}

}  // namespace ec::feedback
