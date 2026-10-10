#include "ec/feedback/framer.hpp"

#include <algorithm>
#include <cstddef>

namespace ec::feedback {
namespace {

bool matching_word_at(const std::vector<std::uint8_t>& buf, std::size_t offset) {
    return is_matching_word(std::span<const std::uint8_t, 4>(buf.data() + offset, 4));
}

std::ptrdiff_t as_offset(std::size_t n) { return static_cast<std::ptrdiff_t>(n); }

}  // namespace

void Framer::push(std::span<const std::uint8_t> bytes) { buffer_.insert(buffer_.end(), bytes.begin(), bytes.end()); }

std::optional<RawFrame> Framer::pop() {
    // The first matching word at kMatchOffset or later ends a frame.
    for (std::size_t end = kMatchOffset; end + 4 <= buffer_.size(); ++end) {
        if (!matching_word_at(buffer_, end)) continue;
        const std::size_t start = end - kMatchOffset;
        if (start > 0) {
            ++stats_.resyncs;
            stats_.bytes_discarded += start;
        }
        RawFrame frame{};
        std::copy_n(buffer_.begin() + as_offset(start), kFrameSize, frame.begin());
        buffer_.erase(buffer_.begin(), buffer_.begin() + as_offset(start + kFrameSize));
        ++stats_.frames;
        return frame;
    }

    // No frame ends in the buffer yet. Only its last kFrameSize - 1 bytes can
    // still be the start of one; anything before them is dropped.
    if (buffer_.size() >= kFrameSize) {
        const std::size_t drop = buffer_.size() - (kFrameSize - 1);
        stats_.bytes_discarded += drop;
        buffer_.erase(buffer_.begin(), buffer_.begin() + as_offset(drop));
    }
    return std::nullopt;
}

}  // namespace ec::feedback
