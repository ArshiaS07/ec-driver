// Framer: whatever way the byte stream is chopped up by TCP, every frame comes
// out whole, in order, and misalignment is recovered.

#include "ec/feedback/framer.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "support/frame_builder.hpp"

namespace {

using namespace ec::feedback;
using ec::test::make_frame;

std::vector<std::uint8_t> concat(std::initializer_list<RawFrame> frames) {
    std::vector<std::uint8_t> out;
    for (const auto& f : frames) out.insert(out.end(), f.begin(), f.end());
    return out;
}

std::uint64_t timestamp_of(const RawFrame& raw) {
    const auto f = decode(raw);
    return f ? f->timestamp_ms : 0;
}

TEST(FeedbackFramer, NeedsAWholeFrame) {
    Framer framer;
    const auto f = make_frame();
    framer.push(std::span(f.data(), kFrameSize - 1));
    EXPECT_FALSE(framer.pop().has_value());
    framer.push(std::span(f.data() + kFrameSize - 1, 1));
    const auto out = framer.pop();
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(*out, f);
    EXPECT_EQ(framer.buffered(), 0u);
}

TEST(FeedbackFramer, SeveralFramesInOnePush) {
    Framer framer;
    const auto bytes = concat({make_frame(1), make_frame(2), make_frame(3)});
    framer.push(bytes);
    for (std::uint64_t ts : {1u, 2u, 3u}) {
        const auto out = framer.pop();
        ASSERT_TRUE(out.has_value());
        EXPECT_EQ(timestamp_of(*out), ts);
    }
    EXPECT_FALSE(framer.pop().has_value());
    EXPECT_EQ(framer.stats().frames, 3u);
    EXPECT_EQ(framer.stats().resyncs, 0u);
}

TEST(FeedbackFramer, ByteByByte) {
    Framer framer;
    const auto bytes = concat({make_frame(1), make_frame(2)});
    std::vector<std::uint64_t> seen;
    for (std::uint8_t b : bytes) {
        framer.push(std::span(&b, 1));
        while (const auto out = framer.pop()) seen.push_back(timestamp_of(*out));
    }
    EXPECT_EQ(seen, (std::vector<std::uint64_t>{1, 2}));
}

TEST(FeedbackFramer, ConnectedMidFrameResyncs) {
    // Typical on connect: the stream starts somewhere inside a frame.
    Framer framer;
    const auto bytes = concat({make_frame(1), make_frame(2), make_frame(3)});
    framer.push(std::span(bytes).subspan(300));  // frame 1 is cut
    std::vector<std::uint64_t> seen;
    while (const auto out = framer.pop()) seen.push_back(timestamp_of(*out));
    EXPECT_EQ(seen, (std::vector<std::uint64_t>{2, 3}));
    EXPECT_EQ(framer.stats().resyncs, 1u);
    EXPECT_EQ(framer.stats().bytes_discarded, kFrameSize - 300);
}

TEST(FeedbackFramer, GarbageWithoutMatchingWordIsDropped) {
    Framer framer;
    std::vector<std::uint8_t> bytes(3000, 0x01);
    framer.push(bytes);
    EXPECT_FALSE(framer.pop().has_value());
    EXPECT_LT(framer.buffered(), 4u);  // only a possible partial word is kept

    const auto f = make_frame(7);
    framer.push(f);
    // The good frame arrives after garbage with no matching word in it: the
    // framer re-aligns on the frame's own matching word, loses that frame, and
    // is aligned for the next one.
    const auto g = make_frame(8);
    framer.push(g);
    std::vector<std::uint64_t> seen;
    while (const auto out = framer.pop()) seen.push_back(timestamp_of(*out));
    EXPECT_EQ(seen, (std::vector<std::uint64_t>{8}));
}

TEST(FeedbackFramer, ResetDropsBufferedBytes) {
    Framer framer;
    const auto f = make_frame();
    framer.push(std::span(f.data(), 500));
    framer.reset();
    EXPECT_EQ(framer.buffered(), 0u);
}

}  // namespace
