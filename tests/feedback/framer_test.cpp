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

TEST(FeedbackFramer, GarbageIsDroppedAndTheNextFrameKept) {
    Framer framer;
    std::vector<std::uint8_t> bytes(3000, 0x01);
    framer.push(bytes);
    EXPECT_FALSE(framer.pop().has_value());
    EXPECT_EQ(framer.buffered(), kFrameSize - 1);  // only bytes that may still start a frame

    framer.push(make_frame(7));
    framer.push(make_frame(8));
    std::vector<std::uint64_t> seen;
    while (const auto out = framer.pop()) seen.push_back(timestamp_of(*out));
    EXPECT_EQ(seen, (std::vector<std::uint64_t>{7, 8}));
    EXPECT_EQ(framer.stats().resyncs, 1u);
    EXPECT_EQ(framer.stats().bytes_discarded, 3000u);
}

TEST(FeedbackFramer, MatchingWordInsidePayloadCostsAtMostOneFrame) {
    // Frame 2 carries the matching word in its reserved bytes. Joining the
    // stream mid-frame, the framer cuts once in the wrong place (that "frame"
    // fails to decode) and is aligned again from frame 3 on. Once aligned, the
    // same payload no longer matters.
    auto with_word_inside = [](std::uint64_t ts) {
        auto f = make_frame(ts);
        f[900] = 0xEC;
        f[901] = 0x80;
        f[902] = 0x56;
        f[903] = 0xEC;
        return f;
    };
    Framer framer;
    const auto bytes = concat({make_frame(1), with_word_inside(2), make_frame(3), with_word_inside(4)});
    framer.push(std::span(bytes).subspan(300));

    std::vector<std::uint64_t> decoded;
    int failed = 0;
    while (const auto out = framer.pop()) {
        if (const auto f = decode(*out))
            decoded.push_back(f->timestamp_ms);
        else
            ++failed;
    }
    EXPECT_EQ(decoded, (std::vector<std::uint64_t>{3, 4}));
    EXPECT_EQ(failed, 1);
}

TEST(FeedbackFramer, ResetDropsBufferedBytes) {
    Framer framer;
    const auto f = make_frame();
    framer.push(std::span(f.data(), 500));
    framer.reset();
    EXPECT_EQ(framer.buffered(), 0u);
}

}  // namespace
