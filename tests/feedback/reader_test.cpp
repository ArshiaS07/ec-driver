// Reader against a scripted server on 127.0.0.1: whole frames come out, in
// order, however TCP cut them, each with its arrival time.

#include "ec/feedback/reader.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <string_view>

#include "support/frame_builder.hpp"
#include "support/loopback_server.hpp"

namespace {

using namespace std::chrono_literals;
using ec::feedback::RawFrame;
using ec::feedback::Reader;
using ec::net::NetError;
using ec::test::LoopbackServer;
using ec::test::make_frame;

std::string_view as_text(const RawFrame& f) { return {reinterpret_cast<const char*>(f.data()), f.size()}; }

Reader connect_to(const LoopbackServer& server) {
    auto reader = Reader::connect("127.0.0.1", server.port(), 2s);
    if (!reader) throw std::runtime_error("connect failed: " + reader.error().message());
    return std::move(*reader);
}

TEST(FeedbackReader, HandsOutWholeFramesHoweverTheyAreCut) {
    LoopbackServer server;
    Reader reader = connect_to(server);
    auto peer = server.accept();
    const RawFrame a = make_frame(1);
    const RawFrame b = make_frame(2);

    peer.send(as_text(a).substr(0, 300));
    peer.send(std::string(as_text(a).substr(300)) + std::string(as_text(b)));
    const auto first = reader.next(1s);
    ASSERT_TRUE(first) << first.error().message();
    EXPECT_EQ(first->raw, a);
    const auto second = reader.next(1s);
    ASSERT_TRUE(second) << second.error().message();
    EXPECT_EQ(second->raw, b);
    EXPECT_GE(second->arrival, first->arrival);
    EXPECT_EQ(reader.stats().frames, 2u);
}

TEST(FeedbackReader, PartialFrameTimesOutAndReadingCarriesOn) {
    LoopbackServer server;
    Reader reader = connect_to(server);
    auto peer = server.accept();
    const RawFrame a = make_frame(1);

    peer.send(as_text(a).substr(0, 500));
    const auto partial = reader.next(100ms);
    ASSERT_FALSE(partial);
    EXPECT_EQ(partial.error().kind, NetError::Kind::Timeout);

    peer.send(as_text(a).substr(500));
    const auto whole = reader.next(1s);
    ASSERT_TRUE(whole) << whole.error().message();
    EXPECT_EQ(whole->raw, a);
}

TEST(FeedbackReader, PeerClosingIsClosed) {
    LoopbackServer server;
    Reader reader = connect_to(server);
    auto peer = server.accept();
    peer.close();

    const auto r = reader.next(1s);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, NetError::Kind::Closed);
}

}  // namespace
