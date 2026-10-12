// Receiver against a scripted server on 127.0.0.1: every frame reaches the
// callback in order, lost and silent connections are replaced, and stop() is
// prompt.

#include "ec/feedback/receiver.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "support/frame_builder.hpp"
#include "support/loopback_server.hpp"

namespace {

using namespace std::chrono_literals;
using ec::feedback::RawFrame;
using ec::feedback::Receiver;
using ec::feedback::Sample;
using ec::net::NetError;
using ec::test::LoopbackServer;
using ec::test::make_frame;

std::string as_text(const RawFrame& f) { return {reinterpret_cast<const char*>(f.data()), f.size()}; }

// Short delays, so the tests run quickly; waits of seconds, so slow
// (sanitized) builds still pass.
constexpr auto kPatience = 5s;

Receiver::Options quick(std::uint16_t port) {
    Receiver::Options options;
    options.port = port;
    options.retry_delay = 10ms;
    return options;
}

// What the callback received.
class Frames {
public:
    void add(const Sample& s) {
        {
            const std::lock_guard lock(mutex_);
            frames_.push_back(s.raw);
        }
        added_.notify_all();
    }

    // The frames received, once there are at least `n` (or after kPatience).
    std::vector<RawFrame> wait_for(std::size_t n) {
        std::unique_lock lock(mutex_);
        added_.wait_for(lock, kPatience, [&] { return frames_.size() >= n; });
        return frames_;
    }

private:
    std::mutex mutex_;
    std::condition_variable added_;
    std::vector<RawFrame> frames_;
};

// Whether `condition` holds within kPatience.
bool eventually(const std::function<bool()>& condition) {
    const auto deadline = std::chrono::steady_clock::now() + kPatience;
    while (!condition()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(5ms);
    }
    return true;
}

TEST(FeedbackReceiver, HandsEveryFrameToTheCallbackInOrder) {
    LoopbackServer server;
    Frames frames;
    const Receiver receiver("127.0.0.1", quick(server.port()), [&](const Sample& s) { frames.add(s); });
    auto peer = server.accept(kPatience);
    const RawFrame a = make_frame(1);
    const RawFrame b = make_frame(2);
    const RawFrame c = make_frame(3);

    peer.send(as_text(a) + as_text(b).substr(0, 100));
    peer.send(as_text(b).substr(100) + as_text(c));
    EXPECT_EQ(frames.wait_for(3), (std::vector{a, b, c}));
    ASSERT_TRUE(eventually([&] { return receiver.status().samples == 3; }));
    const auto status = receiver.status();
    EXPECT_TRUE(status.connected);
    EXPECT_EQ(status.connections, 1u);
    EXPECT_EQ(status.framing.frames, 3u);
    EXPECT_FALSE(status.last_error);
}

TEST(FeedbackReceiver, ConnectsAgainWhenTheControllerCloses) {
    LoopbackServer server;
    Frames frames;
    const Receiver receiver("127.0.0.1", quick(server.port()), [&](const Sample& s) { frames.add(s); });
    const RawFrame a = make_frame(1);
    const RawFrame b = make_frame(2);
    {
        auto first = server.accept(kPatience);
        first.send(as_text(a));
        frames.wait_for(1);
    }  // closed here
    auto second = server.accept(kPatience);
    second.send(as_text(b));

    EXPECT_EQ(frames.wait_for(2), (std::vector{a, b}));
    ASSERT_TRUE(eventually([&] { return receiver.status().samples == 2; }));
    const auto status = receiver.status();
    EXPECT_TRUE(status.connected);
    EXPECT_EQ(status.connections, 2u);
    EXPECT_EQ(status.framing.frames, 2u);  // over both connections
    ASSERT_TRUE(status.last_error);
    EXPECT_EQ(status.last_error->kind, NetError::Kind::Closed);
}

TEST(FeedbackReceiver, ReplacesAConnectionThatFallsSilent) {
    LoopbackServer server;
    auto options = quick(server.port());
    options.silence_timeout = 100ms;
    const Receiver receiver("127.0.0.1", options, [](const Sample&) {});

    auto silent = server.accept(kPatience);       // stays open and sends nothing
    auto replacement = server.accept(kPatience);  // throws unless the receiver connects again
    ASSERT_TRUE(eventually([&] { return receiver.status().connections >= 2; }));
    const auto status = receiver.status();
    ASSERT_TRUE(status.last_error);
    EXPECT_EQ(status.last_error->kind, NetError::Kind::Timeout);
}

TEST(FeedbackReceiver, StopIsPromptWhileWaitingToRetry) {
    std::uint16_t port = 0;
    {
        const LoopbackServer gone;
        port = gone.port();
    }  // nothing listens there any more
    auto options = quick(port);
    options.retry_delay = 10s;
    Receiver receiver("127.0.0.1", options, [](const Sample&) {});
    ASSERT_TRUE(eventually([&] { return receiver.status().last_error.has_value(); }));

    const auto start = std::chrono::steady_clock::now();
    receiver.stop();
    EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
    const auto status = receiver.status();
    EXPECT_FALSE(status.connected);
    EXPECT_EQ(status.connections, 0u);
    EXPECT_EQ(status.last_error->kind, NetError::Kind::Connect);
}

TEST(FeedbackReceiver, ACallbackThatThrowsStopsTheReceiver) {
    LoopbackServer server;
    const Receiver receiver("127.0.0.1", quick(server.port()), [](const Sample&) { throw std::runtime_error("boom"); });
    auto peer = server.accept(kPatience);
    peer.send(as_text(make_frame(1)) + as_text(make_frame(2)));

    ASSERT_TRUE(eventually([&] { return !receiver.status().failure.empty(); }));
    const auto status = receiver.status();
    EXPECT_EQ(status.failure, "the callback threw: boom");
    EXPECT_FALSE(status.connected);
    EXPECT_EQ(status.samples, 0u);
}

TEST(FeedbackReceiver, StopFromTheCallbackEndsTheThreadAfterIt) {
    LoopbackServer server;
    std::atomic<Receiver*> self{nullptr};
    std::atomic<int> calls{0};
    Receiver receiver("127.0.0.1", quick(server.port()), [&](const Sample&) {
        ++calls;
        if (Receiver* r = self.load()) r->stop();
    });
    self.store(&receiver);
    auto peer = server.accept(kPatience);
    peer.send(as_text(make_frame(1)) + as_text(make_frame(2)));

    ASSERT_TRUE(eventually([&] { return calls > 0 && !receiver.status().connected; }));
    std::this_thread::sleep_for(50ms);
    EXPECT_EQ(calls, 1);  // the second frame was never handed over
}

}  // namespace
