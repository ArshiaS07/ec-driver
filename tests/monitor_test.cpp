// Monitor against a scripted 8056 on 127.0.0.1: every frame reaches on_sample,
// every decoded one becomes the newest State (numbered, handed to on_state),
// frames that do not decode are counted and skipped.

#include "ec/monitor.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "support/frame_builder.hpp"
#include "support/loopback_server.hpp"

namespace {

using namespace std::chrono_literals;
using ec::Monitor;

constexpr auto kPatience = 5s;

std::string as_text(const ec::feedback::RawFrame& f) { return {reinterpret_cast<const char*>(f.data()), f.size()}; }

TEST(Monitor, IdleReceivesNothing) {
    const Monitor monitor;
    EXPECT_FALSE(monitor.latest());
    EXPECT_FALSE(monitor.next(0, 10ms));
    const auto status = monitor.status();
    EXPECT_FALSE(status.receiver.connected);
    EXPECT_EQ(status.states, 0u);
}

TEST(Monitor, EveryDecodedFrameBecomesTheNewestState) {
    ec::test::LoopbackServer server;
    std::atomic<int> samples{0};
    std::atomic<int> states{0};
    std::vector<std::uint64_t> seqs;  // on the monitor's thread; read after it is stopped
    {
        Monitor::Options options;
        options.receiver.port = server.port();
        options.on_sample = [&](const ec::feedback::Sample&) { ++samples; };
        options.on_state = [&](const ec::State& s) {
            seqs.push_back(s.seq);
            ++states;
        };
        const Monitor monitor("127.0.0.1", std::move(options));
        auto peer = server.accept(std::chrono::duration_cast<std::chrono::milliseconds>(kPatience));

        ec::test::FrameFields fields;
        fields.joints_deg = {1, 2, 3, 4, 5, 6};
        peer.send(as_text(ec::test::make_frame(fields)));
        const auto first = monitor.next(0, kPatience);
        ASSERT_TRUE(first);
        EXPECT_EQ(first->seq, 1u);
        EXPECT_LT(first->age(), kPatience);
        EXPECT_DOUBLE_EQ(first->joints[5].position, ec::deg_to_rad(6.0));
        EXPECT_EQ(first->system.mode, ec::RobotMode::Remote);

        auto bad = ec::test::make_frame(fields);
        bad[2] = 0;  // message size 1024 -> 0: framed (the matching word is right), not decoded
        peer.send(as_text(bad));
        fields.joints_deg[5] = 7;
        peer.send(as_text(ec::test::make_frame(fields)));
        const auto second = monitor.next(first->seq, kPatience);
        ASSERT_TRUE(second);
        EXPECT_EQ(second->seq, 2u);
        EXPECT_DOUBLE_EQ(second->joints[5].position, ec::deg_to_rad(7.0));
        EXPECT_EQ(monitor.latest()->seq, 2u);
        EXPECT_FALSE(monitor.next(second->seq, 50ms));  // nothing newer

        const auto status = monitor.status();
        EXPECT_TRUE(status.receiver.connected);
        EXPECT_EQ(status.states, 2u);
        EXPECT_EQ(status.undecodable, 1u);
    }
    EXPECT_EQ(samples.load(), 3);
    EXPECT_EQ(states.load(), 2);
    EXPECT_EQ(seqs, (std::vector<std::uint64_t>{1, 2}));
}

}  // namespace
