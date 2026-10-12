// Latest: the newest value with its sequence number, and waiting for the next
// one across threads.

#include "ec/latest.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <thread>

namespace {

using namespace std::chrono_literals;

TEST(Latest, IsEmptyUntilTheFirstPublish) {
    const ec::Latest<int> latest;
    EXPECT_FALSE(latest.get());
    EXPECT_FALSE(latest.wait_newer(0, 10ms));
}

TEST(Latest, KeepsTheNewestAndCountsEveryPublish) {
    ec::Latest<int> latest;
    latest.publish(7);
    latest.publish(8);
    const auto e = latest.get();
    ASSERT_TRUE(e);
    EXPECT_EQ(e->seq, 2u);
    EXPECT_EQ(e->value, 8);
}

TEST(Latest, WaitNewerReturnsAtOnceWhenANewerValueIsThere) {
    ec::Latest<int> latest;
    latest.publish(1);
    const auto e = latest.wait_newer(0, 0ms);
    ASSERT_TRUE(e);
    EXPECT_EQ(e->value, 1);
    EXPECT_FALSE(latest.wait_newer(e->seq, 20ms));  // nothing newer than what was seen
}

TEST(Latest, WaitNewerWakesWhenAnotherThreadPublishes) {
    ec::Latest<int> latest;
    const std::jthread writer([&] {
        std::this_thread::sleep_for(20ms);
        latest.publish(42);
    });
    const auto e = latest.wait_newer(0, 5s);
    ASSERT_TRUE(e);
    EXPECT_EQ(e->value, 42);
}

TEST(Latest, AReaderSeesWholeValuesInOrderAndTheLastOne) {
    struct Pair {
        std::uint64_t a;
        std::uint64_t b;  // equal to `a` when published: a torn copy would differ
    };
    constexpr std::uint64_t kCount = 20000;
    ec::Latest<Pair> latest;
    const std::jthread writer([&] {
        for (std::uint64_t i = 1; i <= kCount; ++i) latest.publish({i, i});
    });
    std::uint64_t seen = 0;
    while (seen < kCount) {
        const auto e = latest.wait_newer(seen, 5s);
        ASSERT_TRUE(e);
        ASSERT_GT(e->seq, seen);
        ASSERT_EQ(e->value.a, e->seq);
        ASSERT_EQ(e->value.b, e->seq);
        seen = e->seq;
    }
}

TEST(Latest, AnUnboundedTimeoutDoesNotOverflow) {
    ec::Latest<int> latest;
    latest.publish(1);
    EXPECT_TRUE(latest.wait_newer(0, std::chrono::nanoseconds::max()));
}

}  // namespace
