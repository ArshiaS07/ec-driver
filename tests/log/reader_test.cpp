// ec::log::read against a scripted log port on 127.0.0.1: the manual's
// exchange (type, count or "all", reply, "exit"), line ends, an empty reply,
// a port that is not there.

#include "ec/log/reader.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "support/loopback_server.hpp"

namespace {

using namespace std::chrono_literals;
using ec::test::LoopbackServer;

ec::log::Options quick(std::uint16_t port) {
    ec::log::Options o;
    o.port = port;
    o.reply_within = 1s;
    o.quiet = 100ms;
    return o;
}

// Serves one exchange: records the three lines it is sent, answers `reply`.
std::jthread serve(LoopbackServer& server, std::string reply, std::vector<std::string>& received) {
    return std::jthread([&server, &received, reply = std::move(reply)] {
        auto peer = server.accept(std::chrono::milliseconds{5'000});
        received.push_back(peer.receive_line());
        received.push_back(peer.receive_line());
        if (!reply.empty()) peer.send(reply);
        received.push_back(peer.receive_line());
    });
}

TEST(LogReader, SendsTheTypeAndCountAndReturnsTheLines) {
    LoopbackServer server;
    std::vector<std::string> received;
    std::vector<std::string> lines;
    {
        const auto thread = serve(server, "first entry\r\nsecond entry\nthird, no end", received);
        const auto read =
            ec::log::read("127.0.0.1", {.level = ec::log::Level::Warning, .lines = 10}, quick(server.port()));
        ASSERT_TRUE(read) << read.error().message();
        lines = *read;
    }
    EXPECT_EQ(received, (std::vector<std::string>{"Type=Warning", "10", "exit"}));
    EXPECT_EQ(lines, (std::vector<std::string>{"first entry", "second entry", "third, no end"}));
}

TEST(LogReader, AllAndAnEmptyReply) {
    LoopbackServer server;
    std::vector<std::string> received;
    {
        const auto thread = serve(server, "", received);
        const auto read =
            ec::log::read("127.0.0.1", {.level = ec::log::Level::Error, .lines = std::nullopt}, quick(server.port()));
        ASSERT_TRUE(read) << read.error().message();
        EXPECT_TRUE(read->empty());
    }
    EXPECT_EQ(received, (std::vector<std::string>{"Type=Error", "all", "exit"}));
}

TEST(LogReader, NoPortIsAnError) {
    std::uint16_t closed = 0;
    {
        const LoopbackServer gone;
        closed = gone.port();
    }
    const auto read = ec::log::read("127.0.0.1", {}, quick(closed));
    ASSERT_FALSE(read);
    EXPECT_EQ(read.error().kind, ec::net::NetError::Kind::Connect);
}

}  // namespace
