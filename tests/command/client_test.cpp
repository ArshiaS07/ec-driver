// Client against a scripted server on 127.0.0.1 (support/loopback_server.hpp).
// Ids start at 1 on every connection, so a test can send a response before the
// call that it answers: the response then waits in the client's socket buffer,
// and each test runs on one thread.

#include "ec/command/client.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <stdexcept>
#include <string>
#include <string_view>

#include "ec/command/movement.hpp"
#include "ec/command/param.hpp"
#include "support/loopback_server.hpp"

namespace {

using namespace std::chrono_literals;
using namespace ec::command;
using ec::test::LoopbackServer;
using Clock = std::chrono::steady_clock;

Client connect_to(const LoopbackServer& server, Client::Options options = {}) {
    options.port = server.port();
    auto client = Client::connect("127.0.0.1", options);
    if (!client) throw std::runtime_error("connect failed: " + client.error().message);
    return std::move(*client);
}

std::string line(std::string_view json) { return std::string(json) + "\n"; }

TEST(Client, SendsTheRequestAndDecodesTheReply) {
    LoopbackServer server;
    Client client = connect_to(server);
    auto peer = server.accept();

    peer.send(line(R"({"jsonrpc":"2.0","result":"95.431","id":1})"));
    const auto dh = client.call(GetDH{.index = 0}, 1s);
    ASSERT_TRUE(dh) << dh.error().message;
    EXPECT_DOUBLE_EQ(*dh, 95.431);
    EXPECT_EQ(peer.receive_line(), R"({"jsonrpc":"2.0","method":"getDH","params":{"index":0},"id":1})");
}

TEST(Client, IdsIncreaseWithEveryCall) {
    LoopbackServer server;
    Client client = connect_to(server);
    auto peer = server.accept();

    peer.send(line(R"({"jsonrpc":"2.0","result":"2","id":1})") + line(R"({"jsonrpc":"2.0","result":"0","id":2})"));
    EXPECT_EQ(client.call(GetRobotMode{}, 1s), ec::RobotMode::Remote);
    EXPECT_EQ(client.call(GetRobotMode{}, 1s), ec::RobotMode::Teach);
    EXPECT_EQ(peer.receive_line(), R"({"jsonrpc":"2.0","method":"getRobotMode","id":1})");
    EXPECT_EQ(peer.receive_line(), R"({"jsonrpc":"2.0","method":"getRobotMode","id":2})");
    EXPECT_EQ(client.stats().calls, 2u);
}

TEST(Client, TimeoutLeavesTheConnectionUsableAndTheLateReplyIsDiscarded) {
    LoopbackServer server;
    Client client = connect_to(server);
    auto peer = server.accept();

    const auto first = client.call(GetRobotMode{}, 100ms);
    ASSERT_FALSE(first);
    EXPECT_EQ(first.error().kind, Error::Kind::Timeout);
    EXPECT_TRUE(client.is_connected());

    peer.send(line(R"({"jsonrpc":"2.0","result":"2","id":1})"));  // the first call's reply, too late
    peer.send(line(R"({"jsonrpc":"2.0","result":"0","id":2})"));
    EXPECT_EQ(client.call(GetRobotMode{}, 1s), ec::RobotMode::Teach);
    EXPECT_EQ(client.stats().timeouts, 1u);
    EXPECT_EQ(client.stats().stale_replies, 1u);
}

TEST(Client, LinesThatAreNotResponsesAreSkippedAndCounted) {
    LoopbackServer server;
    Client client = connect_to(server);
    auto peer = server.accept();

    peer.send(line("garbage") + line(R"({"jsonrpc":"2.0","result":"\"V3.15.3\"","id":1})"));
    const auto v = client.call(Request{"getSoftVersion", nullptr}, 1s);
    ASSERT_TRUE(v) << v.error().message;
    EXPECT_EQ(*v, Json("V3.15.3"));
    EXPECT_EQ(client.stats().bad_lines, 1u);
}

TEST(Client, ErrorWithNullIdIsTheErrorOfTheCallInFlight) {
    LoopbackServer server;
    Client client = connect_to(server);
    auto peer = server.accept();

    peer.send(line(R"({"jsonrpc":"2.0","error":{"code":-32700,"message":"Parse error"},"id":null})"));
    const auto r = client.call(GetRobotMode{}, 1s);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Remote);
    EXPECT_EQ(r.error().code, -32700);
}

TEST(Client, ControllerErrorsAndRefusalsAreReturned) {
    LoopbackServer server;
    Client client = connect_to(server);
    auto peer = server.accept();

    peer.send(line(R"({"jsonrpc":"2.0","error":{"code":-32693,"message":"not in remote mode"},"id":1})"));
    peer.send(line(R"({"jsonrpc":"2.0","result":"false","id":2})"));
    const auto rejected = client.call(Stop{}, 1s);
    ASSERT_FALSE(rejected);
    EXPECT_EQ(rejected.error().kind, Error::Kind::Rejected);
    EXPECT_EQ(rejected.error().message, "not in remote mode");
    const auto refused = client.call(Stop{}, 1s);
    ASSERT_FALSE(refused);
    EXPECT_EQ(refused.error().kind, Error::Kind::Refused);
    EXPECT_TRUE(client.is_connected());
}

TEST(Client, ReplyOfTheWrongTypeIsBadReply) {
    LoopbackServer server;
    Client client = connect_to(server);
    auto peer = server.accept();

    peer.send(line(R"({"jsonrpc":"2.0","result":"\"remote\"","id":1})"));
    const auto r = client.call(GetRobotMode{}, 1s);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::BadReply);
}

TEST(Client, PerCallTimeoutOverridesTheDefault) {
    LoopbackServer server;
    Client client = connect_to(server, Client::Options{.reply_timeout = 10s});
    auto peer = server.accept();

    const auto start = Clock::now();
    const auto r = client.call(GetRobotMode{}, 100ms);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Timeout);
    EXPECT_LT(Clock::now() - start, 5s);
}

TEST(Client, PeerClosingDisconnectsForGood) {
    LoopbackServer server;
    Client client = connect_to(server);
    auto peer = server.accept();
    peer.close();

    const auto first = client.call(GetRobotMode{}, 1s);
    ASSERT_FALSE(first);
    EXPECT_EQ(first.error().kind, Error::Kind::Disconnected);
    EXPECT_FALSE(client.is_connected());

    const auto second = client.call(GetRobotMode{}, 1s);
    ASSERT_FALSE(second);
    EXPECT_EQ(second.error().kind, Error::Kind::Disconnected);
}

TEST(Client, SendThatTimesOutClosesTheConnection) {
    // The peer never reads: large requests fill the socket buffers until a send
    // times out. Part of a request may then be on the wire, so the client must
    // close the connection rather than append the next request to it.
    LoopbackServer server;
    Client client = connect_to(server);
    auto peer = server.accept();

    const Request big{"setBlob", Json{{"blob", std::string(std::size_t{8} << 20, 'a')}}};
    std::expected<Json, Error> r;
    for (int i = 0; i < 16 && client.is_connected(); ++i) r = client.call(big, 100ms);
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().kind, Error::Kind::Disconnected);
    EXPECT_FALSE(client.is_connected());
}

TEST(Client, ConnectFailureIsDisconnected) {
    std::uint16_t port = 0;
    {
        const LoopbackServer server;
        port = server.port();
    }  // nothing listens on `port` any more
    const auto client = Client::connect("127.0.0.1", Client::Options{.port = port, .connect_timeout = 5s});
    ASSERT_FALSE(client);
    EXPECT_EQ(client.error().kind, Error::Kind::Disconnected);
}

}  // namespace
