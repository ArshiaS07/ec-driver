// JSON-RPC framing. Reply lines below are copied verbatim from the EA66
// (firmware V3.15.3), so these tests pin down the controller's real format.

#include "ec/command/codec.hpp"

#include <gtest/gtest.h>

#include "ec/command/movement.hpp"
#include "ec/command/param.hpp"

namespace {

using namespace ec::command;

// ------------------------------------------------------------------ encode

TEST(CommandCodec, EncodesWithoutParamsLikeTheManual) {
    EXPECT_EQ(encode(to_request(GetRobotState{}), 4), R"({"jsonrpc":"2.0","method":"getRobotState","id":4})" "\n");
}

TEST(CommandCodec, EncodesParamsInManualOrder) {
    EXPECT_EQ(encode(to_request(GetDH{.index = 0}), 3),
              R"({"jsonrpc":"2.0","method":"getDH","params":{"index":0},"id":3})" "\n");
}

TEST(CommandCodec, EncodedLineEndsWithExactlyOneNewline) {
    const auto line = encode(to_request(Stop{}), 1);
    ASSERT_FALSE(line.empty());
    EXPECT_EQ(line.back(), '\n');
    EXPECT_EQ(line.find('\n'), line.size() - 1);
}

// ------------------------------------------------------------------ parse

TEST(CommandCodec, UnwrapsQuotedString) {
    const auto r = parse_reply(R"({"jsonrpc":"2.0","result":"\"V3.15.3\"","id":1})");
    ASSERT_TRUE(r);
    EXPECT_EQ(r->id, 1u);
    ASSERT_TRUE(r->result);
    EXPECT_EQ(*r->result, Json("V3.15.3"));
}

TEST(CommandCodec, UnwrapsArray) {
    const auto r = parse_reply(
        R"({"jsonrpc":"2.0","result":"[2.0880681818181817,-90.538739669421489,0.12809405940595298,-90.212962962962962,89.920910493827165,-0.785108024691358]","id":2})");
    ASSERT_TRUE(r);
    ASSERT_TRUE(r->result);
    ASSERT_TRUE(r->result->is_array());
    EXPECT_EQ(r->result->size(), 6u);
    EXPECT_DOUBLE_EQ((*r->result)[1].get<double>(), -90.538739669421489);
}

TEST(CommandCodec, UnwrapsNumbers) {
    const auto dh = parse_reply(R"({"jsonrpc":"2.0","result":"95.431","id":3})");
    ASSERT_TRUE(dh && dh->result);
    EXPECT_DOUBLE_EQ(dh->result->get<double>(), 95.431);

    const auto state = parse_reply(R"({"jsonrpc":"2.0","result":"0","id":4})");
    ASSERT_TRUE(state && state->result);
    EXPECT_TRUE(state->result->is_number_integer());
}

TEST(CommandCodec, KeepsUnquotedTextResultAsString) {
    const auto r = parse_reply(R"({"jsonrpc":"2.0","result":"not json","id":7})");
    ASSERT_TRUE(r && r->result);
    EXPECT_EQ(*r->result, Json("not json"));
}

TEST(CommandCodec, ErrorReplyKeepsIdAndMapsCode) {
    const auto r = parse_reply(R"({"jsonrpc":"2.0","error":{"code":-32601,"message":"Method not found."},"id":5})");
    ASSERT_TRUE(r);  // a well-formed error reply parses fine
    EXPECT_EQ(r->id, 5u);
    ASSERT_FALSE(r->result);
    EXPECT_EQ(r->result.error().kind, Error::Kind::MethodNotFound);
    EXPECT_EQ(r->result.error().code, -32601);
    EXPECT_EQ(r->result.error().message, "Method not found.");
}

TEST(CommandCodec, MapsRejectedAndOtherCodes) {
    const auto rejected = parse_reply(R"({"jsonrpc":"2.0","error":{"code":-32693,"message":"x"},"id":1})");
    ASSERT_TRUE(rejected && !rejected->result);
    EXPECT_EQ(rejected->result.error().kind, Error::Kind::Rejected);

    const auto other = parse_reply(R"({"jsonrpc":"2.0","error":{"code":-32000,"message":"y"},"id":1})");
    ASSERT_TRUE(other && !other->result);
    EXPECT_EQ(other->result.error().kind, Error::Kind::Remote);
}

TEST(CommandCodec, AcceptsTrailingNewline) {
    EXPECT_TRUE(parse_reply("{\"jsonrpc\":\"2.0\",\"result\":\"0\",\"id\":4}\n"));
}

TEST(CommandCodec, RejectsLinesThatAreNotReplies) {
    for (const char* line : {"", "garbage", "[1,2]", R"({"jsonrpc":"2.0","result":"0"})",
                             R"({"jsonrpc":"2.0","result":"0","id":"4"})", R"({"jsonrpc":"2.0","id":4})",
                             R"({"jsonrpc":"2.0","error":"oops","id":4})"}) {
        const auto r = parse_reply(line);
        ASSERT_FALSE(r) << line;
        EXPECT_EQ(r.error().kind, Error::Kind::BadReply) << line;
    }
}

TEST(CommandCodec, DecodeForwardsControllerError) {
    const auto r = parse_reply(R"({"jsonrpc":"2.0","error":{"code":-32693,"message":"not remote"},"id":9})");
    ASSERT_TRUE(r);
    const auto stop = decode<Stop>(*r);
    ASSERT_FALSE(stop);
    EXPECT_EQ(stop.error().kind, Error::Kind::Rejected);
}

}  // namespace
