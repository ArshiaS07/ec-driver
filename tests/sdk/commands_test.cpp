// The command structs: request encoding against the manual's templates, reply
// decoding against the EA66's real replies, and rejection of replies with the
// wrong shape.

#include <gtest/gtest.h>

#include <expected>
#include <string_view>

#include "ec/sdk/codec.hpp"
#include "ec/sdk/movement.hpp"
#include "ec/sdk/param.hpp"
#include "ec/sdk/system.hpp"

namespace {

using namespace ec::sdk;

// Parses a real response line and decodes it as the reply of command C.
template <Command C>
std::expected<typename C::Reply, Error> decode_line(std::string_view line) {
    const auto response = parse_response(line);
    if (!response) return std::unexpected(response.error());
    if (!response->result) return std::unexpected(response->result.error());
    return decode_reply<C>(*response->result);
}

TEST(Commands, GetSoftVersion) {
    const auto v = decode_line<GetSoftVersion>(R"({"jsonrpc":"2.0","result":"\"V3.15.3\"","id":1})");
    ASSERT_TRUE(v);
    EXPECT_EQ(*v, "V3.15.3");
}

TEST(Commands, GetJointPos) {
    const auto q = decode_line<GetJointPos>(
        R"({"jsonrpc":"2.0","result":"[2.0880681818181817,-90.538739669421489,0.12809405940595298,-90.212962962962962,89.920910493827165,-0.785108024691358]","id":2})");
    ASSERT_TRUE(q);
    EXPECT_DOUBLE_EQ((*q)[0], 2.0880681818181817);
    EXPECT_DOUBLE_EQ((*q)[5], -0.785108024691358);
}

TEST(Commands, GetJointPosRejectsWrongLength) {
    const auto q = decode_line<GetJointPos>(R"({"jsonrpc":"2.0","result":"[1,2,3]","id":2})");
    ASSERT_FALSE(q);
    EXPECT_EQ(q.error().kind, Error::Kind::BadReply);
}

TEST(Commands, GetDH) {
    const auto d = decode_line<GetDH>(R"({"jsonrpc":"2.0","result":"95.431","id":3})");
    ASSERT_TRUE(d);
    EXPECT_DOUBLE_EQ(*d, 95.431);
}

TEST(Commands, GetRobotState) {
    const auto s = decode_line<GetRobotState>(R"({"jsonrpc":"2.0","result":"0","id":4})");
    ASSERT_TRUE(s);
    EXPECT_EQ(*s, ec::RobotState::Stop);
}

TEST(Commands, GetRobotMode) {
    const auto m = decode_line<GetRobotMode>(R"({"jsonrpc":"2.0","result":"2","id":1})");
    ASSERT_TRUE(m);
    EXPECT_EQ(*m, ec::RobotMode::Remote);
}

TEST(Commands, EnumsRejectUndefinedValues) {
    for (const char* line :
         {R"({"jsonrpc":"2.0","result":"7","id":1})", R"({"jsonrpc":"2.0","result":"-1","id":1})",
          R"({"jsonrpc":"2.0","result":"99999999999","id":1})", R"({"jsonrpc":"2.0","result":"1.5","id":1})"}) {
        const auto m = decode_line<GetRobotMode>(line);
        ASSERT_FALSE(m) << line;
        EXPECT_EQ(m.error().kind, Error::Kind::BadReply) << line;
    }
}

TEST(Commands, MoveByJointEncodesInManualOrder) {
    const MoveByJoint mv{.target = {0, -90, 0, -90, 90, 0}, .speed_pct = 10, .acc_pct = 50, .dec_pct = std::nullopt};
    EXPECT_EQ(
        encode(to_request(mv), 8),
        R"({"jsonrpc":"2.0","method":"moveByJoint","params":{"targetPos":[0.0,-90.0,0.0,-90.0,90.0,0.0],"speed":10.0,"acc":50},"id":8})"
        "\n");
}

TEST(Commands, ControllerErrorIsForwarded) {
    const auto stop = decode_line<Stop>(R"({"jsonrpc":"2.0","error":{"code":-32693,"message":"not remote"},"id":9})");
    ASSERT_FALSE(stop);
    EXPECT_EQ(stop.error().kind, Error::Kind::Rejected);
    EXPECT_EQ(stop.error().message, "not remote");
}

TEST(Commands, TrueIsOkFalseIsRefused) {
    const auto ok = decode_line<Stop>(R"({"jsonrpc":"2.0","result":"true","id":1})");
    EXPECT_TRUE(ok);

    const auto refused = decode_line<MoveByJoint>(R"({"jsonrpc":"2.0","result":"false","id":1})");
    ASSERT_FALSE(refused);
    EXPECT_EQ(refused.error().kind, Error::Kind::Refused);

    const auto odd = decode_line<Stop>(R"({"jsonrpc":"2.0","result":"1","id":1})");
    ASSERT_FALSE(odd);
    EXPECT_EQ(odd.error().kind, Error::Kind::BadReply);
}

}  // namespace
