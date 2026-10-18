// ReplyDecoder: every reply type decodes what it should and rejects, without
// throwing, what it should not. Plus the Command concept's rules for optional
// params() and decode().

#include "ec/sdk/reply.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <expected>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ec/sdk/request.hpp"
#include "ec/status.hpp"

namespace {

using namespace ec::sdk;

template <typename T>
std::expected<T, Error> decode(std::string_view json) {
    return ReplyDecoder<T>::decode(Json::parse(json));
}

template <typename T>
void expect_bad_reply(std::string_view json) {
    const auto r = decode<T>(json);
    ASSERT_FALSE(r) << json;
    EXPECT_EQ(r.error().kind, Error::Kind::BadReply) << json;
}

TEST(ReplyDecoder, OkIsTrueAndFalseIsRefused) {
    EXPECT_TRUE(decode<Ok>("true"));
    const auto refused = decode<Ok>("false");
    ASSERT_FALSE(refused);
    EXPECT_EQ(refused.error().kind, Error::Kind::Refused);
    expect_bad_reply<Ok>("1");
}

TEST(ReplyDecoder, BoolFalseIsAValueNotAFailure) {
    EXPECT_EQ(decode<bool>("false"), false);
    EXPECT_EQ(decode<bool>("true"), true);
    expect_bad_reply<bool>("0");
}

TEST(ReplyDecoder, IntegersAreRangeChecked) {
    EXPECT_EQ(decode<int>("-5"), -5);
    EXPECT_EQ(decode<std::uint8_t>("255"), 255);
    expect_bad_reply<int>("3000000000");
    expect_bad_reply<std::uint8_t>("256");
    expect_bad_reply<std::uint32_t>("-1");
    expect_bad_reply<int>("1.5");
    expect_bad_reply<int>("\"7\"");
}

TEST(ReplyDecoder, FloatingPointAcceptsIntegers) {
    EXPECT_EQ(decode<double>("95.431"), 95.431);
    EXPECT_EQ(decode<double>("2"), 2.0);
    expect_bad_reply<double>("\"2\"");
}

TEST(ReplyDecoder, Strings) {
    EXPECT_EQ(decode<std::string>("\"V3.15.3\""), "V3.15.3");
    expect_bad_reply<std::string>("3");
}

TEST(ReplyDecoder, EnumsAcceptOnlyDefinedValues) {
    EXPECT_EQ(decode<ec::RobotState>("5"), ec::RobotState::Collision);
    EXPECT_EQ(decode<ec::CycleMode>("2"), ec::CycleMode::Continuous);
    expect_bad_reply<ec::RobotState>("6");
    expect_bad_reply<ec::CycleMode>("-1");
    expect_bad_reply<ec::RobotMode>("true");
}

TEST(ReplyDecoder, ArraysNeedTheExactLength) {
    EXPECT_EQ((decode<std::array<double, 3>>("[1, 2.5, -3]")), (std::array<double, 3>{1, 2.5, -3}));
    expect_bad_reply<std::array<double, 3>>("[1, 2]");
    expect_bad_reply<std::array<double, 3>>("[1, 2, 3, 4]");
    expect_bad_reply<std::array<double, 3>>("{}");

    const auto bad_element = decode<std::array<double, 3>>(R"([1, "x", 3])");
    ASSERT_FALSE(bad_element);
    EXPECT_TRUE(bad_element.error().message.starts_with("element 1:")) << bad_element.error().message;
}

TEST(ReplyDecoder, VectorsTakeAnyLength) {
    EXPECT_EQ(decode<std::vector<int>>("[]"), std::vector<int>{});
    EXPECT_EQ(decode<std::vector<int>>("[1, 2, 3]"), (std::vector<int>{1, 2, 3}));
    expect_bad_reply<std::vector<int>>("[1, 2.5]");
    expect_bad_reply<std::vector<int>>("7");
}

TEST(ReplyDecoder, NestedContainers) {
    using Rows = std::vector<std::array<int, 2>>;
    EXPECT_EQ(decode<Rows>("[[1, 2], [3, 4]]"), (Rows{{1, 2}, {3, 4}}));
    expect_bad_reply<Rows>("[[1, 2], [3]]");
}

// ------------------------------------------------------------ Command concept

struct Plain {
    static constexpr std::string_view method = "plain";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
};
struct WithParams {
    static constexpr std::string_view method = "withParams";
    static constexpr Kind kind = Kind::Query;
    using Reply = Ok;
    int index = 3;
    [[nodiscard]] Json params() const { return Json{{"index", index}}; }
};
struct WithDecode {
    static constexpr std::string_view method = "withDecode";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::pair<int, int>;  // no ReplyDecoder for pairs: needs its own decode()
    static std::expected<Reply, Error> decode(const Json& r) {
        if (!r.is_array() || r.size() != 2) return bad_reply("expected a pair");
        return Reply{r[0].get<int>(), r[1].get<int>()};
    }
};
struct NoReplyType {
    static constexpr std::string_view method = "x";
    static constexpr Kind kind = Kind::Query;
};
struct NoKind {
    static constexpr std::string_view method = "x";
    using Reply = Ok;
};
struct UndecodableReply {
    static constexpr std::string_view method = "x";
    static constexpr Kind kind = Kind::Query;
    using Reply = std::pair<int, int>;
};
struct ParamsNotJson {
    static constexpr std::string_view method = "x";
    static constexpr Kind kind = Kind::Query;
    using Reply = Ok;
    [[nodiscard]] int params() const { return 0; }
};
struct DecodeWithWrongSignature {
    static constexpr std::string_view method = "x";
    static constexpr Kind kind = Kind::Query;
    using Reply = int;
    static int decode(const Json&) { return 0; }  // must return std::expected<Reply, Error>
};

static_assert(Command<Plain>);
static_assert(Command<WithParams>);
static_assert(Command<WithDecode>);
static_assert(!Command<NoReplyType>);
static_assert(!Command<NoKind>);
static_assert(!Command<UndecodableReply>);
static_assert(!Command<ParamsNotJson>);
static_assert(!Command<DecodeWithWrongSignature>);

TEST(CommandConcept, ParamsAreOptional) {
    EXPECT_TRUE(to_request(Plain{}).params.is_null());
    EXPECT_EQ(to_request(WithParams{}).params, Json({{"index", 3}}));
    EXPECT_EQ(to_request(WithParams{}).method, "withParams");
}

TEST(CommandConcept, OwnDecodeIsUsedWhenPresent) {
    EXPECT_EQ(decode_reply<WithDecode>(Json::parse("[4, 5]")), (std::pair{4, 5}));
    EXPECT_EQ(decode_reply<Plain>(Json(7)), 7);
}

}  // namespace
