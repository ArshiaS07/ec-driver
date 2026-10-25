// The call log: a header, then one JSON line per exchange, results and errors.

#include "ec/record/calls.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <string>
#include <vector>

#include "support/temp_file.hpp"

namespace {

using namespace std::chrono_literals;

std::vector<ec::sdk::Json> lines_of(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::vector<ec::sdk::Json> out;
    for (std::string line; std::getline(in, line);) out.push_back(ec::sdk::Json::parse(line));
    return out;
}

TEST(CallLog, AHeaderThenOneLinePerCall) {
    const ec::test::TempFile file("calls.jsonl");
    const auto t0 = ec::Clock::now();
    {
        auto log = ec::record::CallWriter::create(file.path());
        ASSERT_TRUE(log) << log.error();
        log->write(
            {.method = "getRobotMode", .params = nullptr, .reply = ec::sdk::Json(2), .sent = t0, .answered = t0 + 2ms});
        log->write(
            {.method = "moveByJoint",
             .params = {{"speed", 10}},
             .reply = std::unexpected(ec::Error{ec::Error::Kind::Command, "moveByJoint: controller returned false"}),
             .sent = t0 + 5ms,
             .answered = t0 + 9ms});
        EXPECT_EQ(log->written(), 2u);
        EXPECT_FALSE(log->error());
    }
    const auto lines = lines_of(file.path());
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_EQ(lines[0]["format"], "ec-driver.calls/1");
    EXPECT_TRUE(lines[0].contains("steady_ns_at_start"));
    EXPECT_EQ(lines[1]["method"], "getRobotMode");
    EXPECT_EQ(lines[1]["result"], 2);
    EXPECT_EQ(lines[1]["answered"].get<std::int64_t>() - lines[1]["sent"].get<std::int64_t>(), 2'000'000);
    EXPECT_EQ(lines[2]["params"]["speed"], 10);
    EXPECT_EQ(lines[2]["error"]["kind"], "command failed");
    EXPECT_FALSE(lines[2].contains("result"));
}

TEST(CallLog, AnUnwritablePathIsAnError) {
    EXPECT_FALSE(ec::record::CallWriter::create(std::filesystem::current_path() / "no_such_directory" / "c.jsonl"));
}

}  // namespace
