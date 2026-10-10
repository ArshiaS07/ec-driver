// Status enums: which values the manual defines, and how values print.

#include "ec/status.hpp"

#include <gtest/gtest.h>

#include <format>

namespace {

using ec::CycleMode;
using ec::RobotMode;
using ec::RobotState;

static_assert(ec::is_valid(RobotState::Collision));  // usable at compile time

TEST(Status, IsValidMatchesTheManual) {
    for (int v = 0; v <= 5; ++v) EXPECT_TRUE(ec::is_valid(static_cast<RobotState>(v))) << v;
    EXPECT_FALSE(ec::is_valid(static_cast<RobotState>(6)));
    EXPECT_FALSE(ec::is_valid(static_cast<RobotState>(-1)));

    for (int v = 0; v <= 2; ++v) EXPECT_TRUE(ec::is_valid(static_cast<RobotMode>(v))) << v;
    EXPECT_FALSE(ec::is_valid(static_cast<RobotMode>(3)));

    for (int v = 0; v <= 2; ++v) EXPECT_TRUE(ec::is_valid(static_cast<CycleMode>(v))) << v;
    EXPECT_FALSE(ec::is_valid(static_cast<CycleMode>(3)));
}

TEST(Status, FormatsNamesAndUnknownValues) {
    EXPECT_EQ(std::format("{}", RobotState::EmergencyStop), "e-stop");
    EXPECT_EQ(std::format("{}", RobotMode::Remote), "remote");
    EXPECT_EQ(std::format("{}", CycleMode::Step), "single step");
    EXPECT_EQ(std::format("{}", static_cast<RobotState>(9)), "unknown (9)");
    EXPECT_EQ(std::format("{}", static_cast<CycleMode>(200)), "unknown (200)");
    EXPECT_EQ(std::format("[{:>8}]", RobotMode::Auto), "[    auto]");
}

}  // namespace
