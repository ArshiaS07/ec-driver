// to_si: every unit conversion of the SI boundary, against a frame built field
// by field (support/frame_builder.hpp).

#include "ec/feedback/state.hpp"

#include <gtest/gtest.h>

#include <numbers>

#include "support/frame_builder.hpp"

namespace {

using namespace ec::feedback;

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kRpm = 2.0 * std::numbers::pi / 60.0;

TEST(FeedbackState, ConvertsEveryFieldToSi) {
    const auto frame = decode(ec::test::make_frame());
    ASSERT_TRUE(frame.has_value());
    const State s = to_si(*frame);

    EXPECT_EQ(s.controller_time_ms, 1791200688449);  // kept as sent

    EXPECT_DOUBLE_EQ(s.joint_position[0], 10.0 * kDeg);
    EXPECT_DOUBLE_EQ(s.joint_position[5], 15.0 * kDeg);
    EXPECT_DOUBLE_EQ(s.joint_velocity[0], 600.0 * kDeg);
    EXPECT_DOUBLE_EQ(s.joint_acceleration[5], 705.0 * kDeg);
    EXPECT_DOUBLE_EQ(s.joint_torque[0], -1.5);
    EXPECT_DOUBLE_EQ(s.joint_torque_external[5], 5.25);
    EXPECT_DOUBLE_EQ(s.joint_torque_rated[0], 0.300);
    EXPECT_DOUBLE_EQ(s.motor_speed[0], -4.0 * kRpm);
    EXPECT_DOUBLE_EQ(s.joint_temperature[5], 35.0);  // degC, kept as sent

    EXPECT_DOUBLE_EQ(s.tcp_position_base[0], 0.100);
    EXPECT_DOUBLE_EQ(s.tcp_rotation_base[0], 103.0);  // rotations are radians already
    EXPECT_DOUBLE_EQ(s.tcp_position_user[2], 0.202);
    EXPECT_DOUBLE_EQ(s.flange_position_base[1], 0.401);
    EXPECT_DOUBLE_EQ(s.flange_rotation_user[2], 505.0);
    EXPECT_DOUBLE_EQ(s.tcp_speed, 0.0125);
    EXPECT_DOUBLE_EQ(s.tcp_acceleration, -0.00325);
    EXPECT_DOUBLE_EQ(s.tcp_force[2], 802.0);
    EXPECT_DOUBLE_EQ(s.tcp_torque[0], 803.0);

    EXPECT_EQ(s.robot_state, ec::RobotState::Running);
    EXPECT_EQ(s.robot_mode, ec::RobotMode::Remote);
    EXPECT_EQ(s.cycle_mode, ec::CycleMode::Continuous);
    EXPECT_TRUE(s.brakes_released);
    EXPECT_TRUE(s.motors_synced);
    EXPECT_TRUE(s.collision);
    EXPECT_TRUE(s.emergency_stop);
    EXPECT_TRUE(s.drag_enabled);
    EXPECT_FALSE(s.force_sensor_connected);

    EXPECT_EQ(s.digital_input, 0x8001u);
    EXPECT_EQ(s.digital_output, 0x00F0u);
    EXPECT_DOUBLE_EQ(s.analog_input[2], 3.0);
    EXPECT_DOUBLE_EQ(s.analog_output[4], 9.0);
}

}  // namespace
