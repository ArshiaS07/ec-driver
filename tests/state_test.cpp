// state_from: every field of a frame lands in its part of ec::State, in SI
// units, against a frame built field by field (support/frame_builder.hpp).

#include "ec/state.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <numbers>

#include "ec/pose.hpp"
#include "support/frame_builder.hpp"

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kRpm = 2.0 * std::numbers::pi / 60.0;

TEST(State, EveryFieldLandsInItsPart) {
    const auto frame = ec::feedback::decode(ec::test::make_frame());
    ASSERT_TRUE(frame.has_value());
    const auto arrival = ec::Clock::now();
    const ec::State s = ec::state_from(*frame, 7, arrival);

    EXPECT_EQ(s.seq, 7u);
    EXPECT_EQ(s.arrival, arrival);
    EXPECT_EQ(s.controller_time.time_since_epoch(), std::chrono::milliseconds{1791200688449});

    EXPECT_DOUBLE_EQ(s.joints[0].position, 10.0 * kDeg);
    EXPECT_DOUBLE_EQ(s.joints[5].position, 15.0 * kDeg);
    EXPECT_DOUBLE_EQ(s.joints[0].velocity, 600.0 * kDeg);
    EXPECT_DOUBLE_EQ(s.joints[5].acceleration, 705.0 * kDeg);
    EXPECT_DOUBLE_EQ(s.joints[0].torque, -1.5);
    EXPECT_DOUBLE_EQ(s.joints[5].external_torque, 5.25);
    EXPECT_DOUBLE_EQ(s.joints[0].torque_of_rated, 0.300);
    EXPECT_DOUBLE_EQ(s.joints[1].motor_speed, -3.0 * kRpm);
    EXPECT_DOUBLE_EQ(s.joints[5].temperature, 35.0);
    EXPECT_EQ(s.positions(), (ec::Joints{10 * kDeg, 11 * kDeg, 12 * kDeg, 13 * kDeg, 14 * kDeg, 15 * kDeg}));

    // Poses: x, y, z [mm] to m; rx, ry, rz [rad] as R = Rz Ry Rx (ec/pose.hpp).
    EXPECT_TRUE(s.tcp.pose.isApprox(ec::from_mm_rad({100, 101, 102, 103, 104, 105})));
    EXPECT_TRUE(s.tcp.pose_in_user.isApprox(ec::from_mm_rad({200, 201, 202, 203, 204, 205})));
    EXPECT_TRUE(s.tcp.flange_pose.isApprox(ec::from_mm_rad({400, 401, 402, 403, 404, 405})));
    EXPECT_TRUE(s.tcp.flange_pose_in_user.isApprox(ec::from_mm_rad({500, 501, 502, 503, 504, 505})));
    EXPECT_DOUBLE_EQ(s.tcp.pose.translation().x(), 0.100);
    EXPECT_DOUBLE_EQ(s.tcp.speed, 0.0125);
    EXPECT_DOUBLE_EQ(s.tcp.acceleration, -0.00325);
    EXPECT_EQ(s.tcp.force, Eigen::Vector3d(800, 801, 802));
    EXPECT_EQ(s.tcp.torque, Eigen::Vector3d(803, 804, 805));

    EXPECT_TRUE(s.drives.servo_on);
    EXPECT_TRUE(s.drives.synced);
    EXPECT_EQ(s.motion.state, ec::RobotState::Running);
    EXPECT_TRUE(s.motion.hand_guiding);
    EXPECT_TRUE(s.safety.emergency_stop);
    EXPECT_TRUE(s.safety.collision);
    EXPECT_EQ(s.io.digital_inputs.to_ullong(), 0x8001u);
    EXPECT_TRUE(s.io.digital_inputs.test(15));
    EXPECT_EQ(s.io.digital_outputs.to_ullong(), 0x00F0u);
    EXPECT_DOUBLE_EQ(s.io.analog_inputs[2], 3.0);
    EXPECT_DOUBLE_EQ(s.io.analog_outputs[4], 9.0);
    EXPECT_EQ(s.programs.cycle_mode, ec::CycleMode::Continuous);
    EXPECT_EQ(s.system.mode, ec::RobotMode::Remote);
    EXPECT_FALSE(s.system.force_sensor_connected);
}

TEST(State, DefaultIsAtTheOriginWithNothingOn) {
    const ec::State s;
    EXPECT_EQ(s.seq, 0u);
    EXPECT_TRUE(s.tcp.pose.isApprox(Eigen::Isometry3d::Identity()));
    EXPECT_TRUE(s.tcp.force.isZero());
    EXPECT_FALSE(s.drives.servo_on);
    EXPECT_EQ(s.positions(), ec::Joints{});
}

}  // namespace
