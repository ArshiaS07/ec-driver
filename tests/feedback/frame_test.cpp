// Decoder round trip: a frame built field by field in wire order (see
// support/frame_builder.hpp) must decode into the right members, and broken
// frames must be rejected with the right reason.

#include "ec/feedback/frame.hpp"

#include <gtest/gtest.h>

#include "support/frame_builder.hpp"

namespace {

using namespace ec::feedback;

TEST(FeedbackFrame, DecodesEveryField) {
    const auto bytes = ec::test::make_frame();
    const auto r = decode(bytes);
    ASSERT_TRUE(r.has_value());
    const Frame& f = *r;

    EXPECT_EQ(f.message_size, 1024u);
    EXPECT_EQ(f.timestamp_ms, 1791200688449ull);
    EXPECT_EQ(f.cycle_mode, CycleMode::Continuous);
    EXPECT_EQ(f.joint_pos_deg[0], 10.0);
    EXPECT_EQ(f.joint_pos_deg[7], 17.0);
    EXPECT_EQ(f.tcp_pose_base[5], 105.0);
    EXPECT_EQ(f.tcp_pose_user[0], 200.0);
    EXPECT_EQ(f.joint_torque_permille[7], 307.0);
    EXPECT_EQ(f.robot_state, RobotState::Running);
    EXPECT_TRUE(f.servo_ready);
    EXPECT_TRUE(f.motors_synced);
    EXPECT_EQ(f.motor_speed_rpm[0], -4);
    EXPECT_EQ(f.motor_speed_rpm[7], 3);
    EXPECT_EQ(f.robot_mode, RobotMode::Remote);
    EXPECT_EQ(f.analog_in_v[2], 3.0);
    EXPECT_EQ(f.analog_out_v[4], 9.0);
    EXPECT_EQ(f.digital_in, 0x8001ull);
    EXPECT_EQ(f.digital_out, 0x00F0ull);
    EXPECT_TRUE(f.collision);
    EXPECT_EQ(f.flange_pose_base[0], 400.0);
    EXPECT_EQ(f.flange_pose_user[5], 505.0);
    EXPECT_TRUE(f.emergency_stop);
    EXPECT_EQ(f.tcp_speed_mm_s, 12.5);
    EXPECT_EQ(f.joint_vel_deg_s[7], 607.0);
    EXPECT_EQ(f.tcp_acc_mm_s2, -3.25);
    EXPECT_EQ(f.joint_acc_deg_s2[0], 700.0);
    EXPECT_EQ(f.joint_temp_c[5], 35.0);
    EXPECT_EQ(f.joint_torque_nm[0], -1.5);
    EXPECT_EQ(f.joint_torque_ext_nm[5], 5.25);
    EXPECT_EQ(f.tcp_wrench_ext[5], 805.0);
    EXPECT_TRUE(f.drag_enabled);
    EXPECT_FALSE(f.force_sensor_connected);
}

TEST(FeedbackFrame, RejectsBadMatchingWord) {
    auto bytes = ec::test::make_frame();
    bytes[kFrameSize - 1] ^= 0xFF;
    const auto r = decode(bytes);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), DecodeError::BadMatchingWord);
}

TEST(FeedbackFrame, RejectsBadMessageSize) {
    auto bytes = ec::test::make_frame();
    bytes[2] = 0;  // 0x00000400 -> 0x00000000
    const auto r = decode(bytes);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), DecodeError::BadMessageSize);
}

}  // namespace
