#include "ec/state.hpp"

#include <cstddef>
#include <numbers>

#include "ec/pose.hpp"

namespace ec {
namespace {

constexpr double kRadPerSecondPerRpm = 2.0 * std::numbers::pi / 60.0;
constexpr double kMmPerMetre = 1000.0;  // as from_mm_rad
constexpr double kPerMille = 1e-3;

}  // namespace

Joints State::positions() const noexcept {
    Joints q{};
    for (std::size_t i = 0; i < kJointCount; ++i) q[i] = joints[i].position;
    return q;
}

State state_from(const feedback::Frame& f, std::uint64_t seq, Clock::time_point arrival) noexcept {
    State s;
    s.seq = seq;
    s.arrival = arrival;
    s.controller_time = std::chrono::sys_time<std::chrono::milliseconds>{
        std::chrono::milliseconds{static_cast<std::int64_t>(f.timestamp_ms)}};

    for (std::size_t i = 0; i < kJointCount; ++i) {
        JointState& j = s.joints[i];
        j.position = deg_to_rad(f.joint_pos_deg[i]);
        j.velocity = deg_to_rad(f.joint_vel_deg_s[i]);
        j.acceleration = deg_to_rad(f.joint_acc_deg_s2[i]);
        j.torque = f.joint_torque_nm[i];
        j.external_torque = f.joint_torque_ext_nm[i];
        j.torque_of_rated = f.joint_torque_permille[i] * kPerMille;
        j.motor_speed = static_cast<double>(f.motor_speed_rpm[i]) * kRadPerSecondPerRpm;
        j.temperature = f.joint_temp_c[i];
    }

    s.tcp.pose = from_mm_rad(f.tcp_pose_base);
    s.tcp.pose_in_user = from_mm_rad(f.tcp_pose_user);
    s.tcp.flange_pose = from_mm_rad(f.flange_pose_base);
    s.tcp.flange_pose_in_user = from_mm_rad(f.flange_pose_user);
    s.tcp.speed = f.tcp_speed_mm_s / kMmPerMetre;
    s.tcp.acceleration = f.tcp_acc_mm_s2 / kMmPerMetre;
    const auto& w = f.tcp_wrench_ext;
    s.tcp.force = {w[0], w[1], w[2]};
    s.tcp.torque = {w[3], w[4], w[5]};

    s.drives = {.servo_on = f.servo_ready, .synced = f.motors_synced};
    s.motion = {.state = f.robot_state, .hand_guiding = f.drag_enabled};
    s.safety = {.emergency_stop = f.emergency_stop, .collision = f.collision};
    s.io.digital_inputs = f.digital_in;
    s.io.digital_outputs = f.digital_out;
    s.io.analog_inputs = f.analog_in_v;
    s.io.analog_outputs = f.analog_out_v;
    s.programs.cycle_mode = f.cycle_mode;
    s.system = {.mode = f.robot_mode, .force_sensor_connected = f.force_sensor_connected};
    return s;
}

}  // namespace ec
