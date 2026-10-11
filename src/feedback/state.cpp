#include "ec/feedback/state.hpp"

#include <cstddef>
#include <numbers>

namespace ec::feedback {
namespace {

constexpr double kRadPerDeg = std::numbers::pi / 180.0;
constexpr double kRadPerSecondPerRpm = 2.0 * std::numbers::pi / 60.0;
constexpr double kMetrePerMm = 1e-3;
constexpr double kPerMille = 1e-3;

// The arm's six joints of a wire array, scaled.
template <typename T, std::size_t N>
Joints arm(const std::array<T, N>& wire, double scale) {
    static_assert(N >= kArmAxes);
    Joints out{};
    for (std::size_t i = 0; i < kArmAxes; ++i) out[i] = static_cast<double>(wire[i]) * scale;
    return out;
}

// A wire pose's (or wrench's) first three values, scaled, and its last three.
Vec3 head(const WirePose& v, double scale) { return {v[0] * scale, v[1] * scale, v[2] * scale}; }
Vec3 tail(const WirePose& v) { return {v[3], v[4], v[5]}; }

}  // namespace

State to_si(const Frame& f) noexcept {
    State s;
    s.controller_time_ms = static_cast<std::int64_t>(f.timestamp_ms);

    s.joint_position = arm(f.joint_pos_deg, kRadPerDeg);
    s.joint_velocity = arm(f.joint_vel_deg_s, kRadPerDeg);
    s.joint_acceleration = arm(f.joint_acc_deg_s2, kRadPerDeg);
    s.joint_torque = arm(f.joint_torque_nm, 1.0);
    s.joint_torque_external = arm(f.joint_torque_ext_nm, 1.0);
    s.joint_torque_rated = arm(f.joint_torque_permille, kPerMille);
    s.motor_speed = arm(f.motor_speed_rpm, kRadPerSecondPerRpm);
    s.joint_temperature = arm(f.joint_temp_c, 1.0);

    s.tcp_position_base = head(f.tcp_pose_base, kMetrePerMm);
    s.tcp_rotation_base = tail(f.tcp_pose_base);
    s.tcp_position_user = head(f.tcp_pose_user, kMetrePerMm);
    s.tcp_rotation_user = tail(f.tcp_pose_user);
    s.flange_position_base = head(f.flange_pose_base, kMetrePerMm);
    s.flange_rotation_base = tail(f.flange_pose_base);
    s.flange_position_user = head(f.flange_pose_user, kMetrePerMm);
    s.flange_rotation_user = tail(f.flange_pose_user);
    s.tcp_speed = f.tcp_speed_mm_s * kMetrePerMm;
    s.tcp_acceleration = f.tcp_acc_mm_s2 * kMetrePerMm;
    s.tcp_force = head(f.tcp_wrench_ext, 1.0);
    s.tcp_torque = tail(f.tcp_wrench_ext);

    s.robot_state = f.robot_state;
    s.robot_mode = f.robot_mode;
    s.cycle_mode = f.cycle_mode;
    s.brakes_released = f.servo_ready;
    s.motors_synced = f.motors_synced;
    s.collision = f.collision;
    s.emergency_stop = f.emergency_stop;
    s.drag_enabled = f.drag_enabled;
    s.force_sensor_connected = f.force_sensor_connected;

    s.digital_input = f.digital_in;
    s.digital_output = f.digital_out;
    s.analog_input = f.analog_in_v;
    s.analog_output = f.analog_out_v;
    return s;
}

}  // namespace ec::feedback
