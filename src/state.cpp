#include "ec/state.hpp"

#include <cstddef>

#include "ec/feedback/state.hpp"
#include "ec/pose.hpp"

namespace ec {

Joints State::positions() const noexcept {
    Joints q{};
    for (std::size_t i = 0; i < kJointCount; ++i) q[i] = joints[i].position;
    return q;
}

State state_from(const feedback::Frame& frame, std::uint64_t seq, Clock::time_point arrival) noexcept {
    const feedback::State si = feedback::to_si(frame);
    State s;
    s.seq = seq;
    s.arrival = arrival;
    s.controller_time =
        std::chrono::sys_time<std::chrono::milliseconds>{std::chrono::milliseconds{si.controller_time_ms}};

    for (std::size_t i = 0; i < kJointCount; ++i) {
        JointState& j = s.joints[i];
        j.position = si.joint_position[i];
        j.velocity = si.joint_velocity[i];
        j.acceleration = si.joint_acceleration[i];
        j.torque = si.joint_torque[i];
        j.external_torque = si.joint_torque_external[i];
        j.torque_of_rated = si.joint_torque_rated[i];
        j.motor_speed = si.motor_speed[i];
        j.temperature = si.joint_temperature[i];
    }

    s.tcp.pose = isometry(si.tcp_position_base, si.tcp_rotation_base);
    s.tcp.pose_in_user = isometry(si.tcp_position_user, si.tcp_rotation_user);
    s.tcp.flange_pose = isometry(si.flange_position_base, si.flange_rotation_base);
    s.tcp.flange_pose_in_user = isometry(si.flange_position_user, si.flange_rotation_user);
    s.tcp.speed = si.tcp_speed;
    s.tcp.acceleration = si.tcp_acceleration;
    s.tcp.force = {si.tcp_force[0], si.tcp_force[1], si.tcp_force[2]};
    s.tcp.torque = {si.tcp_torque[0], si.tcp_torque[1], si.tcp_torque[2]};

    s.drives = {.servo_on = si.brakes_released, .synced = si.motors_synced};
    s.motion = {.state = si.robot_state, .hand_guiding = si.drag_enabled};
    s.safety = {.emergency_stop = si.emergency_stop, .collision = si.collision};
    s.io.digital_inputs = si.digital_input;
    s.io.digital_outputs = si.digital_output;
    s.io.analog_inputs = si.analog_input;
    s.io.analog_outputs = si.analog_output;
    s.programs.cycle_mode = si.cycle_mode;
    s.system = {.mode = si.robot_mode, .force_sensor_connected = si.force_sensor_connected};
    return s;
}

}  // namespace ec
