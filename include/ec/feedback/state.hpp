#pragma once
/// @file
/// A monitoring frame in SI units: the one place where the 8056 stream's units
/// are converted.

#include <array>
#include <cstdint>

#include "ec/feedback/frame.hpp"
#include "ec/status.hpp"

namespace ec::feedback {

using Joints = std::array<double, kArmAxes>;  ///< J1..J6
using Vec3 = std::array<double, 3>;

/// A frame's contents for the six arm joints, in SI units (m, rad, s, N, N m, V).
/// Kept as the controller sends them: its timestamp (ms) and temperatures (degC).
/// Rotations are the controller's rx, ry, rz [rad], composed as
/// R = Rz(rz) * Ry(ry) * Rx(rx) (measured on the EA66). Left out: the wire's
/// external axes 7 and 8, and its constant size field. Each comment names the
/// manual's field.
struct State {
    std::int64_t controller_time_ms = 0;  ///< timestamp: controller clock, ms since 1970-01-01

    Joints joint_position{};         ///< machinePos [rad]
    Joints joint_velocity{};         ///< jointSpeed [rad/s]
    Joints joint_acceleration{};     ///< jointAcc [rad/s^2]
    Joints joint_torque{};           ///< jointTorque: actual output torque [N m]
    Joints joint_torque_external{};  ///< extJointTorques: estimated external torque [N m]
    Joints joint_torque_rated{};     ///< torque: fraction of the rated torque [1]; per mille on the wire
    Joints motor_speed{};            ///< motor_speed [rad/s]; rpm on the wire
    Joints joint_temperature{};      ///< jointTemperature [degC]

    Vec3 tcp_position_base{};       ///< machinePose x, y, z [m]
    Vec3 tcp_rotation_base{};       ///< machinePose rx, ry, rz [rad]
    Vec3 tcp_position_user{};       ///< machineUserPose [m]
    Vec3 tcp_rotation_user{};       ///< machineUserPose [rad]
    Vec3 flange_position_base{};    ///< machineFlangePose [m]
    Vec3 flange_rotation_base{};    ///< machineFlangePose [rad]
    Vec3 flange_position_user{};    ///< machineUserFlangePose [m]
    Vec3 flange_rotation_user{};    ///< machineUserFlangePose [rad]
    double tcp_speed = 0.0;         ///< tcpSpeed [m/s]
    double tcp_acceleration = 0.0;  ///< tcpAcc [m/s^2]
    Vec3 tcp_force{};               ///< exTcpForceInTool fx, fy, fz, current tool frame [N]
    Vec3 tcp_torque{};              ///< exTcpForceInTool tx, ty, tz, current tool frame [N m]

    RobotState robot_state = RobotState::Stop;  ///< robotState
    RobotMode robot_mode = RobotMode::Teach;    ///< robotMode
    CycleMode cycle_mode = CycleMode::Step;     ///< autorun_cycelMode
    bool brakes_released = false;               ///< servoReady
    bool motors_synced = false;                 ///< can_motor_run
    bool collision = false;                     ///< collision
    bool emergency_stop = false;                ///< emergencyStopState
    bool drag_enabled = false;                  ///< dragState
    bool force_sensor_connected = false;        ///< sensor_connected_state

    std::uint64_t digital_input = 0;        ///< digital_ioInput: bit i is DI i
    std::uint64_t digital_output = 0;       ///< digital_ioOutput: bit i is DO i
    std::array<double, 3> analog_input{};   ///< analog_ioInput [V]
    std::array<double, 5> analog_output{};  ///< analog_ioOutput [V]
};

/// The frame in SI units. Pure; never fails.
[[nodiscard]] State to_si(const Frame& frame) noexcept;

}  // namespace ec::feedback
