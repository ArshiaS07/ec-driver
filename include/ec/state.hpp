#pragma once
/// @file
/// ec::State: what the robot reported in one monitoring frame (port 8056),
/// grouped by the part of the robot each value belongs to (docs/sdk_map.md).
/// A mirror of the controller's report and nothing more: the driver adds no
/// value of its own. What the frame does not carry (precise, arm power, the
/// alarms, the speed slider, the tool number) is not here: ask the controller
/// over 8055.
///
///     ec::State s = robot.monitor().latest().value();
///     double t3 = s.joints[2].temperature;   // J3 [degC]
///     auto T_base_tcp = s.tcp.pose;

#include <Eigen/Geometry>
#include <array>
#include <bitset>
#include <chrono>
#include <cstdint>

#include "ec/feedback/frame.hpp"
#include "ec/stamped.hpp"
#include "ec/status.hpp"
#include "ec/units.hpp"

namespace ec {

/// One joint (SDK manual 3.1; docs/ec66.md §3.2). SI units.
struct JointState {
    double position = 0.0;         ///< machinePos [rad], output side
    double velocity = 0.0;         ///< jointSpeed [rad/s]
    double acceleration = 0.0;     ///< jointAcc [rad/s^2]
    double torque = 0.0;           ///< jointTorque: actual output torque [N m]
    double external_torque = 0.0;  ///< extJointTorques: the controller's estimate [N m]
    double torque_of_rated = 0.0;  ///< torque: fraction of the motor's rated torque [1]; per mille on the wire
    double motor_speed = 0.0;      ///< motor_speed [rad/s], motor side; rpm on the wire
    double temperature = 0.0;      ///< jointTemperature [degC]
};

/// The tool centre point and the flange. Poses as in ec/pose.hpp: `pose` is
/// T_base_tcp; "in_user" ones are in the current user frame.
struct TcpState {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();                 ///< machinePose
    Eigen::Isometry3d pose_in_user = Eigen::Isometry3d::Identity();         ///< machineUserPose
    Eigen::Isometry3d flange_pose = Eigen::Isometry3d::Identity();          ///< machineFlangePose
    Eigen::Isometry3d flange_pose_in_user = Eigen::Isometry3d::Identity();  ///< machineUserFlangePose
    double speed = 0.0;                                                     ///< tcpSpeed [m/s]
    double acceleration = 0.0;                                              ///< tcpAcc [m/s^2]
    Eigen::Vector3d force = Eigen::Vector3d::Zero();  ///< exTcpForceInTool: estimated external force [N], tool frame
    Eigen::Vector3d torque =
        Eigen::Vector3d::Zero();  ///< exTcpForceInTool: estimated external torque [N m], tool frame
};

/// Servo enable and encoders.
struct DriveState {
    bool servo_on = false;  ///< servoReady: the servos are on, the brakes released (getServoStatus)
    bool synced = false;    ///< can_motor_run: the encoders are synchronised (getMotorStatus)
};

/// The motion controller.
struct MotionState {
    RobotState state = RobotState::Stop;  ///< robotState (getRobotState); may be a value the manual does not define
    bool hand_guiding = false;            ///< dragState (get_drag_state)
};

/// Stops the safety system reports.
struct SafetyState {
    bool emergency_stop = false;  ///< emergencyStopState (get_estop_status)
    bool collision = false;       ///< collision (getCollisionState)
};

/// The control box's I/O.
struct IoState {
    std::bitset<64> digital_inputs{};        ///< digital_ioInput: bit i is DI i
    std::bitset<64> digital_outputs{};       ///< digital_ioOutput: bit i is DO i
    std::array<double, 3> analog_inputs{};   ///< analog_ioInput [V]
    std::array<double, 5> analog_outputs{};  ///< analog_ioOutput [V]
};

/// The program engine.
struct ProgramState {
    CycleMode cycle_mode = CycleMode::Step;  ///< autorun_cycelMode (getCycleMode)
};

/// The controller as a whole.
struct SystemState {
    RobotMode mode = RobotMode::Teach;    ///< robotMode: the key switch (getRobotMode)
    bool force_sensor_connected = false;  ///< sensor_connected_state
};

/// One monitoring frame, as the robot's parts. Each comment names the manual's
/// field (SDK manual 3.1).
struct State {
    std::uint64_t seq = 0;        ///< 1 for the first frame decoded, one more for each after it
    Clock::time_point arrival{};  ///< when the frame arrived, on this PC's steady clock
    /// timestamp: the controller's own clock (ms since 1970), not this PC's.
    std::chrono::sys_time<std::chrono::milliseconds> controller_time{};

    std::array<JointState, kJointCount> joints{};  ///< J1..J6: joints[0] is J1
    TcpState tcp{};
    DriveState drives{};
    MotionState motion{};
    SafetyState safety{};
    IoState io{};
    ProgramState programs{};
    SystemState system{};

    /// The six joint positions [rad], as the model's calls take them.
    [[nodiscard]] Joints positions() const noexcept;
    /// How long ago the frame arrived.
    [[nodiscard]] Clock::duration age() const noexcept { return Clock::now() - arrival; }
};

/// The state a decoded frame reports, its units converted by feedback::to_si
/// (the one place they are); `seq` and `arrival` as given. Pure.
[[nodiscard]] State state_from(const feedback::Frame& frame, std::uint64_t seq, Clock::time_point arrival) noexcept;

}  // namespace ec
