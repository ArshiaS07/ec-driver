# The SDK by component

The 184 calls of the 3.11.2 SDK, typed in `include/ec/sdk/`, regrouped by the part of the robot they
belong to (`docs/architecture.md` §2) instead of the vendor's services. Generated from the headers by
`python3 tools/sdk_map.py`; do not edit by hand.

Columns: the call; its kind in the code (Query, Compute, Setting, Action, Motion, Session); the manual's
section; **8056** when every monitor frame already carries the value, so the observed state needs no call.

| Component | Calls | of them in 8056 |
|---|---|---|
| State: joints and TCP (observed) | 20 | 16 |
| Drives: power, enable, brakes, encoders, alarms | 13 | 2 |
| Motion: the motion controller and its families | 36 | 2 |
| Model: kinematics, tools, payload, frames | 25 | 0 |
| Safety: stops, collision, limits | 17 | 2 |
| Programs: JBI, cycle, variables | 21 | 1 |
| I/O: box, flange, virtual bits, fieldbus registers | 24 | 4 |
| System: mode, versions, alarms, clock | 5 | 1 |
| Peripherals: serial ports, force sensor, force control | 23 | 0 |
| **All** | **184** | **28** |

## State: joints and TCP (observed)

| Call | Kind | § | 8056 |
|---|---|---|---|
| `get_joint_pos` | Query | 2.2.2.3 | 8056 |
| `get_actual_joint` | Query | 2.2.2.69 | 8056 |
| `get_target_joint` | Query | 2.2.2.70 |  |
| `get_motor_pos` | Query | 2.2.2.63 |  |
| `get_joint_speed` | Query | 2.2.2.57 | 8056 |
| `get_joint_acc` | Query | 2.2.2.59 | 8056 |
| `get_motor_speed` | Query | 2.2.2.5 | 8056 |
| `get_motor_torque` | Query | 2.2.2.14 | 8056 |
| `get_joint_torques` | Query | 2.2.2.46 | 8056 |
| `get_joint_temp` | Query | 2.2.2.72 | 8056 |
| `getCurrentEncode` | Query | 2.2.2.9 |  |
| `get_tcp_pose` | Query | 2.2.2.4 | 8056 |
| `getTcpPoseInUser` | Query | 2.2.2.55 | 8056 |
| `get_actual_tcp` | Query | 2.2.2.67 | 8056 |
| `get_target_tcp` | Query | 2.2.2.68 |  |
| `get_base_flange_pose` | Query | 2.2.2.38 | 8056 |
| `get_user_flange_pose` | Query | 2.2.2.39 | 8056 |
| `get_tcp_speed` | Query | 2.2.2.60 | 8056 |
| `get_tcp_acc` | Query | 2.2.2.58 | 8056 |
| `get_tcp_force` | Query | 2.2.2.45 | 8056 |

## Drives: power, enable, brakes, encoders, alarms

| Call | Kind | § | 8056 |
|---|---|---|---|
| `getServoStatus` | Query | 2.2.1.1 | 8056 |
| `set_servo_status` | Action | 2.2.1.2 |  |
| `getMotorStatus` | Query | 2.2.1.5 | 8056 |
| `syncMotorStatus` | Action | 2.2.1.3 |  |
| `get_servo_precise_position_status` | Query | 2.2.2.64 |  |
| `calibrate_encoder_zero_position` | Action | 2.2.3.21 |  |
| `clearAlarm` | Action | 2.2.1.4 |  |
| `reset_robot_status` | Action | 2.2.11.5 |  |
| `get_servo_alarm_state` | Query | 2.2.2.65 |  |
| `get_servo_brake_off_status` | Query | 2.2.11.1 |  |
| `get_backdrive_status` | Query | 2.2.11.2 |  |
| `enter_backdrive` | Action | 2.2.11.3 |  |
| `exit_backdrive` | Action | 2.2.11.4 |  |

## Motion: the motion controller and its families

| Call | Kind | § | 8056 |
|---|---|---|---|
| `getRobotState` | Query | 2.2.2.1 | 8056 |
| `stop` | Motion | 2.2.3.9 |  |
| `getSpeed` | Query | 2.2.2.34 |  |
| `setSpeed` | Setting | 2.2.3.15 |  |
| `moveByJoint` | Motion | 2.2.3.1 |  |
| `moveByLine` | Motion | 2.2.3.2 |  |
| `moveByArc` | Motion | 2.2.3.3 |  |
| `moveByRotate` | Motion | 2.2.3.4 |  |
| `moveByLineCoord` | Motion | 2.2.3.20 |  |
| `clearPathPoint` | Motion | 2.2.3.6 |  |
| `addPathPoint` | Motion | 2.2.3.5 |  |
| `moveByPath` | Motion | 2.2.3.7 |  |
| `getPathPointIndex` | Query | 2.2.2.15 |  |
| `jog` | Motion | 2.2.3.8 |  |
| `getCurrentCoord` | Query | 2.2.2.6 |  |
| `setCurrentCoord` | Setting | 2.2.2.16 |  |
| `moveBySpeedj` | Motion | 2.2.3.16 |  |
| `stopj` | Motion | 2.2.3.17 |  |
| `moveBySpeedl` | Motion | 2.2.3.18 |  |
| `stopl` | Motion | 2.2.3.19 |  |
| `transparent_transmission_init` | Motion | 2.2.7.1 |  |
| `tt_set_current_servo_joint` | Motion | 2.2.7.2 |  |
| `get_transparent_transmission_state` | Query | 2.2.7.3 |  |
| `tt_put_servo_joint_to_buf` | Motion | 2.2.7.4 |  |
| `tt_clear_servo_joint_buf` | Motion | 2.2.7.5 |  |
| `start_push_pos` | Motion | 2.2.9.1 |  |
| `push_pos` | Motion | 2.2.9.2 |  |
| `stop_push_pos` | Motion | 2.2.9.3 |  |
| `check_trajectory` | Query | 2.2.9.4 |  |
| `start_trajectory` | Motion | 2.2.9.5 |  |
| `pause_trajectory` | Motion | 2.2.9.6 |  |
| `resume_trajectory` | Motion | 2.2.9.7 |  |
| `stop_trajectory` | Motion | 2.2.9.8 |  |
| `flush_trajectory` | Motion | 2.2.9.9 |  |
| `drag_teach_switch` | Action | 2.2.2.17 |  |
| `get_drag_state` | Query | 2.2.2.18 | 8056 |

## Model: kinematics, tools, payload, frames

| Call | Kind | § | 8056 |
|---|---|---|---|
| `inverseKinematic` | Compute | 2.2.4.1 |  |
| `inverseKinematicAll` | Compute | 2.2.2.73 |  |
| `positiveKinematic` | Compute | 2.2.4.2 |  |
| `poseMul` | Compute | 2.2.4.5 |  |
| `poseInv` | Compute | 2.2.4.6 |  |
| `convertPoseFromCartToUser` | Compute | 2.2.4.3 |  |
| `convertPoseFromUserToCart` | Compute | 2.2.4.4 |  |
| `get_interp_pose` | Compute | 2.2.2.71 |  |
| `getDH` | Query | 2.2.2.29 |  |
| `getRobotType` | Query | 2.2.2.28 |  |
| `getRobotSubtype` | Query | 2.2.2.40 |  |
| `getToolNumber` | Query | 2.2.2.10 |  |
| `setToolNumber` | Setting | 2.2.2.11 |  |
| `getAutoRunToolNumber` | Query | 2.2.2.36 |  |
| `setAutoRunToolNumber` | Setting | 2.2.2.37 |  |
| `getTcpPos` | Query | 2.2.2.25 |  |
| `cmd_set_tcp` | Setting | 2.2.2.20 |  |
| `cmd_set_payload` | Setting | 2.2.2.19 |  |
| `get_tool_payload` | Query | 2.2.2.62 |  |
| `getPayload` | Query | 2.2.2.26 |  |
| `getCentreMass` | Query | 2.2.2.27 |  |
| `getUserNumber` | Query | 2.2.2.12 |  |
| `setUserNumber` | Setting | 2.2.2.13 |  |
| `getUserFrame` | Query | 2.2.2.22 |  |
| `setUserFrame` | Setting | 2.2.2.24 |  |

## Safety: stops, collision, limits

| Call | Kind | § | 8056 |
|---|---|---|---|
| `get_estop_status` | Query | 2.2.2.61 | 8056 |
| `getCollisionState` | Query | 2.2.2.21 | 8056 |
| `resetCollisionState` | Action | 2.2.2.35 |  |
| `get_collision_enable_status` | Query | 2.2.2.53 |  |
| `setCollisionEnable` | Setting | 2.2.2.30 |  |
| `getCollisionSensitivity` | Query | 2.2.2.54 |  |
| `setCollisionSensitivity` | Setting | 2.2.2.31 |  |
| `getRobotSafetyParamsEnabled` | Query | 2.2.2.41 |  |
| `getRobotSafetyPower` | Query | 2.2.2.42 |  |
| `getRobotSafetyMomentum` | Query | 2.2.2.43 |  |
| `getRobotSafetyToolForce` | Query | 2.2.2.44 |  |
| `getRobotSafetyElbowForce` | Query | 2.2.2.47 |  |
| `getRobotSpeedPercentage` | Query | 2.2.2.48 |  |
| `getRobotDragStartupMaxSpeed` | Query | 2.2.2.49 |  |
| `getRobotTorqueErrorMaxPercents` | Query | 2.2.2.50 |  |
| `get_remote_sys_password` | Query | 2.2.2.32 |  |
| `setSafetyParams` | Setting | 2.2.2.33 |  |

## Programs: JBI, cycle, variables

| Call | Kind | § | 8056 |
|---|---|---|---|
| `checkJbiExist` | Query | 2.2.3.12 |  |
| `runJbi` | Motion | 2.2.3.13 |  |
| `getJbiState` | Query | 2.2.3.14 |  |
| `run` | Motion | 2.2.3.10 |  |
| `pause` | Motion | 2.2.3.11 |  |
| `getCurrentJobLine` | Query | 2.2.2.8 |  |
| `getCycleMode` | Query | 2.2.2.7 | 8056 |
| `setCycleMode` | Setting | 2.2.2.23 |  |
| `book_program_clear` | Action | 2.2.2.66 |  |
| `getSysVarB` | Query | 2.2.6.1 |  |
| `setSysVarB` | Setting | 2.2.6.2 |  |
| `getSysVarI` | Query | 2.2.6.3 |  |
| `setSysVarI` | Setting | 2.2.6.4 |  |
| `getSysVarD` | Query | 2.2.6.5 |  |
| `setSysVarD` | Setting | 2.2.6.6 |  |
| `getSysVarPState` | Query | 2.2.6.7 |  |
| `getSysVarP` | Query | 2.2.6.8 |  |
| `setSysVarP` | Setting | 2.2.6.9 |  |
| `getSysVarV` | Query | 2.2.6.10 |  |
| `setSysVarV` | Setting | 2.2.6.11 |  |
| `save_var_data` | Action | 2.2.6.12 |  |

## I/O: box, flange, virtual bits, fieldbus registers

| Call | Kind | § | 8056 |
|---|---|---|---|
| `getInput` | Query | 2.2.5.1 | 8056 |
| `getOutput` | Query | 2.2.5.2 | 8056 |
| `setOutput` | Setting | 2.2.5.3 |  |
| `getAnalogInput` | Query | 2.2.5.8 | 8056 |
| `get_analog_output` | Query | 2.2.5.9 | 8056 |
| `setAnalogOutput` | Setting | 2.2.5.10 |  |
| `getVirtualInput` | Query | 2.2.5.4 |  |
| `getVirtualOutput` | Query | 2.2.5.5 |  |
| `setVirtualOutput` | Setting | 2.2.5.6 |  |
| `getRegisters` | Query | 2.2.5.7 |  |
| `checkFlangeButton` | Query | 2.2.2.52 |  |
| `setFlangeButton` | Setting | 2.2.2.51 |  |
| `get_profinet_int_input_registers` | Query | 2.2.10.1 |  |
| `get_profinet_int_output_registers` | Query | 2.2.10.2 |  |
| `get_profinet_float_input_registers` | Query | 2.2.10.3 |  |
| `get_profinet_float_output_registers` | Query | 2.2.10.4 |  |
| `set_profinet_int_output_registers` | Setting | 2.2.10.5 |  |
| `set_profinet_float_output_registers` | Setting | 2.2.10.6 |  |
| `get_eip_int_input_registers` | Query | 2.2.12.1 |  |
| `get_eip_int_output_registers` | Query | 2.2.12.2 |  |
| `get_eip_float_input_registers` | Query | 2.2.12.3 |  |
| `get_eip_float_output_registers` | Query | 2.2.12.4 |  |
| `set_eip_int_output_registers` | Setting | 2.2.12.5 |  |
| `set_eip_float_output_registers` | Setting | 2.2.12.6 |  |

## System: mode, versions, alarms, clock

| Call | Kind | § | 8056 |
|---|---|---|---|
| `getRobotMode` | Query | 2.2.2.2 | 8056 |
| `getSoftVersion` | Query | 2.2.8.1 |  |
| `getJointVersion` | Query | 2.2.8.2 |  |
| `getAlarmNum` | Query | 2.2.2.56 |  |
| `set_system_time` | Setting | 2.2.2.74 |  |

## Peripherals: serial ports, force sensor, force control

| Call | Kind | § | 8056 |
|---|---|---|---|
| `open_serial_port` | Session | 2.2.14.1 |  |
| `setopt_serial_port` | Session | 2.2.14.2 |  |
| `recv_serial_port` | Session | 2.2.14.3 |  |
| `send_serial_port` | Session | 2.2.14.4 |  |
| `flush_serial_port` | Session | 2.2.14.5 |  |
| `close_serial_port` | Session | 2.2.14.6 |  |
| `open_tci` | Session | 2.2.15.1 |  |
| `setopt_tci` | Session | 2.2.15.2 |  |
| `recv_tci` | Session | 2.2.15.3 |  |
| `send_tci` | Session | 2.2.15.4 |  |
| `flush_tci` | Session | 2.2.15.5 |  |
| `close_tci` | Session | 2.2.15.6 |  |
| `start_push_force` | Session | 2.2.13.1 |  |
| `push_external_force` | Session | 2.2.13.2 |  |
| `stop_push_force` | Session | 2.2.13.3 |  |
| `get_force_ctrl_mode` | Query | 2.2.13.4 |  |
| `get_tcp_force_by_sensor` | Query | 2.2.13.5 |  |
| `start_force_mode` | Motion | 2.2.16.1 |  |
| `end_force_mode` | Motion | 2.2.16.2 |  |
| `get_force_mode_state` | Query | 2.2.16.3 |  |
| `zero_force_sensor` | Action | 2.2.16.4 |  |
| `fc_act` | Motion | 2.2.16.5 |  |
| `fc_deact` | Motion | 2.2.16.6 |  |
