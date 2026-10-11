# SDK call catalog

All 184 JSON-RPC calls on port 8055, from the SDK manual Ver3.11.2 (English translation).
Generated from the manual's section headings and request templates; classification is ours.

## Kinds

| Kind | Meaning | Count |
|---|---|---|
| `query` | Read-only. No state change. Safe any time. | 84 |
| `compute` | Read-only computation on the controller (kinematics). Safe any time. | 7 |
| `setting` | Changes stored or selected configuration (tool, frames, payload, safety, IO, variables). Only while stopped. | 26 |
| `action` | One-shot change of controller state (servo on, sync, clear alarm, backdrive, ...). | 12 |
| `motion` | Part of a motion family protocol; see the family table below. | 40 |
| `session` | Part of a non-motion open/data/close protocol (serial ports, external force feed). | 15 |

"Remote" ✓ means the manual states the call only works in remote mode. Most motion and setting
calls fail outside remote mode in practice even where the manual does not say so; treat ✓ as
documented, not exhaustive. "8056" names the monitoring-frame field that carries the same value;
prefer the stream and use the call only for verification.

## Motion families

Each family is one controller protocol with a lifecycle. `stop` ends any family.

| Family | open | data | start / control | close | query |
|---|---|---|---|---|---|
| PointToPoint | robot stopped (manual) | | `moveByJoint`, `moveByLine`, `moveByArc`, `moveByLineCoord` | `stop` or arrival | |
| Waypoints | `clearPathPoint` | `addPathPoint` × N | `moveByPath` | `stop` or arrival, `clearPathPoint` | `getPathPointIndex` |
| Jog | — | `jog` every < 1 s | | `stop`, or 1 s silence | |
| Velocity | — | `moveBySpeedj` / `moveBySpeedl`, each valid for `t` | | `stopj` / `stopl`, or `t` expiry | |
| ServoStream | `transparent_transmission_init` | `tt_put_servo_joint_to_buf` per cycle | | `tt_clear_servo_joint_buf` | `get_transparent_transmission_state` |
| TimedTrajectory | `start_push_pos` | `push_pos` × N, `stop_push_pos` | `start_trajectory`, `pause_trajectory`, `resume_trajectory`, `stop_trajectory` | `flush_trajectory` | `check_trajectory` |
| Program (JBI) | | | `runJbi`, `pause`, `run` | `stop` | `checkJbiExist`, `getJbiState`, `getCurrentJobLine` |
| ForceControl (modifier) | `start_force_mode` | | `fc_act`, `fc_deact` | `end_force_mode` | `get_force_mode_state` |

ForceControl is not a motion family of its own: it modifies how the other families move.

## All calls


### 2.2.1 Servo Service (ServoService)

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.1.1 | `getServoStatus` | Get Arm Servo Status | query |  | 8056: `servoReady` |
| 2.2.1.2 | `set_servo_status` | Set Arm Servo Status | action | ✓ |  |
| 2.2.1.3 | `syncMotorStatus` | Synchronize Servo Encoder Data | action | ✓ |  |
| 2.2.1.4 | `clearAlarm` | Clear Alarm | action | ✓ |  |
| 2.2.1.5 | `getMotorStatus` | Get Synchronization Status | query |  | 8056: `can_motor_run` |

### 2.2.2 Parameter Service (ParamService)

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.2.1 | `getRobotState` | Get Robot State | query |  | 8056: `robotState` |
| 2.2.2.2 | `getRobotMode` | Get Robot Mode | query |  | 8056: `robotMode` |
| 2.2.2.3 | `get_joint_pos` | Get Output-Side Joint Positions | query |  | 8056: `machinePos` |
| 2.2.2.4 | `get_tcp_pose` | Get Current Robot Pose | query |  | 8056: `machinePose` |
| 2.2.2.5 | `get_motor_speed` | Get Robot Motor Speeds | query |  | 8056: `motor_speed` |
| 2.2.2.6 | `getCurrentCoord` | Get Current Robot Coordinate System | query |  |  |
| 2.2.2.7 | `getCycleMode` | Get Robot Cycle Mode | query |  | 8056: `autorun_cycelMode` |
| 2.2.2.8 | `getCurrentJobLine` | Get Line Number of the Currently Running Job | motion |  | Program · query |
| 2.2.2.9 | `getCurrentEncode` | Get Current Encoder Values | query |  |  |
| 2.2.2.10 | `getToolNumber` | Get Current Tool Number in Teach Mode | query |  |  |
| 2.2.2.11 | `setToolNumber` | Switch Current Tool Number in Teach Mode | setting | ✓ |  |
| 2.2.2.12 | `getUserNumber` | Get Current User Frame Number | query |  |  |
| 2.2.2.13 | `setUserNumber` | Switch Current User Frame Number | setting | ✓ |  |
| 2.2.2.14 | `get_motor_torque` | Get Current Robot Torque | query |  | 8056: `torque` |
| 2.2.2.15 | `getPathPointIndex` | Get Index of the Currently Executing Waypoint | motion |  | Waypoints · query |
| 2.2.2.16 | `setCurrentCoord` | Set Coordinate System | setting | ✓ |  |
| 2.2.2.17 | `drag_teach_switch` | Drag-Teach Switch | action | ✓ |  |
| 2.2.2.18 | `get_drag_state` | Get Robot Drag Enable State | query |  | 8056: `dragState` |
| 2.2.2.19 | `cmd_set_payload` | Set Arm Payload and Center of Gravity | setting | ✓ |  |
| 2.2.2.20 | `cmd_set_tcp` | Set Arm Tool Center Point | setting | ✓ |  |
| 2.2.2.21 | `getCollisionState` | Get Collision State | query |  | 8056: `collision` |
| 2.2.2.22 | `getUserFrame` | Get User Frame Data | query |  | ≥ v2.15.2 |
| 2.2.2.23 | `setCycleMode` | Set Cycle Mode | setting | ✓ |  |
| 2.2.2.24 | `setUserFrame` | Set User Frame Data | setting | ✓ | ≥ v2.15.2 |
| 2.2.2.25 | `getTcpPos` | Get Tool Frame Data | query |  |  |
| 2.2.2.26 | `getPayload` | Get Tool Payload Mass | query |  | **deprecated** (use get_tool_payload) |
| 2.2.2.27 | `getCentreMass` | Get Tool Center of Mass | query |  | **deprecated** (use get_tool_payload) |
| 2.2.2.28 | `getRobotType` | Get Robot Type | query |  |  |
| 2.2.2.29 | `getDH` | Get Robot DH Parameters | query |  |  |
| 2.2.2.30 | `setCollisionEnable` | Set Collision Enable | setting |  | **deprecated** (use setSafetyParams) |
| 2.2.2.31 | `setCollisionSensitivity` | Set Collision Sensitivity | setting | ✓ | **deprecated** (use setSafetyParams); lower = more sensitive (10 % triggers with the smallest force, per the EC66 pendant) |
| 2.2.2.32 | `get_remote_sys_password` | Get Auto-Generated Encrypted String | query |  | for setSafetyParams password |
| 2.2.2.33 | `setSafetyParams` | Set Safety Parameters | setting | ✓ |  |
| 2.2.2.34 | `getSpeed` | Get Robot Running Speed | query |  |  |
| 2.2.2.35 | `resetCollisionState` | Clear Collision State | action | ✓ |  |
| 2.2.2.36 | `getAutoRunToolNumber` | Get Current Tool Number in Remote Mode | query |  | ≥ v2.14.4 |
| 2.2.2.37 | `setAutoRunToolNumber` | Set Current Tool Number in Remote Mode | setting | ✓ | ≥ v2.14.4 |
| 2.2.2.38 | `get_base_flange_pose` | Get Flange Center Pose in the Base Frame | query |  | 8056: `machineFlangePose`; ≥ v2.14.4 |
| 2.2.2.39 | `get_user_flange_pose` | Get Flange Center Pose in the User Frame | query |  | 8056: `machineUserFlangePose`; ≥ v2.14.4 |
| 2.2.2.40 | `getRobotSubtype` | Get Robot Subtype | query |  | ≥ v2.16.2 |
| 2.2.2.41 | `getRobotSafetyParamsEnabled` | Get Safety Parameter Enable State | query |  | ≥ v2.16.2 |
| 2.2.2.42 | `getRobotSafetyPower` | Get Safety Power | query |  | ≥ v2.16.2 |
| 2.2.2.43 | `getRobotSafetyMomentum` | Get Safety Momentum | query |  | ≥ v2.16.2 |
| 2.2.2.44 | `getRobotSafetyToolForce` | Get Safety Tool Force | query |  | ≥ v2.16.2 |
| 2.2.2.45 | `get_tcp_force` | Get "External" Force and Torque in the Current TCP Frame | query |  | 8056: `exTcpForceInTool` |
| 2.2.2.46 | `get_joint_torques` | Get Current Joint Torques | query |  | 8056: `extJointTorques` |
| 2.2.2.47 | `getRobotSafetyElbowForce` | Get Safety Elbow Force | query |  | ≥ v2.16.2 |
| 2.2.2.48 | `getRobotSpeedPercentage` | Get Speed Percentage | query |  | ≥ v2.16.2 |
| 2.2.2.49 | `getRobotDragStartupMaxSpeed` | Get Maximum Drag Start Speed | query |  | ≥ v2.16.2 |
| 2.2.2.50 | `getRobotTorqueErrorMaxPercents` | Get Maximum Torque Error Percentage | query |  | ≥ v2.16.2 |
| 2.2.2.51 | `setFlangeButton` | Set End-Effector Button State | setting | ✓ | ≥ v2.16.2 |
| 2.2.2.52 | `checkFlangeButton` | Get End-Effector Button State | query |  | ≥ v2.16.2 |
| 2.2.2.53 | `get_collision_enable_status` | Get Collision Detection Enable State | query |  |  |
| 2.2.2.54 | `getCollisionSensitivity` | Get Collision Sensitivity | query |  | lower = more sensitive (EC66 pendant) |
| 2.2.2.55 | `getTcpPoseInUser` | Get Current TCP Pose in the Current User Frame | query |  | 8056: `machineUserPose` |
| 2.2.2.56 | `getAlarmNum` | Get Robot Alarm Serial Numbers | query |  |  |
| 2.2.2.57 | `get_joint_speed` | Get Joint Velocities | query |  | 8056: `jointSpeed` |
| 2.2.2.58 | `get_tcp_acc` | Get TCP Acceleration | query |  | 8056: `tcpAcc` |
| 2.2.2.59 | `get_joint_acc` | Get Joint Accelerations | query |  | 8056: `jointAcc` |
| 2.2.2.60 | `get_tcp_speed` | Get TCP Speed | query |  | 8056: `tcpSpeed` |
| 2.2.2.61 | `get_estop_status` | Get Robot Emergency-Stop State | query |  | 8056: `emergencyStopState` |
| 2.2.2.62 | `get_tool_payload` | Get Tool Payload and Center of Mass | query |  |  |
| 2.2.2.63 | `get_motor_pos` | Get Input-Side Joint Positions | query |  |  |
| 2.2.2.64 | `get_servo_precise_position_status` | Get Servo Encoder Precision State | query |  |  |
| 2.2.2.65 | `get_servo_alarm_state` | Get Servo Alarm State | query |  |  |
| 2.2.2.66 | `book_program_clear` | Clear Queued Programs | action | ✓ |  |
| 2.2.2.67 | `get_actual_tcp` | Get Actual End-Effector Pose | query |  |  |
| 2.2.2.68 | `get_target_tcp` | Get Target (Interpolated) End-Effector Pose | query |  |  |
| 2.2.2.69 | `get_actual_joint` | Get Actual Joint Data | query |  | 8056: `machinePos` |
| 2.2.2.70 | `get_target_joint` | Get Target (Interpolated) Joint Data | query |  |  |
| 2.2.2.71 | `get_interp_pose` | Get Linearly Interpolated Pose | query |  |  |
| 2.2.2.72 | `get_joint_temp` | Get Joint Temperatures | query |  | 8056: `jointTemperature` |
| 2.2.2.73 | `inverseKinematicAll` | Get All Inverse Kinematics Solutions for a Target Pose | compute |  |  |
| 2.2.2.74 | `set_system_time` | Set System Time | setting |  | ≥ v3.10.2 |

### 2.2.3 Movement Service (MovementService)

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.3.1 | `moveByJoint` | Joint Motion | motion | ✓ | PointToPoint · start |
| 2.2.3.2 | `moveByLine` | Linear Motion | motion | ✓ | PointToPoint · start |
| 2.2.3.3 | `moveByArc` | Circular-Arc Motion | motion | ✓ | PointToPoint · start |
| 2.2.3.4 | `moveByRotate` | Rotational Motion | motion | ✓ | PointToPoint · start; **deprecated** |
| 2.2.3.5 | `addPathPoint` | Add Waypoint 2.0 | motion | ✓ | Waypoints · data |
| 2.2.3.6 | `clearPathPoint` | Clear Waypoints 2.0 | motion | ✓ | Waypoints · open / close |
| 2.2.3.7 | `moveByPath` | Trajectory Motion 2.0 | motion | ✓ | Waypoints · start |
| 2.2.3.8 | `jog` | Jog Motion | motion | ✓ | Jog · data (keep-alive) |
| 2.2.3.9 | `stop` | Stop Robot | motion | ✓ | all families · stop |
| 2.2.3.10 | `run` | Robot Auto Run | motion | ✓ | Program · resume |
| 2.2.3.11 | `pause` | Pause Robot | motion | ✓ | Program · pause |
| 2.2.3.12 | `checkJbiExist` | Check Whether a JBI File Exists | motion |  | Program · query |
| 2.2.3.13 | `runJbi` | Run a JBI File | motion | ✓ | Program · start |
| 2.2.3.14 | `getJbiState` | Get JBI File Run State | motion |  | Program · query |
| 2.2.3.15 | `setSpeed` | Set Robot Running Speed | setting | ✓ | ≥ v2.13.1 |
| 2.2.3.16 | `moveBySpeedj` | Constant-Velocity Joint Motion | motion | ✓ | Velocity · data (keep-alive) |
| 2.2.3.17 | `stopj` | Stop Constant-Velocity Joint Motion | motion |  | Velocity · close |
| 2.2.3.18 | `moveBySpeedl` | Constant-Velocity Linear Motion | motion | ✓ | Velocity · data (keep-alive) |
| 2.2.3.19 | `stopl` | Stop Constant-Velocity Linear Motion | motion |  | Velocity · close |
| 2.2.3.20 | `moveByLineCoord` | Linear Motion in a Specified Coordinate System | motion | ✓ | PointToPoint · start; ≥ v2.16.2 |
| 2.2.3.21 | `calibrate_encoder_zero_position` | Encoder Zero Calibration | action | ✓ | **changes calibration, never call casually** |

### 2.2.4 Kinematics Service (KinematicsService)

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.4.1 | `inverseKinematic` | Inverse Kinematics | compute |  |  |
| 2.2.4.2 | `positiveKinematic` | Forward Kinematics | compute |  |  |
| 2.2.4.3 | `convertPoseFromCartToUser` | Base-to-User Frame Pose Conversion | compute |  |  |
| 2.2.4.4 | `convertPoseFromUserToCart` | User-to-Base Frame Pose Conversion | compute |  |  |
| 2.2.4.5 | `poseMul` | Pose Multiplication | compute |  |  |
| 2.2.4.6 | `poseInv` | Pose Inversion | compute |  |  |

### 2.2.5 IO Service (IOService)

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.5.1 | `getInput` | Get Input IO State | query |  | 8056: `digital_ioInput` |
| 2.2.5.2 | `getOutput` | Get Output IO State | query |  | 8056: `digital_ioOutput` |
| 2.2.5.3 | `setOutput` | Set Output IO State | setting | ✓ |  |
| 2.2.5.4 | `getVirtualInput` | Get Virtual Input IO State | query |  |  |
| 2.2.5.5 | `getVirtualOutput` | Get Virtual Output IO State | query |  |  |
| 2.2.5.6 | `setVirtualOutput` | Set Virtual Output IO State | setting | ✓ |  |
| 2.2.5.7 | `getRegisters` | Read Multiple M Virtual IOs | query |  |  |
| 2.2.5.8 | `getAnalogInput` | Get Analog Input | query |  | 8056: `analog_ioInput` |
| 2.2.5.9 | `get_analog_output` | Get Analog Output | query |  | 8056: `analog_ioOutput` |
| 2.2.5.10 | `setAnalogOutput` | Set Analog Output | setting | ✓ |  |

### 2.2.6 Variable Service (VarService)

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.6.1 | `getSysVarB` | Get System B Variable | query |  |  |
| 2.2.6.2 | `setSysVarB` | Set System B Variable | setting | ✓ |  |
| 2.2.6.3 | `getSysVarI` | Get System I Variable | query |  |  |
| 2.2.6.4 | `setSysVarI` | Set System I Variable | setting | ✓ |  |
| 2.2.6.5 | `getSysVarD` | Get System D Variable | query |  |  |
| 2.2.6.6 | `setSysVarD` | Set System D Variable | setting | ✓ |  |
| 2.2.6.7 | `getSysVarPState` | Check Whether a System P Variable Is Enabled | query |  |  |
| 2.2.6.8 | `getSysVarP` | Get P Variable | query |  |  |
| 2.2.6.9 | `setSysVarP` | Set P Variable | setting | ✓ | ≥ v2.15.2 |
| 2.2.6.10 | `getSysVarV` | Get V Variable | query |  |  |
| 2.2.6.11 | `setSysVarV` | Set V Variable | setting | ✓ |  |
| 2.2.6.12 | `save_var_data` | Save Variable Data | action | ✓ | ≥ v2.15.2 |

### 2.2.7 Transparent Transmission Service (TransparentTransmissionService)

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.7.1 | `transparent_transmission_init` | Initialize Transparent Transmission | motion | ✓ | ServoStream · open |
| 2.2.7.2 | `tt_set_current_servo_joint` | Set Current Transparent Transmission Servo Target Joint Position | motion | ✓ | ServoStream · data; **deprecated** (use tt_put_servo_joint_to_buf) |
| 2.2.7.3 | `get_transparent_transmission_state` | Check Whether the Robot Is in Transparent Transmission State | motion |  | ServoStream · query |
| 2.2.7.4 | `tt_put_servo_joint_to_buf` | Add Servo Target Joint Position to the Buffer | motion | ✓ | ServoStream · data |
| 2.2.7.5 | `tt_clear_servo_joint_buf` | Clear the Transparent Transmission Buffer | motion | ✓ | ServoStream · close |

### 2.2.8 System Service (SystemService)

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.8.1 | `getSoftVersion` | Get Controller Software Version | query |  |  |
| 2.2.8.2 | `getJointVersion` | Get Servo Version | query |  |  |

### 2.2.9 Timestamp Service (TrajectoryService)

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.9.1 | `start_push_pos` | Initialize Motion | motion |  | TimedTrajectory · open |
| 2.2.9.2 | `push_pos` | Add Motion Point | motion |  | TimedTrajectory · data |
| 2.2.9.3 | `stop_push_pos` | Stop Adding Points | motion |  | TimedTrajectory · seal |
| 2.2.9.4 | `check_trajectory` | Check Execution State | motion |  | TimedTrajectory · query |
| 2.2.9.5 | `start_trajectory` | Start Timestamped Motion | motion |  | TimedTrajectory · start |
| 2.2.9.6 | `pause_trajectory` | Pause Motion | motion |  | TimedTrajectory · pause |
| 2.2.9.7 | `resume_trajectory` | Resume Motion | motion |  | TimedTrajectory · resume |
| 2.2.9.8 | `stop_trajectory` | Stop Motion | motion |  | TimedTrajectory · stop |
| 2.2.9.9 | `flush_trajectory` | Clear Buffer | motion |  | TimedTrajectory · close |

### 2.2.10 Profinet Service (ProfinetService)

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.10.1 | `get_profinet_int_input_registers` | Get Profinet int Input Registers | query |  |  |
| 2.2.10.2 | `get_profinet_int_output_registers` | Get Profinet int Output Registers | query |  |  |
| 2.2.10.3 | `get_profinet_float_input_registers` | Get Profinet float Input Registers | query |  |  |
| 2.2.10.4 | `get_profinet_float_output_registers` | Get Profinet float Output Registers | query |  |  |
| 2.2.10.5 | `set_profinet_int_output_registers` | Set Profinet int Output Registers | setting | ✓ |  |
| 2.2.10.6 | `set_profinet_float_output_registers` | Set Profinet float Output Registers | setting | ✓ | manual request template says `set_profinet_int_…` (copy-paste error); example uses this name |

### 2.2.11 Backdrive Service (BackdriveService)

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.11.1 | `get_servo_brake_off_status` | Get Servo Brake Release State | query |  |  |
| 2.2.11.2 | `get_backdrive_status` | Check Whether in Backdrive Mode | query |  |  |
| 2.2.11.3 | `enter_backdrive` | Enter Backdrive Mode | action | ✓ |  |
| 2.2.11.4 | `exit_backdrive` | Exit Backdrive Mode | action | ✓ |  |
| 2.2.11.5 | `reset_robot_status` | Reset Controller State | action | ✓ |  |

### 2.2.12 Ethernet/IP

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.12.1 | `get_eip_int_input_registers` | Get Ethernet/IP int Input Registers | query |  | ≥ v3.5.2 |
| 2.2.12.2 | `get_eip_int_output_registers` | Get Ethernet/IP int Output Registers | query |  | ≥ v3.5.2 |
| 2.2.12.3 | `get_eip_float_input_registers` | Get Ethernet/IP float Input Registers | query |  | ≥ v3.5.2 |
| 2.2.12.4 | `get_eip_float_output_registers` | Get Ethernet/IP float Output Registers | query |  | ≥ v3.5.2 |
| 2.2.12.5 | `set_eip_int_output_registers` | Set Ethernet/IP int Output Registers | setting |  |  |
| 2.2.12.6 | `set_eip_float_output_registers` | Set Ethernet/IP float Output Registers | setting | ✓ | ≥ v3.5.2 |

### 2.2.13 External Force Sensor Service

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.13.1 | `start_push_force` | Mark the Start of Torque Data Transmission | session |  | ExternalForce · open |
| 2.2.13.2 | `push_external_force` | Transmit Torque Data | session |  | ExternalForce · data |
| 2.2.13.3 | `stop_push_force` | End the Current Torque Data Transmission | session |  | ExternalForce · close |
| 2.2.13.4 | `get_force_ctrl_mode` | Get Current Torque Data Source | query |  |  |
| 2.2.13.5 | `get_tcp_force_by_sensor` | Get TCP Force and Torque from the External Force Sensor | query |  | ≥ v3.9.2 |

### 2.2.14 Hardware Communication Control Service

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.14.1 | `open_serial_port` | Open Hardware Serial Port | session |  | SerialPort · open |
| 2.2.14.2 | `setopt_serial_port` | Configure Hardware Serial Port | session | ✓ | SerialPort · configure; ≥ v3.6.2 |
| 2.2.14.3 | `recv_serial_port` | Receive Data | session | ✓ | SerialPort · data; ≥ v3.6.2 |
| 2.2.14.4 | `send_serial_port` | Send Data | session | ✓ | SerialPort · data; ≥ v3.6.2 |
| 2.2.14.5 | `flush_serial_port` | Clear Hardware Buffer | session | ✓ | SerialPort · data; ≥ v3.6.2 |
| 2.2.14.6 | `close_serial_port` | Close Hardware Serial Port | session | ✓ | SerialPort · close; ≥ v3.6.2 |

### 2.2.15 TCI Communication Control Service

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.15.1 | `open_tci` | Open TCI Serial Port | session |  | TciPort · open |
| 2.2.15.2 | `setopt_tci` | Configure TCI Serial Port | session | ✓ | TciPort · configure; ≥ v3.6.2 |
| 2.2.15.3 | `recv_tci` | Receive Data | session | ✓ | TciPort · data; ≥ v3.6.2 |
| 2.2.15.4 | `send_tci` | Send Data | session | ✓ | TciPort · data; ≥ v3.6.2 |
| 2.2.15.5 | `flush_tci` | Clear TCI Buffer | session | ✓ | TciPort · data; ≥ v3.6.2 |
| 2.2.15.6 | `close_tci` | Close TCI Serial Port | session | ✓ | TciPort · close; ≥ v3.6.2 |

### 2.2.16 Force Control Service

| § | method | what | kind | remote | notes |
|---|---|---|---|---|---|
| 2.2.16.1 | `start_force_mode` | Enable Force Control Mode | motion |  | ForceControl (modifier) · open |
| 2.2.16.2 | `end_force_mode` | Disable Force Control Mode | motion |  | ForceControl (modifier) · close; ≥ v3.6.2 |
| 2.2.16.3 | `get_force_mode_state` | Get Force Control State | motion |  | ForceControl (modifier) · query; ≥ v3.6.2 |
| 2.2.16.4 | `zero_force_sensor` | Zero the Force Sensor | action |  | ≥ v3.6.2 |
| 2.2.16.5 | `fc_act` | Start Force-Seeking Motion | motion | ✓ | ForceControl (modifier) · start |
| 2.2.16.6 | `fc_deact` | End Force-Seeking Motion | motion | ✓ | ForceControl (modifier) · stop; ≥ v3.9.2 |
