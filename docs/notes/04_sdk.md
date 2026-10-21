# SDK socket manual: vendor English 3.19.2 [S] and Chinese 3.11.2 (Annuo "EA series", robotanno.com) [S-zh]
## Ports [S 1, S-zh 1]
- 8055 control: JSON-RPC 2.0, one JSON object per line ("both sending and receiving end with \n"), id echoed.
  Errors: -32601 method not found; -32693 internal: "parameters not found / out of range / execution conditions not
  met" [S 2]. Result is a JSON string (double-encoded) [S 2.1 example json.loads(jdata["result"])].
- 8056 monitor: 1024-byte frames every 8 ms, "long connection: if the access time exceeds 8 ms, the data will pile up
  in the buffer" [S 3]. Fields (big endian): size, timestamp ms since 1970, cycle mode, machinePos[8] deg,
  machinePose (base, mm/rad), machineUserPose, torque[8] per-mille of rated, robotState (0 stop,1 pause,2 E-stop,
  3 run,4 alarm), servoReady, can_motor_run (sync status), motor_speed[8] rpm, robotMode (0 teach,1 auto/play,
  2 remote), AI[3] V, AO[5] V, DI u64, DO u64, collision, flange pose base/user, emergencyStopState, tcpSpeed,
  jointSpeed[8] deg/s, tcpAcc, jointAcc[8], jointTemperature[6] C, jointTorque[6] Nm, extJointTorques[6] Nm
  (estimated external), exTcpForceInTool[6] N/Nm, dragState, sensor_connected_state, 219 reserved, matchingWord
  0xec8056ec [S 3.1].
- 8058 log interface: text; send "Type=Info|Warning|Error\n" then "all\n" or "N\n" (last N lines) or "exit\n" [S 4].
- 8059 raw log interface (v2.14.0+): "all" / N / exit [S 5]. Both listed in the 3.11.2 port table [S-zh 1].
## Mode gating: "This command is only supported in remote mode" on: set_servo_status, syncMotorStatus, clearAlarm,
  setToolNumber, setUserNumber, setCurrentCoord, drag_teach_switch, cmd_set_payload, cmd_set_tcp, setCycleMode,
  setUserFrame, resetCollisionState, setAutoRunToolNumber, setFlangeButton, book_program_clear, all moves, path,
  jog, stop, run, pause, runJbi, setSpeed, speedj/l, moveByLineCoord, calibrate_encoder_zero_position, setOutput,
  setVirtualOutput, setAnalogOutput, set var B/I/D/P/V, save_var_data, TT, profinet/EIP set, backdrive enter/exit,
  reset_robot_status, serial, force control. setSafetyParams: remote AND play [S 2.2.2.33]. Queries work in any mode.
## Servo / state / bring-up
- getServoStatus; set_servo_status{status 0|1}; syncMotorStatus ("synchronise servo encoder data");
  getMotorStatus ("synchronisation status"); clearAlarm [S 2.2.1]. Examples: state 4 -> clearAlarm + 0.5 s;
  not synced -> syncMotorStatus + 0.5 s; 0.5 s; servo off -> set_servo_status 1 (+1 s in 2.2.1.2/2.2.7.6/2.3.1)
  [S 2.2.9.10, 2.2.7.6, 2.3.1]. calibrate_encoder_zero_position: servo on first, then calibrate [S 2.2.3.21].
- getRobotState: 0 stop,1 pause,2 E-stop,3 run,4 alarm,5 collision; "The emergency stop status ... will only exist
  for a short time and will be covered by the alarm soon. If you need it, use get_estop_status" [S 2.2.2.1, .61].
- getRobotMode 0 teach / 1 play ("operating") / 2 remote [S 2.2.2.2].
- get_servo_precise_position_status (1 exact/0 inexact) = the pendant's precise/imprecise [S 2.2.2.64];
  get_servo_alarm_state [.65]; getAlarmNum: serial numbers of the last 5 alarms [.56]; getCollisionState /
  resetCollisionState [.21/.35]; get_estop_status [.61].
- reset_robot_status: "the robot must be in a stopped or error state (excluding emergency stop alarm)", remote only
  [S 2.2.11.5]. Backdrive: get_servo_brake_off_status (brake open per joint), get_backdrive_status,
  enter_backdrive ("must be in the reset state"), exit_backdrive [S 2.2.11].
- set_robot_power_status{status 0 off|1 on}, get_robot_power_status (0 off, 1 powering on, 2 on) - V3.12.2+, NOT in
  3.11.2 [S 2.2.2.87/.88] => arm power on/off from the SDK on our V3.15.3 [I: probably works; check].
- set_brake_voltage_type (MINI controller only, V3.14.2+) [S 2.2.2.89].
- Status flags also readable as virtual outputs (getVirtualOutput 400..1535; getRegisters for 16-bit blocks):
  M472 precise, M473 initialized, M476 ready-for-servo, M465 sync, M466 collision alarm, M416-418 mode, M424-428.
## Motion (all remote-only; "make sure the robot is in a stopped state; if running send stop and wait") [S 2.2.3]
- moveByJoint/Line/Arc/Rotate: joint targets (3.19.2 also target_pose with ref_pos; not in 3.11.2); speed % for
  joints [0.01,100]; Cartesian speed_type V mm/s [1,3000], VR deg/s [1,300], AV/AVR within the controller's
  kinematic parameters; acc/dec % [1,100] default 80/acc; cond_* stop condition (UNTIL syntax) [S 2.2.3.1-4].
- Path: clearPathPoint, addPathPoint (moveType joint/line/rotate/arc, speed, circular_radius blend [0,2608] mm (last
  must be 0), smooth deprecated, trigger IO at STARTTIME/ENDTIME/STARTLEN/ENDLEN, FPT), moveByPath -> count or -1,
  getPathPointIndex [S 2.2.3.5-7, 2.2.2.15].
- jog{index 0..11, speed [0.05,100]}: "if the next jog command is not received for more than 1 s ... jog stops";
  stop immediately with "stop" [S 2.2.3.8]. stop / run / pause (program-level) [S 2.2.3.9-11].
- checkJbiExist, runJbi, getJbiState (0 stop,1 pause,2 E-stop,3 run,4 error), getCurrentJobLine [S 2.2.3.12-14].
- setSpeed [0.05,100] (= the PLAY/REMOTE speed slider), getSpeed [S 2.2.3.15, 2.2.2.34].
- moveBySpeedj{vj deg/s, acc deg/s^2, t s} / stopj{acc}; moveBySpeedl{v mm/s & deg/s, acc, arot, t} / stopl:
  "the robot will not decelerate after executing the moveBySpeedj instruction, and continue to execute the last
  instruction sent" [S 2.2.3.16-19] => latest command wins, runs for t then stops [I].
- moveByLineCoord (target in a given user frame) [S 2.2.3.20].
- Hand-guiding-in-pause helpers (3.16.2+) [S 2.2.3.22-25].
## Kinematics: inverseKinematic{targetPose, referencePos}, inverseKinematicAll, positiveKinematic,
  convertPoseFromCartToUser/UserToCart, poseMul, poseInv, get_interp_pose [S 2.2.4, 2.2.2.71/.73]; getDH{index 0..11}.
## TT (transparent transmission) [S 2.2.7]: init{lookahead [10,1000] ms, t [2,100] ms, smoothness [0,1],
  response_enable}; tt_put_servo_joint_to_buf{targetPos} (or targetPose); tt_clear_servo_joint_buf;
  get_transparent_transmission_state; tt_set_current_servo_joint deprecated. Example 1 answers each point, sleeps
  10 ms between points; example 2 uses response_enable 0 and does not wait for replies. Queue has a capacity
  ("TTTARGETJOINT failed, the queue is full" [Sv]).
## Timestamp service [S 2.2.9]: start_push_pos{path_lenth>=3, pos_type, ref_joint_pos, ref_frame, ret_flag},
  push_pos{timestamp s (first 0, increasing), pos}, stop_push_pos (true only if all points correct),
  check_trajectory (0 ok,-1 count,-2 format,-3 timestamps), start_trajectory{speed_percent >= 0.1},
  pause/resume/stop_trajectory, flush_trajectory; "as long as flush_trajectory or start_push_pos is not executed,
  the current trajectory can be run in a loop without re-transmission". Example 2 (ret_flag 0) pushes without
  waiting. Both examples end with sleep 5 s then flush_trajectory.
## Other services: IO (DI 0..19, 46..51; DO 0..19, 48..51; virtual in 0..399, out 400..1535 (set 528..799);
  analog), variables B/I/D/P/V + save_var_data, system (getSoftVersion, getJointVersion axis 0..7, set_timezone,
  set_ntpServer - 3.19 only), Profinet/EIP registers, external force push (start_push_force/push_external_force/
  stop_push_force, index 0..65535), serial (RS232/485 on the box, TCI on the flange; recv has a timeout - blocks the
  channel [I]), force control (start_force_mode..., fc_act), safety params (setSafetyParams with MD5 of
  get_remote_sys_password + remote password; ranges power 80-1500 W, momentum 5-90, forces 100-400 N, speed %,
  collision sensitivity 10-100), welding, flange buttons, scan button.
