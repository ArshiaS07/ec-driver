# User manual EC66 (B2 control box) Ver3.19.2 (2025-01-15) [U]
Versions in manual: software V3.19.2, servo V11.54.1, mechanical V4.B2.1, hardware V1.11.0 (our EA66: V3.15.3).
## System
- Three parts: controller (control box, "ERB2E series"), arm, teach pendant (TP); three cables (mains, robot, TP)
  [U 3]. Controller = control board, power supplies, I/O incl. safety, digital, analog [U 3.2]. Robot cable max 6 m;
  controller runs Linux with an sFTP server (/rbctrl, user root/elite2014, port 22) for JBI files [U 6.7.1]; 3 V coin
  battery on the core board keeps the system time [U 2, 5.4.3]; web TP at http://IP:6680/ and VNC (pw 333333) [U 11,
  12.1.3]; TP can be set to "Disconnect" (then an external E-stop must be wired) [U 11.4/11.5, 12.3.2].
- Arm: 6 revolute joints, J1 base, J2 shoulder, J3 elbow, J4-J6 wrist; joint limits +-360 deg each, adjustable soft
  limits [U 3.1.2, 12.3.3]. DH (a, d in m; alpha): J1 d=0.096 a=-pi/2; J2 a=0.418; J3 a=0.398; J4 d=0.122 a=-pi/2;
  J5 d=0.098 a=-pi/2; J6 d=0.089 [U 3.1.2.2]. Singular zone: the cylinder directly above and below the base
  [U 3.1.2.1]. Payload 6 kg, CoG < 260 mm from flange axis (payload diagram) [U 5.4.4]. Tech spec TCP speed 2.8 m/s
  (datasheet: typical 1.5 m/s) [U 14.6]. Mount 4x M8 at 20 Nm; any angle; mount config (tilt alpha about X, rotation
  gamma about Z) feeds gravity compensation, hand guiding, collision detection [U 5.4.1, 12.2.2].
- Tool flange: 12-pin connector (HR10A-10R-12P): DO Y48/Y49 (sink, 600 mA typ, 1 A max), DI X48/X49, AI003 (0-10 V,
  10 bit), AO005 (0-10 V), RS485 (2-4 ms latency each way; 2.4k..115.2k), 24 V (1 A total in 7.7; 2 A in 14.6);
  two flange buttons (blue X50, green X51: drag/record/disable) [U 7.7, 6.11.1, 14.1].
- Controller I/O: safety inputs dual channel: X0/X1 emergency stop, X2/X3 safeguard stop (X0-X3 occupied), DI
  X4-X19, DO Y0-Y19 (NPN), AI001-002, AO001-004 (+-10 V, 12 bit), RS485 up to 2 Mbit/s, Ethernet (SDK, Modbus TCP
  502, Lua). SDK needs the key switch on REMOTE [U 7.9]. Network cable recommended <= 3 m [U 7.9].
## Modes, operator devices
- Key switch on TP: TEACH / PLAY / REMOTE [U 3.3.1, 6.2]. TEACH: create/modify programs, variables, I/O, settings
  (settings only changeable in TEACH); jog only in TEACH, with the 3-position enabling switch on the TP back held in
  the middle [U 6.6]. PLAY: run programs from the TP (yellow servo button, green start, red pause; double-click pause
  within 1 s = stop) [U 3.3.1]. REMOTE: controlled by an external method (SDK over Ethernet) [U 6.2]. Moving the switch
  from PLAY to TEACH ends the running program [U 3.3.1]. Speed: manual (teach) speed separate; PLAY speed = REMOTE
  speed (same slider) [U 6.5.12]. Hand guiding: hold the button on the flange (no enabling switch needed) [U 6.6.3].
- Status shown: stop, pause, run, alarm, "stop(Imprecise)" [U 6.5.10]. Status control buttons: Initial (only at
  boot), Reset/Alarm, Imprecise, Shutdown, backdrive [U 6.5.5].
## Start-up / shut-down (official)
- Power on: key into power switch, turn on; fans + TP LED; then "press and hold the three-position switch on the
  back of the TP and click the imprecise button so that the robot is in an initialized state. The user can calibrate
  the encoder at the time. If it failed to finish the calibration, please manually move each axis until the robot is
  in the precise mode" [U 4.3, 5.5, 6.1]. => after power-up the controller is "uninitialized/imprecise"; encoder
  calibration (= SDK syncMotorStatus) makes it precise. M472 precise flag, M473 initialized flag, M476 "normal state:
  ready to switch on the servo", M465 sync status [U 14.1].
- Power off: "first turn the TP mode switch from PLAY or REMOTE to TEACH to stop the current program. Select the
  shutdown icon, OK. When the robot has come to a complete stop, turn the power switch off" [U 4.2, 4.7].
- Information log is not written to the file system in real time: power off right after an alarm may lose it
  [U 6.5.4].
- Backdrive mode: move the arm by hand after power-on (beyond soft limit, brake cannot release for encoder
  calibration); only when initialized [U 6.5.7]; brakes released per joint while moved [Sv 2.1.2].
## Stops and safety
- E-stop: TP button + external E_STOP (dual channel); stop category 1 (controlled, then power off), motor power OFF,
  program STOP, manual reset; arm "may slowly fall a short distance while the brakes engage" [U 1.7, 7.4, Table 7-2].
  Stop distance/time at 100% speed, full extension, 6 kg: ~0.18-0.22 rad, ~0.145 s [U Tables 7-4/7-5]. After E-stop:
  remove the cause, release the button, reset the alarm (TP), restart; main/sub-program call relations cleared
  [U 6.8.2]. Configurable "emergency stop deceleration stop time 1-200 ms" [U 12.3.2].
- Safeguard stop (X2/X3): stop category 2 (controlled, power stays on), program PAUSE, reset automatic (PLC S19/M48/
  M525) or manual (green play) [U 7.4.3, Table 7-2].
- Program pause: slows down and stops at the current line [U 6.8.2]. Alarm during a program: program ends, manipulator
  stops immediately [U 6.8.2].
- Safety limits: normal and reduced mode (reduced via M526/M527 = PLC S17/S18): power, momentum, tool force, elbow
  force, speed ratio [U 12.2.3]. Collision detection on/off + sensitivity 0..100 % [U 12.2.4]; depends on the dynamics
  identification being valid ("force control status green") [U 6.11.2.1]. Collision detection "is not a guarantee of
  safety" [U 1.7]. Interference zones: 16 (cube in base frame or joint ranges) -> M440-M455 (signals only) [U 6.11.6].
- Kinematic parameters: max/min joint speed, max/min line speed (mm/s), max/min pose rotation speed (deg/s) -
  these bound the AV/AVR "absolute" speeds of the SDK [U 12.3.1].
- Remote password (default "123456") gates safety parameters in remote mode [U 12.1.4].
- User levels Operator/Expert(111111)/Super(222222)/Admin(333333) [U 6.4].
## Programs and variables
- JBI programs (TP or PC/sFTP), Lua scripts (8 slots, background, /rbctrl/luadir), user PLC (ladder, 200 rungs,
  always running; safety relays S0..S26 incl. S3 external servo power up, S4 clear alarm, S7 servo off, S22 encoder
  calibration, S23 reset system, S24 enter reverse drive, S25 collision detection enable) [U 6.7, 9, 10].
- Global variables (power-retained, 256 each): B int [0,2^31-1], I int16, D double, P joint arrays [-360,360],
  V poses (rx,ry,rz in [-180,180]) [U 6.8.3.3]. Virtual I/O M0-M1535 (M0-M527 system, read only via Modbus;
  M528-M799 editable) [U 8, 14.1].
- Run cycles: single step / single cycle / continuous [U 6.8.1.2]. Program home (M401), mechanical home, 20-point zero
  calibration, 7-point TCP calibration, 3-point user frames, 8 tools (0-7) with TCP + mass + CoG (affects gravity
  compensation) [U 6.11-6.13].
- Drag (hand-guide) recording up to 20 min -> MOVEDRAG; payload identification (3-min motion, PLAY + servo; vertical
  mounting only) [U 6.11.2, 6.11.4]; force control needs a 6-axis F/T sensor at the flange (data source internal /
  Lua / SDK / back-end) [U 6.5.9].
- Monitoring on TP: raw pulse (sent/feedback/difference), motor speed, motor torque (per mille of rated), joint
  temperature [U 6.8.3].
## Virtual I/O map (selected) [U 14.1]
M0 E-stop (0 when X0/X1 active); M1 external E-stop; M3 external start; M4/M7 external pause; M5 external servo
enable; M6 clear alarm externally; M24 external stop; M26/M27 inconsistent E-stop/safeguard channels (400 ms);
M36 external synchronization (>=20 ms); M38/M400 servo-enabled state; M40 switch servo off from outside; M41 encoder
calibration; M42 system reset; M43 backdrive; M44 collision detection enable; M48/M525 auto reset after safeguard
stop; M49 safeguard stop status; M50 external pause status; M51 TP pause status; M80 back to first line; M401 at
program home; M416 TEACH, M417 PLAY, M418 REMOTE; M419-M422 TP run/pause/servo/clear-alarm buttons; M424 stop,
M425 pause, M427 program running, M428 warning; M440-M455 interference zones; M465 sync status; M466 collision
alarm; M468-M470 cycle mode; M472 precise (0 imprecise/1 precise); M473 initialized; M474 backdrive; M475 collision
detection on; M476 normal state (ready to switch the servo on); M526/M527 reduced mode.
