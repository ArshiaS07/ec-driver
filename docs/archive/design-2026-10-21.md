# An honest driver for the EC66: who owns what, and the design

A proposal, written 2026-10-21 after reading every manual in `docs/EC66`
(the facts and their sources are in `docs/ec66.md`; this document cites its
sections as §R.n). It starts from the hardware rather than from earlier
decisions, says where the existing code fits and where it doesn't, and ends
with a plan for the next few weeks. Nothing here is decided until the user
says so.

## 1. The honest premise

The EC66's control box is a complete robot controller (§R.4.2). It plans
and interpolates every motion, solves the kinematics, runs the dynamics
model, owns the certified safety functions, runs programs, and keeps the
state machine of power, encoders, alarms and modes. Its drives sit in the
joints, behind a field bus the PC never sees (§R.3.2). The Mini controller
is the same software in a smaller DC box, not a lower-level interface
(§R.2.1).

So the driver is a client of that controller, and its job is narrow:

1. speak the controller's protocols correctly and safely (8055, 8056, the
   logs), with every call typed, in SI at the API;
2. carry out the vendor's documented workflows (the ceremonies), in order,
   with their pauses;
3. report the controller's own state and its own reasons, so that an
   application knows why something did not happen;
4. stay out of what the controller does: no planning, no safety, no
   control loops of our own.

What the hardware does not offer, the driver cannot add. There is no
torque or position loop at kHz rates as on a Franka or a UR's RTDE: the
fastest path is transparent transmission, a queue of joint targets played
every 2–100 ms after at least 10 ms of lookahead, smoothed by the controller
(§R.5.7). It is a streamed trajectory, not servoing. Applications that close
a loop around the arm (hardware in the loop) close it at that rate, through
that queue, with the 125 Hz state stream for feedback.

## 2. Who owns what

| Concern | Controller (we call it) | Driver (we own it) | Application |
|---|---|---|---|
| Servo loops, commutation, encoders, brakes | ✓ (§R.3) | read the states | |
| Initialisation, precise, synchronisation, alarms | the state machine (§R.3.4, §R.5.2) | the vendor's sequence (`bring_up`), and reporting | the pendant steps the SDK cannot do |
| Modes (key switch) | ✓ (§R.5.1) | read and explain; never work around | the operator turns the key |
| Safety: E-stop, safeguard, collision, limits, reduced mode | ✓, certified (§R.5.8) | report it; nothing of ours | risk assessment |
| Motion: planning, interpolation, timing, blending | ✓ for every family (§R.5.7) | each family's call sequence, waits, stops | which motion, when |
| Kinematics, frames, tools, payload | ✓ (§R.5.6) | typed calls in SI and Eigen | |
| Trajectories from TCP motions (planning) | IK per point, if asked | nothing | ✓ (`apps/planning.hpp` today, capsnav later) |
| Protocols: framing, ids, timeouts, reconnecting | | ✓ | |
| One call at a time, never unconfirmed bursts | | ✓ (learned on the robot, §R.6.2) | |
| Units (deg, mm, %) to SI | | ✓, at one boundary | |
| State stream, recording | sends frames | decode, timestamp, record | analysis |
| Programs (JBI), Lua, PLC, fieldbuses | ✓ | start a program, read its state | writes the programs |
| Explaining failures | the alarm numbers, its log (§R.5.9) | fetch and attach them | show them |

## 3. Where we are, candidly

What is solid: the wire layer. All 184 calls are typed with their kinds;
the client never has two calls in flight; the 8056 decoder, the receiver,
the raw and HDF5 recorders, the fakes and the end-to-end tests all work, and
the timestamped circle ran on the robot. The motion families (milestone A)
are close to right: each makes its family's calls in the manual's order.

What went wrong, and why:

- **The library came before the application.** Without concrete use cases
  the API was guessed at, then rebuilt (a control thread with activities,
  jobs and sessions; then families; then planning moved in and out again).
  Part of that is mine: I added structure, checks and planning that nobody
  had asked for.
- **We under-read the controller.** Several things that shape the design
  only surfaced in this full reading: the two encoders and the
  precise/synchronised states (§R.3.4); the speed slider scaling relative
  speeds (§R.5.5); `get_estop_status`, because the E-stop state is overwritten
  by the alarm (§R.5.3); the controller's own logs on 8058/8059 and
  `getAlarmNum` (§R.5.9); the arm's power call (§R.5.2).
- **No session lifecycle.** We bring the robot up but never end a session:
  tools disconnect and leave the servo on, in REMOTE, with nobody watching.
  The pendant's log shows the connections coming and going. A session should
  end as deliberately as it starts.
- **Wrong line for checks.** First too many of our own (limits, arrival,
  first point), then none. The right line: check the preconditions the
  controller documents, using the controller's own state; add a guard of our
  own only where an unchecked mistake moves the arm somewhere unplanned, and
  then say it is ours.

## 4. Principles

1. **The controller is the source of truth.** State comes from 8056 and its
   queries; reasons come from its alarms and its log. The driver keeps no
   model of the robot of its own.
2. **Ceremonies are the vendor's.** Each documented sequence is carried out
   as written, with its pauses; where the manuals give none (ending a
   session), the driver's sequence is built from documented calls and says
   so.
3. **Fail with the controller's reason.** A refused call is answered with
   the controller's message, its alarm numbers and the state that explains
   it, gathered after the failure, never with extra calls before each
   command.
4. **Our own guards are few, explicit and switchable,** each marked as the
   driver's in its documentation (§5.4).
5. **SI and Eigen at the API**, controller units only at the wire; the typed
   call (`robot.call(ec::sdk::X{...})`) is always there for everything.
6. **One call in flight; never unconfirmed bursts.**
7. **A motion lives as long as its object;** a session as long as its
   `Robot` (§5.5).

## 5. The design

The shape stays: one library, the wire layer underneath, `ec::Robot` for
one controller, small motion families above it. What changes is what
`Robot` does for a session, and a few guards.

### 5.1 The session: `ec::Robot`

```cpp
namespace ec {

class Robot {
public:
    static std::expected<Robot, Error> connect(RobotOptions);   // 8055 + 8056

    // Session (the vendor's ceremonies)
    std::expected<BringUpReport, Error> bring_up(BringUpOptions = {});
    std::expected<void, Error> shut_down(ShutDownOptions = {});
    std::expected<Status, Error> status();               // the controller's state, gathered
    void stop() noexcept;                                // any thread

    // State and the controller's computations, in SI (a thin view over typed calls)
    std::expected<Joints, Error> joints();               // get_joint_pos
    std::expected<Eigen::Isometry3d, Error> tcp_pose();  // get_tcp_pose, base, current tool
    std::expected<Joints, Error> ik(const Eigen::Isometry3d& T_base_tcp, const Joints& near);
    std::expected<std::vector<Joints>, Error> ik_all(const Eigen::Isometry3d& T_base_tcp);
    std::expected<Eigen::Isometry3d, Error> fk(const Joints&);
    std::expected<double, Error> speed_scale();          // getSpeed: the PLAY/REMOTE slider, 0..1
    std::expected<void, Error> set_speed_scale(double);  // setSpeed

    // Every SDK call, typed
    template <sdk::Command C> std::expected<typename C::Reply, Error> call(const C&);

    // The 8056 stream
    std::optional<Feedback> feedback() const;
    std::optional<Feedback> next_feedback(std::uint64_t after, std::chrono::nanoseconds timeout) const;

    ~Robot();   // stops an open motion: nobody supervises it any more (§5.5)
};

struct Status {                 // all of it reported by the controller
    RobotMode mode;             // the key switch
    RobotState state;           // stop, pause, E-stop, running, alarm, collision
    bool estop, servo_on, synced, precise, collision;
    double speed_scale;         // the slider
    std::vector<std::int64_t> alarms;   // getAlarmNum: the last five
    // Why the arm cannot take a motion command now, in words; empty when it can.
    std::vector<std::string> not_ready() const;
};

}
```

- **`bring_up()`** is today's sequence (§R.5.10): clear an alarm, synchronise,
  switch the servo on and wait for it; it now also reads `precise` and
  reports what it did and what it could not do (not REMOTE, emergency stop
  latched, imprecise: steps that need the operator).
- **`shut_down()`** ends a session: the open motion's stop, `stop`, wait for
  state stop, `set_servo_status 0`. With `{.power_off = true}`, also
  `set_robot_power_status 0` once the call is confirmed on V3.15.3 (§R.8).
  It does not touch the key switch or the box's power.
- **`status()`** reads 8056 where it can and asks for the rest (sync,
  precise, E-stop, alarms, slider). `not_ready()` turns it into reasons: "the
  key is on PLAY", "emergency stop pressed", "servo off", "imprecise: move
  each axis or calibrate", "alarm 0-1030-ca".
- **The view in SI** is the user's request: `robot.joints()` reads better
  than `robot.call(ec::sdk::GetJointPos{})` and a unit conversion. It is a
  view, not a second path: each member is one typed call and its conversion,
  documented as such, and `call<C>` stays for the rest.

### 5.2 Motion families

Same shape as now: the constructor takes the inputs and sends nothing; the
methods are the family's calls in the manual's order; waits end on state
stop, fault or `Robot::stop()`.

| Family | Methods | Calls |
|---|---|---|
| `Move` (exists) | `joint`, `line`, `arc`, `rotate`, `line_in(frame)`; `wait`, `stop` | `moveByJoint`, `moveByLine`, `moveByArc`, `moveByRotate`, `moveByLineCoord` |
| `Path` (new) | `add(waypoint)`, `run`, `index`, `wait`, `stop` | `clearPathPoint`, `addPathPoint`, `moveByPath`, `getPathPointIndex` |
| `Jog` (new) | `jog(axis, direction, speed)` re-sent by the application, `stop` | `jog`, `stop` (the controller stops 1 s after the last `jog`) |
| `SpeedStream` (new) | `joints(qd)`, `tcp(v)`, `stop` | `moveBySpeedj`, `moveBySpeedl`, `stopj`, `stopl` |
| `ServoStream` (exists) | `open`, `add_point`, `close`, `stop` | transparent transmission |
| `Timestamped` (exists) | `upload`, `start`, `pause`, `resume`, `wait`, `stop`, `flush` | the timestamp service |
| `Program` (new) | `run(name)`, `pause`, `resume`, `wait`, `stop` | `checkJbiExist`, `runJbi`, `getJbiState`, `pause`, `run`, `stop` |

Changes to the existing ones:

- **Waits read 8056** when the stream is on: the same states as
  `getRobotState`, without using the command channel, so `stop()` never
  queues behind a wait. Without the stream they poll as the manual does.
- **A refused command is explained** (principle 3): on −32693 or `false`,
  the error carries `status().not_ready()` and the last alarms.
- **Speeds of the types VJ, V and VR are scaled by the slider** (§R.5.5);
  documented on each option, with `speed_scale()` beside.

### 5.3 What the application gets

```cpp
auto robot = ec::Robot::connect({.host = "192.168.0.200"}).value();
if (auto up = robot.bring_up(); !up) { /* the report says what the operator must do */ }

ec::motion::Move move(robot);
move.joint(trajectory.front().value, {.speed = 0.1}).value();
move.wait().value();

ec::motion::Timestamped run(robot, trajectory);   // the application planned it
run.upload().value();
run.start().value();         // guarded: the arm at the first point, speed ≤ 1 (§5.4)
if (auto done = run.wait(); !done) std::println("{}", describe(done.error()));  // with the controller's alarms

robot.shut_down().value();   // stop, servo off; the key and the power stay with the operator
```

### 5.4 The driver's own guards

Each is ours, not the controller's, documented as such, with a default and
a way to switch it off:

| Guard | Why | Default |
|---|---|---|
| `Timestamped::start`: the arm within a tolerance of the first point | if the arm is elsewhere, what the controller does is unknown (§R.8 #7); MoveIt checks the same before executing (`allowed_start_tolerance`, 0.01 rad) | 0.01 rad, from the 8056 joints (one `get_joint_pos` without the stream) |
| `Timestamped::start`: speed at most 1 | the manual bounds `speed_percent` only from below (≥ 0.1 %); above 100 % the trajectory runs faster than it was planned | 1, `allow_faster` to lift it |
| Never send a call before the last one is answered | the controller wedged once (§R.6.2) | always |

Everything else (limits, singularities, smoothness) is the application's
planning or the controller's safety.

### 5.5 Lifetimes

- A motion object owns its motion: destroying an open one sends its stop
  (`ServoStream` does this again since 2026-10-21; the proposal extends it to
  all families). Keep the object alive for as long as the motion should run.
- The `Robot` owns the session: destroying it stops an open motion, since
  nobody supervises it any more. It does not switch the servo off;
  `shut_down()` does that deliberately.
- If the process dies, nothing on the PC runs: the controller finishes a
  timestamped trajectory, plays out a streaming queue, stops a jog after
  1 s and a velocity command after its `t` (§R.5.7). Applications should
  know this; the emergency stop stays the safety device.

### 5.6 Diagnostics

- Errors carry the controller's message and, on a fault, the last alarm
  numbers (`getAlarmNum`).
- A small client for the 8058 log (`ec::ControllerLog`: the last N lines of a
  type), and a tool `ec_log`, so that an alarm can be read as the controller
  wrote it.
- A call log: every request and reply with its PC time, recorded next to the
  8056 frames (`ec_record`), so that a session can be replayed and explained.

## 6. Where I push back

- **"Servo off after every motion"**: no. The controller holds the arm still
  under servo control for as long as needed; every servo-on releases and
  checks the brakes, about a second with a small motion; the brakes are for
  power-off and emergencies (§R.3.3, §R.5.10). Servo off belongs to
  `shut_down()`, at the end of a session or before leaving the arm.
- **A planner check from the controller**: it offers less than hoped. There
  is inverse kinematics per pose (nearest to a reference, or all solutions),
  forward kinematics, frame conversions and linear interpolation between two
  poses; `check_trajectory` checks only the count, the format and the
  timestamps (§R.5.7). There is no dry run and no "would this move succeed":
  a move that cannot be done fails when it runs, with an alarm such as "the
  target pose is singular" or "joint speed exceeds limit". The IK view in
  §5.1 is what the controller can check in advance; the rest stays the
  application's.
- **An upper speed limit "from the manuals"**: the manual gives none for
  `start_trajectory`. The cap at 1 is our policy, said so and switchable
  (§5.4).
- **Two encoders do not mean the driver must manage them**: precise and
  synchronised are the controller's states. The driver reads them, runs the
  vendor's sequence and says what is missing; calibration that moves the arm
  stays an explicit, separate call.
- **Unconfirmed sends, even where the manual's examples use them**: not
  after what happened on 2026-10-14. If transparent transmission needs more
  throughput, measure the round trip first (`robot_checks.py rtt`).
- **A second connection** (for a separate stop path, or a split
  receive/control interface as in ur_rtde): not until §R.8 #4 is measured.
- **The ADRs and old documents**: keep them as history, but make this
  document and `docs/ec66.md` the reference. `docs/motion.md` §5.1–§6 (the
  removed runtime) can go to an archive.

## 7. Plan

About three weeks, each ending with a hardware session the user runs from
commands written for it.

| Week | Driver | On the robot |
|---|---|---|
| 1 | `Status` and `not_ready()`; `bring_up()` with its report; `shut_down()`; the SI view (`joints`, `tcp_pose`, `ik`, `ik_all`, `fk`, `speed_scale`); errors with alarms; `Timestamped` guards; waits on 8056; the `Robot` and motion lifetimes; tests against the fakes | the read-only checks (§R.8), then bring-up, one joint move, the circle, `shut_down()` |
| 2 | `Path`, `Jog`, `SpeedStream`, `Program`; the 8058 log client and `ec_log`; the call log in `ec_record` | each family at 10 % with a stop; what a disconnect does mid-motion; the streaming queue running empty; the slider's effect on trajectories |
| 3 | the first real application against the API (a capsnav trajectory through `Timestamped`; a live stream through `ServoStream`), fixing what it finds awkward; a short guide; pruning the documents | the application's runs |

## 8. Decisions for the user

1. **Lifetimes**: every motion object stops its motion when destroyed, and
   the `Robot` stops an open motion when destroyed (recommended); or only
   `ServoStream`, as now.
2. **`shut_down()`**: servo off only (recommended until the power call is
   confirmed), or the arm's power off too.
3. **The start guard's tolerance**: 0.01 rad, as MoveIt (recommended), or
   another value.
4. **Go ahead with week 1?**
