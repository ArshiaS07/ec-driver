# Motion and control

**Status:** motion families on one connection, no control thread (ADR 0018,
proposed), 2026-10-14, after the first runs on the robot (§5.0). Sections
1-4 describe the controller and the field and still hold. §5.0 is the design
as built now. §5.1-5.10 and §6 describe the earlier runtime (ADR 0014: one
control thread, activities, jobs, sessions), superseded and kept for its
reasoning. §5.11 (TCP targets, planned in joint space) still holds. Section 7
is what is assumed until measured on the robot.

**Sources**, in order of authority (§5.1): the vendor's manuals, cited as
[S] SDK socket manual 3.19.2 (English) / 3.11.2 (Chinese original),
[U] EC66 user manual 3.19.2, [J] JBI manual 3.19.2, [D] EC66 datasheet;
then our measurements on the EA66; then the references in §8 ([R1] ...).
A citation like [S 2.2.7.1] names a manual section.

---

## 1. Summary

- The controller offers two kinds of motion. **Planned** (offline): the whole
  motion is handed over, and the controller times and executes it
  (point-to-point, waypoints, timestamped trajectories, programs).
  **Streamed** (online): the PC decides while the arm moves (position
  streaming, velocity, jog).
- Each motion family has its own ceremony in the SDK manual (set-up, its
  calls, its stop, its tear-down). The driver has one small class per family
  that sends exactly that sequence over one connection, every call answered
  before the next (§5.0).
- `ec::Robot` is one controller: the command connection, the monitoring
  stream's newest frame, the controller's kinematics, bring-up, and a stop
  callable from any thread. One motion is open at a time.
- No control thread: set-up calls block, waits poll `getRobotState` as the
  manual's examples do, and streams are paced by the application's loop.
  Safety is the controller's (E-stop, pendant) as much as the driver's.

---

## 2. How the controller works

### 2.1 Modes, power, states, stops

- **Modes** [U 6.2]: TEACH (pendant jog, programming), PLAY (stored programs),
  REMOTE (external control, including the SDK), chosen with the key switch on
  the pendant. The SDK can read the mode (`getRobotMode`, 8056 `robotMode`) but
  not change it: a person decides who controls the arm.
- **Power and readiness**: the vendor's examples bring the arm up in this order
  [S 2.2.7.6, 2.2.9.10]: clear an alarm (`clearAlarm`), synchronise the encoders
  if needed (`syncMotorStatus`; 8056 `can_motor_run`), switch the servos on
  (`set_servo_status`; 8056 `servoReady`). It is the robot-level form of the
  drive power state machine of CiA 402 (switch-on disabled, ready, operation
  enabled; fault, fault reset) [R3].
- **Robot state** [S 2.2.2.1]: 0 stop, 1 pause, 2 emergency stop, 3 running,
  4 alarm, 5 collision. One motion source is active at a time. The vendor's
  examples take "state back to 0" as "motion finished" [S 2.2.3.1].
- **Stops** [U 7.4, 14.2; R13]: emergency stop is IEC 60204-1 category 1 (controlled
  stop, then power off; PLd); safeguard stop is category 2 (controlled stop,
  power kept; PLd); an alarm stops the arm at once; collision detection stops it
  (lower sensitivity value = triggers at smaller force). The SDK's `stop` is an
  ordinary command, **not a safety function**, and nothing in this driver is one.
- **Speed and limits**: PLAY and REMOTE share one speed setting [U 6.5.12]
  (`setSpeed`, 0.05–100 % [S 2.2.3.15]). EC66 [D]: joints ±360°; J1–J2 150°/s,
  J3 190°/s, J4–J6 260°/s; typical TCP speed 1.5 m/s; payload 6 kg; reach
  914 mm; repeatability ±0.03 mm.

### 2.2 Two ports, two directions

- **8055, commands**: JSON-RPC, one request per line, one reply each. A reply
  means *accepted*, not *done*: the vendor's examples send `moveByJoint`, then
  poll the state until it is back to stop.
- **8056, state**: a 1024-byte frame pushed about every 8 ms; "if the access
  time exceeds 8 ms, the data will pile up in the buffer" [S 3]. Measured on
  the EA66: 120–122 frames/s (8.2–8.3 ms), so frames are not an exact clock.
  The driver reads 8056 continuously on its own thread (ADR 0013) and judges
  completion, readiness and faults from it.

### 2.3 Motion families

| Family | The PC sends | Timed by | Ends when | If the PC dies |
|---|---|---|---|---|
| Point-to-point: `moveByJoint`, `moveByLine`, `moveByArc`, `moveByLineCoord` [S 2.2.3] | one target, speed, acc/dec % | controller | state back to stop | runs to the target |
| Waypoints: `clearPathPoint`, `addPathPoint` × N, `moveByPath` | points, move types, blend radii | controller | state back to stop | runs to the end |
| Timestamped trajectory: `start_push_pos`, `push_pos` × N, `stop_push_pos`, `check_trajectory`, `start_trajectory` [S 2.2.9] | ≥ 3 points (joint or pose) with times from 0 s; validated before start; time scaling, pause, resume; replayable until `flush_trajectory` | controller | trajectory end | expected to run to the end (to verify) |
| Program: `runJbi`, `pause`, `run` | a program name | controller | program end | runs on |
| Position stream (transparent transmission) [S 2.2.7, J 2.4.11] | one joint target per sampling period | **controller takes one point per T; the PC must keep up** | the PC closes it | undocumented (to verify) |
| Velocity: `moveBySpeedj`, `moveBySpeedl`, `stopj`, `stopl` [S 2.2.3.16–19] | velocity, acceleration, duration t; the newest command replaces the last | controller | t expires, or `stopj`/`stopl` | stops when t expires |
| Jog: `jog` [S 2.2.3.8] | axis direction and speed %, repeated | controller | `stop`, or 1 s without a jog | stops after 1 s |

The first four are **planned**: the whole motion is known before it starts,
and the controller needs nothing more from the PC. The last three are
**streamed**: the PC decides while the arm moves.

### 2.4 Transparent transmission (position streaming)

The JBI manual states the mechanism [J 2.4.11]: "T: Sampling time. Robot takes
position information once in this time interval. LOOKAHEAD: Forward time.
Robot waits for this time before start taking position in queue. SMOOTHNESS:
Trajectory gain. It will be used to smooth the path." The SDK's
`transparent_transmission_init` takes the same three values (lookahead
10–1000 ms, t 2–100 ms, smoothness 0–1) plus `response_enable` [S 2.2.7.1].
So the stream is a queue between two clocks:

```
PC: one point every T  --8055-->  [ queue, ~L/T points ]  -->  controller: starts L after the
    tt_put_servo_joint_to_buf                                    first point, then takes one point
                                                                 every T, smooths, drives the servos
```

What follows from it, each point a requirement on the design:

1. **Each point stands for one period.** Points sent at another rate distort
   time: a camera pushing 30 targets/s into a T = 8 ms queue makes the arm move
   about 4× too fast, then run dry. The application's rate must be converted to
   1/T: **resampling is mandatory**, by whatever sits between the application
   and the queue.
2. **The consumer sets the pace.** On average exactly one point per T. Faster
   fills the queue (latency grows); slower empties it (underrun, behaviour
   undocumented). The queue absorbs short-term jitter of about ±L.
3. **LOOKAHEAD is latency.** A target reaches the arm about L after it was sent
   (plus network and smoothing): the controller gets its "future" by delaying
   the present. This is a playout buffer, with the same trade-off as audio over
   a network [R14]: a small L (2–5 periods) for closed-loop work such as vision
   or force, a large L for robust playback (the vendor's examples use 400 ms).
4. **The two clocks drift.** The queue integrates the small rate difference
   between the PC's clock and the controller's. That matters only for long
   continuous streams (§7.1).
5. **T is a whole number of ms**, while frames come every 8.2–8.3 ms: the
   producer cannot be clocked by the frames. It runs on its own timer, with
   absolute deadlines.
6. **The vendor's sequence** [S 2.2.7.4–7]: `moveByJoint` to the first point,
   wait for stop, `transparent_transmission_init`, one
   `tt_put_servo_joint_to_buf` per period, `tt_clear_servo_joint_buf`. With
   `response_enable = 0` the puts get no reply.

The parameter names resemble Universal Robots' `servoj` (`lookahead_time`,
`gain`) [R6], but Elite defines LOOKAHEAD as a queue start delay, not as UR's
prediction horizon. The 3.11.2 manual already accepts a Cartesian target
(`targetPose`) instead of joints [S 2.2.7.4], without giving its units;
whether firmware V3.15.3 does is untested (§7, #8). The driver streams joints
only; a TCP target is solved first (§5.11).

### 2.5 A session, end to end

1. A person sets REMOTE on the pendant.
2. The PC connects 8056 (state) and 8055 (commands) and checks that the state
   is fresh.
3. Bring-up: alarm cleared, encoders synchronised, servos on, each confirmed in 8056.
4. Motion: planned motions run one at a time from standstill; a stream opens at
   standstill, runs, and is closed.
5. Stop at any time; on a fault the controller or the driver stops the arm, and
   a person decides what happens next.

---

## 3. What is logically required

1. **Causality.** No controller can follow a future it has not been given. A
   streamed motion becomes smooth in three ways, which combine:
   - *delay*: buffer, and fit a smooth curve through what is buffered (LOOKAHEAD);
   - *filtering*: damp fast changes, at the cost of phase lag (SMOOTHNESS);
   - *limits*: move toward the newest target as fast as velocity, acceleration
     and jerk limits allow, re-planned every cycle. This is online trajectory
     generation [R11, R12]: smooth by construction and reactive, with no
     future needed. Classical trajectory planning [R1] assumes the whole path
     is known; online generation drops that assumption.

   A target that jumps is never forwarded as a jump.
2. **Two clocks, one resampler.** The application produces on its clock and the
   controller consumes on its own; exactly one component, in one thread,
   converts between them, at the consumer's rate.
3. **Observation is the truth.** A reply means accepted; 8056 shows what
   happened. Completion, readiness and faults come from the newest state, never
   from what was sent.
4. **Every stream needs a guard.** If targets stop, the arm must stop. Velocity
   and jog have it built in (t, 1 s); position streaming documents none, so the
   driver provides one.
5. **Soft real time.** Periods are milliseconds, and the network and the OS
   scheduler dominate the jitter. The loop keeps the cheap real-time rules
   (absolute deadlines, bounded work, no disk I/O, no unbounded waits); heavier
   measures (real-time priority, a PREEMPT_RT kernel, allocation-free paths)
   wait for measurements that ask for them.

---

## 4. How others do it

| System | Planned motion | Streamed motion | Who runs the fixed-rate loop | Guard on silence |
|---|---|---|---|---|
| CiA 402 drives [R3] | profile position (pp) | cyclic synchronous position, velocity (csp, csv); interpolated position, buffered (ip) | the fieldbus cycle | EtherCAT sync-manager watchdog |
| PLCopen motion [R2] | MC_MoveAbsolute etc. (Execute, Busy, Done, Error) | MC_MoveVelocity, synchronized motion | the PLC scan | errors latch in ErrorStop until MC_Reset |
| ros2_control [R4] | trajectory controller | position and velocity command interfaces | the controller manager: read, update, write | command timeouts (e.g. `cmd_vel_timeout`); commands enter through a newest-wins real-time buffer |
| MoveIt Servo [R5] | | twist or joint-jog commands | its own fixed-rate loop | `incoming_command_timeout`: halt |
| Universal Robots, ur_rtde [R6] | moveJ, moveL (blocking or asynchronous) | servoJ (lookahead, gain), speedJ | the application, at 500 Hz | `setWatchdog`, `kickWatchdog` |
| Franka, libfranka [R7] | | a 1 kHz callback returns the next command | the robot calls the application (pull) | a missed deadline stops the arm |
| KUKA RSI, ABB EGM, FANUC stream motion [R8] | | cyclic corrections or positions, 4–12 ms | the robot (pull) | late or missing packets stop the arm |
| **Elite EC (this robot)** | point-to-point, waypoints, timestamped trajectory, programs | position stream, velocity, jog | **open: the controller only empties a queue** | velocity: t; jog: 1 s; stream: undocumented |

What they share:

1. Planned and streamed motion are always separate interfaces.
2. Some fixed-rate loop at the robot's period always exists. Either the robot
   calls the application every period (pull: libfranka, RSI, EGM, which demand
   hard real time from the application), or the application must run that loop
   itself (ur_rtde's servoJ), or a driver-side loop runs it and the application
   pushes at any rate (ros2_control, MoveIt Servo).
3. Only the last tolerates an application that is slow or irregular (Python,
   vision, planners). It needs a newest-wins buffer and a guard on the age of
   the newest command.
4. Errors latch and need an explicit reset.

This robot is the soft-real-time case: a queue of 8 ms periods, a network in
between, applications of any kind. That points to the third option.

---

## 5. Design

### 5.0 Motion families (ADR 0018), as built

**Why the change.** The first runs on the EA66 (2026-10-14) met three
problems, all from the runtime of §5.1-5.10 rather than the controller:

- `set_servo_status` answered later than the loop's 50 ms reply timeout, so
  bring-up failed.
- The timestamp upload ran inside the 8 ms cycle. Confirmed points made
  cycles late, and the late-cycle rule, made for streams, stopped a planned
  motion.
- Unconfirmed points backed up on the controller. The clean-up and the
  disconnect then left both ports refusing connections until the
  controller's power was cut.

The manual's own example, sent call by call, drew the circle.

**The design.**

| Part | Where | What |
|---|---|---|
| `ec::Robot` | `ec/robot.hpp` | `connect` (8055; 8056 if `monitor`), `call<C>` (one typed call, answered; while a motion is open, queries and computations only), `feedback`/`next_feedback` (the newest frame, decoded; `on_feedback` with every frame), `ik`, `fk`, `joints`, `bring_up` (the manual's sequence, waiting for the servos), `stop` (any thread: the open motion's stop, then `stop`) |
| Errors | `ec/error.hpp` | one `ec::Error` (invalid, busy, command failed, fault, stopped, timeout, disconnected) |
| Point-to-point | `ec/motion/move.hpp` | `Move::joints` (moveByJoint); `wait_for`, `wait` (until it rests at the target), `stop` |
| Timestamped trajectory | `ec/motion/timestamped.hpp` | `Timestamped::upload`: checked, the arm at the first point; start_push_pos, 0.2 s, push_pos per point, 1 s, stop_push_pos, check_trajectory. Then `start`, `pause`, `resume`, `wait_for`, `wait`, `stop` (stop_trajectory) |
| Servo streaming | `ec/motion/servo_stream.hpp` | `ServoStream::open`: a stream left open cleared first, then transparent_transmission_init. Then `add_point` (tt_put_servo_joint_to_buf) and `close` (tt_clear_servo_joint_buf); paced by the application, no watchdog yet |
| TCP motions | `ec/motion/move_along.hpp` | `plan(robot, poses)` (§5.11); `move_along`: plan, a joint move to the first pose, the timestamped run |

Each family's factory does the manual's set-up and returns the motion or
why not. A failed set-up sends nothing to clean up. Waits ask
`getRobotState` every 10 ms. Until the motion is seen running, a stop
counts only a second after the start, since a motion just started may not
show yet. The only thread in the driver is the optional 8056 receiver.
`apps/interrupt.hpp` turns Ctrl+C into `Robot::stop()` for the tools.

Following, in milestone B: the waypoint queue, jog, velocity streaming,
stored programs, `Move` line and arc, a watchdog for `ServoStream`, and a
log of every call and answer.

### 5.1 Order of authority

1. **What the robot does**: the vendor's manuals, overruled by observed
   hardware behaviour.
2. **What is possible**: causality and timing (§3). Not a matter of taste.
3. **How the software is structured**: industry practice (§4) and established
   principles: SOLID [R16], functional core and imperative shell [R18], I/O-free
   protocol code [R19], one writer per piece of data [R20]. They are the
   default because they carry field experience and make the code familiar.
4. **Our own patterns**: only where 1–3 leave a gap, each with its reason in an ADR.

Safety standards are not a choice on this list: the driver never stands in for
the robot's safety functions.

### 5.2 Threads and data flow

```
application threads               control thread, one per robot                  receiver thread
-------------------               -----------------------------                  ---------------
job = robot.run(...)  --request-> every period T, at absolute deadlines:         8056 -> Sample
stream.target(q)      --mailbox->  1 read    newest state, requests, targets  <-- Latest<Sample>
robot.stop()          --flag--->   2 decide  supervision + the activity (pure)
robot.state()         <--copy---   3 act     send this cycle's commands (8055)
job.wait()            <--status--  4 publish status, faults, log records
```

- **Receiver thread** (built, ADR 0013): 8056 into `Latest<Sample>`.
- **Control thread**: the only user of the 8055 connection, so "one source
  moves the robot" holds by construction, without lease objects. Its period is
  the stream sampling period T, set per connection (8 ms by default).
- **Application threads** never touch a socket and never run inside the loop.
  They submit requests, write targets into newest-wins mailboxes (`Latest<T>`),
  and read copies of the state. A one-off query is a request too, sent by the
  control thread with the cycle's commands.

### 5.3 Activities: one contract for whatever the control thread runs

An activity is a state machine without I/O. Each cycle it reads the newest
state and the replies to what it sent the cycle before, and appends the
commands to send now. The motion families are activities; so are bring-up,
compositions and an application's own motions.

```cpp
namespace ec::motion {

struct Cycle {                             // one cycle's inputs, copied at its start
    Clock::time_point now;
    std::chrono::nanoseconds period;       // the loop's nominal period
    feedback::State state;                 // newest 8056 state, SI
    std::uint64_t state_seq;
    std::chrono::nanoseconds state_age;
    Conditions conditions;                 // observe(state, state_age, thresholds)
    std::span<const Reply> replies;        // to the commands sent in the previous cycle, in order
};

enum class Phase { Running, Done };
enum class End { Done, Cancelled, Stopped, Faulted };  // why close() is called
struct Needs { Conditions to_start; Conditions while_running; };

template <class A>
concept Activity = std::move_constructible<A> && requires(A& a, const A& ca, const Cycle& c, Outbox& out, End why) {
    { ca.needs() } -> std::convertible_to<Needs>;                       // may change as it advances
    { a.step(c, out) } -> std::same_as<std::expected<Phase, Error>>;   // opens, then runs
    { a.close(c, out, why) } -> std::same_as<bool>;                    // true once closed
};

}  // namespace ec::motion
```

- **Lifecycle**, enforced by the control thread: `step` every cycle until
  `Done` (the first steps open the family's protocol), then `close` every cycle
  until it returns true. Close also runs after a cancel, a stop or a fault,
  like `finally`. Its contract: **leave the arm at rest and the controller's
  protocol clean**, whatever the reason. Opening and closing are phases, not
  single calls, because they wait for replies or for standstill.
- **Outbox**: activities write typed commands (`out.send(MoveByJoint{...})`,
  or `out.send_without_reply(...)` after asking the controller not to answer).
  Whether the control thread then sends them one by one, pipelined, or over a
  second connection is its own business; activities do not change.
- **Specs**: the library's motions are plain data, like the command structs
  (`MoveJoint{.target = q, .speed = 0.2}`); `make_activity(spec)`, found by
  argument-dependent lookup, builds the activity that runs one.
- **AnyActivity**: the control thread holds the current activity in a
  type-erased wrapper (§5.8); it accepts activities and specs alike.

### 5.4 Planned motion: jobs

```cpp
ec::motion::Job job = robot.run(ec::motion::MoveJoint{.target = q, .speed = 0.2});  // returns at once
if (auto done = job.wait(); !done) report(done.error());  // arrival seen in 8056, or why not
auto maybe = job.wait_for(10s);                            // nullopt: still running
job.cancel();                                              // stop the arm, then close
```

- `run` accepts any `Activity`; the job is its handle (busy, then done, failed
  or cancelled: the PLCopen pattern [R2]).
- One activity at a time: `run` while busy is refused, not queued, because the
  controller starts a point-to-point move only from standstill anyway.
  `Sequence{a, b, ...}`, itself an activity, runs several back to back (PLCopen's
  "buffered" mode).
- Dropping a job handle leaves the motion running, as dropping a `std::future`
  leaves the work running; `cancel()` stops it.

### 5.5 Streamed motion: sessions the application pushes into

```cpp
auto servo = robot.servo({.lookahead = 40ms, .smoothness = 0.1}).value();  // a motion::Session<Joints>
servo.target(q);           // any thread, any time: the newest wins, stamped on arrival
servo.close();             // decelerate, let the queue drain, clear it; the destructor does the same
```

`robot.servo()` returns once the controller has accepted the stream (or with
why it did not). For TCP targets the application solves each one with
`robot.ik()` and pushes the joints (§5.11).

Each cycle, the stream activity:

1. takes the newest reference (a target, or a trajectory) from the mailbox;
2. **guards** it: once its age passes `watchdog.target_timeout`, the reference
   becomes "stop where you are" (hold); after `watchdog.hold_for` more, the
   activity fails with `Starved` and closes. This is the watchdog, and all it checks is that the
   application keeps pushing;
3. **shapes** it: the next setpoint moves toward the reference within velocity
   and acceleration limits. A rate limiter is the default; online trajectory
   generation such as Ruckig [R12] is a drop-in alternative. Jumps become ramps,
   and a hold becomes a smooth stop;
4. checks joint limits, and tracking (measured position against the setpoint
   sent about L earlier);
5. sends one `tt_put_servo_joint_to_buf`.

A stream opens only at standstill and starts from the **measured** position
(bumpless transfer), so nothing moves until the application pushes a target.
Velocity streams (`robot.velocity(...)`: speedj or speedl re-sent every cycle
with t of a few periods; a stale target becomes zero velocity, then
`stopj`) follow the same pattern.

### 5.6 Supervision

- **Remembered**: idle, busy (one activity), fault (latched). Nothing else.
- **Recomputed every cycle** by a pure function of the newest state: the set of
  conditions that hold (a bitset).

| Condition | From 8056 | Needed by |
|---|---|---|
| Fresh | state age below 50 ms (about 6 frames) | everything except stop |
| Remote | robot mode remote | every activity |
| NoEmergencyStop | E-stop flag clear, state not 2 | every activity |
| NoAlarm, NoCollision | state not 4 or 5, collision flag clear | motion |
| ServoOn, Synced | `servoReady`, `can_motor_run` | motion |
| Standstill | state stop, joint speeds near 0 | starting a planned motion or a stream |

- An activity declares what it needs (`A::needs`). A missing condition refuses
  a start (an error, no fault) or, during a run, ends the run with a fault.
- **Faults** latch with what was observed: a condition lost, a starved stream,
  a tracking error, a refused command, a job past its time limit, a late loop.
  The activity's close runs, the arm is stopped if the controller has not done
  so, and further runs are refused.
- **Recovery is manual by design**: a person fixes the cause, the application
  calls `acknowledge()`, then `bring_up()`, itself an activity: clear alarm,
  reset collision, synchronise, servo on, each confirmed in 8056. No automatic
  retries, no retreat moves. `acknowledge()` clears the latch whatever the
  state shows: it records a person's decision, while the start conditions keep
  guarding what may run. (Refusing it while the cause is visible, as first
  proposed, would deadlock after a collision: the collision flag clears only
  through bring-up, which could not start while the fault was latched.)
  Bring-up needs only a fresh state, remote mode and no emergency stop; every
  motion still needs everything else.
- `robot.stop()`, from any thread: an atomic flag, seen at the next cycle
  (within T). The activity's close then sends its family's fastest stop
  (`stop`, `stopj`, `stop_trajectory`, ...).

### 5.7 Changes it absorbs

| Change | What is touched | What is not |
|---|---|---|
| A new controller family or firmware feature | one new activity, and its commands | control thread, supervision, the facade's core |
| A new control law in an application (vision, teleoperation, force) | application code, which pushes targets | the driver |
| A custom motion (move until contact, a square, a search) | an activity or a sequence, in the application | the driver |
| Another smoothing method | a shaper (strategy) | the stream activity |
| A new safety condition | one condition and its check | everything else |
| Another transport (pipelining, a second connection for stop) | the control thread's I/O | activities |
| Slaving the stream to the controller's clock | the cycle timer | activities |
| Another log format | a new `Sink` | everything else |

The same in SOLID terms [R16]:

- **S**: each component has one job (receiver, mailbox, supervision, each
  activity, shaper, the control thread's I/O, the facade).
- **O**: the table above. The control thread depends on the `Activity`
  contract, never on a family.
- **L**: any activity can take any other's place in the control thread; the
  wrapper enforces one contract.
- **I**: small contracts: an activity is two functions and a constant, a sink is
  one function, and application handles expose only their use (`Job`, `Stream`).
- **D**: the facade composes concrete parts and hands them in; tests hand in
  fakes.

A custom motion, for illustration: a move that stops on contact, wrapping an
existing activity (a decorator [R15]):

```cpp
struct MoveUntilContact {                   // satisfies ec::motion::Activity; no base class
    ec::motion::PointToPoint move;          // make_activity(ec::motion::MoveJoint{...})
    double force_limit_n = 20.0;

    ec::motion::Needs needs() const { return move.needs(); }
    std::expected<Phase, Error> step(const Cycle& c, Outbox& out) {
        const auto& f = c.state.tcp_force;
        if (std::hypot(f[0], f[1], f[2]) > force_limit_n) return Phase::Done;  // contact: close() stops the arm
        return move.step(c, out);
    }
    bool close(const Cycle& c, Outbox& out, End why) { return move.close(c, out, why); }
};
```

### 5.8 The C++ shape

**Why a concept, and why not only a concept.** A concept is a compile-time
contract: `template <Activity A>` documents and checks the requirement where a
type is used, with direct calls, no base class and no heap. It does not create
a run-time type, though: a concept alone cannot be stored in a member, chosen
at run time, or put in one container with others. A virtual base class creates
that run-time type, but ties every implementation to our class hierarchy
(inheritance, pointers, ownership and slicing questions) and to our ABI. The
established answer uses both [R17]. The concept is the public contract; one
wrapper, `AnyActivity`, erases it at the single place that needs run-time
choice (the control thread), with a private virtual interface inside.
Applications write plain values; tests drive concrete activities directly;
`Sequence<A, B>` composes at compile time. `std::function` is the same idea for
callables.

**Where `std::variant` fits.** For closed sets that we own: the supervision
states, a reference that is either a target or a trajectory. `std::visit` then
checks that every case is handled. It is the wrong tool for activities: an
application must be able to add one without editing our variant.

**Language baseline.** C++23 as accepted by GCC 13, Clang 19 (the first that
sees libstdc++'s `std::expected`) and MSVC 19.36 (Visual Studio 2022 17.6),
following the C++ Core Guidelines [R21]. Used, each for a reason:

- concepts, for contracts;
- `std::expected`: errors as values, and no exceptions through the loop;
- `std::jthread` and `std::stop_token`: threads that always join and stop
  cooperatively;
- `std::chrono` with `sleep_until`: typed time and absolute deadlines;
- `std::span`;
- designated initialisers, so options read like a datasheet
  (`{.lookahead = 40ms, .smoothness = 0.2}`);
- `std::variant`;
- `std::atomic`, for the stop flag;
- `std::move_only_function`, where a callback owns move-only state.

Left out:

- deducing `this`, `std::generator` and `std::print`: they need GCC 14;
- coroutines in the loop: their frames allocate, and the explicit state
  machine a coroutine would compile to is easier to test cycle by cycle.

### 5.9 Push and pull

At the bottom, the controller pulls: it takes one point every T. The design
question is only where pulling turns into pushing.

- **In the pull design** (libfranka, and this project's earlier drafts),
  the application's function runs inside the loop. It must answer within every
  period, and if it hangs, the loop hangs with it, so a second watchdog thread
  with its own connection is needed.
- **In the push design**, the driver's loop absorbs the controller's pull, and
  the application pushes when it can. Application code never runs in the loop:
  a slow or crashed application is just a stale target, and the watchdog is one
  comparison in a loop the driver controls. The application may be as slow,
  irregular or high-level as it likes.

A control law synchronous with the robot remains possible: the application
waits for each new state (`robot.next_state`) and pushes its output, which the
loop picks up within one period (§6.2 b).

### 5.10 As built

| Part | Where | Notes |
|---|---|---|
| Conditions, errors | `ec/motion/conditions.hpp`, `error.hpp` | `AtRest` (joints still, any state) was added next to `Standstill` (at rest and state stop): closing waits for rest, starting for standstill |
| The contract | `ec/motion/activity.hpp` | `Activity`, `Spec`, `AnyActivity`, `Cycle`, `Outbox`, `Reply` |
| Supervision | `ec/motion/executor.hpp` | idle, running, closing, fault; a close that does not finish within 2 s ends with a last-resort stop and a fault |
| Planned motion | `point_to_point.hpp`, `trajectory.hpp`, `sequence.hpp`, `bring_up.hpp` | `MoveJoint`, `PlayTrajectory` (with `JointTrajectory` and `validate()`), `Sequence`, `BringUp` |
| Streamed motion | `stream.hpp`, `shaper.hpp`, `session.hpp` | `ServoStream`, `VelocityStream`; `RateLimiter`; the application's `Session<T>` |
| The control thread | `ec/motion/control_loop.hpp` | `ControlLoop`, `Job`, `Status` |
| The facade | `ec/robot.hpp` | `ec::Robot`: `connect`, `state`, `next_state`, `status`, `ik`, `fk`, `move_joints`, `plan`, `play`, `run`, `bring_up`, `servo`, `velocity`, `stop`, `acknowledge`, `call<C>` |
| Poses | `ec/pose.hpp`, `ec/stamped.hpp` | `Eigen::Isometry3d` in the T_ab notation; `from_mm_rad`, `to_mm_rad`, `rotation_from_rxyz`, `rxyz_from_rotation` at the wire; `PoseTrajectory` (poses stamped with their time since the start, `ec::Stamped`), `validate()`, `interpolate()`, `pose_at()` |
| Planning | `ec/motion/planning.hpp` | `plan()`, `check_point()`, `check_step()`, `JointSpline`, `singularity_margins()` (§5.11) |
| Tools | `apps/ec_move.cpp`, `apps/sbx_trajectory.cpp` | `ec_move`: status, bring-up, joint, nudge, sine, stop, acknowledge; nothing moves without `--yes`. `sbx_trajectory`: a circle of the TCP from a free function (stamped poses), brought up, verified and executed by the timestamp service. `ec_move` has `--record` |
| Test bench | `tools/fake_robot.py`, `tools/kinematics.py` | a simulated arm on 8055 and 8056, with fault injection, an EC66-like kinematic model and a trace of its TCP |

Refinements of the design above, each for a reason:

- `Needs` has two sets (to start, while running) and is asked every cycle, so
  an activity can need standstill to start but not while it moves.
- `close()`'s contract is "leave the arm at rest and the protocol clean,
  whatever the reason": a wrapper that ends its inner activity early (the
  decorator example in §5.7) then gets a stop for free.
- A one-off call (`robot.call(command)`) is refused for motion commands, and
  for settings, actions and sessions while an activity runs: the command kind
  (ADR 0015) decides.
- The loop counts a cycle as late when it starts more than half a period after
  its deadline; three in a row while an activity runs are an overrun fault.
- The loop publishes its status before it completes the jobs that ended in the
  cycle, so a thread woken by its job reads the status of the cycle that ended
  it (CI found the other order racing, under ThreadSanitizer).
- The age of the state is measured when the loop reads it, not at the cycle's
  start: a frame that arrived in between had a negative age, which `observe()`
  rightly distrusts as coming from the future, and a running motion stopped
  with "fresh state" lost, at random (found by the circle tests on the fake).
- One-off calls are answered between cycles, as soon as they come, not at the
  next cycle: a path's inverse kinematics costs a round trip per point.
- From a review before the first robot session: `PlayTrajectory` stops the arm
  on any ending once `start_trajectory` went out, even before its reply; a closed
  path (ending where it starts) counts as arrived only after the arm moved; a
  shutdown that outlasts its time limit sends a last-resort stop.

Not built yet: slaving the stream to the controller's clock; an online
trajectory generator as the shaper (it would also take away the live streams'
lag, §5.11); logging of commands and events; a check before motion that the
controller's tool and frames are the ones the application expects (§5.11).
Left to the command layer, by choice (ADR 0017): the controller's own
Cartesian moves (`moveByLine`, `moveByLineCoord`, `moveByArc`, waypoint
paths) and jog.

### 5.11 TCP targets, in joint space

The arm moves in joint space only; a TCP target becomes joints through the
controller's inverse kinematics before anything moves (ADR 0017). This is how
ROS/MoveIt executes Cartesian paths: the robot never solves a pose while it
moves, so a singular or unreachable pose is found on the PC, at planning time.
The controller's own Cartesian moves stay typed commands for those who want
them.

**Poses** are `Eigen::Isometry3d`. `T_ab` is the pose of frame b in frame a:
p_a = T_ab · p_b; its translation is r_ab_a, its rotation q_ab (v_a = q_ab · v_b);
poses chain as T_ac = T_ab · T_bc. The robot's pose is `T_base_tcp`. Only at the
wire do they become the controller's x, y, z [mm] and rx, ry, rz [rad], with
R = Rz·Ry·Rx (ADR 0012).

```cpp
Joints q = robot.ik(T_base_tcp, near).value();               // inverseKinematic, nearest `near`
ec::motion::Move::joints(robot, q, {.speed = 0.2})->wait();  // moveByJoint
ec::PoseTrajectory motion{{0s, T_base_tcp}, {2s, T_base_goal}};  // poses stamped with their time
auto trajectory = ec::motion::plan(robot, motion).value();   // solved, checked, every 20 ms
ec::motion::move_along(robot, motion);                       // plan, move to the start, run timestamped
auto servo = ec::motion::ServoStream::open(robot).value();   // live: solve each target, check_step(), add_point()
servo.add_point(robot.ik(T_base_tcp, q).value());
```

**A motion** of the TCP is a `PoseTrajectory`: poses, each stamped with its time
since the start (a `Clock::duration`, the steady clock's), the first at 0 and
where the arm is. The application fills in the times; the library has no timing
law (yet). Between two poses the TCP goes straight, its orientation turning
evenly; `pose_at(motion, t)` reads it so, and `plan()` too. The shape follows
the navigation library this may later take motions from (capsnav: a value
stamped with its time, sampled at a duration since the start), without
depending on it: a pose there converts to an `Eigen::Isometry3d` and back.

`plan()` turns a motion into a joint trajectory:

1. **Fill in.** Neighbouring points further apart than 3 mm or 1° get points
   between them, on the straight line and turning evenly (slerp), timed in
   proportion.
2. **Solve every point** with the controller, each near the solution before (the
   first near the arm's joints), a solution given within one turn moved by whole
   turns to the nearest (`nearest_turn`).
3. **Check every solution** (`check_step()`, which a stream of poses uses too):
   no joint more than 10° from the one before (a change of arm configuration,
   or a singularity in between: MoveIt's jump threshold), J5 and J3 at least 5°
   from 0 and 180° (the wrist and elbow singularities of this arm, whose layout
   is Universal Robots'), within the joint limits. A pose stamped twice is a
   pause: it is not solved again, its joints are held.
4. **Interpolate over time** (`JointSpline`: cubic Hermite, Bessel's slopes; at
   rest at both ends and at both ends of a pause, the segments into and out of
   rest kept monotone so that the arm neither steps backwards from rest nor
   overshoots into it) and sample every 20 ms, ending exactly at the last time.
   The arm passes through every other pose without stopping: to stop there, the
   stamps must slow down to it.
5. **Check the trajectory**: joint speeds within the datasheet's, accelerations
   within the limit (from rest at the start, to rest at the end).

Each point costs one call; the control loop answers calls between its cycles,
so the sandbox's circle with its approach (412 poses over 20.5 s) plans in 0.1 to 0.3 s on the fake.
Interpolating over time across long gaps where a motion is slow strays from it
(tens of micrometres on a circle, measured before): motions stamp their poses
densely, the sandbox's every 50 ms.

The shoulder singularity (the wrist above or below the base) is not checked by a
margin: near it J1 turns fast, which the jump and speed checks catch. A true
distance from singularity (the Jacobian's smallest singular value, as MoveIt
Servo uses) needs a kinematic model; the controller's 12 DH values (`getDH`),
checked against `positiveKinematic`, would give one.

**Streams** of TCP targets are the application's loop: `robot.ik()` per target,
checked with `check_step()` against the solution before, the joints into a
`ServoStream` with `add_point()`, one every period. Since ADR 0018 the driver
neither shapes nor resamples them: the application sends smooth targets, as
the manual's example does. (The earlier stream's rate limiter trailed a moving
target by its braking distance: on the fake a 30 mm circle strayed 0.4 mm
streamed, 0.01 mm played.)

**Tools and frames** [S 2.2.2, U 6.11]: the controller keeps 8 tools (a TCP
offset from the flange, x, y, z within ±500 mm and rx, ry, rz, with the payload's
mass and centre of mass) and 8 user frames (full poses in the base frame, taught
by three points or set numerically). Two tool numbers are active: one in teach
mode (`getToolNumber`) and one in remote and auto mode (`getAutoRunToolNumber`);
SDK motion runs in remote mode. `inverseKinematic` and `positiveKinematic` take
neither tool nor frame: the base frame and the active tool (§7, #14). The driver
works in the base frame and leaves work frames to the application, as poses it
composes (T_base_work · T_work_tcp); a frame taught on the pendant can be read
once with `getUserFrame`.

`apps/sbx_trajectory.cpp` is the worked example, a sandbox to scroll through: a
free function returns the circle as stamped poses (centre and tool orientation
read on the pendant, radius, speed, number of poses); `main` connects, brings
the robot up, verifies the poses (`plan()`), moves to the first point with a
joint move, and executes the circle by the timestamp service. Ctrl+C stops the
arm.

---

## 6. The control loop, idiomatically (the superseded runtime, ADR 0014)

### 6.1 The driver's loop

```cpp
void ControlThread::run(std::stop_token stop) {
    auto deadline = Clock::now();
    while (!stop.stop_requested()) {
        deadline += period_;                           // absolute deadlines: no drift
        std::this_thread::sleep_until(deadline);
        const auto now = Clock::now();
        if (now - deadline > overrun_limit_) note_overrun(now - deadline);

        const Cycle cycle = read_inputs(now);          // 1 read: newest state, replies; copies only
        take_requests();                               //   run, stop, acknowledge, queries

        outbox_.clear();
        supervise(check(cycle), cycle);                // 2 decide (pure): conditions, then step
                                                       //   or close the activity into outbox_
        replies_ = send(outbox_, deadline + budget_);  // 3 act: bounded network I/O
        publish(cycle);                                // 4 status, faults, log records (to other threads)
    }
}
```

The rules, each standard in cyclic control [R4, R9, R10]:

1. Absolute deadlines on a monotonic clock (`next += T`). Never `sleep(T)`,
   which accumulates drift.
2. A fixed order: read every input, compute, then write every output. Inputs
   are copied once per cycle, so the computation sees one consistent snapshot.
3. Bounded work: no disk I/O, no waiting on other threads, no unbounded calls;
   every network call has a deadline within the cycle's budget.
4. A defined overrun policy: count late cycles; several in a row are a fault,
   since the stream's queue may be running dry.
5. Bumpless start, graceful end: start from the measured state; end by holding
   and decelerating, never by just ceasing to send.
6. Measure before tuning: histograms of cycle start times and command round
   trips on the real setup. Real-time priority (`SCHED_FIFO`), CPU pinning or a
   PREEMPT_RT kernel only if those numbers ask for it.

### 6.2 The application's loops

```cpp
// a) Application-clocked: targets come at the application's own rate.
auto v = robot.velocity({.acceleration = 1.0}).value();  // rad/s^2
while (running) v.target(visual_servo(camera.grab(), robot.state()));  // 30 Hz and jittery: fine
v.close();

// b) Robot-clocked: one output per 8056 frame.
auto servo = robot.servo({.lookahead = 24ms}).value();
for (std::uint64_t seq = 0;;) {
    const auto s = robot.next_state(seq, 50ms);          // waits for the next frame
    if (!s) break;                                       // no fresh state: the driver is already holding
    seq = s->seq;
    servo.target(control_law(*s));
}

// c) Move, then go online: plain sequential code.
robot.run(ec::motion::MoveJoint{.target = start, .speed = 0.2}).wait().value();
auto stream = robot.servo({}).value();                   // opens at standstill, from the measured position

// d) A recorded TCP path: solved, checked, and played by the controller (§5.11).
auto trajectory = robot.plan(timed_poses).value();      // the controller's IK, every point
robot.move_joints({.target = trajectory.position.front(), .speed = 0.2}).wait().value();
robot.play(trajectory).wait().value();                  // the timestamp service
```

---

## 7. Assumptions until measured

Each open question has a default that the design runs on, chosen so that a
different answer changes one place, never the structure. Four can be measured
now, read-only, with `tools/robot_checks.py`, and two (#10, #13) by planning a
circle without `--yes`. The others need motion; `ec_move` runs them against the
moving fake first.

| # | Question | Default until measured | If the answer differs | Measured by |
|---|---|---|---|---|
| 1 | Stream underrun: does the arm stop, decelerate, or raise an alarm? | The driver never lets the queue run dry on purpose: it sends a point every cycle (holding repeats the last setpoint) and closes deliberately (decelerate, wait for standstill, clear). An underrun then means the PC or the network stopped for longer than L; the arm is assumed to stop at the last point. | An alarm is an ordinary fault, seen in 8056. Any other answer changes only what the notes say the arm does if the PC dies. | motion |
| 2 | Round trip of `tt_put_servo_joint_to_buf` | One request and its reply per cycle (`response_enable = 1`), as in the vendor's first example at T = 10 ms; every point is confirmed. | `response_enable = 0`, as in the vendor's second example: an option of the stream, and the control thread's way of sending. | `robot_checks.py rtt` and `pipeline` now (queries, as a baseline); the put itself with motion |
| 3 | Drift between the controller's clock and the PC's | No correction. A stream measures the skew from the 8056 timestamps while it runs and reports it; the tracking check sees a growing lag. | The cycle timer trims its period by the measured skew. | `ec_record --seconds 600`, then `robot_checks.py drift` |
| 4 | Two 8055 connections at once | One connection, owned by the control thread. `stop()` is handled first in every cycle, ahead of anything queued, so it waits at most one cycle and one round trip. | A second connection as an independent stop path, inside the control thread's I/O. | `robot_checks.py connections` |
| 5 | Stop latency per family | Numbers only: the options in §7.2. | Tuned values. | motion |
| 6 | Does the speed setting scale streams and timestamped trajectories? (It scales point-to-point speeds of types VJ, V and VR, not AV and AVR [J 2.4]; the linear moves send AV.) | Not relied on. Its value is logged when a stream or trajectory starts; trajectory time limits assume it applies (the slower case); streams are bounded by the driver's own limits. | If it slows the queue's consumption, streams require 100 %: one more condition. | motion |
| 7 | Does a timestamped trajectory run on if the PC disconnects? | Yes, the cautious assumption: trajectories are validated before upload (joint limits, speeds, a start at the current position), and first runs use a low `speed_percent`. | Nothing. | motion |
| 8 | Does V3.15.3 accept `targetPose` in a stream [S 2.2.7.4], or poses in an upload (`pos_type` 1), and in which units? | Not used: the driver sends joints only (ADR 0017). | The controller could solve live targets itself, without a call per target. | motion, without moving: stream the current pose as `targetPose` (a later milestone) |
| 9 | Step response for several L, T and smoothness | The defaults in §7.2. | Tuned values, and the lag the tracking check expects. | motion |
| 10 | Does `inverseKinematic` return the solution nearest `referencePos`, and within which range? | The nearest; any range, moved by whole turns to the reference (`nearest_turn`). A solution from another configuration shows up as a jump, which planning refuses. | Nothing structural: solving reports where the jump is. | `sbx_trajectory`, answered no (computations only) |
| 11 | How do arc waypoints (`moveType` 3) group in a path? The JBI manual has MOVC in threes [J 2.4.3]. | Not used: the driver moves in joint space (ADR 0017). | Nothing, unless the controller's own paths come back. | motion |
| 12 | How does the controller interpolate an uploaded trajectory between its points? | Linearly or better; points are sent 20 ms apart. | A different spacing. | `sbx_trajectory`, recorded by `ec_record` in a second terminal |
| 13 | Is the TCP in 8056 (`machinePose`) the current tool's, as `positiveKinematic` computes it? | Yes. | Planning would compare against the reported TCP's own tool. | `ec_query positiveKinematic` with the joints now, against `ec_monitor`'s TCP |
| 14 | Which tool do `inverseKinematic` and `positiveKinematic` use in remote mode: the teach-mode tool (`getToolNumber`) or the remote one (`getAutoRunToolNumber`)? | The remote one, the tool remote-mode motion uses. | A check before motion sets or compares the other. | `ec_query getToolNumber`, `getAutoRunToolNumber`, then `ec_query positiveKinematic` with the two different |
| 15 | Joint accelerations the arm accepts (the datasheet gives none) | Each joint's datasheet speed reached in 0.5 s. | Tuned limits. | motion |

### 7.1 Why drift matters only for long streams

Timing precision is not the issue: at 125 Hz, a rate error of 100 ppm changes
each period by less than a microsecond, far below the network's jitter. The
stream's queue, however, *integrates* the rate difference between producer and
consumer: its level moves by skew × elapsed time, 3 ms per minute at 50 ppm. A
40 ms lookahead therefore lasts about 13 minutes of continuous streaming before
the queue runs dry or the latency doubles. Planned motion and sessions of a few
minutes are unaffected. Estimating the clock offset online is the cure if it is
ever needed: the slope of that offset is the skew to correct.

### 7.2 Numbers to tune on the robot

Every number is an option with a default chosen before measuring. Tuning them
is a roadmap step (11 and 16), not a code change.

| Number | Default | Used by |
|---|---|---|
| Cycle period T | 8 ms, the 8056 rate | control thread; stream sampling period |
| Reply timeout | 2 s per command: a reply that takes longer fails it (50 ms was too short for the EA66's `set_servo_status`, first robot run) | control thread |
| Trajectory upload | 5 points per cycle, each confirmed (`ret_flag` 1): the EA66 takes them at its own pace (unconfirmed, 20 a cycle backed up and blocked the loop, first robot run); 0.2 s after `start_push_pos` and 1 s before `stop_push_pos`, as the manual's examples (2.2.9.10, 11) | `PlayTrajectory` |
| Late cycles before a fault | 3 in a row, each starting later than T/2, while a stream runs (other activities only run later: first robot run) | control thread |
| State freshness | 50 ms, about 6 frames | condition Fresh |
| Standstill | every joint below 0.5 °/s for 5 frames | condition Standstill |
| Stream lookahead L | 40 ms (5 T) | position stream |
| Stream smoothness | 0.1, as in the vendor's examples | position stream |
| Target timeout, then hold | 100 ms: applications push at 10 Hz or faster | stream watchdog |
| Hold before giving up | 500 ms, then `Starved` | stream watchdog |
| Stream velocity limit | 10 % of the datasheet maxima: J1–J2 15 °/s, J3 19 °/s, J4–J6 26 °/s | stream shaper |
| Stream acceleration limit | the velocity limit reached in 0.25 s | stream shaper |
| Tracking tolerance | 5° between the measured position and the setpoint sent L earlier | stream guard |
| Velocity command duration t | 40 ms (5 T), re-sent every cycle | velocity stream |
| Stop timeout | 2 s from the stop command to standstill | every family |
| Job time limit | given per job; 30 s otherwise | planned motion |
| Planning | points at most 3 mm and 1° apart; at most 10° per joint between solutions; J5 and J3 at least 5° from 0 and 180°; a trajectory point every 20 ms; joint accelerations: the datasheet speed in 0.5 s; at most 10 000 points | `plan()` |
| Sandbox circle | centre and tool orientation as read on the pendant; 30 mm radius, 20 mm/s, 200 poses; a joint move to the start at 10 % | `apps/sbx_trajectory.cpp` |

---

## 8. References

- [R1] B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo. *Robotics: Modelling,
  Planning and Control.* Springer, 2009 (ch. 4, trajectory planning).
- [R2] PLCopen. *Function Blocks for Motion Control*, Part 1 (v2.0) and Part 4
  (coordinated motion).
- [R3] IEC 61800-7-201/-301, the CiA 402 drive profile: power state machine;
  modes pp, pv, ip, csp, csv.
- [R4] S. Chitta et al. "ros_control: A generic and simple control framework
  for ROS." JOSS 2(20), 2017; ros2_control documentation (controller manager,
  real-time buffer, controller lifecycle).
- [R5] MoveIt Servo documentation (`incoming_command_timeout`, smoothing plugins).
- [R6] Universal Robots, URScript manual (`servoj`, `speedj`); ur_rtde
  documentation (`servoJ`, `speedJ`, `setWatchdog`).
- [R7] libfranka documentation (`Robot::control`, real-time requirements, rate
  limiting); S. Haddadin et al. "The Franka Emika Robot." IEEE RAM 29(2), 2022.
- [R8] KUKA Robot Sensor Interface; ABB Externally Guided Motion; FANUC Stream
  Motion (J519).
- [R9] T. P. Baker, A. Shaw. "The cyclic executive model and Ada." Real-Time
  Systems 1(1), 1989.
- [R10] G. Buttazzo. *Hard Real-Time Computing Systems*, 3rd ed. Springer, 2011.
- [R11] T. Kröger, F. M. Wahl. "Online Trajectory Generation: Basic Concepts
  for Instantaneous Reactions to Unforeseen Events." IEEE T-RO 26(1), 2010.
- [R12] L. Berscheid, T. Kröger. "Jerk-limited Real-time Trajectory Generation
  with Arbitrary Target States." RSS 2021 (Ruckig).
- [R13] IEC 60204-1 (stop categories); ISO 10218-1/-2; ISO 13849-1; ISO/TS 15066.
- [R14] R. Ramjee, J. Kurose, D. Towsley, H. Schulzrinne. "Adaptive playout
  mechanisms for packetized audio applications in wide-area networks." IEEE
  INFOCOM 1994.
- [R15] E. Gamma, R. Helm, R. Johnson, J. Vlissides. *Design Patterns.* 1994
  (Strategy, Decorator, Command, State).
- [R16] R. C. Martin. *Agile Software Development: Principles, Patterns, and
  Practices.* 2002 (SOLID).
- [R17] S. Parent. "Inheritance Is the Base Class of Evil." GoingNative 2013;
  "Better Code: Runtime Polymorphism." NDC London 2017.
- [R18] G. Bernhardt. "Functional Core, Imperative Shell." 2012.
- [R19] "Writing I/O-Free (Sans-I/O) Protocol Implementations."
  sans-io.readthedocs.io.
- [R20] M. Thompson. "Single Writer Principle." Mechanical Sympathy, 2011.
- [R21] B. Stroustrup, H. Sutter (eds.). C++ Core Guidelines.
