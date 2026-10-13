# Motion and control

**Status:** built, 2026-10-13 (ADR 0014, accepted); Cartesian paths, following
a trajectory and the Cartesian stream the same day (§5.11, ADR 0016 proposed).
Sections 1–4 describe the controller and the field; sections 5–6 the design,
as built, with the refinements listed in §5.10; section 7 what is assumed
until measured on the robot. Nothing of sections 5–6 has run on the robot yet.

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
- Position streaming is a queue that the controller empties at a fixed rate:
  one point per sampling period. The application's targets come at its own
  rate, so something must convert them into that regular sequence. In this
  design, the driver does it.
- The application **pushes**: it hands over a target, or a trajectory, whenever
  it has one, from any thread. The driver's control loop runs at the
  controller's pace, takes the newest target, limits it and feeds the queue.
  The watchdog is a guard in that loop: a target that gets too old makes the
  stream hold, then stop.
- One **control thread** owns the command connection and runs a fixed-period
  cycle (read, decide, act), like a PLC scan. Whatever it runs (a move, a
  stream, bring-up) is an **activity**: a small state machine without I/O,
  behind one C++ concept. Adding a motion adds an activity; the loop does not
  change.
- **Supervision** remembers three things (idle, busy, fault) and recomputes
  everything else from the newest state every cycle. Faults latch; a person
  fixes the cause, acknowledges, and runs bring-up again.

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
whether firmware V3.15.3 does is untested (§7, #8). The driver's Cartesian
stream solves poses into joints itself (§5.11).

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
why it did not). `robot.servo_pose()` streams TCP poses the same way, and a
known timed trajectory is streamed by `FollowTrajectory` (§5.11).

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
`stopj`) and jog follow the same pattern.

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

A custom motion, for illustration: a linear move that stops on contact,
wrapping an existing activity (a decorator [R15]):

```cpp
struct MoveUntilContact {                   // satisfies ec::motion::Activity; no base class
    ec::motion::PointToPoint move;          // make_activity(ec::motion::MoveLine{...})
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
| Planned motion | `point_to_point.hpp`, `trajectory.hpp`, `sequence.hpp`, `bring_up.hpp` | `MoveJoint`, `MoveLine`, `MoveLineToPose`, `PlayTrajectory` (with `JointTrajectory` and `validate()`), `Sequence`, `BringUp` |
| Streamed motion | `stream.hpp`, `shaper.hpp`, `session.hpp` | `ServoStream`, `VelocityStream`, `JogStream`; `RateLimiter`; the application's `Session<T>` |
| The control thread | `ec/motion/control_loop.hpp` | `ControlLoop`, `Job`, `Status` |
| The facade | `ec/robot.hpp` | `ec::Robot`: `connect`, `state`, `next_state`, `status`, `run`, `bring_up`, `servo`, `servo_pose`, `velocity`, `jog`, `stop`, `acknowledge`, `call<C>`, `inverse_kinematics`, `forward_kinematics` |
| Cartesian paths | `ec/path.hpp`, `ec/motion/planning.hpp` | `CartesianPath` (a concept: `length`, `pose_at`), `Circle`; `SpeedProfile`, `JointSpline`, `solve()`, `TimedPath` (§5.11) |
| More motion | `follow.hpp`, `waypoints.hpp`, `point_to_point.hpp`, `stream.hpp` | `FollowTrajectory` (a trajectory streamed by the PC), `MovePath` (waypoints), `MoveArc`; `ServoLink` (the transparent-transmission protocol the position streams share), `PoseServoStream` |
| Tool | `apps/ec_move.cpp`, `apps/circle.cpp` | status, bring-up, joint, nudge, sine, circle (seven modes), stop, acknowledge; nothing moves without `--yes`; `--record` |
| Test bench | `tools/fake_robot.py`, `tools/kinematics.py` | a simulated arm on 8055 and 8056, with fault injection, an EC66-like kinematic model, Cartesian moves and a trace of its TCP |

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

Not built yet: slaving the stream to the controller's clock; an online
trajectory generator as the shaper (it would also take away the live streams'
lag, §5.11); logging of commands and events; the stream's own Cartesian
targets (§7, #8); arc waypoints in paths (§7, #11).

### 5.11 Cartesian paths

A path of the TCP becomes motion in four pure steps, then one of several
families runs it:

```cpp
const ec::Circle circle{.center = c, .radius = 0.05, .rotation = tool};       // ec/path.hpp: geometry only
auto ik = [&](const Pose& p, const Joints& near) { return robot.inverse_kinematics(p, near); };
auto path = ec::motion::solve(circle, robot.state()->state.joint_position, ik).value();  // every 3 mm
auto timed = ec::motion::TimedPath::make(std::move(path), 0.02, 0.5).value();  // 20 mm/s, 0.5 s ramps
robot.run(ec::motion::PlayTrajectory{.path = timed.sample(0.02)});      // uploaded, the controller's clock
robot.run(ec::motion::FollowTrajectory{.path = timed.sample(0.008)});   // or streamed by the PC
```

1. **Geometry.** A path is anything with `length(path)` and `pose_at(path, s)`
   (the `CartesianPath` concept, found by argument-dependent lookup): a new
   shape needs nothing else. `Circle` is the first: centre, radius, normal
   (counter-clockwise about it), a start direction, a constant tool orientation,
   turns.
2. **Inverse kinematics, by the controller.** `solve()` asks `inverseKinematic`
   for a pose every 3 mm, each near the solution before, and refuses a path whose
   neighbouring solutions differ by more than 10° on a joint (a change of arm
   configuration, or a singularity in between). Why the controller's: the
   manual's model is the arm's, with its calibration and its current tool; a
   model of our own could differ from it in ways only the robot would show
   (ADR 0016). The price is one call per point, about one cycle each: a 50 mm
   circle takes about a second to solve. A solution given within one turn is
   moved by whole turns to the one nearest the last (`nearest_turn`).
3. **Interpolation in joint space.** `JointSpline` joins the solutions with
   cubic Hermite segments (Bessel's slopes: continuous velocity, exact for
   quadratics). Between points 3 mm apart it stays within micrometres of the
   path.
4. **Timing.** `SpeedProfile` moves along the path from rest to rest: the speed
   rises over a ramp as a half cosine (zero acceleration at both ends of the
   ramp), cruises, and falls. `TimedPath` gives positions and velocities at any
   time, and samples them into a `JointTrajectory`.

How each family runs the result (all seven are modes of `ec_move circle`):

| Mode | Family | Who times it | Exact on the path? |
|---|---|---|---|
| `arc` | `MoveArc`: `moveByArc` half a turn at a time, through solved points | controller | yes (its arcs); stops between arcs |
| `path` | `MovePath`: lines between solved points, corners blended | controller | chords, cut at the corners |
| `lines` | `Sequence` of `MoveLineToPose` (`moveByLineCoord`, the controller solves) | controller | chords; stops at each corner |
| `trajectory` | `PlayTrajectory`: the sampled trajectory, uploaded | controller | yes |
| `follow` | `FollowTrajectory`: the same trajectory, a point per cycle into transparent transmission | PC | yes |
| `pose-stream` | `PoseServoStream`: poses pushed live, solved per cycle | PC | trails (below) |
| `velocity` | `VelocityStream`: planned joint velocities plus a position correction, pushed live | PC | trails slightly |

`FollowTrajectory` sends its points unshaped. The live streams' rate limiter
plans to be able to stop at each target, so it trails a moving target by the
braking distance, v²/2a per joint, and the joints trail by different amounts:
the path bends. For a planned trajectory that only costs accuracy, so the
trajectory is checked against the stream's limits before anything moves
(speed, and acceleration from rest to rest), and then sent as it is. Live
targets keep the shaper. On the fake, a 20 mm circle at 60 mm/s strays 1.3 mm in
`pose-stream` mode; the lag grows with the square of the speed. An online
trajectory generator that also takes the target's velocity would remove it.

`PoseServoStream` asks `inverseKinematic` for each new pose near the last
solution and uses the reply in the next cycle: one period more latency than a
joint stream, and two commands per cycle on the command port (§7, #2). A pose
without a solution, or a solution more than 10° from the last on a joint,
ends the stream gracefully (`Kind::Invalid`).

`ec_move circle` checks first that the controller's forward kinematics of the
current joints give the TCP that 8056 reports (within 1 mm and 0.5°), so that
planning and checking use the same tool; then it plans, prints points to
check by hand (drag mode), and without `--yes` stops there. With `--yes` it
moves in a straight line to the first point, draws the circle, and reports
how far every monitoring frame strayed from it.

---

## 6. The control loop, idiomatically

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

// d) A recorded path: played by the controller, or streamed by the driver.
robot.run(ec::motion::MoveJoint{.target = path.position.front(), .speed = 0.2}).wait().value();
robot.run(ec::motion::PlayTrajectory{.path = path}).wait().value();    // timestamped upload
robot.run(ec::motion::FollowTrajectory{.path = path}).wait().value();  // or streamed by the PC (§5.11)
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
| 8 | Does V3.15.3 accept `targetPose` in a stream [S 2.2.7.4], and in which units? | Not used: Cartesian streams solve each pose with `inverseKinematic` and stream joints (§5.11). | A second way to feed `ServoLink`, without the extra call per cycle. | motion, without moving: stream the current pose as `targetPose` (a later milestone) |
| 9 | Step response for several L, T and smoothness | The defaults in §7.2. | Tuned values, and the lag the tracking check expects. | motion |
| 10 | Does `inverseKinematic` return the solution nearest `referencePos`, and within which range? | The nearest; any range, moved by whole turns to the reference (`nearest_turn`). A solution from another configuration shows up as a jump, which planning refuses. | Nothing structural: solving reports where the jump is. | `ec_move circle MODE --radius R` without `--yes` (computations only) |
| 11 | How do arc waypoints (`moveType` 3) group in a path? The JBI manual has MOVC in threes [J 2.4.3]. | Not used: paths are lines with blended corners; arcs use `moveByArc`. | `PathMove::Arc` in `MovePath`. | motion |
| 12 | How does the controller interpolate an uploaded trajectory between its points? | Linearly or better; points are sent 20 ms apart. | A different spacing. | `ec_move circle trajectory`, recorded |
| 13 | Is the TCP in 8056 (`machinePose`) the current tool's, as `positiveKinematic` computes it? | Yes; `ec_move circle` checks it before planning (1 mm, 0.5°). | Planning would compare against the reported TCP's own tool. | `ec_move circle` without `--yes` |

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
| Reply timeout | 50 ms per command: a reply that takes longer fails it | control thread |
| Late cycles before a fault | 3 in a row, each starting later than T/2, while an activity runs | control thread |
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
| Jog repeat | every 200 ms (the controller stops after 1 s) | jog |
| Stop timeout | 2 s from the stop command to standstill | every family |
| Job time limit | given per job; 30 s otherwise | planned motion |
| Path solving | a point every 3 mm; at most 10° per joint between points; at most 5000 points | `solve()` |
| Pose stream | at most 10° per joint between solutions | `PoseServoStream` |
| Following | 0.5° from the first point to start; 0.2° to arrive; 2 s to settle | `FollowTrajectory` |
| Circle defaults | 20 mm/s, 0.5 s ramps; approach at 20 mm/s, at most 300 mm; path: 36 lines a turn, blends of 0.3 of a line; lines: 12 a turn; uploads: a point every 20 ms; velocity correction 4/s | `ec_move circle` |

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
