# Architecture: the robot as a composition of parts

Written 2026-10-21 from the manuals (`docs/ec66.md`, cited as §R.n, with the
reading notes in `docs/notes/`) and nothing else: earlier decisions and the
archived ADRs do not apply. It answers four questions: what the robot is made
of, how the driver should be built around that, which workflow each task
takes, and how to get there in a few weeks. Agreed in outline the same day
(§12 records the user's answers); details may still change as it is built.

## 1. The short answer

**Yes: see the robot as a composition of parts. But of the parts the
controller really has, and with joints as state, not as things you command.**

- The robot is a few parts: six joints and a tool flange (hardware), and
  inside the controller the drives (power, enable, brakes, encoders), the
  motion controller, the model (kinematics, tools, payload, frames), the
  safety system, the I/O, the program engine, and the system itself (mode,
  versions, alarms, logs) (§R.2–§R.5). The driver should have the same parts.
- **Joints are observed, not commanded.** Every motion is coordinated by the
  controller; the SDK has no command addressed to one joint, except jogging
  one axis (§R.5.7). So a joint is a value in the state (position, speed,
  torque, temperature, ...), not an object with methods.
- **A state mirror, yes; a digital twin, no.** The controller streams almost
  its whole state every 8 ms on 8056 (§R.6.3). The driver turns each frame
  into one snapshot made of the parts' states: a mirror of what the controller
  reports, and nothing it doesn't (what IoT platforms call a device
  shadow). Joints are stateful mirrors: you read them, you don't move them.
  A twin that simulates the arm would duplicate the controller's
  kinematics and dynamics, which it owns. Our simulator is `tools/fake_robot.py`,
  for tests.
- **Why the SDK looked huge and scattered**: the vendor groups its 184 calls
  by internal service. One of them, "ParamService", holds 74 unrelated calls
  (state, tools, safety, power, settings). Regrouped by part
  (`docs/sdk_map.md`):

  | Part | Calls | of them streamed on 8056 |
  |---|---|---|
  | Joints and TCP (observed) | 20 | 16 |
  | Drives | 13 | 2 |
  | Motion controller | 36 | 2 |
  | Model | 25 | 0 |
  | Safety | 17 | 2 |
  | Programs | 21 | 1 |
  | I/O | 24 | 4 |
  | System | 5 | 1 |
  | Peripherals (serial ports, force sensor, force control) | 23 | 0 |
  | **All** | **184** | **28** |

  28 calls only repeat the stream, so the observed state needs no calls at
  all. Each part is then 5 to 36 operations: small enough to understand.
- **Not from scratch.** The basic components are already right and tested:
  the socket, the JSON-RPC client, the 184 typed calls, the 8056 decoder and
  receiver, the recorders, the fakes. What was missing is the middle: a model
  of the robot's parts, and explicit workflows. That is what this proposes to
  build.

## 2. What the robot is made of

| Part | What it is | Observed (8056 / queries) | Commanded | Configured |
|---|---|---|---|---|
| **Joints** ×6 | motor, reducer, brake, two encoders, a drive, in each joint (§R.3.2) | position (output and motor side), speed, acceleration, torque, external torque, motor speed, temperature; brake released | — (only through the motion controller; jogging an axis is the motion controller's too) | soft limits (pendant) |
| **Tool flange / TCP** | the end of the arm, with the tool's TCP | TCP and flange poses (base and user frame), speed, acceleration, external wrench | — | through the model (tools) |
| **Drives** | power, servo enable, brakes, encoder states, joint alarms (§R.3.3–§R.3.4, §R.5.2) | servo on, synchronised, E-stop, collision; precise, servo alarm, power (queries) | arm power, servo on/off, synchronise, calibrate, clear alarm, reset, backdrive | — |
| **Motion controller** | plans, interpolates and times every motion (§R.5.7) | its state (stop, pause, running, alarm), the speed slider, hand guiding | the families: point-to-point, path, jog, speed, servo stream, timestamped trajectory, hand guiding; stop | speed slider, jog frame |
| **Model** | kinematics, dynamics, tools, payload, frames (§R.5.6) | — | computations: IK, all IK, FK, pose arithmetic | tool data and number, payload, user frames |
| **Safety system** | E-stop, safeguard, collision, limits, reduced mode (§R.5.8) | E-stop, collision; limits and settings (queries) | reset collision | collision on/off and sensitivity, safety limits (password) |
| **I/O** | box, flange, virtual bits, fieldbus registers (§R.4.3) | digital and analog I/O; virtual bits, registers (queries) | outputs, virtual outputs, registers | flange buttons |
| **Program engine** | JBI programs, cycle mode, variables (§R.4.2) | program state and line; variables | run, pause, resume, stop | cycle mode, variables |
| **System** | the controller as a whole: mode, versions, alarms, logs, clock (§R.5.1, §R.5.9) | key-switch mode; versions; last alarms; the log (8058) | — | clock |
| *Peripherals* | serial ports, a force sensor, force control | (later, when an application needs them) | | |

The operator owns three things no software can touch: the key switch, the
emergency stop, and the box's power switch.

## 3. The architecture

```
 applications, tools (ec_query, ec_monitor, ec_record, ec_move, sandboxes)
 ───────────────────────────────────────────────────────────────────────────
 workflows      bring_up  shut_down  recover  calibrate  run_trajectory ...
                (the operating manuals' sequences, built from parts; reports)
 ───────────────────────────────────────────────────────────────────────────
 robot          Robot: the composition root (one session; the latest observation)
                parts:  Joint(i)  Drives  Motion(+families)  Model  Safety  Io  Programs  System
                state:  State = { joints[6], tcp, drives, motion, io, system }   (from 8056)
 ───────────────────────────────────────────────────────────────────────────
 wire           sdk: the 184 typed calls, the JSON-RPC client (8055)
                feedback: 8056 frames, decoder, reader, receiver (a component of its own)
                controller log: the 8058 client (new, small)
 ───────────────────────────────────────────────────────────────────────────
 transport      net::TcpSocket
```

Rules, one sentence each:

1. **The wire mirrors the manual** (calls as the manual names and groups
   them); **the robot layer mirrors the robot** (its parts); **workflows
   mirror the operating manuals** (their sequences).
2. Each layer uses only the layers below it. Parts don't call each other;
   workflows compose them. So no part hides a ceremony.
3. One source of truth: the controller. The state is what it reports; the
   driver keeps no model of the robot of its own.
4. One owner per resource: the `Robot` owns the connection and the state
   stream; the motion controller part owns "one motion at a time".
5. Everything that is not a part's operation stays reachable as a typed call
   (`robot.call(ec::sdk::X{...})`): parts grow when an application needs
   more, guided by `docs/sdk_map.md`.

## 4. The observed state

Built on days 2–3 (`include/ec/state.hpp`, `monitor.hpp`, `readiness.hpp`).

**Two paths to the same robot, each honest about its source** (the user's
call). The command connection (8055) asks the controller now, one call at a
time: `robot.call(...)` and the parts' getters (`robot.joint(3).temperature()`,
`robot.drives().servo_on()`). The monitoring path (8056) is what the
controller pushes every 8 ms: `robot.monitor()`, an `ec::Monitor`, a component
of its own that the robot holds. Reading it sends nothing and never waits for
a call, so the code says which path a value came from:

```cpp
const ec::State s = robot.monitor().latest().value();   // 8056: the newest snapshot, at once
double t3 = s.joints[2].temperature;                     // J3 [degC], as of s.arrival
auto next = robot.monitor().next(s.seq, 100ms);          // the next one, for loops paced by the robot
auto mode = robot.call(ec::sdk::GetRobotMode{});         // 8055: asked now
```

The monitor (`ec::Monitor`) is the receiver (`feedback::Receiver`, its own
thread, reconnecting), the decoder and the newest value (`Latest<State>`),
nothing more. Without a robot it runs on its own (`ec::Monitor m(host, {})`).
`status()` says whether the stream is connected, how many frames it decoded or
skipped, and why the last connection ended; a lost stream shows as states
that stop arriving (`State::age()`). Two hooks run on its thread:
`on_sample`, every frame as received (recordings, lossless, undecodable frames
included), and `on_state`, every state. Subscribing at run time waits until an
application needs it.

**`ec::State`**: one snapshot per frame, made of the parts' states, grouped as
`docs/sdk_map.md` groups the calls that read the same values:

```cpp
struct State {
    std::uint64_t seq;                     // 1 for the first frame decoded, one more per frame
    Clock::time_point arrival;             // this PC's steady clock
    sys_time<milliseconds> controller_time;// the controller's clock
    std::array<JointState, 6> joints;      // position, velocity, acceleration, torque, external_torque,
                                           // torque_of_rated, motor_speed, temperature
    TcpState tcp;                          // pose, pose_in_user, flange_pose, flange_pose_in_user
                                           // (Isometry3d), speed, acceleration, force, torque
    DriveState drives;                     // servo_on, synced
    MotionState motion;                    // state (stop, pause, running, ...), hand_guiding
    SafetyState safety;                    // emergency_stop, collision
    IoState io;                            // digital_inputs/outputs (bitset<64>), analog_inputs/outputs
    ProgramState programs;                 // cycle_mode
    SystemState system;                    // mode (the key switch), force_sensor_connected
    Joints positions() const;              // the six positions, for the model's calls
};
State state_from(const feedback::Frame&, std::uint64_t seq, Clock::time_point arrival);   // pure
```

`state_from` is the one place where the stream's units are converted, and
`ec::State` the one SI view of a frame: the HDF5 recording stores it member by
member (`docs/recording-format.md`). `ec::feedback` is the wire below it,
the frame as the manual lays it out (`Frame`, `decode`, framing, reader,
receiver), as `ec::sdk` is for 8055; `ec_monitor` prints frames at that level,
in the controller's units, to compare with the pendant.

What 8056 does not carry (precise, power, the last alarms, the speed slider,
the tool numbers, which brakes are released) has only the 8055 path, read on
demand and never polled behind the application's back: polling would occupy
the one command channel.

**`ec::readiness(state, precise)`** is a pure function: the first step that
stands between the robot and a motion command (`Blocker`), with `remedy()`
in words and `needs_operator()` for the key and the button. The order: the
key at REMOTE, the emergency stop, a collision (reset before its alarm), an
alarm, synchronisation, the servo (calibrating needs it on), precise (only if
the caller read it), the state back at stop (docs/ec66.md §5.2). It decides
nothing for the controller, which checks every command itself; `ec_move status`
prints it, and the start-up workflow will follow it.

## 5. The parts

Each part is a small handle that shares the robot's session (copyable, valid
while the `Robot` lives). Its operations are the typed calls of its part,
converted to SI, each documented with the manual's section and "controller"
or "driver" for who does the work.

```cpp
auto robot  = ec::Robot::connect({.host = "192.168.0.200"}).value();
ec::State s = robot.monitor().latest().value();   // newest snapshot (8056)
double t3   = s.joints[2].temperature;        // °C

auto drives = robot.drives();
drives.switch_servo(true);                    // set_servo_status 1, waits until reported on
auto q = robot.model().ik(T_base_tcp, s.positions()).value();
```

| Part | Operations (first version) | Calls behind them |
|---|---|---|
| `Joint` (`robot.joint(i)`, i = 1..6) | read only, over 8055: `position()`, `motor_position()`, `velocity()`, `torque()`, `motor_torque()`, `temperature()`, `brake_released()`, `firmware()` | §R.3.2 |
| `Drives` | `servo_on()`, `switch_servo(bool)`, `synced()`, `synchronise()`, `precise()`, `calibrate_encoders()` (moves the arm a little), `clear_alarm()`, `reset()`, `servo_alarm()`, `brakes_released()`, `enter_backdrive()`, `exit_backdrive()`; `power()` and `switch_power(bool)` once confirmed on V3.15.3 | §R.5.2 |
| `Motion` | `state()`, `wait_until_stopped()`, `stop()`, `speed_scale()`, `set_speed_scale()`, `hand_guiding(bool)`; the families `Move`, `Path`, `Jog`, `SpeedStream`, `ServoStream`, `Timestamped` | §R.5.5, §R.5.7 |
| `Model` | `ik`, `ik_all`, `fk`, `interpolate`, `to_user`, `from_user`, `dh`; `tool(n)`, `set_tool(n, tcp)`, `active_tool()`, `select_tool(n)`; `payload(n)`, `set_payload(n, mass, cog)`; `user_frame(n)`, `set_user_frame(n, pose)`, `select_user_frame(n)` | §R.5.6 |
| `Safety` | `collision()`, `reset_collision()`, `collision_detection()`, `limits()` (normal and reduced); setters later (they need the remote password) | §R.5.8 |
| `Io` | `input(i)`, `output(i)`, `set_output(i, on)`, `analog_input(i)`, `analog_output(i)`, `set_analog_output(i, v)`, `virtual_bit(i)`, `set_virtual_bit(i, on)`, `flange_button(b)` | §R.4.3 |
| `Programs` | `exists(name)`, `run(name)`, `state()`, `line()`, `pause()`, `resume()`, `stop()`, `cycle_mode()`, variables `b/i/d/p/v(n)` and setters, `save_variables()` | §R.4.2 |
| `System` | `mode()`, `versions()`, `alarms()` (the last five), `log(lines, level)` (8058), `set_clock(t)` | §R.5.1, §R.5.9 |

The motion families keep their present shape (the constructor takes the
inputs and sends nothing; the methods are the family's calls in the manual's
order), now under the motion controller part, with waits that read the state
stream instead of polling 8055.

## 6. Workflows: what each task takes

A workflow is a free function over the parts that carries out one task as
the manuals describe it, and returns a report: each step, what it did or why
it stopped, and what the operator must do. Applications may use them or
compose the parts themselves.

| Task | Steps | Ends when | Source |
|---|---|---|---|
| **Start a session** (`bring_up`) | mode must be REMOTE, else stop: "turn the key"; E-stop pressed: stop, "release it"; alarm: clear, 0.5 s; collision: reset it, clear; not synchronised: synchronise, 0.5 s; not precise: stop, "calibrate (moves the arm) or move each axis on the pendant", unless asked to calibrate; servo off: switch on, wait until reported on; report the speed slider | the servo is on and nothing is missing | §R.5.10 [S 2.2.9.10] |
| **End a session** (`shut_down`) | the open motion's stop, `stop`; wait for state stop; servo off, wait until reported off; arm power off if asked | the servo is off | §R.5.10 (built from documented calls) |
| **Recover after an emergency stop** | the operator releases the button; clear the alarm; synchronise; servo on | ready | [Sv servo alarm 40] [U 6.8.2] |
| **Recover after a collision** | reset the collision state; clear the alarm; synchronise if needed; servo on | ready | [S 2.2.2.35] |
| **Recover after an alarm** | read the alarms and the log; clear; synchronise if needed; servo on; report what the controller said | ready, or the alarm persists | §R.5.9 |
| **Make the encoders precise** | servo on; `calibrate_encoder_zero_position`; read precise | precise | [S 2.2.3.21] [U 4.3] |
| **Set up a tool or frame** | at rest, REMOTE: write the tool, payload or frame; select its number; read it back | read back equal | §R.5.6 |
| **Point move** | (ready) `Move`: one call; wait | state stop | [S 2.2.3.1-4] |
| **Waypoint path** | clear; add each point; run; follow the index; wait | state stop | [S 2.2.3.5-7] |
| **Timestamped trajectory** | (ready) a joint move to the first point; upload; start; wait; optionally flush (a check of the start: postponed, §7) | state stop | [S 2.2.9.10] |
| **Servo stream** | (ready, at the first point) open; one point per period; close | closed | [S 2.2.7.6] |
| **Velocity stream** | (ready) a velocity before each `t` runs out; `stopj`/`stopl` | stopped | [S 2.2.3.16-19] |
| **Jog** | (ready) repeat `jog` within 1 s; `stop` | stopped | [S 2.2.3.8] |
| **Run a program** | check it exists; run; follow its state and line | its state stop | [S 2.2.3.12-14] |
| **Hand guiding** | (REMOTE, servo on) switch on; the operator moves the arm; switch off | off | [S 2.2.2.17] |
| **Record a session** | the state stream into a raw or HDF5 file; the call log beside it | stopped by the application | §8 |
| **Explain a failure** | the controller's message; the last alarms; the last error lines of its log; the state at that moment | — | §R.5.9 |

## 7. Concurrency, errors, lifetimes

- **Threads**: one, the monitor's (8056), which builds each `State` and hands
  it to the latest-value store and to the hooks (recordings, `on_state`). Commands run on
  the caller's thread, one at a time on 8055 (a mutex), each answered before
  the next. `robot.stop()` works from any thread; it waits at most for the
  call in flight. If a second 8055 client proves allowed (§R.8), a separate
  stop channel can be added inside the session without changing the API.
- **Errors**: `std::expected<T, ec::Error>` everywhere. On a refusal or a
  fault the error carries the controller's own message, its last alarm
  numbers and the state at that moment: the driver explains with the
  controller's reasons, never with extra calls before every command.
- **The driver's own guards** are few and named as ours: never two calls in
  flight or unconfirmed bursts (learned on the robot, §R.6.2); a timestamped
  trajectory's speed factor at most 1 unless lifted. Postponed (the user,
  2026-10-21): checking that the arm is at a trajectory's first point before
  it starts, or moving it there. What the controller does when it is not is an
  open question (§R.8 #7); it may handle it itself.
- **Lifetimes** (recommended, and agreed): a motion object owns its motion,
  so destroying an open one sends its stop; the `Robot` owns the session, so
  destroying it stops an open motion (it does not switch the servo off,
  `shut_down` does). The reason: a motion should never outlive the code that
  can supervise it. Exceptions, early returns and Ctrl+C then stop the arm
  instead of leaving it running unwatched, as `std::jthread` asks its thread
  to stop when it goes out of scope. The cost: a move meant to run on must
  keep its object, which shows in the code. If the process dies, nothing on
  the PC runs; the controller finishes or drains what was running (§R.5.7).

## 8. How we build it

**The engineering workflow, for each feature:**

1. Find the facts: the manual's sections, `docs/ec66.md`, the open
   questions.
2. Write the header: the operation, its units, its errors, who does the work
   (controller or driver), the manual's section.
3. Pure logic first (conversions, `state_from`, `readiness`), with unit tests.
4. The calls through the typed catalogue, tested against the scripted
   controller (`tests/support/mini_controller.hpp`).
5. End to end against the simulated arm (`tools/fake_robot.py`), extending
   the fake when the robot does something new.
6. A hardware check written as commands for the user; its result goes into
   `docs/ec66.md` §7.

**The toolchain stays:** CMake presets, Ninja, vcpkg in manifest mode,
GCC 15, clang-format and clang-tidy, the CI. It is not what made the project
feel heavy. Editor support is local: `.vscode/` (not tracked, the user's call)
holds the settings for CMake Tools on the presets, IntelliSense (or clangd) on
`build/linux-debug/compile_commands.json`, format on save with the pinned
clang-format, debug launches for `ec_tests` and the tools, and tasks for the
format, clang-tidy and the app tests.

## 9. What happens to the current code

| Keep as it is | Reshape | New | Archive |
|---|---|---|---|
| `net/`, the 184 typed calls and the client (`sdk/`), the 8056 decoder, reader and receiver, the recorders (`record/`), `Latest`, units, poses, status enums, the fakes and their tests, `apps/planning.hpp` (the sandbox's) | `feedback::State` → `ec::State` made of parts, the HDF5 recording on it (done); `Robot` → the composition root; `detail::Connection` → the session; the motion families under the motion part, with waits on the stream; `bring_up` → a workflow; `ec_move` on the workflows | the parts; `Monitor` and `readiness` (done); the workflows; the 8058 log client; `Path`, `Jog`, `SpeedStream`; `ec_log` | done 2026-10-21: `docs/motion.md`, the old architecture and design, the ADRs (local) moved to `docs/archive/` |

Also done on day 1: `ec::log` (our recordings) is now `ec::record`, so that
"log" means the controller's log; ADR citations in the code now point to
`docs/ec66.md` or are gone. Kept after all: `tools/poses.py` (the fakes' pose
library), `tools/rotation_convention.py` (how the convention was measured,
with its test), and `docs/sdk_calls.md` beside `docs/sdk_map.md` (the manual's
view of the calls beside the robot's).

## 10. Plan

About two and a half weeks; each block is tested before the next.

| Days | Work |
|---|---|
| 1 | Done 2026-10-21: clean-up (§9), README, the VS Code files (local) |
| 2–3 | Done 2026-10-21: `State` and `state_from`, `readiness`, the `Monitor` (the 8056 path) in `Robot`; unit tests |
| 4–5 | `Drives`, `System` (with the 8058 client), `Model`: thin, against the scripted controller |
| 6–7 | `Motion` with the existing families moved in, waits on the stream, the guards; `Io` |
| 8 | Workflows: start, end, recover, with reports; `ec_move` on them |
| 9 | End to end against the fake (taught the flags it lacks: precise, power, alarms) |
| 10 | Hardware session: the read-only checks (§R.8), start, one move, the circle, end |
| 11–13 | `Path`, `Jog`, `SpeedStream`, `Programs`, `Safety` (reading); the call log; a second hardware session |
| after | The first real application on top; fix what it finds awkward |

## 11. Where I push back

- **Joints as objects with commands**: no; the hardware has none. A joint is
  a value in the state, and a read-only handle that asks the controller
  (`robot.joint(i)`).
- **A simulating twin**: no. The controller owns the kinematics and
  dynamics; duplicating them gives two answers to every question. The mirror
  in §4 is the useful part, and the fake robot is our simulator.
- **Starting again from the socket**: no. The bottom layers are the basic
  components, built and tested; rebuild the middle.
- **Wrapping all 184 calls in parts now**: no. The parts start with what
  applications use (about 60 operations); the map says where the rest
  belongs, and the typed call reaches it meanwhile.
- **Peripherals** (serial ports, force sensor, force control): later, when an
  application needs them.

## 12. The user's answers (2026-10-21)

1. The parts and their boundaries: almost; to be refined while building.
   Jogging stays with the motion controller, not with a joint.
2. Parts as handles and workflows as free functions with reports: go ahead,
   change it if it does not work out.
3. Lifetimes as recommended in §7: yes. The start check of a trajectory:
   postponed.
4. The VS Code files: local only, not tracked; IntelliSense must work.
5. Two paths for observed values: the latest 8056 observation in
   `robot.state()` "or something", the controller asked over 8055 by the
   parts' getters (§4).
   The 8056 reader stays a component of its own.
6. Start with day 1, once and cleanly; nothing from the older talks carries
   over unless this document takes it up.
7. Before days 2–3: the 8056 path should show in the code as a separate path
   on the same robot (the user, not sure it is right): it is `robot.monitor()`
   (§4). The existing stream code works, so change little. Where there is a
   choice, take the safer, more idiomatic one.
