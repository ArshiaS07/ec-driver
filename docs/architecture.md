# Architecture

Controller: Elite EC series (Annuo EA66 = EC66), firmware V3.15.3, SDK manual
Ver3.11.2 (`docs/sdk_calls.md` lists every call). Language: C++23
(`std::expected` for every fallible call, `std::byteswap`, `std::format`). One robot,
one host.

Rule for this document: every abstraction states why it exists. If the reason
is missing, the abstraction should be.

---

## 1. What the controller exposes

| Port | Direction | Protocol | Who initiates |
|---|---|---|---|
| 8055 | PC → robot → PC | JSON-RPC 2.0, one request per line, one reply per request | always the PC |
| 8056 | robot → PC | 1024-byte big-endian frames, ~every 8 ms | always the robot |

That is the whole hardware interface for this driver (Modbus 502, EtherNet/IP
44818 and the log ports are out of scope). Everything below is built on these
two endpoints.

---

## 2. Namespaces

Named by direction of data, the same split ros2_control uses (state
interfaces versus command interfaces):

| Namespace | Contains | Talks to |
|---|---|---|
| `ec::feedback` | everything flowing robot → PC on 8056 | port 8056 |
| `ec::command` | one type per 8055 call (all 184, queries included) | port 8055 |
| `ec::motion` | motion families: the controller's motion protocols, built from commands | `ec::command` |
| `ec::supervisor` | controller state mirror, gate, faults | all of the above |
| `ec::net` | TCP transport shared by both ports | the network |
| `ec` | status enums both ports report (`RobotState`, `RobotMode`, `CycleMode`) | |
| `ec::detail` | internals (byte readers) | |

A query such as `getRobotMode` is still a `command`: it is a request sent on
the command port. Its reply is data, but the call is a command.

---

## 3. Components, one job each

Bottom-up, from the wire. "Pure" means no I/O and no threads: testable with
plain unit tests.

| Layer | Component | Its one job | Pure | Status |
|---|---|---|---|---|
| bytes | `detail::BigEndianReader` | read big-endian scalars from a span | yes | **done** |
| bytes | `net::TcpSocket` | RAII TCP socket, Linux + Windows: connect, send, receive, all with timeouts | no | **done** |
| bytes | `feedback::Framer` | 8056 byte stream → aligned 1024-byte frames, with resync | yes | **done** |
| bytes | `command::LineFramer` | 8055 byte stream → newline-terminated lines | yes | next |
| codec | `feedback::Frame`, `feedback::decode` | 1024 bytes → typed frame (wire units) | yes | **done** |
| codec | `command::*` structs | describe one SDK call: method, params, reply type | yes | **first 7 done** |
| codec | `command::encode`, `parse_reply` | request → JSON line; reply line → id + result or error | yes | **done** |
| endpoint | `command::Client` | send one request, wait for its reply, enforce the timeout | no | |
| endpoint | `feedback::Receiver` | own the 8056 thread: socket → framer → decode → publish | no | |
| endpoint | `feedback::Latest` | hold the newest frame; copy-out, `wait_next()`, age | no | |
| motion | `motion::PointToPoint`, `Waypoints`, `Jog`, `Velocity`, `ServoStream`, `TimedTrajectory`, `Program` | one controller motion protocol each (see `sdk_calls.md`) | yes | |
| motion | `motion::ServoLoop` | own the clock for ServoStream: each tick, pull a setpoint and send it | no | |
| gate | `supervisor::step` | (state, event) → (next state, actions) | yes | |
| gate | `supervisor::Supervisor` | run `step` on one thread; execute its actions | no | |

Validation (`TrajectoryValidator`) and monitoring (`TrackingMonitor`) are pure
functions that sit next to the motion layer. A convenience facade that wires
everything for applications comes last, once there is something worth wrapping.

---

## 4. Who initiates a call: push versus pull

The wire is fixed: the robot never asks the PC for anything on 8055. So "does
the robot request values from my code" is a host-side design choice, and the
answer differs by family.

**Push (your code drives).** Calling a method sends the command now, through the
supervisor's gate. Right for everything with no timing requirement:
PointToPoint, Waypoints, TimedTrajectory (one upload, then the controller owns
timing), Program, settings, queries.

**Pull (a loop drives).** ServoStream needs a setpoint every `t` ms, on time.
If arbitrary application code called `send()` whenever it felt like it, the
controller buffer would starve or flood. So `motion::ServoLoop` owns a
real-time clock and, every tick, *pulls* the next setpoint from a source you
give it:

```cpp
// Your source: any callable. Return nullopt when there is no new setpoint.
auto source = [&](std::chrono::nanoseconds t) -> std::optional<JointPosition> { ... };
```

`ServoStream` itself stays a pure protocol (`setpoint(q)` returns the request;
one job: the protocol). `ServoLoop` is the pull side (one job: timing). Keeping them apart means the
protocol is testable without a clock, and the timing is testable without a
robot. A nullopt (or a late source) triggers the starvation policy: decelerate,
hold, report.

Velocity used for visual servoing is the same pull pattern and reuses
`ServoLoop` with a velocity source.

---

## 5. Shape of a motion family

The manual's protocols are lifecycles: an **open** sequence, **data** calls,
**start/control** calls, a **close** sequence, plus queries. A family is a small
**pure protocol state machine**: its methods take what you want and return the
commands to send, advancing its own phase. It performs no I/O.

```cpp
class TimedTrajectory {
public:
    expected<Requests, ProtocolError> open(const TrajectorySpec&);   // start_push_pos
    expected<Requests, ProtocolError> push(const TimedPoint&);       // push_pos
    expected<Requests, ProtocolError> seal();                        // stop_push_pos, check_trajectory
    expected<Requests, ProtocolError> start(SpeedFraction);          // start_trajectory
    Requests close();                                                // stop_trajectory, flush_trajectory
};
```

Why commands are returned rather than sent:

- The supervisor executes every command, so it can reject any of them; the
  family cannot bypass the gate even by mistake.
- Calling in the wrong order (start before seal) is a `ProtocolError` from a pure
  function, caught in unit tests, never on the robot.

Why `open`/`close` methods and not constructor/destructor: closing needs RPC
calls, waiting for the robot to stop, and error reporting. Destructors can do
none of that safely.

The supervisor holds the open family as
`std::variant<std::monostate, PointToPoint, Waypoints, ..., TimedTrajectory>`.
A variant rather than a virtual base: the set of families is closed and known
at compile time.

The exception is the real-time path: `ServoLoop` sends `tt_put_servo_joint_to_buf`
directly through a lease-checked sender, because a round trip through the
supervisor's queue each cycle would add latency and jitter. The lease (section
7) keeps that path gated.

---

## 6. Units

- **Wire layer** (`feedback::Frame`, `command::*`): controller units (deg, mm,
  rad, %). Mirrors the manual; checkable against the pendant.
- **Everything above**: SI (m, rad, s, N, N·m, kg), the ROS / ur_rtde /
  libfranka convention.
- **One conversion boundary**, nowhere else converts.
- Percentages and per-mille have no SI equivalent: fractions in [0, 1] with
  named types (`SpeedFraction`), never bare `double`.
- Orientation above the wire is a rotation type (e.g. Eigen), converted to the
  vendor's rx/ry/rz at the boundary once its convention is verified.
- Commands always send `unit_type` explicitly: the manual's defaults differ per
  method (`getTcpPos` defaults to degrees, most others to radians).

---

## 7. Supervisor

The top of the hardware side: it **mirrors the controller's operating state**
and **gates every command against it**. Applications never bypass it.

### States

Mapped to the PLCopen motion state diagram, the de facto standard:

| State | PLCopen | Invariants on every frame | Leaves on |
|---|---|---|---|
| Disconnected | | none | both ports up + fresh frame → Preflight |
| Preflight | | fresh | identity + config verified → Standby; mismatch → Fault |
| Standby | Disabled | fresh | bring-up requested → BringUp |
| BringUp | Power on | fresh, remote, no E-stop | each step observed → Ready; timeout → Fault |
| Ready | Standstill | fresh, remote, servo on, synced, no alarm/collision, expected tool/frame, stopped | family opened → Executing |
| Executing | Motion | Ready's set minus "stopped", plus family monitors | done → Stopping; violation → Fault |
| Stopping | Stopping | fresh | stopped observed + family closed → Ready; timeout → Fault |
| Fault | ErrorStop | none | acknowledge → Standby (never directly Ready) |

### Rules

1. **Stop is always allowed.** First check in `step`, before dispatching on the
   state. `stop()` is callable from any thread and jumps the event queue. It is
   an IEC 60204-1 category 2 software stop, not a safety function; E-stop and
   S_STOP are.
2. **Any failed invariant goes straight to Fault.** Conditions are named
   `constexpr` entries in one table per state, evaluated centrally on every
   frame; the first failed name becomes the fault reason.
3. **Precondition versus invariant.** The same conditions checked on a request
   reject the request (no state change, not a fault); checked on a frame they
   fault.
4. **Observed, not assumed.** A reply means "accepted". "Done" is when the
   frame shows it.
5. **Every waiting state has a timeout.**
6. **Families switch only from rest.** Requesting a different family while
   moving is rejected with a reason, never answered with a surprise stop.

### Implementation

- Core: `Step step(const State&, const Event&)`, pure. States are a
  `std::variant` because most carry data that must not outlive them
  (`Executing{lease, family}`, `Fault{record}`, `Stopping{deadline}`).
- Shell: one thread, one event queue; executes the returned actions; logs every
  transition with its cause. A logged event sequence replays exactly.
- Motion lease: only the supervisor mints one, at most one at a time, only in
  Ready. Motion sends require it; revoking it is the first action of every
  fault.

---

## 8. Threads

| Thread | Does | Must never |
|---|---|---|
| feedback receiver | 8056 recv, decode, publish, post event | block on anything else |
| supervisor | event loop, command client, family open/close | be skipped by stop |
| servo loop (online only) | pull setpoint, safety-filter, send | allocate, lock, log, wait for replies |
| application | plan, request, `stop()` | touch sockets |

---

## 9. Build order

1. **done:** repository, build system, CI; `net::TcpSocket`, `feedback::Framer`,
   `feedback::decode`, `apps/ec_monitor`, `tools/fake_8056.py`.
2. **done:** `command::*` structs for 5 queries + `MoveByJoint` + `Stop`, and the
   codec (nlohmann-json). Tested against the manual's templates and real EA66
   replies; every result arrives wrapped in a JSON string and is unwrapped.
3. `command::LineFramer` + `command::Client` + `apps/ec_query` (replaces `nc`).
4. `feedback::Receiver` + `feedback::Latest`; `ec_monitor` moves onto them; SI
   boundary.
5. `supervisor::step` with tests; then the shell in observe-only mode.
6. `motion::PointToPoint` behind the supervisor, 10 % speed.
7. `motion::TimedTrajectory` + validator + tracking monitor.
8. `motion::ServoStream` + `motion::ServoLoop`.

---

## 10. Open questions for the robot

- Does the controller accept two simultaneous 8055 connections?
- What does ServoStream do when starved? What does a TimedTrajectory do if the
  host dies mid-run?
- Do the close sequences in `sdk_calls.md` leave the controller ready for every
  other family?
- Which tool does `machinePose` refer to; what convention are rx/ry/rz?
