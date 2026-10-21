#!/usr/bin/env python3
"""A fake EC controller that moves: ports 8055 and 8056 of one simulated arm,
for testing the driver without the robot (the robot: docs/ec66.md).

    python3 tools/fake_robot.py                  # 127.0.0.1: command 8055, monitor 8056, control 8057
    python3 tools/fake_robot.py --port 0         # free ports, printed on the first line

Simulated, at the monitor rate (8 ms), on an EC66-like arm (tools/kinematics.py):
  - modes, servo power, encoder synchronisation, alarms, collision, E-stop;
  - joint moves (moveByJoint), stop, the speed setting (setSpeed), which scales
    joint moves;
  - forward and inverse kinematics (positiveKinematic, inverseKinematic, the
    solution nearest the reference), get_tcp_pose, and the TCP pose in frames;
  - transparent transmission: a queue consumed one point per t after the
    lookahead; an empty queue holds the last point (smoothing is ignored);
  - moveBySpeedj / stopj, each command valid for its t;
  - timestamped trajectories: upload, check, start, pause, resume, stop, flush.
Not simulated: Cartesian moves (moveByLine, moveByArc, paths: the driver moves
in joint space), jog, tools and user frames (the TCP is the flange, poses are
in the base frame), dynamics, the controller's smoothing. Where the real
controller is not yet measured (docs/ec66.md §8), the fake makes the assumption
noted where it does it.

Faults are injected through the control port, one command per line, answered
with "ok" (or the state, as JSON):
    estop on | estop off        emergency stop: the arm stops, the servos go off; state 2, then alarm
    collision                   the arm stops; state 5 and the collision flag until resetCollisionState
    alarm                       the arm stops; state 4 until clearAlarm
    mode teach | play | remote  the pendant's key switch
    freeze on | freeze off      stop / resume sending monitor frames
    drop                        close every command connection
    reject METHOD               answer METHOD with false from now on
    delay METHOD SECONDS        answer METHOD that much later from now on (its effect is at once)
    ready                       servos on, synchronised, no alarm (skips bring-up)
    joints J1 .. J6             put the arm there [deg] (while it stands still)
    trace start | trace         record the TCP pose every period from now | the poses recorded, as JSON
    state                       the simulated state, as JSON

Standard library only; Linux and Windows.
"""

import argparse
import collections
import json
import math
import socket
import struct
import threading
import time

import fake_8055
import kinematics
import poses

FRAME_SIZE = 1024
MATCHING_WORD = 0xEC8056EC
PERIOD = 0.008
MAX_SPEED = [150.0, 150.0, 190.0, 260.0, 260.0, 260.0]  # deg/s, the EC66 datasheet
HOME = [0.0, -90.0, 0.0, -90.0, 90.0, 0.0]

STOP, PAUSE, ESTOP, RUNNING, ALARM, COLLISION = 0, 1, 2, 3, 4, 5
TEACH, PLAY, REMOTE = 0, 1, 2


class RpcError(Exception):
    def __init__(self, code: int, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


def not_remote() -> RpcError:
    return RpcError(-32693, "this command is only supported in remote mode")


def joints(values) -> list:
    """Six joint positions [deg] from a request, or RpcError."""
    q = [float(v) for v in values]
    if len(q) != 6 or any(abs(v) > 360 for v in q):
        raise RpcError(-32693, "joint positions out of range")
    return q


def recorded(method: str, params: dict):
    """A call the fake does not simulate: a computed pose, or a reply recorded from the EA66."""
    text = fake_8055.computed(method, params, argparse.Namespace(convention="ZYX", product="12"))
    if text is None:
        text = fake_8055.RESULTS.get(method)
    if text is None:
        raise RpcError(-32601, "Method not found.")
    return json.loads(text)


# ---------------------------------------------------------------- motions

class PointToPoint:
    """A joint-space move with a smooth (cubic) profile, all joints arriving together."""

    def __init__(self, start, target, speed_fraction):
        self.start = list(start)
        self.delta = [t - s for s, t in zip(start, target)]
        need = max(abs(d) / (v * speed_fraction) for d, v in zip(self.delta, MAX_SPEED))
        self.duration = max(0.05, 1.5 * need)  # a cubic's peak speed is 1.5x its mean
        self.t = 0.0

    def tick(self, arm, dt):
        self.t = min(self.t + dt, self.duration)
        u = self.t / self.duration
        s, ds = 3 * u * u - 2 * u ** 3, (6 * u - 6 * u * u) / self.duration
        arm.q = [a + d * s for a, d in zip(self.start, self.delta)]
        arm.qd = [d * ds for d in self.delta]
        return self.t < self.duration


class Brake:
    """Stops the arm within 0.1 s."""

    def __init__(self, qd):
        self.qd0 = list(qd)
        self.t = 0.0

    def tick(self, arm, dt):
        self.t += dt
        k = max(0.0, 1.0 - self.t / 0.1)
        arm.qd = [v * k for v in self.qd0]
        arm.q = [q + v * dt for q, v in zip(arm.q, arm.qd)]
        return k > 0.0


class Stream:
    """Transparent transmission: one queued point per period, once the lookahead has passed."""

    def __init__(self, lookahead_ms, t_ms, replies):
        self.lookahead = lookahead_ms / 1000.0
        self.period = t_ms / 1000.0
        self.replies = replies
        self.queue = collections.deque()
        self.next_take = None  # when the next point is taken
        self.last_taken = None

    def put(self, point, now):
        if self.next_take is None:
            self.next_take = now + self.lookahead
        self.queue.append(list(point))

    def tick(self, arm, dt, now):
        while self.next_take is not None and now >= self.next_take:
            if not self.queue:  # ran dry: hold the last point, and keep the pace from now on
                self.next_take = now + self.period
                break
            point = self.queue.popleft()
            arm.qd = [(p - q) / self.period for p, q in zip(point, arm.q)]
            arm.q = point
            self.last_taken = now
            self.next_take += self.period
        if self.last_taken is None or now - self.last_taken > 2 * self.period:
            arm.qd = [0.0] * 6
        return any(abs(v) > 1e-9 for v in arm.qd)


class SpeedJ:
    """Joint velocity, ramped at acc, valid for t."""

    def __init__(self):
        self.target = [0.0] * 6
        self.acc = 1.0
        self.until = 0.0

    def command(self, vj, acc, t, now):
        self.target, self.acc, self.until = list(vj), float(acc), now + float(t)

    def tick(self, arm, dt, now):
        goal = self.target if now < self.until else [0.0] * 6
        step = self.acc * dt
        arm.qd = [v + max(-step, min(step, g - v)) for v, g in zip(arm.qd, goal)]
        arm.q = [q + v * dt for q, v in zip(arm.q, arm.qd)]
        return now < self.until or any(abs(v) > 1e-9 for v in arm.qd)


class Trajectory:
    def __init__(self):
        self.expected = 0
        self.points = []  # (time, joints)
        self.sealed = False
        self.playing = False
        self.paused = False
        self.t = 0.0
        self.scale = 1.0

    def check(self):
        if len(self.points) != self.expected:
            return -1
        times = [t for t, _ in self.points]
        if not times or times[0] != 0 or any(b <= a for a, b in zip(times, times[1:])):
            return -3
        return 0

    def tick(self, arm, dt):
        if not self.playing or self.paused:
            arm.qd = [0.0] * 6
            return self.playing
        self.t += dt * self.scale
        last_t, last_q = self.points[-1]
        if self.t >= last_t:
            arm.qd = [0.0] * 6
            arm.q = list(last_q)
            self.playing = False
            return False
        k = next(i for i, (t, _) in enumerate(self.points) if t > self.t)
        (t0, q0), (t1, q1) = self.points[k - 1], self.points[k]
        u = (self.t - t0) / (t1 - t0)
        new_q = [a + (b - a) * u for a, b in zip(q0, q1)]
        arm.qd = [(n - q) / dt for n, q in zip(new_q, arm.q)]
        arm.q = new_q
        return True


# ------------------------------------------------------------------- the arm

class Arm:
    def __init__(self, ready: bool):
        self.lock = threading.Lock()
        self.q = list(HOME)
        self.qd = [0.0] * 6
        self.mode = REMOTE
        self.state = STOP
        self.servo = ready
        self.synced = ready
        self.collision = False
        self.estop = False
        self.speed_pct = 100.0
        self.motion = None  # PointToPoint or Brake
        self.stream = None
        self.speedj = None
        self.trajectory = Trajectory()
        self.rejected = set()
        self.delays = {}  # method -> seconds before its reply
        self.pending = []  # (when, function): delayed effects
        self.estop_at = None
        self.pose = kinematics.forward(self.q)  # the TCP: the flange [mm, rad]
        self.tcp_speed = 0.0
        self.trace = None  # the poses recorded since "trace start"

    # -- the simulation

    def tick(self, now: float) -> None:
        for when, effect in [p for p in self.pending if p[0] <= now]:
            effect()
        self.pending = [p for p in self.pending if p[0] > now]
        if self.estop_at is not None and now - self.estop_at > 0.1 and self.state == ESTOP:
            self.state = ALARM  # the manual: E-stop shows briefly, then alarm
        moving = False
        if self.motion is not None:
            moving = self.motion.tick(self, PERIOD)
            if not moving:
                self.motion = None
                self.qd = [0.0] * 6
        elif self.stream is not None:
            moving = self.stream.tick(self, PERIOD, now)
        elif self.speedj is not None:
            moving = self.speedj.tick(self, PERIOD, now)
            if not moving:
                self.speedj = None
        elif self.trajectory.playing:
            moving = self.trajectory.tick(self, PERIOD)
        else:
            self.qd = [0.0] * 6
        if self.state in (STOP, RUNNING):
            self.state = RUNNING if moving else STOP
        pose = kinematics.forward(self.q)
        self.tcp_speed = math.dist(pose[:3], self.pose[:3]) / PERIOD
        self.pose = pose
        if self.trace is not None:
            self.trace.append(pose)

    def halt(self, state: int) -> None:
        """Faults: the arm stops at once and every motion ends."""
        self.motion = self.stream = self.speedj = None
        self.trajectory.playing = False
        self.qd = [0.0] * 6
        self.state = state

    def stop(self) -> None:
        self.stream = self.speedj = None
        self.trajectory.playing = False
        if any(abs(v) > 1e-9 for v in self.qd):
            self.motion = Brake(self.qd)
        else:
            self.motion = None

    def frame(self) -> bytes:
        q = self.q + [0.0, 0.0]
        qd = self.qd + [0.0, 0.0]
        b = struct.pack(">IQB", FRAME_SIZE, int(time.time() * 1000), 2)
        b += struct.pack(">8d", *q)
        b += struct.pack(">6d", *self.pose) * 2  # machinePose, machineUserPose: the TCP is the flange
        b += struct.pack(">8d", *[1.0] * 8)
        b += struct.pack(">iii", self.state, 1 if self.servo else 0, 1 if self.synced else 0)
        b += struct.pack(">8i", *[0] * 8)
        b += struct.pack(">i", self.mode)
        b += struct.pack(">3d", 0.0, 0.0, 0.0)
        b += struct.pack(">5d", *[0.0] * 5)
        b += struct.pack(">QQ", 0, 0)
        b += b"\x01" if self.collision else b"\x00"
        b += struct.pack(">6d", *self.pose) * 2  # machineFlangePose, machineUserFlangePose
        b += b"\x01" if self.estop else b"\x00"
        b += struct.pack(">d", self.tcp_speed)
        b += struct.pack(">8d", *qd)
        b += struct.pack(">d", 0.0)
        b += struct.pack(">8d", *[0.0] * 8)
        b += struct.pack(">6d", *[35.0] * 6)
        b += struct.pack(">6d", *[0.0] * 6)
        b += struct.pack(">6d", *[0.0] * 6)
        b += struct.pack(">6d", *[0.0] * 6)
        b += b"\x00\x00"
        assert len(b) == 801, len(b)
        return b + bytes(219) + struct.pack(">I", MATCHING_WORD)

    # -- commands

    def can_move(self) -> bool:
        return (self.servo and self.synced and not self.estop and not self.collision
                and self.state == STOP and self.motion is None and self.stream is None
                and not self.trajectory.playing)

    def handle(self, method: str, params, now: float):
        """The result of one call (a Python value), or RpcError."""
        p = params if isinstance(params, dict) else {}
        remote = self.mode == REMOTE
        if method in self.rejected:
            return False

        # Queries.
        queries = {
            "getRobotState": lambda: self.state,
            "getRobotMode": lambda: self.mode,
            "getServoStatus": lambda: self.servo,
            "getMotorStatus": lambda: self.synced,
            "get_joint_pos": lambda: self.q,
            "get_actual_joint": lambda: self.q,
            "get_joint_speed": lambda: self.qd,
            "getSpeed": lambda: self.speed_pct,
            "get_estop_status": lambda: 1 if self.estop else 0,
            "getCollisionState": lambda: 1 if self.collision else 0,
            "get_transparent_transmission_state": lambda: 1 if self.stream is not None else 0,
            "check_trajectory": self.trajectory.check,
        }
        if method in queries:
            return queries[method]()
        if method in ("get_tcp_pose", "positiveKinematic", "inverseKinematic"):
            return self.kinematics(method, p)

        # Everything else changes the controller: remote mode only.
        if method in ("set_servo_status", "syncMotorStatus", "clearAlarm", "resetCollisionState", "setSpeed",
                      "moveByJoint", "stop", "moveBySpeedj", "stopj",
                      "transparent_transmission_init", "tt_put_servo_joint_to_buf", "tt_clear_servo_joint_buf",
                      "start_push_pos", "push_pos", "stop_push_pos", "start_trajectory", "pause_trajectory",
                      "resume_trajectory", "stop_trajectory", "flush_trajectory"):
            if not remote:
                raise not_remote()
        else:
            return recorded(method, p)

        if method == "set_servo_status":
            on = int(p.get("status", 0)) == 1
            if on and (self.estop or self.state == ALARM or not self.synced):
                return False
            self.pending.append((now + 0.05, lambda: setattr(self, "servo", on)))
            return True
        if method == "syncMotorStatus":
            if self.estop:
                return False
            self.pending.append((now + 0.05, lambda: setattr(self, "synced", True)))
            return True
        if method == "clearAlarm":
            if self.estop:
                return False
            if self.state == ALARM:
                self.state = STOP
            return True
        if method == "resetCollisionState":
            self.collision = False
            if self.state == COLLISION:
                self.state = STOP
            return True
        if method == "setSpeed":
            self.speed_pct = float(p["value"])
            return True
        if method == "stop":
            self.stop()
            return True
        if method == "moveByJoint":
            target = joints(p["targetPos"])
            if not self.can_move():
                return False
            fraction = float(p["speed"]) / 100.0 * self.speed_pct / 100.0
            self.motion = PointToPoint(self.q, target, max(fraction, 1e-4))
            self.state = RUNNING
            return True
        if method == "moveBySpeedj":
            if not self.servo or not self.synced or self.estop or self.collision or self.state not in (STOP, RUNNING):
                return False
            if self.speedj is None:
                if self.motion is not None or self.stream is not None:
                    return False
                self.speedj = SpeedJ()
            self.speedj.command(p["vj"], p["acc"], p["t"], now)
            return True
        if method == "stopj":
            if self.speedj is not None:
                self.speedj.command([0.0] * 6, p["acc"], 0.0, now)
            return True
        if method == "transparent_transmission_init":
            if not self.can_move():
                return False
            self.stream = Stream(int(p["lookahead"]), int(p["t"]), int(p.get("response_enable", 1)) == 1)
            return True
        if method == "tt_put_servo_joint_to_buf":
            if self.stream is None:
                return False
            self.stream.put([float(v) for v in p["targetPos"]], now)
            return True if self.stream.replies else None
        if method == "tt_clear_servo_joint_buf":
            self.stream = None
            return True
        if method == "start_push_pos":
            self.trajectory = Trajectory()
            self.trajectory.expected = int(p["path_lenth"])
            self.trajectory.replies = int(p.get("ret_flag", 1)) == 1
            return True
        if method == "push_pos":
            self.trajectory.points.append((float(p["timestamp"]), [float(v) for v in p["pos"]]))
            return True if getattr(self.trajectory, "replies", True) else None
        if method == "stop_push_pos":
            self.trajectory.sealed = True
            return self.trajectory.check() == 0
        if method == "start_trajectory":
            if self.trajectory.check() != 0 or not self.can_move():
                return False
            first = self.trajectory.points[0][1]
            if max(abs(a - b) for a, b in zip(first, self.q)) > 1.0:
                return False
            self.trajectory.scale = float(p["speed_percent"]) / 100.0
            self.trajectory.t = 0.0
            self.trajectory.paused = False
            self.trajectory.playing = True
            self.state = RUNNING
            return True
        if method == "pause_trajectory":
            self.trajectory.paused = True
            return True
        if method == "resume_trajectory":
            self.trajectory.paused = False
            return True
        if method == "stop_trajectory":
            self.trajectory.playing = False
            return True
        if method == "flush_trajectory":
            self.trajectory = Trajectory()
            return True
        raise RpcError(-32601, "Method not found.")

    def kinematics(self, method: str, p):
        degrees = int(p.get("unit_type", 1)) == 0
        if method == "get_tcp_pose":
            pose = list(self.pose)
        elif method == "positiveKinematic":
            pose = kinematics.forward(joints(p["targetPos"]))
        else:
            target = [float(v) for v in p["targetPose"]]
            if degrees:
                target = poses.to_radians(target)
            reference = joints(p["referencePos"]) if "referencePos" in p else self.q
            q = kinematics.inverse(target, reference)
            if q is None:
                raise RpcError(-32693, "inverse kinematics: no solution near the reference")
            return q
        return poses.to_degrees(pose) if degrees else pose

    def inject(self, words, now: float) -> str:
        cmd = words[0] if words else ""
        arg = words[1] if len(words) > 1 else ""
        if cmd == "estop":
            self.estop = arg == "on"
            if self.estop:
                self.halt(ESTOP)
                self.servo = False
                self.estop_at = now
            return "ok"
        if cmd == "collision":
            self.collision = True
            self.halt(COLLISION)
            return "ok"
        if cmd == "alarm":
            self.halt(ALARM)
            return "ok"
        if cmd == "mode":
            self.mode = {"teach": TEACH, "play": PLAY, "remote": REMOTE}[arg]
            return "ok"
        if cmd == "reject":
            self.rejected.add(arg)
            return "ok"
        if cmd == "delay":
            self.delays[arg] = float(words[2]) if len(words) > 2 else 0.0
            return "ok"
        if cmd == "ready":
            self.servo = self.synced = True
            self.collision = self.estop = False
            self.state = STOP
            return "ok"
        if cmd == "joints":
            if self.motion is not None or self.stream is not None or self.speedj is not None:
                return "moving"
            self.q = [float(v) for v in words[1:7]]
            self.qd = [0.0] * 6
            self.pose = kinematics.forward(self.q)
            return "ok"
        if cmd == "trace":
            if arg == "start":
                self.trace = []
                return "ok"
            return json.dumps(self.trace or [])
        if cmd == "state":
            return json.dumps({"q": self.q, "qd": self.qd, "pose": self.pose, "mode": self.mode, "state": self.state,
                               "servo": self.servo, "synced": self.synced, "collision": self.collision,
                               "estop": self.estop, "speed": self.speed_pct,
                               "streaming": self.stream is not None})
        return f"unknown command: {' '.join(words)}"


# --------------------------------------------------------------- the servers

class Robot:
    def __init__(self, ready: bool):
        self.arm = Arm(ready)
        self.monitors = []
        self.commands = []
        self.frozen = False
        self.lock = threading.Lock()  # guards the connection lists

    def simulate(self) -> None:
        next_t = time.monotonic()
        while True:
            next_t += PERIOD
            time.sleep(max(0.0, next_t - time.monotonic()))
            with self.arm.lock:
                self.arm.tick(time.monotonic())
                frame = None if self.frozen else self.arm.frame()
            if frame is None:
                continue
            with self.lock:
                for conn in list(self.monitors):
                    try:
                        conn.sendall(frame)
                    except OSError:
                        self.monitors.remove(conn)
                        conn.close()

    def serve_commands(self, conn: socket.socket) -> None:
        buffer = b""
        with conn:
            while True:
                try:
                    chunk = conn.recv(65536)
                except OSError:
                    return
                if not chunk:
                    return
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    try:
                        request = json.loads(line)
                    except ValueError:
                        continue
                    request_id = request.get("id")
                    method = request.get("method", "")
                    try:
                        with self.arm.lock:
                            result = self.arm.handle(method, request.get("params"), time.monotonic())
                            delay = self.arm.delays.get(method, 0.0)
                        if result is None:
                            continue  # asked not to reply
                        if delay > 0.0:
                            time.sleep(delay)  # a slow answer: this connection waits
                        reply = {"jsonrpc": "2.0", "result": json.dumps(result), "id": request_id}
                    except RpcError as e:
                        reply = {"jsonrpc": "2.0", "error": {"code": e.code, "message": e.message}, "id": request_id}
                    except (KeyError, TypeError, ValueError) as e:
                        reply = {"jsonrpc": "2.0", "error": {"code": -32693, "message": f"bad params: {e}"},
                                 "id": request_id}
                    try:
                        conn.sendall((json.dumps(reply, separators=(",", ":")) + "\n").encode())
                    except OSError:
                        return

    def serve_control(self, conn: socket.socket) -> None:
        with conn, conn.makefile("rw", encoding="utf-8", newline="\n") as f:
            for line in f:
                words = line.split()
                if words[:1] == ["freeze"]:
                    self.frozen = words[1:2] == ["on"]
                    answer = "ok"
                elif words[:1] == ["drop"]:
                    with self.lock:
                        for c in self.commands:
                            try:
                                c.shutdown(socket.SHUT_RDWR)
                            except OSError:
                                pass
                        self.commands.clear()
                    answer = "ok"
                else:
                    with self.arm.lock:
                        answer = self.arm.inject(words, time.monotonic())
                f.write(answer + "\n")
                f.flush()


def listen(host: str, port: int) -> socket.socket:
    return socket.create_server((host, port))


def accept_forever(server: socket.socket, on_connection) -> None:
    while True:
        conn, _ = server.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=on_connection, args=(conn,), daemon=True).start()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8055,
                   help="the command port; monitor and control follow it (0: three free ports)")
    p.add_argument("--ready", action="store_true", help="start with the servos on and synchronised")
    args = p.parse_args()

    if args.port == 0:
        servers = [listen(args.host, 0) for _ in range(3)]
    else:
        servers = [listen(args.host, args.port + i) for i in range(3)]
    command, monitor, control = servers
    robot = Robot(args.ready)

    def on_command(conn):
        with robot.lock:
            robot.commands.append(conn)
        robot.serve_commands(conn)

    def on_monitor(conn):
        with robot.lock:
            robot.monitors.append(conn)

    threading.Thread(target=robot.simulate, daemon=True).start()
    threading.Thread(target=accept_forever, args=(command, on_command), daemon=True).start()
    threading.Thread(target=accept_forever, args=(monitor, on_monitor), daemon=True).start()
    ports = [s.getsockname()[1] for s in servers]
    print(f"fake robot on {args.host}: command {ports[0]}, monitor {ports[1]}, control {ports[2]}, "
          f"Ctrl+C to quit", flush=True)
    accept_forever(control, robot.serve_control)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
