#!/usr/bin/env python3
"""The command-line tools, end to end, against the fake servers in tools/.

Run by ctest (tests/CMakeLists.txt) on native builds:

    test_apps.py --ec-query PATH --ec-monitor PATH --ec-record PATH --ec-move PATH --tools DIR

Every invocation names 127.0.0.1 explicitly, so not even a command line that
parses by mistake can reach the robot's default address. Standard library only.
"""

import argparse
import json
import math
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

PATHS = argparse.Namespace()  # filled in by main()
LOCAL = "127.0.0.1"


class FakeServer:
    """One of the fake controller ports, on a free port chosen by the OS."""

    def __init__(self, script: str, *options: str):
        self.proc = subprocess.Popen(
            [sys.executable, str(PATHS.tools / script), "--host", LOCAL, "--port", "0", *options],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        first = self.proc.stdout.readline()  # "fake 8055 on 127.0.0.1:PORT, Ctrl+C to quit"
        match = re.search(r":(\d+),", first)
        if not match:
            self.proc.kill()
            self.proc.wait()
            self.proc.stdout.close()
            raise RuntimeError(f"{script} did not start: {first!r}")
        self.port = int(match.group(1))
        self.drain = threading.Thread(target=self.proc.stdout.read, daemon=True)  # keep the pipe from filling
        self.drain.start()

    def stop(self) -> None:
        self.proc.kill()
        self.proc.wait()
        self.drain.join(timeout=5)  # the pipe reaches end of file once the process is gone
        self.proc.stdout.close()


class FakeRobot:
    """tools/fake_robot.py on free ports: a simulated arm on its command,
    monitor and control ports."""

    def __init__(self, *options: str):
        self.proc = subprocess.Popen(
            [sys.executable, str(PATHS.tools / "fake_robot.py"), "--host", LOCAL, "--port", "0", *options],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        first = self.proc.stdout.readline()  # "fake robot on 127.0.0.1: command P, monitor P, control P, ..."
        match = re.search(r"command (\d+), monitor (\d+), control (\d+)", first)
        if not match:
            self.proc.kill()
            self.proc.wait()
            self.proc.stdout.close()
            raise RuntimeError(f"fake_robot.py did not start: {first!r}")
        self.command, self.monitor, self.control_port = map(int, match.groups())
        self.drain = threading.Thread(target=self.proc.stdout.read, daemon=True)
        self.drain.start()

    def control(self, line: str) -> str:
        with socket.create_connection((LOCAL, self.control_port), timeout=5) as s:
            s.sendall((line + "\n").encode())
            answer = b""
            while not answer.endswith(b"\n"):
                chunk = s.recv(4096)
                if not chunk:
                    break
                answer += chunk
        return answer.decode().strip()

    def state(self) -> dict:
        return json.loads(self.control("state"))

    def stop(self) -> None:
        self.proc.kill()
        self.proc.wait()
        self.drain.join(timeout=5)
        self.proc.stdout.close()


def run(tool: Path, *args: str, timeout: float = 20.0) -> subprocess.CompletedProcess:
    return subprocess.run([str(tool), *args], capture_output=True, text=True, timeout=timeout)


def closed_port() -> int:
    """A port nothing listens on (free a moment ago)."""
    with socket.create_server((LOCAL, 0)) as s:
        return s.getsockname()[1]


class EcQuery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fake = FakeServer("fake_8055.py", "--split", "--stale", "--garbage")

    @classmethod
    def tearDownClass(cls):
        cls.fake.stop()

    def query(self, *args: str, port: int = 0) -> subprocess.CompletedProcess:
        return run(PATHS.ec_query, "--host", LOCAL, "--port", str(port or self.fake.port), *args)

    def test_reads_through_split_stale_and_garbage_replies(self):
        r = self.query("getSoftVersion")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stdout.strip(), '"V3.15.3"')

    def test_params_as_pairs_or_as_one_json_object(self):
        for params in (["index=0"], ['{"index":0}']):
            with self.subTest(params=params):
                r = self.query("getDH", *params)
                self.assertEqual(r.returncode, 0, r.stderr)
                self.assertEqual(r.stdout.strip(), "95.431")

    def test_controller_error_is_a_failure(self):
        r = self.query("--untyped", "--write", "getNothing")
        self.assertEqual(r.returncode, 1)
        self.assertIn("method not found", r.stderr)

    def test_unknown_methods_are_refused_before_sending(self):
        r = self.query("getNothing")
        self.assertEqual(r.returncode, 2)
        self.assertIn("not a call of the SDK manual", r.stderr)
        r = self.query("--untyped", "getNothing")
        self.assertEqual(r.returncode, 2)
        self.assertIn("--write", r.stderr)

    def test_lists_every_call_of_the_catalog(self):
        r = run(PATHS.ec_query, "--list")
        self.assertEqual(r.returncode, 0, r.stderr)
        listed = dict(line.split("\t") for line in r.stdout.splitlines())
        self.assertEqual(len(listed), 184)
        self.assertEqual(listed["getRobotState"], "query")
        self.assertEqual(listed["moveByJoint"], "motion")
        self.assertEqual(listed["poseMul"], "compute")
        catalog = (PATHS.tools.parent / "docs" / "sdk_calls.md").read_text(encoding="utf-8")
        documented = set(re.findall(r"^\| 2\.2\.[0-9.]+ \| `([A-Za-z_0-9]+)`", catalog, re.M))
        self.assertEqual(set(listed), documented)

    def test_checks_every_query_against_its_type(self):
        r = self.query("--check-all")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("ok        getSoftVersion", r.stdout)
        self.assertIn("ok        get_joint_pos", r.stdout)
        self.assertRegex(r.stdout, r"\d+ fit their types, 0 did not, \d+ failed")
        self.assertNotIn("moveByJoint", r.stdout)  # queries only

    def test_computations_are_read_only(self):
        r = self.query("poseMul", "pose1=[100,0,0,0,0,90]", "pose2=[10,0,0,0,0,0]", "unit_type=0")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual([round(v, 6) for v in json.loads(r.stdout)], [100, 10, 0, 0, 0, 90])

    def test_writes_need_the_flag(self):
        r = self.query("stop")
        self.assertEqual(r.returncode, 2)
        self.assertIn("--write", r.stderr)
        r = self.query("--write", "stop")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stdout.strip(), "true")

    def test_late_reply_is_a_timeout(self):
        slow = FakeServer("fake_8055.py", "--delay-ms", "1000")
        try:
            start = time.monotonic()
            r = self.query("--timeout", "300", "getRobotMode", port=slow.port)
            elapsed = time.monotonic() - start
        finally:
            slow.stop()
        self.assertEqual(r.returncode, 1)
        self.assertIn("timeout", r.stderr)
        self.assertLess(elapsed, 5.0)

    def test_nothing_listening_is_a_failure(self):
        r = self.query("getRobotMode", port=closed_port())
        self.assertEqual(r.returncode, 1)
        self.assertIn("connect", r.stderr)

    def test_bad_command_lines(self):
        port = str(closed_port())  # never reached: each line fails before connecting
        for args in (
            [],
            ["--port", "abc", "getRobotMode"],
            ["--port", "70000", "getRobotMode"],
            ["--timeout", "0", "getRobotMode"],
            ["getDH", "index"],
            ["getDH", "{not json"],
        ):
            with self.subTest(args=args):
                r = run(PATHS.ec_query, "--host", LOCAL, *(["--port", port] if "--port" not in args else []), *args)
                self.assertEqual(r.returncode, 2, r.stderr)

    def test_help(self):
        r = run(PATHS.ec_query, "--help")
        self.assertEqual(r.returncode, 0)
        self.assertIn("--write", r.stdout)


class EcMonitor(unittest.TestCase):
    def test_prints_frames_then_stops(self):
        fake = FakeServer("fake_8056.py", "--chop", "--garbage")
        try:
            r = run(PATHS.ec_monitor, LOCAL, "--port", str(fake.port), "--all", "--frames", "20")
        finally:
            fake.stop()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stdout.count("---- #"), 20)
        self.assertIn("stopped: 20 frames", r.stderr)
        self.assertIn("0 decode errors", r.stderr)

    def test_bad_command_lines(self):
        for args in (["--rate", "0"], ["--rate", "5", "--all"], ["--port", "0"], ["--frames", "x"]):
            with self.subTest(args=args):
                r = run(PATHS.ec_monitor, LOCAL, *args)
                self.assertEqual(r.returncode, 2, r.stderr)


class EcRecord(unittest.TestCase):
    FRAMES = 30
    RAW_HEADER, RAW_RECORD = 24, 8 + 1024

    def setUp(self):
        # Inside the build tree, next to the tool, never in the source tree.
        self.tmp = tempfile.TemporaryDirectory(dir=PATHS.ec_record.parent)
        self.dir = Path(self.tmp.name)
        self.has_h5 = "--h5" in run(PATHS.ec_record, "--help").stdout

    def tearDown(self):
        self.tmp.cleanup()

    def test_records_live_then_replays_into_hdf5(self):
        raw = self.dir / "run.ecraw"
        h5_options = ["--h5", str(self.dir / "live.h5")] if self.has_h5 else []
        fake = FakeServer("fake_8056.py", "--chop")
        try:
            r = run(PATHS.ec_record, LOCAL, "--port", str(fake.port), "--raw", str(raw), *h5_options,
                    "--frames", str(self.FRAMES))
        finally:
            fake.stop()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn(f"stopped: {self.FRAMES} frames", r.stderr)
        self.assertEqual(raw.stat().st_size, self.RAW_HEADER + self.FRAMES * self.RAW_RECORD)

        if not self.has_h5:
            self.skipTest("built without HDF5")
        self.assertGreater((self.dir / "live.h5").stat().st_size, 0)
        r = run(PATHS.ec_record, "--replay", str(raw), "--h5", str(self.dir / "replayed.h5"))
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn(f"stopped: {self.FRAMES} frames", r.stderr)

    def test_bad_command_lines(self):
        out = str(self.dir / "x.ecraw")
        for args in (
            [LOCAL],  # nothing to write
            [LOCAL, "--raw", out, "--seconds", "-1"],
            ["--replay", str(self.dir / "missing.ecraw"), "--raw", out],
            [LOCAL, "--port", "0", "--raw", out],
        ):
            with self.subTest(args=args):
                r = run(PATHS.ec_record, *args)
                self.assertEqual(r.returncode, 2, r.stderr)


class RotationConvention(unittest.TestCase):
    """tools/rotation_convention.py names the convention the fake computes with."""

    def find(self, convention: str, product: str) -> subprocess.CompletedProcess:
        fake = FakeServer("fake_8055.py", "--convention", convention, "--product", product)
        try:
            return subprocess.run(
                [sys.executable, str(PATHS.tools / "rotation_convention.py"), "--ec-query", str(PATHS.ec_query),
                 "--host", LOCAL, "--port", str(fake.port)],
                capture_output=True, text=True, timeout=60)
        finally:
            fake.stop()

    def test_finds_fixed_xyz_with_t1_t2(self):
        r = self.find("ZYX", "12")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("R = Rz(rz).Ry(ry).Rx(rx)", r.stdout)
        self.assertIn("poseMul(pose1, pose2) = T1.T2", r.stdout)

    def test_finds_another_convention(self):
        r = self.find("XYZ", "21")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("R = Rx(rx).Ry(ry).Rz(rz)", r.stdout)
        self.assertIn("poseMul(pose1, pose2) = T2.T1", r.stdout)


class RobotChecks(unittest.TestCase):
    """tools/robot_checks.py: the read-only measurements, against the fakes."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=PATHS.ec_record.parent)  # in the build tree
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def check(self, *args: str) -> subprocess.CompletedProcess:
        return subprocess.run([sys.executable, str(PATHS.tools / "robot_checks.py"), *args],
                              capture_output=True, text=True, timeout=60)

    def fake(self, *options: str) -> FakeServer:
        fake = FakeServer("fake_8055.py", *options)
        self.addCleanup(fake.stop)
        return fake

    def test_round_trip(self):
        port = str(self.fake().port)
        r = self.check("rtt", "--host", LOCAL, "--port", port, "--count", "20", "--period-ms", "0")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("controller V3.15.3, robot mode 2", r.stdout)
        self.assertIn("round trip of getRobotState, 20 calls", r.stdout)

    def test_pipelined_requests(self):
        port = str(self.fake("--split").port)
        r = self.check("pipeline", "--host", LOCAL, "--port", port)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("all 20 replies, in order", r.stdout)

    def test_two_connections_at_once(self):
        port = str(self.fake().port)
        r = self.check("connections", "--host", LOCAL, "--port", port, "--rounds", "5")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("both answered through 5 rounds", r.stdout)

    def test_one_connection_at_a_time(self):
        port = str(self.fake("--one-client").port)
        r = self.check("connections", "--host", LOCAL, "--port", port)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("serves one connection at a time", r.stdout)

    def test_drift_of_a_recording_from_ec_record(self):
        raw = self.dir / "live.ecraw"
        fake = FakeServer("fake_8056.py")
        try:
            r = run(PATHS.ec_record, LOCAL, "--port", str(fake.port), "--raw", str(raw), "--frames", "120")
        finally:
            fake.stop()
        self.assertEqual(r.returncode, 0, r.stderr)
        r = self.check("drift", str(raw))
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("120 frames over", r.stdout)

    def test_drift_finds_a_known_skew(self):
        # Frames every 8.25 ms with up to 0.6 ms of arrival jitter; the controller's
        # clock runs 100 ppm fast and stamps whole milliseconds.
        raw = self.dir / "skewed.ecraw"
        with raw.open("wb") as f:
            f.write(b"ECRAW001" + bytes(16))
            for k in range(20000):
                ideal = k * 8.25e-3
                arrival = int((ideal + (k * 7919 % 600) * 1e-6) * 1e9)
                controller = 1_791_200_688_449 + int(ideal * (1 + 100e-6) * 1000)
                f.write(arrival.to_bytes(8, "little", signed=True) + bytes(4)
                        + controller.to_bytes(8, "big") + bytes(1024 - 12))
        r = self.check("drift", str(raw))
        self.assertEqual(r.returncode, 0, r.stderr)
        ppm = float(re.search(r"([+-][0-9.]+) ppm", r.stdout).group(1))
        self.assertAlmostEqual(ppm, 100.0, delta=2.0)
        self.assertIn("8.251 ms on average", r.stdout)

    def test_failures(self):
        r = self.check("rtt", "--host", LOCAL, "--port", str(closed_port()))
        self.assertEqual(r.returncode, 1)
        self.assertIn("connect to", r.stderr)
        junk = self.dir / "junk.bin"
        junk.write_bytes(b"not a recording")
        r = self.check("drift", str(junk))
        self.assertEqual(r.returncode, 1)
        self.assertIn("not a raw recording", r.stderr)
        for args in (["rtt"], ["rtt", "--host", LOCAL, "--count", "0"], ["nothing"]):
            with self.subTest(args=args):
                self.assertEqual(self.check(*args).returncode, 2)


class EcMove(unittest.TestCase):
    """ec_move against the moving fake: plans, refusals, bring-up, moves, a
    streamed sine, and an emergency stop in the middle of a move."""

    def robot(self, *options: str) -> FakeRobot:
        fake = FakeRobot(*options)
        self.addCleanup(fake.stop)
        return fake

    def move(self, fake: FakeRobot, *args: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
        return run(PATHS.ec_move, "--host", LOCAL, "--command-port", str(fake.command), "--monitor-port",
                   str(fake.monitor), *args, timeout=timeout)

    def test_status_reads_only(self):
        fake = self.robot()
        r = self.move(fake, "status")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("mode remote, state stop, servos off", r.stdout)
        self.assertIn("conditions:", r.stdout)

    def test_a_move_without_yes_only_prints_its_plan(self):
        fake = self.robot("--ready")
        r = self.move(fake, "joint", "10", "-90", "0", "-90", "90", "0")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("nothing moved", r.stdout)
        self.assertAlmostEqual(fake.state()["q"][0], 0.0)

    def test_a_move_before_bring_up_is_refused_naming_what_is_missing(self):
        fake = self.robot()
        r = self.move(fake, "--yes", "joint", "10", "-90", "0", "-90", "90", "0")
        self.assertEqual(r.returncode, 1)
        self.assertIn("cannot start without: servo on, encoders synchronised", r.stderr)

    def test_bring_up_then_move(self):
        fake = self.robot()
        r = self.move(fake, "--yes", "bring-up")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertTrue(fake.state()["servo"])
        r = self.move(fake, "--yes", "joint", "10", "-90", "0", "-90", "90", "0", "--speed", "0.3")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertAlmostEqual(fake.state()["q"][0], 10.0, places=3)

    def test_nudge_and_a_streamed_sine(self):
        fake = self.robot("--ready")
        r = self.move(fake, "--yes", "nudge", "6", "5")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertAlmostEqual(fake.state()["q"][5], 5.0, places=3)
        r = self.move(fake, "--yes", "sine", "6", "2", "0.5", "--seconds", "1")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("sine: done", r.stdout)
        state = fake.state()
        self.assertFalse(state["streaming"])  # the stream was cleared
        self.assertLess(abs(state["q"][5] - 5.0), 3.0)  # stopped near where the sine was

    def test_an_emergency_stop_ends_a_move(self):
        fake = self.robot("--ready")
        timer = threading.Timer(0.5, lambda: fake.control("estop on"))
        timer.start()
        self.addCleanup(timer.cancel)
        r = self.move(fake, "--yes", "joint", "60", "-90", "0", "-90", "90", "0", "--speed", "0.02")
        self.assertEqual(r.returncode, 1)
        self.assertIn("condition lost", r.stderr)
        self.assertIn("no emergency stop", r.stderr)
        self.assertLess(fake.state()["q"][0], 60.0)

    def test_teach_mode_refuses_motion(self):
        fake = self.robot("--ready")
        fake.control("mode teach")
        r = self.move(fake, "--yes", "nudge", "1", "2")
        self.assertEqual(r.returncode, 1)
        self.assertIn("remote mode", r.stderr)

    def test_bad_command_lines(self):
        fake = self.robot()
        for args in (["nudge", "7", "5"], ["nudge", "1", "45"], ["sine", "6", "25", "1"], ["joint", "1", "2"], []):
            with self.subTest(args=args):
                self.assertEqual(self.move(fake, *args).returncode, 2)


class EcMoveCircle(unittest.TestCase):
    """ec_move circle against the moving fake: the plan alone, then the circle
    drawn in every mode, checked against the fake's own trace of its TCP."""

    RADIUS = 20.0  # mm
    READY = "joints 0 -90 90 -90 -90 0"  # elbow bent, the tool pointing down

    def robot(self) -> FakeRobot:
        fake = FakeRobot("--ready")
        self.addCleanup(fake.stop)
        self.assertEqual(fake.control(self.READY), "ok")
        return fake

    def circle(self, fake: FakeRobot, *args: str, timeout: float = 60.0) -> subprocess.CompletedProcess:
        return run(PATHS.ec_move, "--host", LOCAL, "--command-port", str(fake.command), "--monitor-port",
                   str(fake.monitor), *args, timeout=timeout)

    def test_without_yes_it_only_plans(self):
        fake = self.robot()
        tmp = tempfile.TemporaryDirectory(dir=PATHS.ec_move.parent)  # in the build tree
        self.addCleanup(tmp.cleanup)
        plan = Path(tmp.name) / "plan.csv"
        before = fake.state()["q"]
        r = self.circle(fake, "circle", "follow", "--radius", "20", "--save", str(plan))
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("points to check: start (0 deg) (-476.0, -122.0, 424.4) mm", r.stdout)  # the TCP is the centre
        self.assertIn("nothing moved", r.stdout)
        self.assertEqual(fake.state()["q"], before)
        rows = plan.read_text().splitlines()
        self.assertTrue(rows[0].startswith("# ec_move circle follow"))
        self.assertEqual(rows[2].split(",")[:4], ["t_s", "x_mm", "y_mm", "z_mm"])
        self.assertEqual(rows[3].split(",")[:2], ["0.000", "-476.0000"])

    def test_every_mode_draws_the_circle(self):
        # mode, and how far its TCP may stray [mm]: chords for path and lines;
        # for the live pose stream, the shaper's lag, which bends the path (it
        # trails each joint's target by its braking distance: 1.3 mm here at
        # 60 mm/s, a ninth of that at 20); the correction's lag for velocity.
        for mode, tolerance in (("arc", 0.05), ("path", 0.15), ("lines", 0.8), ("trajectory", 0.05),
                                ("follow", 0.05), ("pose-stream", 2.0), ("velocity", 0.5)):
            with self.subTest(mode=mode):
                fake = self.robot()
                x, y, z = fake.state()["pose"][:3]
                cx = x - self.RADIUS  # the TCP is the circle's first point: no approach
                fake.control("trace start")
                r = self.circle(fake, "--yes", "circle", mode, "--radius", str(self.RADIUS), "--center", str(cx),
                                str(y), str(z), "--start-toward", "1", "0", "0", "--speed", "60", "--ramp", "0.3")
                self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
                self.assertIn(f"circle ({mode}): done", r.stdout)
                self.assertNotIn("approach", r.stdout)
                trace = json.loads(fake.control("trace"))
                radial = [abs(math.hypot(p[0] - cx, p[1] - y) - self.RADIUS) for p in trace]
                self.assertLess(max(radial), tolerance)
                self.assertLess(max(abs(p[2] - z) for p in trace), max(tolerance, 0.05))
                angles = sorted(math.atan2(p[1] - y, p[0] - cx) for p in trace)
                gaps = [b - a for a, b in zip(angles, angles[1:])] + [angles[0] + 2 * math.pi - angles[-1]]
                self.assertLess(max(gaps), math.radians(45))  # all the way round
                end = trace[-1]
                self.assertLess(math.hypot(end[0] - x, end[1] - y), 0.5)  # back at the start

    def test_an_unreachable_circle_is_refused_before_anything_moves(self):
        fake = self.robot()
        before = fake.state()["q"]
        r = self.circle(fake, "--yes", "circle", "follow", "--radius", "500", "--max-approach", "2000")
        self.assertEqual(r.returncode, 1)
        self.assertIn("planning: invalid: no joint positions for the pose", r.stderr)
        self.assertEqual(fake.state()["q"], before)

    def test_too_fast_for_a_stream_is_refused(self):
        fake = self.robot()
        r = self.circle(fake, "--yes", "circle", "follow", "--radius", "5", "--speed", "250", "--ramp", "0")
        self.assertEqual(r.returncode, 1)
        self.assertIn("too fast for a stream", r.stderr)

    def test_an_emergency_stop_ends_a_streamed_circle_and_the_recording_has_it(self):
        fake = self.robot()
        tmp = tempfile.TemporaryDirectory(dir=PATHS.ec_move.parent)
        self.addCleanup(tmp.cleanup)
        recording = Path(tmp.name) / "circle.ecraw"
        timer = threading.Timer(2.5, lambda: fake.control("estop on"))
        timer.start()
        self.addCleanup(timer.cancel)
        r = self.circle(fake, "--yes", "--record", str(recording), "circle", "follow", "--radius", "20")
        self.assertEqual(r.returncode, 1)
        self.assertIn("condition lost", r.stderr)
        self.assertFalse(fake.state()["streaming"])
        self.assertRegex(r.stdout, r"recorded \d+ frames to")
        self.assertGreater(recording.stat().st_size, 100 * 1032)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ec-query", type=Path, required=True)
    p.add_argument("--ec-monitor", type=Path, required=True)
    p.add_argument("--ec-record", type=Path, required=True)
    p.add_argument("--ec-move", type=Path, required=True)
    p.add_argument("--tools", type=Path, required=True)
    args, rest = p.parse_known_args()
    vars(PATHS).update(vars(args))
    unittest.main(argv=[sys.argv[0], *rest], verbosity=2)


if __name__ == "__main__":
    main()
