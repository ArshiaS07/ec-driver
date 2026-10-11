#!/usr/bin/env python3
"""The command-line tools, end to end, against the fake servers in tools/.

Run by ctest (tests/CMakeLists.txt) on native builds:

    test_apps.py --ec-query PATH --ec-monitor PATH --ec-record PATH --tools DIR

Every invocation names 127.0.0.1 explicitly, so not even a command line that
parses by mistake can reach the robot's default address. Standard library only.
"""

import argparse
import json
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
        r = self.query("getNothing")
        self.assertEqual(r.returncode, 1)
        self.assertIn("method not found", r.stderr)

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


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ec-query", type=Path, required=True)
    p.add_argument("--ec-monitor", type=Path, required=True)
    p.add_argument("--ec-record", type=Path, required=True)
    p.add_argument("--tools", type=Path, required=True)
    args, rest = p.parse_known_args()
    vars(PATHS).update(vars(args))
    unittest.main(argv=[sys.argv[0], *rest], verbosity=2)


if __name__ == "__main__":
    main()
