#!/usr/bin/env python3
"""Read-only measurements on the controller, for the assumptions listed in
docs/motion.md section 7. Nothing here moves the arm or changes a setting: the
port-8055 checks send only the queries in QUERIES, and `drift` reads a
recording made by ec_record.

    python3 tools/robot_checks.py rtt --host 192.168.0.200          # round trip of one query
    python3 tools/robot_checks.py pipeline --host 192.168.0.200     # several requests in flight
    python3 tools/robot_checks.py connections --host 192.168.0.200  # two connections at once
    ./build/linux-release/apps/ec_record 192.168.0.200 --raw drift.ecraw --seconds 600
    python3 tools/robot_checks.py drift drift.ecraw                 # frame period, jitter, clock drift

Exit codes: 0 the check ran and printed what it found; 1 it could not run;
2 bad command line. Standard library only; Linux and Windows.
"""

from __future__ import annotations

import argparse
import json
import socket
import statistics
import struct
import sys
import time

QUERIES = ("getRobotState", "getRobotMode", "getSoftVersion")  # the only methods this tool sends
REPLY_TIMEOUT = 2.0  # s
CYCLE_MS = 8.0  # the planned control period (docs/motion.md)
LOOKAHEAD_MS = 40.0  # the planned default stream lookahead (docs/motion.md)

RAW_MAGIC = b"ECRAW001"  # docs/log-format.md
RAW_HEADER = 24  # magic, steady clock at start, system clock at start
RAW_RECORD = 8 + 1024  # arrival time (ns, little-endian), then the frame as received
FRAME_TIMESTAMP = 4  # offset of the controller's timestamp in a frame: uint64 ms, big-endian


class CheckFailed(Exception):
    """The check could not run: no connection, no reply, an unreadable file."""


class Connection:
    """A port-8055 connection that can send only the read-only QUERIES."""

    def __init__(self, host: str, port: int) -> None:
        try:
            self.sock = socket.create_connection((host, port), timeout=REPLY_TIMEOUT)
        except OSError as e:
            raise CheckFailed(f"connect to {host}:{port}: {e}") from e
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buffer = b""

    def close(self) -> None:
        self.sock.close()

    def send(self, *requests: tuple[str, int]) -> None:
        """Sends the (method, id) requests in one write."""
        lines = []
        for method, request_id in requests:
            if method not in QUERIES:
                raise ValueError(f"{method} is not one of the read-only queries")
            lines.append(json.dumps({"jsonrpc": "2.0", "method": method, "id": request_id}, separators=(",", ":")))
        try:
            self.sock.sendall("".join(line + "\n" for line in lines).encode())
        except OSError as e:
            raise CheckFailed(f"send: {e}") from e

    def receive(self, timeout: float = REPLY_TIMEOUT) -> dict | None:
        """The next line, parsed; None if no whole line arrives within `timeout`."""
        deadline = time.monotonic() + timeout
        while b"\n" not in self.buffer:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            self.sock.settimeout(left)
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                return None
            except OSError as e:
                raise CheckFailed(f"receive: {e}") from e
            if not chunk:
                raise CheckFailed("the controller closed the connection")
            self.buffer += chunk
        line, self.buffer = self.buffer.split(b"\n", 1)
        try:
            reply = json.loads(line)
        except ValueError:
            reply = None
        return reply if isinstance(reply, dict) else {"line": line.decode(errors="replace")}

    def call(self, method: str, request_id: int) -> dict:
        """Sends one query and returns its reply, skipping lines with another id."""
        self.send((method, request_id))
        while True:
            reply = self.receive()
            if reply is None:
                raise CheckFailed(f"no reply to {method} within {REPLY_TIMEOUT:g} s")
            if reply.get("id") == request_id:
                return reply


def result(reply: dict) -> str:
    """A reply's result as text; the controller wraps every result in a JSON string."""
    if "result" not in reply:
        return f"error {reply.get('error')}"
    value = reply["result"]
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except ValueError:
            return value
    return value if isinstance(value, str) else json.dumps(value)


def describe(conn: Connection) -> None:
    version = result(conn.call("getSoftVersion", 1))
    mode = result(conn.call("getRobotMode", 2))
    print(f"controller {version}, robot mode {mode} (0 teach, 1 play, 2 remote)")


def percentile(ordered: list[float], p: float) -> float:
    k = (len(ordered) - 1) * p / 100.0
    low = int(k)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (k - low)


def summary(values: list[float]) -> str:
    s = sorted(values)
    return (f"min {s[0]:.2f}, median {percentile(s, 50):.2f}, p95 {percentile(s, 95):.2f}, "
            f"p99 {percentile(s, 99):.2f}, max {s[-1]:.2f}")


def check_rtt(args: argparse.Namespace) -> None:
    """Round trip of getRobotState, one call per period like the planned loop."""
    conn = Connection(args.host, args.port)
    times = []
    try:
        describe(conn)
        period = args.period_ms / 1000.0
        deadline = time.monotonic()
        for i in range(args.count):
            if period > 0:
                deadline += period
                time.sleep(max(0.0, deadline - time.monotonic()))
            start = time.perf_counter_ns()
            conn.call("getRobotState", 100 + i)
            times.append((time.perf_counter_ns() - start) / 1e6)
    finally:
        conn.close()
    budget = CYCLE_MS / 2
    over = sum(t > budget for t in times)
    print(f"round trip of getRobotState, {len(times)} calls, one every {args.period_ms:g} ms [ms]:")
    print(f"  {summary(times)}")
    print(f"  {over} of {len(times)} ({100 * over / len(times):.1f} %) above {budget:g} ms, "
          f"half the planned {CYCLE_MS:g} ms cycle")


def check_pipeline(args: argparse.Namespace) -> None:
    """Several queries in one write: are they all answered, in order?"""
    conn = Connection(args.host, args.port)
    ids = list(range(1000, 1000 + args.count))
    got = []
    ended = ""
    try:
        describe(conn)
        start = time.perf_counter_ns()
        conn.send(*[("getRobotState", i) for i in ids])
        try:
            while len(got) < len(ids):
                reply = conn.receive()
                if reply is None:
                    ended = f"nothing more within {REPLY_TIMEOUT:g} s"
                    break
                got.append(reply.get("id"))
        except CheckFailed as e:  # here, the controller closing the connection is a finding
            ended = str(e)
        elapsed = (time.perf_counter_ns() - start) / 1e6
    finally:
        conn.close()
    n = len(ids)
    if got == ids:
        print(f"all {n} replies, in order, {elapsed:.1f} ms in total: "
              "several requests can be in flight on one connection")
    elif sorted(got) == ids:
        print(f"all {n} replies, out of order: several requests can be in flight, matched by id")
    else:
        print(f"{len(got)} of {n} replies ({ended}): send one request at a time")


def check_connections(args: argparse.Namespace) -> None:
    """Two connections at once, with requests in flight on both."""
    first = Connection(args.host, args.port)
    second = None
    crossed = 0
    try:
        describe(first)
        try:
            second = Connection(args.host, args.port)
        except CheckFailed as e:
            print(f"a second connection is refused while the first is open: {e}")
            return
        second.send(("getRobotState", 1))
        if second.receive() is None:
            print("a second connection is accepted but not answered while the first is open: "
                  "the controller serves one connection at a time")
            return
        for r in range(args.rounds):
            first.send(("getRobotState", 1000 + r))
            second.send(("getRobotMode", 2000 + r))
            on_second = second.receive()  # read in the opposite order
            on_first = first.receive()
            if on_first is None or on_second is None:
                raise CheckFailed(f"round {r + 1}: a reply went missing")
            crossed += on_first.get("id") != 1000 + r or on_second.get("id") != 2000 + r
        first.close()
        first = None
        second.call("getRobotState", 3000)
    finally:
        if first:
            first.close()
        if second:
            second.close()
    if crossed:
        print(f"two connections at once: in {crossed} of {args.rounds} rounds a reply arrived on the wrong one")
    else:
        print(f"two connections at once: both answered through {args.rounds} rounds with requests in flight "
              "on both, and the second kept working after the first closed")


def read_raw(path: str) -> tuple[list[int], list[int]]:
    """Arrival times (PC steady clock, ns) and controller timestamps (ms) of a raw recording."""
    try:
        f = open(path, "rb")
    except OSError as e:
        raise CheckFailed(str(e)) from e
    arrival, controller = [], []
    with f:
        header = f.read(RAW_HEADER)
        if len(header) < RAW_HEADER or header[:8] != RAW_MAGIC:
            raise CheckFailed(f"{path}: not a raw recording from ec_record")
        while len(record := f.read(RAW_RECORD)) == RAW_RECORD:
            arrival.append(struct.unpack_from("<q", record, 0)[0])
            controller.append(struct.unpack_from(">Q", record, 8 + FRAME_TIMESTAMP)[0])
    return arrival, controller


def check_drift(args: argparse.Namespace) -> None:
    """Frame period, arrival jitter, and the controller's clock rate against the PC's."""
    arrival, controller = read_raw(args.file)
    n = len(arrival)
    if n < 100:
        raise CheckFailed(f"{args.file}: {n} frames; record a few minutes")
    t = [(a - arrival[0]) / 1e9 for a in arrival]  # PC steady clock [s]
    c = [(x - controller[0]) / 1e3 for x in controller]  # controller clock [s]
    if t[-1] <= 0:
        raise CheckFailed(f"{args.file}: the arrival times do not advance")
    period = c[-1] / (n - 1) * 1e3  # ms, mean over the recording
    controller_steps = [(b - a) * 1e3 for a, b in zip(c, c[1:])]
    arrival_steps = [(b - a) * 1e3 for a, b in zip(t, t[1:])]
    gaps = sum(s > 1.5 * period for s in controller_steps)
    backwards = sum(s < 0 for s in controller_steps)

    # controller time = offset + rate * PC time, least squares
    mean_t, mean_c = statistics.fmean(t), statistics.fmean(c)
    sxx = sum((x - mean_t) ** 2 for x in t)
    sxy = sum((x - mean_t) * (y - mean_c) for x, y in zip(t, c))
    rate = sxy / sxx
    offset = mean_c - rate * mean_t
    skew_ppm = (rate - 1.0) * 1e6
    worst_ms = max(abs(y - (offset + rate * x)) for x, y in zip(t, c)) * 1e3

    print(f"{n} frames over {t[-1]:.1f} s of PC time, {(n - 1) / t[-1]:.2f} frames/s")
    print(f"frame period, controller clock: {period:.3f} ms on average (timestamps in whole ms); "
          f"{gaps} gaps longer than 1.5 periods"
          + (f"; {backwards} steps backwards (was the clock set?)" if backwards else ""))
    print(f"arrival intervals, PC clock [ms]: {summary(arrival_steps)}")
    print(f"controller clock against the PC clock: {skew_ppm:+.1f} ppm; "
          f"largest deviation from a straight line {worst_ms:.1f} ms")
    if abs(skew_ppm) >= 0.1:
        minutes = LOOKAHEAD_MS / 1e3 / (abs(skew_ppm) * 1e-6) / 60
        print(f"at this rate, a stream queue gains or loses {LOOKAHEAD_MS:g} ms (the planned lookahead) "
              f"in about {minutes:.0f} min of continuous streaming")


def positive(text: str) -> int:
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be at least 1")
    return value


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    checks = p.add_subparsers(dest="check", required=True, metavar="CHECK")

    def on_8055(name: str, text: str) -> argparse.ArgumentParser:
        s = checks.add_parser(name, help=text, description=text)
        s.add_argument("--host", required=True, help="controller address (no default)")
        s.add_argument("--port", type=int, default=8055)
        return s

    rtt = on_8055("rtt", "round trip of one read-only query, paced like the control loop")
    rtt.add_argument("--count", type=positive, default=500)
    rtt.add_argument("--period-ms", type=float, default=CYCLE_MS, help="time between calls; 0: back to back")
    pipeline = on_8055("pipeline", "several read-only queries in one write, then their replies")
    pipeline.add_argument("--count", type=positive, default=20)
    connections = on_8055("connections", "two connections at once, with requests in flight on both")
    connections.add_argument("--rounds", type=positive, default=20)
    drift = checks.add_parser("drift", help="frame period, arrival jitter and clock drift of an ec_record raw file")
    drift.add_argument("file")
    args = p.parse_args()

    run = {"rtt": check_rtt, "pipeline": check_pipeline, "connections": check_connections, "drift": check_drift}
    try:
        run[args.check](args)
    except CheckFailed as e:
        print(f"{args.check}: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"{args.check}: interrupted", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
