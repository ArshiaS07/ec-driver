#!/usr/bin/env python3
"""Fake port-8056 server: streams valid monitoring frames every 8 ms.

For trying ec_monitor (and later the driver) without the robot:

    python3 tools/fake_8056.py                 # listens on 127.0.0.1:8056
    ./build/linux-debug/apps/ec_monitor 127.0.0.1

Joint positions move slowly so the output visibly changes. --chop splits every
frame at a random byte and --garbage sends junk before the first frame, to
exercise the client's framing. Standard library only; works on Linux and
Windows.
"""

import argparse
import math
import random
import socket
import struct
import time

FRAME_SIZE = 1024
MATCHING_WORD = 0xEC8056EC


def build_frame(t_ms: int, k: int) -> bytes:
    q = [20.0 * math.sin(0.01 * k + j) for j in range(8)]  # deg
    b = struct.pack(">IQB", FRAME_SIZE, t_ms, 2)             # size, timestamp, cycle mode
    b += struct.pack(">8d", *q)                              # machinePos
    b += struct.pack(">6d", 400.0, 0.0, 300.0, 3.14, 0.0, 1.57) * 2  # TCP base / user
    b += struct.pack(">8d", *[1.0] * 8)                      # torque per-mille
    b += struct.pack(">iii", 0, 1, 1)                        # robotState, servoReady, can_motor_run
    b += struct.pack(">8i", *[0] * 8)                        # motor_speed
    b += struct.pack(">i", 2)                                # robotMode: remote
    b += struct.pack(">3d", 0.0, 0.0, 0.0)                   # analog in
    b += struct.pack(">5d", *[0.0] * 5)                      # analog out
    b += struct.pack(">QQ", 0b101, 0)                        # DI, DO
    b += b"\x00"                                             # collision
    b += struct.pack(">6d", 400.0, 0.0, 250.0, 3.14, 0.0, 1.57) * 2  # flange base / user
    b += b"\x00"                                             # emergency stop
    b += struct.pack(">d", 0.0)                              # tcpSpeed
    b += struct.pack(">8d", *[0.0] * 8)                      # jointSpeed
    b += struct.pack(">d", 0.0)                              # tcpAcc
    b += struct.pack(">8d", *[0.0] * 8)                      # jointAcc
    b += struct.pack(">6d", *[35.0] * 6)                     # jointTemperature
    b += struct.pack(">6d", *[0.0] * 6)                      # jointTorque
    b += struct.pack(">6d", *[0.0] * 6)                      # extJointTorques
    b += struct.pack(">6d", *[0.0] * 6)                      # exTcpForceInTool
    b += b"\x00\x00"                                         # dragState, sensor state
    assert len(b) == 801, len(b)
    return b + bytes(219) + struct.pack(">I", MATCHING_WORD)


def serve(conn: socket.socket, period: float, chop: bool, garbage: bool) -> None:
    if garbage:
        conn.sendall(bytes(random.randrange(256) for _ in range(300)))
    k = 0
    next_t = time.monotonic()
    while True:
        frame = build_frame(int(time.time() * 1000), k)
        if chop:
            cut = random.randint(1, FRAME_SIZE - 1)
            conn.sendall(frame[:cut])
            conn.sendall(frame[cut:])
        else:
            conn.sendall(frame)
        k += 1
        next_t += period
        time.sleep(max(0.0, next_t - time.monotonic()))


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8056)
    p.add_argument("--period-ms", type=float, default=8.0)
    p.add_argument("--chop", action="store_true", help="split every frame at a random byte")
    p.add_argument("--garbage", action="store_true", help="send junk bytes before the first frame")
    args = p.parse_args()

    with socket.create_server((args.host, args.port)) as srv:
        print(f"fake 8056 on {args.host}:{args.port}, Ctrl+C to quit")
        while True:
            conn, addr = srv.accept()
            print(f"client {addr[0]}:{addr[1]} connected")
            with conn:
                try:
                    serve(conn, args.period_ms / 1000.0, args.chop, args.garbage)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    print("client disconnected")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
