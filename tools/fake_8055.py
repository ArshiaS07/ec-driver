#!/usr/bin/env python3
"""Fake port-8055 server: answers JSON-RPC calls like the EA66 does.

For trying ec_query and the command client without the robot:

    python3 tools/fake_8055.py                     # listens on 127.0.0.1:8055
    ./build/linux-debug/apps/ec_query --host 127.0.0.1 getRobotMode

Replies are copied from the real controller (firmware V3.15.3), including its
habit of wrapping every result in a JSON string. Unknown methods get the real
"Method not found." error. Fault injection for testing the client:

    --delay-ms N   answer every call N ms late (exercises timeouts)
    --stale        send a reply with a wrong id before each real one
    --garbage      send a line that is not JSON-RPC before each real reply
    --split        send every reply in two TCP pieces
    --one-client   serve one connection at a time (default: each on its own thread)

poseMul and poseInv are computed, with a rotation convention chosen by
--convention (the matrix product order of rx, ry, rz, default ZYX) and
--product (12: poseMul is T1.T2, 21: T2.T1), so tools that probe the
convention can be tested.

Standard library only; works on Linux and Windows.
"""

import argparse
import json
import socket
import threading
import time

import poses

# method -> result exactly as the controller encodes it (a JSON string holding JSON text)
RESULTS = {
    "getRobotMode": "2",
    "getRobotState": "0",
    "getServoStatus": "false",
    "getMotorStatus": "true",
    "getSoftVersion": '"V3.15.3"',
    "get_joint_pos": "[2.0880681818181817,-90.538739669421489,0.12809405940595298,"
                     "-90.212962962962962,89.920910493827165,-0.785108024691358]",
    "getDH": "95.431",
    "stop": "true",
    "moveByJoint": "true",
}


def computed(method: str, params: dict, args: argparse.Namespace):
    """The result of a pose computation, or None if `method` is not one."""
    if method not in ("poseMul", "poseInv"):
        return None
    degrees = params.get("unit_type", 1) == 0  # the manual's default is radians
    to_internal = poses.to_radians if degrees else (lambda p: p)
    to_external = poses.to_degrees if degrees else (lambda p: p)
    if method == "poseInv":
        result = poses.invert(args.convention, to_internal(params["pose"]))
    else:
        p1, p2 = to_internal(params["pose1"]), to_internal(params["pose2"])
        if args.product == "21":
            p1, p2 = p2, p1
        result = poses.compose(args.convention, p1, p2)
    return json.dumps(to_external(result))


def reply_for(request: dict, args: argparse.Namespace) -> dict:
    rid = request.get("id")
    method = request.get("method")
    result = computed(method, request.get("params") or {}, args)
    if result is None and method in RESULTS:
        result = RESULTS[method]
    if result is not None:
        return {"jsonrpc": "2.0", "result": result, "id": rid}
    return {"jsonrpc": "2.0", "error": {"code": -32601, "message": "Method not found."}, "id": rid}


def send(conn: socket.socket, message: dict, split: bool) -> None:
    data = (json.dumps(message, separators=(",", ":")) + "\n").encode()
    if split and len(data) > 2:
        half = len(data) // 2
        conn.sendall(data[:half])
        time.sleep(0.005)
        conn.sendall(data[half:])
    else:
        conn.sendall(data)


def serve(conn: socket.socket, args: argparse.Namespace) -> None:
    buffer = b""
    while True:
        chunk = conn.recv(4096)
        if not chunk:
            return
        buffer += chunk
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            try:
                request = json.loads(line)
            except json.JSONDecodeError:
                print(f"  not JSON: {line!r}")
                continue
            print(f"  <- {request}")
            if args.delay_ms:
                time.sleep(args.delay_ms / 1000.0)
            if args.garbage:
                conn.sendall(b"this is not a JSON-RPC response\n")
            if args.stale:
                send(conn, {"jsonrpc": "2.0", "result": "0", "id": 999999}, args.split)
            send(conn, reply_for(request, args), args.split)


def handle(conn: socket.socket, addr, args: argparse.Namespace) -> None:
    print(f"client {addr[0]}:{addr[1]} connected", flush=True)
    with conn:
        try:
            serve(conn, args)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
    print(f"client {addr[0]}:{addr[1]} disconnected", flush=True)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8055)
    p.add_argument("--delay-ms", type=int, default=0)
    p.add_argument("--stale", action="store_true")
    p.add_argument("--garbage", action="store_true")
    p.add_argument("--split", action="store_true")
    p.add_argument("--one-client", action="store_true")
    p.add_argument("--convention", choices=poses.ORDERS, default="ZYX")
    p.add_argument("--product", choices=("12", "21"), default="12")
    args = p.parse_args()

    with socket.create_server((args.host, args.port)) as srv:
        print(f"fake 8055 on {args.host}:{srv.getsockname()[1]}, Ctrl+C to quit", flush=True)
        while True:
            conn, addr = srv.accept()
            if args.one_client:
                handle(conn, addr, args)
            else:
                threading.Thread(target=handle, args=(conn, addr, args), daemon=True).start()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
