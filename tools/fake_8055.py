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

Standard library only; works on Linux and Windows.
"""

import argparse
import json
import socket
import time

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


def reply_for(request: dict) -> dict:
    rid = request.get("id")
    method = request.get("method")
    if method in RESULTS:
        return {"jsonrpc": "2.0", "result": RESULTS[method], "id": rid}
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
            send(conn, reply_for(request), args.split)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8055)
    p.add_argument("--delay-ms", type=int, default=0)
    p.add_argument("--stale", action="store_true")
    p.add_argument("--garbage", action="store_true")
    p.add_argument("--split", action="store_true")
    args = p.parse_args()

    with socket.create_server((args.host, args.port)) as srv:
        print(f"fake 8055 on {args.host}:{srv.getsockname()[1]}, Ctrl+C to quit", flush=True)
        while True:
            conn, addr = srv.accept()
            print(f"client {addr[0]}:{addr[1]} connected")
            with conn:
                try:
                    serve(conn, args)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
            print("client disconnected")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
