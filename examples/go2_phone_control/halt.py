#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Standalone E-STOP for the Go2 — does NOT depend on the running server.

Use this when:
- The phone-control server is unresponsive / crashed.
- The follow loop is still driving and you can't get to the UI.
- Anything is going wrong and you just want the dog to stop NOW.

Faster paths (try them first if available):
1. The physical Unitree remote — L2+B (damp) is the most reliable kill.
2. `curl -X POST http://localhost:8800/api/halt`  (if server is alive; ~50ms)

This script is the deepest fallback: it opens a fresh WebRTC connection
direct to the dog, spams zero-twist, then commands liedown. Takes ~3-5s due
to WebRTC handshake.

Run:
    .venv/bin/python examples/go2_phone_control/halt.py --ip 192.168.12.1 --ap
"""

from __future__ import annotations

import argparse
import time

from unitree_webrtc_connect.constants import WebRTCConnectionMethod

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.connection import UnitreeWebRTCConnection


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2 standalone E-STOP")
    parser.add_argument("--ip", default="192.168.12.1", help="Robot IP")
    parser.add_argument(
        "--ap",
        action="store_true",
        help="Robot is on its own hotspot (LocalAP). Matches `server.py --ap`.",
    )
    parser.add_argument(
        "--mode",
        default="normal",
        help="Motion mode to switch to before halting (default 'normal')",
    )
    args = parser.parse_args()

    method = WebRTCConnectionMethod.LocalAP if args.ap else WebRTCConnectionMethod.LocalSTA
    print(f"[halt] opening fresh WebRTC to {args.ip} ({method.name}) ...")
    conn = UnitreeWebRTCConnection(ip=args.ip, mode=args.mode, connection_method=method)
    print("[halt] connected, sending stop sequence")

    zero = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
    try:
        for i in range(5):  # belt-and-suspenders
            conn.move(zero)
            time.sleep(0.05)
        print("[halt] sent 5x zero-twist; calling liedown ...")
        conn.liedown()
        time.sleep(2.0)
        print("[halt] done.")
    finally:
        try:
            conn.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
