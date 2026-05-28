#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Minimal Go2 motion script: stand up, walk forward ~2 steps, lie down.

No perception, no blueprint framework. Works against real robot (WebRTC),
MuJoCo simulator, or replay dataset, via the same code path.

Run:
    # Real robot
    .venv/bin/python examples/go2_walk_forward.py --ip 192.168.12.1

    # MuJoCo simulator (macOS: requires .venv/bin on PATH for mjpython;
    # script auto-adds it)
    .venv/bin/python examples/go2_walk_forward.py --ip mujoco

    # Replay (no-op moves, just for code-path sanity)
    .venv/bin/python examples/go2_walk_forward.py --ip replay

    # Options: --distance 1.0 --speed 0.3
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.go2.connection import make_connection


def _ensure_venv_on_path() -> None:
    """MuJoCo spawns a subprocess that needs `mjpython` on PATH. When the user
    invokes us as `.venv/bin/python …` instead of activating the venv, the venv
    bin dir isn't on PATH. Add it so the subprocess can find mjpython."""
    venv_bin = Path(sys.executable).parent
    path_parts = os.environ.get("PATH", "").split(os.pathsep)
    if str(venv_bin) not in path_parts:
        os.environ["PATH"] = str(venv_bin) + os.pathsep + os.environ.get("PATH", "")


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2: stand up and walk forward")
    parser.add_argument("--ip", required=True, help="Robot IP, e.g. 192.168.12.1")
    parser.add_argument(
        "--distance", type=float, default=1.0, help="Meters to walk (default: 1.0 = ~2 steps)"
    )
    parser.add_argument(
        "--speed", type=float, default=0.3, help="Linear speed in m/s (default: 0.3)"
    )
    args = parser.parse_args()

    duration = args.distance / args.speed

    if args.ip in ("mujoco", "replay", "fake", "mock"):
        _ensure_venv_on_path()

    print(f"Connecting to {args.ip}...")
    # make_connection routes: 'mujoco' -> MujocoConnection, 'replay'/'fake'/'mock' ->
    # ReplayConnection, anything else -> UnitreeWebRTCConnection(ip)
    conn = make_connection(args.ip, global_config)
    conn.start()

    try:
        print("Standing up...")
        conn.standup()
        time.sleep(3)

        print("Entering balance stand mode...")
        conn.balance_stand()
        time.sleep(1)

        print(f"Walking forward {args.distance:.2f}m at {args.speed:.2f} m/s ({duration:.1f}s)...")
        twist = Twist(linear=Vector3(args.speed, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        conn.move(twist, duration=duration)

        # Belt-and-suspenders: explicitly send a zero twist.
        conn.move(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
        time.sleep(0.5)

        print("Lying down...")
        conn.liedown()
        time.sleep(2)

        print("Done.")
    except KeyboardInterrupt:
        print("\nInterrupted — stopping robot.")
        conn.move(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
    finally:
        conn.stop()


if __name__ == "__main__":
    main()
