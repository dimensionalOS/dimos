#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Minimal Go2 motion script: stand up, walk BACKWARD ~2 meters, lie down.

No perception, no blueprint framework — just a direct WebRTC connection.

⚠️  Walking backward is blind (no rear camera/LiDAR coverage). Make sure
    the path behind the robot is clear. Keep the remote in hand for E-stop.

Run:
    .venv/bin/python examples/go2_walk_backward.py --ip 192.168.12.1
    # Options: --distance 2.0 --speed 0.3
"""

from __future__ import annotations

import argparse
import time

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.connection import UnitreeWebRTCConnection


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2: stand up and walk backward")
    parser.add_argument("--ip", required=True, help="Robot IP, e.g. 192.168.12.1")
    parser.add_argument(
        "--distance",
        type=float,
        default=2.0,
        help="Meters to walk backward (default: 2.0)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.3,
        help="Linear speed magnitude in m/s (default: 0.3)",
    )
    args = parser.parse_args()

    # Both expected positive; we send -speed on x for backward motion.
    if args.distance <= 0 or args.speed <= 0:
        parser.error("--distance and --speed must be positive (direction is hard-coded backward)")

    duration = args.distance / args.speed

    print(f"Connecting to {args.ip}...")
    conn = UnitreeWebRTCConnection(ip=args.ip)

    try:
        print("Standing up...")
        conn.standup()
        time.sleep(3)

        print("Entering balance stand mode...")
        conn.balance_stand()
        time.sleep(1)

        print(
            f"Walking BACKWARD {args.distance:.2f}m at {args.speed:.2f} m/s ({duration:.1f}s)..."
        )
        twist = Twist(linear=Vector3(-args.speed, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
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
