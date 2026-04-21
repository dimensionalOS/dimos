#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Velocity sweep test with emergency stop and distance limiting.

Collects step response data across a range of velocities (0.2 to 2.0 m/s)
with 10 trials per velocity. Safety features:
- Press 'q' at any time to emergency stop the current trial
- Auto-stops when robot travels beyond --max-distance (default 3m)
- Pauses between trials for repositioning

Does NOT require keyboard teleop running — this script is the sole
velocity commander.

Usage:
    .venv/bin/python -m dimos.control.tuning.velocity_sweep_test \\
        --channel vx --output vx_sweep.csv --max-distance 3.0

    .venv/bin/python -m dimos.control.tuning.velocity_sweep_test \\
        --channel wz --output wz_sweep.csv
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
import select
import sys
import termios
import time
import tty

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TRIALS_PER_VELOCITY = 10
STEP_DURATION = 10.0  # max seconds per trial (may stop early on distance)
SETTLE_DURATION = 1.5  # seconds at zero between trials
SAMPLE_RATE = 0.02  # 50 Hz

CSV_FIELDS = [
    "timestamp",
    "channel",
    "amplitude",
    "trial",
    "phase",
    "cmd_vx",
    "cmd_vy",
    "cmd_wz",
    "odom_x",
    "odom_y",
    "odom_yaw",
]

# Velocity targets
VX_VELOCITIES = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0]
VY_VELOCITIES = [0.2, 0.4, 0.6, 0.8]
WZ_VELOCITIES = [0.3, 0.6, 0.9, 1.2, 1.5, 2.0]


@dataclass
class OdomSnapshot:
    timestamp: float
    x: float
    y: float
    yaw: float


class OdomTracker:
    def __init__(self) -> None:
        self._latest: OdomSnapshot | None = None

    def on_odom(self, msg: PoseStamped) -> None:
        self._latest = OdomSnapshot(
            timestamp=time.perf_counter(),
            x=msg.position.x,
            y=msg.position.y,
            yaw=msg.orientation.euler[2],
        )

    @property
    def latest(self) -> OdomSnapshot | None:
        return self._latest


class NonBlockingKeyReader:
    """Reads keypresses without blocking, no pygame needed."""

    def __init__(self) -> None:
        self._old_settings = None

    def __enter__(self) -> NonBlockingKeyReader:
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *args: object) -> None:
        if self._old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    def get_key(self) -> str | None:
        """Return key if pressed, None otherwise. Non-blocking."""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


def _send_twist(pub: LCMTransport, vx: float, vy: float, wz: float) -> None:
    pub.publish(
        Twist(
            linear=Vector3(x=vx, y=vy, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=wz),
        )
    )


def _cmd_for_channel(channel: str, amplitude: float) -> tuple[float, float, float]:
    if channel == "vx":
        return (amplitude, 0.0, 0.0)
    elif channel == "vy":
        return (0.0, amplitude, 0.0)
    else:
        return (0.0, 0.0, amplitude)


def _distance(a: OdomSnapshot, b: OdomSnapshot) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def run_trial(
    cmd_pub: LCMTransport,
    odom: OdomTracker,
    keys: NonBlockingKeyReader,
    channel: str,
    amplitude: float,
    trial: int,
    max_distance: float,
) -> tuple[list[dict], str]:
    """Run one trial. Returns (rows, stop_reason).

    stop_reason: "completed" | "distance_limit" | "emergency_stop"
    """
    rows: list[dict] = []
    cmd_vx, cmd_vy, cmd_wz = _cmd_for_channel(channel, amplitude)

    start_odom = odom.latest
    stop_reason = "completed"

    t0 = time.perf_counter()
    while time.perf_counter() - t0 < STEP_DURATION:
        # Emergency stop check
        key = keys.get_key()
        if key == "q" or key == "Q":
            _send_twist(cmd_pub, 0.0, 0.0, 0.0)
            stop_reason = "emergency_stop"
            break

        # Distance limit check
        current = odom.latest
        if current and start_odom and channel in ("vx", "vy"):
            dist = _distance(start_odom, current)
            if dist >= max_distance:
                _send_twist(cmd_pub, 0.0, 0.0, 0.0)
                stop_reason = "distance_limit"
                break

        # Send command
        _send_twist(cmd_pub, cmd_vx, cmd_vy, cmd_wz)

        # Record
        s = odom.latest
        if s is not None:
            rows.append(
                {
                    "timestamp": s.timestamp,
                    "channel": channel,
                    "amplitude": amplitude,
                    "trial": trial,
                    "phase": "step",
                    "cmd_vx": cmd_vx,
                    "cmd_vy": cmd_vy,
                    "cmd_wz": cmd_wz,
                    "odom_x": s.x,
                    "odom_y": s.y,
                    "odom_yaw": s.yaw,
                }
            )

        time.sleep(SAMPLE_RATE)

    # Always stop the robot
    _send_twist(cmd_pub, 0.0, 0.0, 0.0)

    # Record decay for 1.5s
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < SETTLE_DURATION:
        _send_twist(cmd_pub, 0.0, 0.0, 0.0)
        s = odom.latest
        if s is not None:
            rows.append(
                {
                    "timestamp": s.timestamp,
                    "channel": channel,
                    "amplitude": amplitude,
                    "trial": trial,
                    "phase": "decay",
                    "cmd_vx": 0.0,
                    "cmd_vy": 0.0,
                    "cmd_wz": 0.0,
                    "odom_x": s.x,
                    "odom_y": s.y,
                    "odom_yaw": s.yaw,
                }
            )
        time.sleep(SAMPLE_RATE)

    return rows, stop_reason


def _walk_back_mode(cmd_pub: LCMTransport, keys: NonBlockingKeyReader) -> None:
    """Manual repositioning: WASD to move, ENTER when done.

    W/S: forward/backward
    A/D: rotate left/right
    Q/E: strafe left/right
    ENTER: done, start next trial
    """
    print("    [REPOSITION] WASD to move, Q/E strafe, ENTER when ready")
    speed = 0.3
    turn = 0.5

    _KEY_MAP = {
        "w": (speed, 0.0, 0.0),
        "s": (-speed, 0.0, 0.0),
        "a": (0.0, 0.0, turn),
        "d": (0.0, 0.0, -turn),
        "q": (0.0, speed, 0.0),
        "e": (0.0, -speed, 0.0),
    }

    try:
        while True:
            key = keys.get_key()
            if key == "\n" or key == "\r":
                _send_twist(cmd_pub, 0.0, 0.0, 0.0)
                break
            elif key and key.lower() in _KEY_MAP:
                vx, vy, wz = _KEY_MAP[key.lower()]
                _send_twist(cmd_pub, vx, vy, wz)
            else:
                _send_twist(cmd_pub, 0.0, 0.0, 0.0)
            time.sleep(0.05)
    except Exception:
        _send_twist(cmd_pub, 0.0, 0.0, 0.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Velocity sweep with emergency stop")
    parser.add_argument("--channel", required=True, choices=["vx", "vy", "wz"])
    parser.add_argument("--output", default="velocity_sweep.csv")
    parser.add_argument("--trials", type=int, default=TRIALS_PER_VELOCITY)
    parser.add_argument(
        "--max-distance",
        type=float,
        default=3.0,
        help="Max distance (m) before auto-stop per trial",
    )
    parser.add_argument(
        "--velocities",
        nargs="+",
        type=float,
        default=None,
        help="Custom velocity list (overrides defaults)",
    )
    args = parser.parse_args()

    if args.velocities:
        velocities = args.velocities
    else:
        velocities = {"vx": VX_VELOCITIES, "vy": VY_VELOCITIES, "wz": WZ_VELOCITIES}[args.channel]

    cmd_pub = LCMTransport("/cmd_vel", Twist)
    odom_sub = LCMTransport("/go2/odom", PoseStamped)
    odom = OdomTracker()
    odom_unsub = odom_sub.subscribe(odom.on_odom)

    all_rows: list[dict] = []
    unit = "rad/s" if args.channel == "wz" else "m/s"

    try:
        logger.info("Waiting 3s for odom...")
        time.sleep(3.0)
        if odom.latest is None:
            logger.error("No odom — is the coordinator running?")
            return

        print("\n" + "=" * 50)
        print("  VELOCITY SWEEP TEST")
        print(f"  Channel: {args.channel}")
        print(f"  Velocities: {velocities} {unit}")
        print(f"  Trials per velocity: {args.trials}")
        print(f"  Max distance: {args.max_distance}m")
        print("  Press 'q' during any trial to EMERGENCY STOP")
        print("=" * 50)

        with NonBlockingKeyReader() as keys:
            for vel in velocities:
                for trial in range(1, args.trials + 1):
                    print(f"\n>>> {args.channel}={vel} {unit} (trial {trial}/{args.trials})")
                    _walk_back_mode(cmd_pub, keys)

                    logger.info(f"Running {args.channel}={vel} trial {trial}...")
                    rows, reason = run_trial(
                        cmd_pub,
                        odom,
                        keys,
                        args.channel,
                        vel,
                        trial,
                        args.max_distance,
                    )
                    all_rows.extend(rows)

                    elapsed = len([r for r in rows if r["phase"] == "step"]) * SAMPLE_RATE
                    print(f"    {reason} ({elapsed:.1f}s, {len(rows)} samples)")

                    if reason == "emergency_stop":
                        print("    Robot stopped. Reposition and continue.")

        _send_twist(cmd_pub, 0.0, 0.0, 0.0)

    finally:
        odom_unsub()
        cmd_pub.stop()
        odom_sub.stop()

    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(all_rows)

    logger.info(f"Saved {len(all_rows)} samples to {args.output}")
    print(f"\nDone! Saved to {args.output}")
    print(
        f"Plot with: .venv/bin/python -m dimos.control.tuning.plant_identification --data {args.output}"
    )


if __name__ == "__main__":
    main()
