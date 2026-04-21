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

"""Velocity tracking PID test on real hardware.

Commands constant target velocities on one channel at a time, uses
PID feedback from odom to adjust the command, and records:
- Desired velocity (what the outer loop wants)
- Adjusted command (what PID sends to robot)
- Actual velocity (measured from odom)

Produces comparison plots: open-loop (no PID) vs closed-loop (with PID).

Usage:
    .venv/bin/python -m dimos.control.tuning.velocity_pid_test --channel vx --output vx_pid_test.csv
    .venv/bin/python -m dimos.control.tuning.velocity_pid_test --channel wz --output wz_pid_test.csv
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
import time

import numpy as np

from dimos.control.tasks.velocity_tracking_pid import (
    VelocityPIDConfig,
    VelocityTrackingConfig,
    VelocityTrackingPID,
)
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TEST_DURATION = 10.0  # seconds per velocity target
CONTROL_DT = 0.1  # 10 Hz
SETTLE_DURATION = 2.0  # seconds at zero before each test

CSV_FIELDS = [
    "timestamp",
    "channel",
    "target_vel",
    "mode",
    "trial",
    "desired",
    "adjusted_cmd",
    "actual_vel",
    "odom_x",
    "odom_y",
    "odom_yaw",
]

# Test velocities per channel
VX_TARGETS = [0.2, 0.4, 0.6]
VY_TARGETS = [0.2, 0.4, 0.6]
WZ_TARGETS = [0.3, 0.6, 0.9]


@dataclass
class OdomState:
    timestamp: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    prev_x: float = 0.0
    prev_y: float = 0.0
    prev_yaw: float = 0.0
    prev_ts: float = 0.0

    def body_velocities(self) -> tuple[float, float, float]:
        """Compute body-frame velocities from consecutive odom."""
        dt = self.timestamp - self.prev_ts
        if dt < 1e-6:
            return (0.0, 0.0, 0.0)

        dx = self.x - self.prev_x
        dy = self.y - self.prev_y
        dyaw = self.yaw - self.prev_yaw
        dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi

        mid_yaw = self.prev_yaw + dyaw / 2.0
        cos_y = math.cos(mid_yaw)
        sin_y = math.sin(mid_yaw)

        body_vx = (dx * cos_y + dy * sin_y) / dt
        body_vy = (-dx * sin_y + dy * cos_y) / dt
        body_wz = dyaw / dt

        return (body_vx, body_vy, body_wz)


class OdomTracker:
    """Tracks latest odom and computes body-frame velocity."""

    def __init__(self) -> None:
        self._state = OdomState()
        self._has_data = False

    def on_odom(self, msg: PoseStamped) -> None:
        now = time.perf_counter()
        self._state.prev_x = self._state.x
        self._state.prev_y = self._state.y
        self._state.prev_yaw = self._state.yaw
        self._state.prev_ts = self._state.timestamp

        self._state.timestamp = now
        self._state.x = msg.position.x
        self._state.y = msg.position.y
        self._state.yaw = msg.orientation.euler[2]
        self._has_data = True

    @property
    def ready(self) -> bool:
        return self._has_data and self._state.prev_ts > 0

    @property
    def state(self) -> OdomState:
        return self._state


def _send_twist(pub: LCMTransport, vx: float, vy: float, wz: float) -> None:
    pub.publish(
        Twist(
            linear=Vector3(x=vx, y=vy, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=wz),
        )
    )


def run_test(
    cmd_pub: LCMTransport,
    odom: OdomTracker,
    channel: str,
    target: float,
    mode: str,
    pid: VelocityTrackingPID | None,
    trial: int,
) -> list[dict]:
    """Run one velocity tracking test.

    mode = "open_loop" (no PID, direct command) or "closed_loop" (with PID)
    """
    rows: list[dict] = []

    # Settle at zero
    logger.info(f"  Settling at zero ({SETTLE_DURATION}s)...")
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < SETTLE_DURATION:
        _send_twist(cmd_pub, 0.0, 0.0, 0.0)
        time.sleep(CONTROL_DT)

    if pid:
        pid.reset()

    logger.info(f"  Running {mode} at {channel}={target} for {TEST_DURATION}s...")
    t0 = time.perf_counter()

    while time.perf_counter() - t0 < TEST_DURATION:
        # Get actual velocity from odom
        if odom.ready:
            actual_vx, actual_vy, actual_wz = odom.state.body_velocities()
        else:
            actual_vx, actual_vy, actual_wz = 0.0, 0.0, 0.0

        # Desired velocity
        des_vx = target if channel == "vx" else 0.0
        des_vy = target if channel == "vy" else 0.0
        des_wz = target if channel == "wz" else 0.0

        # Compute command
        if pid and mode == "closed_loop":
            cmd_vx, cmd_vy, cmd_wz = pid.compute(
                des_vx,
                des_vy,
                des_wz,
                actual_vx,
                actual_vy,
                actual_wz,
            )
        else:
            cmd_vx, cmd_vy, cmd_wz = des_vx, des_vy, des_wz

        _send_twist(cmd_pub, cmd_vx, cmd_vy, cmd_wz)

        # Get actual for the channel we care about
        actual = {"vx": actual_vx, "vy": actual_vy, "wz": actual_wz}[channel]

        rows.append(
            {
                "timestamp": time.perf_counter(),
                "channel": channel,
                "target_vel": target,
                "mode": mode,
                "trial": trial,
                "desired": target,
                "adjusted_cmd": {"vx": cmd_vx, "vy": cmd_vy, "wz": cmd_wz}[channel],
                "actual_vel": actual,
                "odom_x": odom.state.x,
                "odom_y": odom.state.y,
                "odom_yaw": odom.state.yaw,
            }
        )

        time.sleep(CONTROL_DT)

    # Stop
    _send_twist(cmd_pub, 0.0, 0.0, 0.0)
    return rows


def _filter_velocity(ts: np.ndarray, vel: np.ndarray, cutoff_hz: float = 2.0) -> np.ndarray:
    """Zero-phase low-pass filter on velocity signal."""
    from scipy.signal import butter, filtfilt

    if len(vel) < 12:
        return vel
    dt = np.diff(ts)
    dt[dt < 1e-6] = 1e-6
    fs = 1.0 / float(np.median(dt))
    nyq = fs / 2.0
    if cutoff_hz >= nyq:
        cutoff_hz = nyq * 0.8
    b, a = butter(2, cutoff_hz / nyq, btype="low")
    return filtfilt(b, a, vel)


def plot_results(csv_path: str, channel: str) -> None:
    """Plot open-loop vs closed-loop velocity tracking."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows_by_key: dict[tuple[float, str], list[dict]] = {}
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            if row["channel"] != channel:
                continue
            key = (float(row["target_vel"]), row["mode"])
            rows_by_key.setdefault(key, []).append(row)

    targets = sorted(set(k[0] for k in rows_by_key.keys()))
    unit = "rad/s" if channel == "wz" else "m/s"
    label = {"vx": "Forward velocity (vx)", "vy": "Lateral velocity (vy)", "wz": "Yaw rate (wz)"}[
        channel
    ]

    n = len(targets)
    fig, axes = plt.subplots(n, 1, figsize=(10, 4 * n), squeeze=False)

    for idx, target in enumerate(targets):
        ax = axes[idx][0]

        for mode, color, _ls in [("open_loop", "C0", "-"), ("closed_loop", "C1", "-")]:
            key = (target, mode)
            if key not in rows_by_key:
                continue
            data = rows_by_key[key]
            ts = np.array([float(r["timestamp"]) for r in data])
            ts_rel = ts - ts[0]
            actual_raw = np.array([float(r["actual_vel"]) for r in data])
            adjusted = np.array([float(r["adjusted_cmd"]) for r in data])

            # Zero-phase filter on actual velocity
            actual = _filter_velocity(ts, actual_raw, cutoff_hz=2.0)

            ax.plot(ts_rel, actual_raw, color=color, linewidth=0.4, alpha=0.25)
            ax.plot(ts_rel, actual, color=color, linewidth=1.5, label=f"{mode} actual (filtered)")
            if mode == "closed_loop":
                ax.plot(
                    ts_rel,
                    adjusted,
                    color="C2",
                    linewidth=0.8,
                    alpha=0.6,
                    linestyle="--",
                    label="PID adjusted cmd",
                )

        ax.axhline(
            y=target, color="red", linestyle=":", alpha=0.5, label=f"Target: {target} {unit}"
        )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"Velocity ({unit})")
        ax.set_title(f"{label} @ {target} {unit} — Open Loop vs PID")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    fig.suptitle(f"Velocity Tracking PID Test — {label}", fontsize=13, fontweight="bold")
    plt.tight_layout()
    out = csv_path.replace(".csv", "_plot.png")
    plt.savefig(out, dpi=150)
    print(f"Saved plot to {out}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Velocity tracking PID test")
    parser.add_argument("--channel", required=True, choices=["vx", "vy", "wz"])
    parser.add_argument("--output", default="velocity_pid_test.csv")
    parser.add_argument("--kp", type=float, default=0.5, help="Proportional gain")
    parser.add_argument("--ki", type=float, default=0.1, help="Integral gain")
    parser.add_argument("--kd", type=float, default=0.0, help="Derivative gain")
    parser.add_argument("--plot-only", action="store_true", help="Just plot existing CSV")
    args = parser.parse_args()

    if args.plot_only:
        plot_results(args.output, args.channel)
        return

    targets = {"vx": VX_TARGETS, "vy": VY_TARGETS, "wz": WZ_TARGETS}[args.channel]

    # Build PID config for the tested channel
    pid_cfg = VelocityPIDConfig(
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        output_min=-1.5,
        output_max=1.5,
    )
    tracking_cfg = VelocityTrackingConfig(dt=CONTROL_DT)
    if args.channel == "vx":
        tracking_cfg.vx = pid_cfg
    elif args.channel == "vy":
        tracking_cfg.vy = pid_cfg
    else:
        tracking_cfg.wz = pid_cfg

    pid = VelocityTrackingPID(tracking_cfg)

    cmd_pub = LCMTransport("/cmd_vel", Twist)
    odom_sub = LCMTransport("/go2/odom", PoseStamped)
    odom = OdomTracker()
    odom_unsub = odom_sub.subscribe(odom.on_odom)

    all_rows: list[dict] = []

    try:
        logger.info("Waiting 3s for odom...")
        time.sleep(3.0)

        if not odom.ready:
            logger.error("No odom — is the coordinator running?")
            return

        for target in targets:
            # Open loop (no PID)
            input(f"\n>>> Open loop: {args.channel}={target} — position robot, press ENTER...")
            logger.info(f"Open loop: {args.channel}={target}")
            rows = run_test(cmd_pub, odom, args.channel, target, "open_loop", None, trial=1)
            all_rows.extend(rows)

            # Closed loop (with PID)
            input(f">>> Closed loop PID: {args.channel}={target} — position robot, press ENTER...")
            logger.info(f"Closed loop: {args.channel}={target}")
            rows = run_test(cmd_pub, odom, args.channel, target, "closed_loop", pid, trial=1)
            all_rows.extend(rows)

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
    plot_results(args.output, args.channel)


if __name__ == "__main__":
    main()
