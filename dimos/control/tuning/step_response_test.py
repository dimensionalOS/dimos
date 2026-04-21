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

"""Step-response test for Go2 plant characterisation.

Records RAW odom (x, y, yaw) with timestamps, trial IDs, and phase markers.
Velocity is computed in post-processing, not during collection.

Each row in the CSV has:
    timestamp, channel, amplitude, trial, phase, cmd_vx, cmd_vy, cmd_wz, odom_x, odom_y, odom_yaw

phase = "baseline" | "step" | "decay"

Usage:
    .venv/bin/python -m dimos.control.tuning.step_response_test --output step_data.csv
    .venv/bin/python -m dimos.control.tuning.step_response_test --channels vx --output vx_only.csv
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import time

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Default test amplitudes (both directions for asymmetry detection)
LINEAR_AMPLITUDES = [0.2, 0.4, 0.6, -0.2, -0.4, -0.6]  # m/s
ANGULAR_AMPLITUDES = [0.3, 0.6, 0.9, -0.3, -0.6, -0.9]  # rad/s
TRIALS_PER_AMPLITUDE = 3

BASELINE_DURATION = 2.0  # seconds
STEP_DURATION = 10.0  # seconds
DECAY_DURATION = 3.0  # seconds
SAMPLE_RATE = 0.02  # 50 Hz polling (odom arrives ~10-30Hz via WebRTC)

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


@dataclass
class OdomSample:
    timestamp: float
    x: float
    y: float
    yaw: float


@dataclass
class StepResponseRecorder:
    """Records raw odom samples."""

    _latest: OdomSample | None = None
    _prev: OdomSample | None = None

    def on_odom(self, msg: PoseStamped) -> None:
        self._prev = self._latest
        self._latest = OdomSample(
            timestamp=time.perf_counter(),
            x=msg.position.x,
            y=msg.position.y,
            yaw=msg.orientation.euler[2],
        )

    @property
    def latest(self) -> OdomSample | None:
        return self._latest

    @property
    def has_new_sample(self) -> bool:
        """True if latest differs from prev (new odom arrived)."""
        if self._latest is None:
            return False
        if self._prev is None:
            return True
        return self._latest.timestamp != self._prev.timestamp


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


def run_single_trial(
    cmd_pub: LCMTransport,
    recorder: StepResponseRecorder,
    channel: str,
    amplitude: float,
    trial: int,
) -> list[dict]:
    """Run one step-response trial. Records raw odom with phase markers."""
    rows: list[dict] = []
    cmd_vx, cmd_vy, cmd_wz = _cmd_for_channel(channel, amplitude)

    def _record(phase: str, cvx: float, cvy: float, cwz: float) -> None:
        s = recorder.latest
        if s is None:
            return
        rows.append(
            {
                "timestamp": s.timestamp,
                "channel": channel,
                "amplitude": amplitude,
                "trial": trial,
                "phase": phase,
                "cmd_vx": cvx,
                "cmd_vy": cvy,
                "cmd_wz": cwz,
                "odom_x": s.x,
                "odom_y": s.y,
                "odom_yaw": s.yaw,
            }
        )

    # Phase 1: baseline (zero command)
    logger.info(f"  Baseline ({BASELINE_DURATION}s)...")
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < BASELINE_DURATION:
        _send_twist(cmd_pub, 0.0, 0.0, 0.0)
        _record("baseline", 0.0, 0.0, 0.0)
        time.sleep(SAMPLE_RATE)

    # Phase 2: step command
    logger.info(f"  Step cmd=({cmd_vx}, {cmd_vy}, {cmd_wz}) for {STEP_DURATION}s...")
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < STEP_DURATION:
        _send_twist(cmd_pub, cmd_vx, cmd_vy, cmd_wz)
        _record("step", cmd_vx, cmd_vy, cmd_wz)
        time.sleep(SAMPLE_RATE)

    # Phase 3: decay (zero command)
    logger.info(f"  Decay ({DECAY_DURATION}s)...")
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < DECAY_DURATION:
        _send_twist(cmd_pub, 0.0, 0.0, 0.0)
        _record("decay", 0.0, 0.0, 0.0)
        time.sleep(SAMPLE_RATE)

    logger.info(f"  Recorded {len(rows)} samples")
    return rows


def _wait_for_ready(channel: str, amp: float, trial: int, total_trials: int) -> None:
    _DIR = {
        ("vx", True): "FORWARD",
        ("vx", False): "BACKWARD",
        ("vy", True): "LEFT",
        ("vy", False): "RIGHT",
        ("wz", True): "ROTATE CCW",
        ("wz", False): "ROTATE CW",
    }
    direction = _DIR.get((channel, amp > 0), channel)
    unit = "rad/s" if channel == "wz" else "m/s"
    print(f"\n>>> Next: {direction} at {abs(amp)} {unit} (trial {trial + 1}/{total_trials})")
    input("    Walk robot back to start position, then press ENTER... ")


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2 step-response plant characterisation")
    parser.add_argument("--output", default="step_response_data.csv", help="Output CSV path")
    parser.add_argument("--trials", type=int, default=TRIALS_PER_AMPLITUDE)
    parser.add_argument(
        "--channels",
        nargs="+",
        default=["vx", "vy", "wz"],
        help="Channels to test (default: vx vy wz)",
    )
    parser.add_argument("--no-pause", action="store_true")
    args = parser.parse_args()

    cmd_pub = LCMTransport("/cmd_vel", Twist)
    odom_sub = LCMTransport("/go2/odom", PoseStamped)
    recorder = StepResponseRecorder()
    odom_unsub = odom_sub.subscribe(recorder.on_odom)

    all_rows: list[dict] = []

    try:
        logger.info("Waiting 3s for odom stream to stabilise...")
        time.sleep(3.0)

        if recorder.latest is None:
            logger.error("No odom received — is the Go2 coordinator running?")
            return

        for channel in args.channels:
            amps = LINEAR_AMPLITUDES if channel in ("vx", "vy") else ANGULAR_AMPLITUDES
            for amp in amps:
                for trial in range(args.trials):
                    if not args.no_pause:
                        _wait_for_ready(channel, amp, trial, args.trials)
                    logger.info(
                        f"Channel={channel}, amplitude={amp}, trial={trial + 1}/{args.trials}"
                    )
                    rows = run_single_trial(cmd_pub, recorder, channel, amp, trial + 1)
                    all_rows.extend(rows)
                    time.sleep(0.5)

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


if __name__ == "__main__":
    main()
