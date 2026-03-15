#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
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

"""Record robot position from /odom and plot the travel path on Ctrl+C."""

from __future__ import annotations

import argparse
import math
import signal
import time

import matplotlib.pyplot as plt
import numpy as np

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

MIN_DIST = 0.05  # minimum distance (m) between recorded points
STUCK_RADIUS = 0.6  # if robot stays within this radius (m) ...
STUCK_TIMEOUT = 60.0  # ... for this many seconds, stop recording


def main() -> None:
    parser = argparse.ArgumentParser(description="Record /odom and plot patrol path")
    parser.add_argument("--output", "-o", default="patrol_path.ignore.png")
    args = parser.parse_args()

    transport: LCMTransport[PoseStamped] = LCMTransport("/odom", PoseStamped)

    xs: list[float] = []
    ys: list[float] = []
    t_start: list[float] = []  # single-element list so closure can mutate
    stuck_anchor: list[float] = [0.0, 0.0]  # (x, y) center for stuck detection
    stuck_since: list[float] = [0.0]  # timestamp when robot entered current stuck zone
    stop = False

    def on_msg(msg: PoseStamped) -> None:
        nonlocal stop
        x, y = msg.position.x, msg.position.y

        # Record start time on first message.
        if not t_start:
            t_start.append(time.time())
            stuck_anchor[0], stuck_anchor[1] = x, y
            stuck_since[0] = time.time()

        # Only record if far enough from the last recorded point.
        if xs:
            dx = x - xs[-1]
            dy = y - ys[-1]
            if math.hypot(dx, dy) < MIN_DIST:
                return

        xs.append(x)
        ys.append(y)

        # Stuck detection: check if robot left the stuck circle.
        dist_from_anchor = math.hypot(x - stuck_anchor[0], y - stuck_anchor[1])
        if dist_from_anchor > STUCK_RADIUS:
            # Robot moved out — reset anchor to current position.
            stuck_anchor[0], stuck_anchor[1] = x, y
            stuck_since[0] = time.time()
        elif time.time() - stuck_since[0] > STUCK_TIMEOUT:
            print(
                f"\nRobot stuck within {STUCK_RADIUS}m radius for >{STUCK_TIMEOUT:.0f}s — stopping."
            )
            stop = True

    transport.start()
    transport.subscribe(on_msg)

    print("Listening on /odom ... recording positions. Press Ctrl+C to stop and plot.")

    def _handle_sigint(_sig: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _handle_sigint)

    while not stop:
        time.sleep(0.05)

    transport.stop()
    t_end = time.time()

    # Compute stats.
    elapsed = t_end - t_start[0]
    mins, secs = divmod(elapsed, 60)

    xs_arr = np.array(xs)
    ys_arr = np.array(ys)
    dists = np.hypot(np.diff(xs_arr), np.diff(ys_arr))
    total_dist = float(np.sum(dists))

    print(f"Recorded {len(xs)} points over {int(mins)}m{secs:.0f}s, {total_dist:.1f}m traveled.")

    fig, ax = plt.subplots(figsize=(10, 10))

    for i in range(len(xs_arr) - 1):
        ax.plot(xs_arr[i : i + 2], ys_arr[i : i + 2], color="blue", alpha=0.2, linewidth=2)

    ax.plot(xs_arr[0], ys_arr[0], "go", markersize=10, label="Start")
    ax.plot(xs_arr[-1], ys_arr[-1], "ro", markersize=10, label="End")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"Patrol Path — {int(mins)}m{secs:.0f}s, {total_dist:.1f}m traveled")
    ax.set_aspect("equal")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    print(f"Saved plot to {args.output}")


if __name__ == "__main__":
    main()
