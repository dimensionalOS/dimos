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

"""Headless verification for the babylon-smoketest pimsim sim.

Assumes ``dimos run babylon-smoketest --daemon`` is already running. Spawns a
headless browser so the in-browser Havok physics ticks, then drives the box
via ``/nav_cmd_vel`` and checks that the base obeys the command and the lidar
streams non-empty clouds:

- yaw test: command an angular rate, confirm ``/odom`` yaw advances (this is
  collision-free, so it isolates command integration from obstacles).
- translation sweep: from the origin, drive +x/-x/+y/-y in turn (respawning
  between each) and confirm at least one open direction travels a clear
  distance. Directions that stall early are the office walls/furniture, which
  the lidar independently confirms are there.

Run: ``.venv/bin/python -m dimos.experimental.pimsim.demo_verify_pimsim``
"""

from __future__ import annotations

import math
import os
import sys
import time

from dimos.core.transport import LCMTransport
from dimos.experimental.pimsim.client import PimSimClient
from dimos.experimental.pimsim.headless import HeadlessBrowser
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# The headless browser renders on CPU swiftshader, so the in-browser physics
# loop runs slower than wall-clock; drive long enough for clear motion margins.
SETTLE_SECONDS = 3.0
DRIVE_SECONDS = 6.0
COMMAND_PERIOD = 0.1
FORWARD_SPEED = 0.6
TURN_RATE = 0.6
MIN_TRAVEL_METERS = 0.5
MIN_YAW_CHANGE_RAD = 0.5
MIN_LIDAR_FRAMES = 5


def _point_count(cloud: PointCloud2) -> int:
    try:
        return int(cloud.points_f32().shape[0])
    except Exception:
        return 0


class _State:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_count = 0
        self.lidar_counts: list[int] = []

    def on_odom(self, msg: PoseStamped) -> None:
        self.x, self.y, self.yaw = float(msg.x), float(msg.y), float(msg.yaw)
        self.odom_count += 1

    def on_lidar(self, msg: PointCloud2) -> None:
        self.lidar_counts.append(_point_count(msg))


def _drive(command: LCMTransport[Twist], twist: Twist, seconds: float) -> None:
    deadline = time.time() + seconds
    while time.time() < deadline:
        command.publish(twist)
        time.sleep(COMMAND_PERIOD)
    command.publish(Twist())
    time.sleep(0.6)


def main() -> int:
    state = _State()
    command = LCMTransport("/nav_cmd_vel", Twist)
    command.start()
    odom = LCMTransport("/odom", PoseStamped)
    odom.subscribe(state.on_odom)
    lidar = LCMTransport("/lidar", PointCloud2)
    lidar.subscribe(state.on_lidar)

    client = PimSimClient()
    client.start()

    print("[verify] launching headless browser (waits for window.__pimsimReady)...")
    browser = HeadlessBrowser()
    browser.start()
    print("[verify] browser ready; settling...")
    time.sleep(SETTLE_SECONDS)

    if state.odom_count == 0:
        print("[verify] FAIL: no /odom received (bridge/browser not publishing)")
        browser.stop()
        return 1

    print("[verify] --- yaw test: command angular.z, expect /odom yaw to move ---")
    client.set_agent_position(0.0, 0.0)
    time.sleep(1.0)
    yaw_start = state.yaw
    turn = Twist()
    turn.angular.z = TURN_RATE
    _drive(command, turn, DRIVE_SECONDS)
    yaw_change = abs(math.atan2(math.sin(state.yaw - yaw_start), math.cos(state.yaw - yaw_start)))
    print(f"[verify] yaw {yaw_start:.3f} -> {state.yaw:.3f} (|Δ|={yaw_change:.3f} rad)")

    print("[verify] --- translation sweep: drive +x/-x/+y/-y from origin ---")
    directions = {
        "+x": (FORWARD_SPEED, 0.0),
        "-x": (-FORWARD_SPEED, 0.0),
        "+y": (0.0, FORWARD_SPEED),
        "-y": (0.0, -FORWARD_SPEED),
    }
    best_travel = 0.0
    for label, (vx, vy) in directions.items():
        client.set_agent_position(0.0, 0.0)
        time.sleep(1.2)
        start_x, start_y = state.x, state.y
        move = Twist()
        move.linear.x = vx
        move.linear.y = vy
        _drive(command, move, DRIVE_SECONDS)
        travel = math.hypot(state.x - start_x, state.y - start_y)
        best_travel = max(best_travel, travel)
        print(
            f"[verify] {label}: ({start_x:.2f},{start_y:.2f}) -> "
            f"({state.x:.2f},{state.y:.2f})  travel={travel:.3f} m"
        )

    lidar_frames = len(state.lidar_counts)
    nonempty = [count for count in state.lidar_counts if count > 0]
    max_points = max(state.lidar_counts) if state.lidar_counts else 0
    print(
        f"[verify] lidar: {lidar_frames} frames, {len(nonempty)} non-empty, "
        f"max {max_points} points/frame"
    )

    browser.stop()
    client.stop()

    yaw_ok = yaw_change >= MIN_YAW_CHANGE_RAD
    travel_ok = best_travel >= MIN_TRAVEL_METERS
    lidar_ok = lidar_frames >= MIN_LIDAR_FRAMES and len(nonempty) > 0 and max_points > 0
    cmd_vel_ok = yaw_ok and travel_ok
    print(
        f"[verify] cmd_vel control: {'PASS' if cmd_vel_ok else 'FAIL'} "
        f"(yaw {'ok' if yaw_ok else 'no'}, best travel {best_travel:.2f} m)"
    )
    print(f"[verify] lidar flowing:   {'PASS' if lidar_ok else 'FAIL'}")
    return 0 if (cmd_vel_ok and lidar_ok) else 1


if __name__ == "__main__":
    # os._exit avoids a nonzero code from lingering playwright/LCM teardown
    # threads after the result is already decided and printed.
    exit_code = main()
    sys.stdout.flush()
    os._exit(exit_code)
