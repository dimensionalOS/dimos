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

"""Cross-wall obstacle check for the babylon-smoketest pimsim sim.

Assumes ``dimos run babylon-smoketest --daemon`` is already running. Proves the
perception primitive a nav stack routes on: a wall spawned with
``PimSimClient.add_wall`` shows up in the ``/lidar`` cloud, so it lands in the
costmap and the planner can steer around it.

Method (driving +y, the most open lane out of the office origin):
  1. baseline: clear obstacles, sample lidar points in the future wall band.
  2. spawn a wall across +y, confirm the band's point count jumps.

Note: the kinematic base raycasts only the static scene mesh, not dynamic
entities, so driving straight at the wall passes through it. That is fine for
nav (the planner avoids the costmap obstacle); it just means add_wall is a
sensor/costmap obstacle, not a physical bumper for the base.

Run: ``.venv/bin/python -m dimos.experimental.pimsim.demo_cross_wall``
"""

from __future__ import annotations

import os
import sys
import time

from dimos.core.transport import LCMTransport
from dimos.experimental.pimsim.client import PimSimClient
from dimos.experimental.pimsim.headless import HeadlessBrowser
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

SETTLE_SECONDS = 3.0
DRIVE_SECONDS = 6.0
COMMAND_PERIOD = 0.1
DRIVE_SPEED = 0.6
WALL_Y = 0.45
WALL_HALF_WIDTH = 1.5
WALL_BAND = 0.2
WALL_X_REACH = 1.0
MIN_WALL_POINTS = 30


class _State:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.latest_cloud: PointCloud2 | None = None
        self.odom_count = 0

    def on_odom(self, msg: PoseStamped) -> None:
        self.x, self.y = float(msg.x), float(msg.y)
        self.odom_count += 1

    def on_lidar(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg


def _wall_band_points(cloud: PointCloud2 | None) -> int:
    if cloud is None:
        return 0
    try:
        points = cloud.points_f32()
    except Exception:
        return 0
    near = (
        (abs(points[:, 1] - WALL_Y) < WALL_BAND)
        & (abs(points[:, 0]) < WALL_X_REACH)
        & (points[:, 1] > 0.0)
    )
    return int(near.sum())


def _drive_plus_y(command: LCMTransport[Twist]) -> None:
    move = Twist()
    move.linear.y = DRIVE_SPEED
    deadline = time.time() + DRIVE_SECONDS
    while time.time() < deadline:
        command.publish(move)
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

    print("[wall] launching headless browser...")
    browser = HeadlessBrowser()
    browser.start()
    time.sleep(SETTLE_SECONDS)
    if state.odom_count == 0:
        print("[wall] FAIL: no /odom received")
        browser.stop()
        return 1

    print("[wall] baseline: clear obstacles, drive +y with no wall")
    client.clear_entities()
    client.set_agent_position(0.0, 0.0)
    time.sleep(1.5)
    base_start_y = state.y
    _drive_plus_y(command)
    baseline_travel = state.y - base_start_y
    print(f"[wall] baseline travel +y: {baseline_travel:.3f} m")

    print(f"[wall] spawn wall across +y at y={WALL_Y}")
    client.set_agent_position(0.0, 0.0)
    time.sleep(1.5)
    base_points = _wall_band_points(state.latest_cloud)
    client.add_wall(-WALL_HALF_WIDTH, WALL_Y, WALL_HALF_WIDTH, WALL_Y)
    time.sleep(2.0)
    wall_points = _wall_band_points(state.latest_cloud)
    print(f"[wall] lidar points in wall band: {base_points} -> {wall_points}")

    # Observation only: the kinematic base raycasts the static scene mesh, not
    # dynamic entities, so driving straight at the wall passes through it. A nav
    # stack never relies on this — it routes around the wall from the lidar
    # costmap, which is the primitive asserted below.
    print("[wall] drive +y at the wall (kinematic base ignores entity colliders)")
    _drive_plus_y(command)
    print(f"[wall] note: base moved to y={state.y:.3f} (passes through entity wall by design)")

    browser.stop()
    client.stop()

    lidar_sees_wall = wall_points >= MIN_WALL_POINTS and wall_points > 2 * base_points
    print(
        f"[wall] lidar detects spawned wall: {'PASS' if lidar_sees_wall else 'FAIL'} "
        f"({base_points} -> {wall_points} points in band)"
    )
    return 0 if lidar_sees_wall else 1


if __name__ == "__main__":
    # os._exit avoids a nonzero code from lingering playwright/LCM teardown
    # threads after the result is already decided and printed.
    exit_code = main()
    sys.stdout.flush()
    os._exit(exit_code)
