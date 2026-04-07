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

"""E2E collision test: SmoothLocalPlanner must not drive through walls.

Loads a pre-recorded costmap (saved with ``bin/save_costmap`` while
driving the sim with the 0.7 m inflation the SimplePlanner uses) and
asserts that the robot's odometry trajectory never enters any cell
marked occupied in that costmap. The costmap includes inflation, so a
robot center inside a blocked cell means the robot body is against a
wall.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import threading
import time

import lcm as lcmlib
import numpy as np
import pytest

os.environ.setdefault("DISPLAY", ":1")

ODOM_TOPIC = "/odometry#nav_msgs.Odometry"
GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"

GOAL = (11.1, 1.7, 0.0)
GOAL_THRESHOLD = 2.0
GOAL_TIMEOUT = 180.0
WARMUP_SEC = 15.0
# Don't fault-check during the first SETTLE_SEC of the leg — the spawn
# pose may clip an inflated cell before the robot has moved.
SETTLE_SEC = 3.0

COSTMAP_PATH = Path(__file__).resolve().parents[1] / "modules" / "simple_planner" / "costmap.npz"

pytestmark = [pytest.mark.slow]


class Occupancy:
    def __init__(self, path: Path) -> None:
        data = np.load(path)
        self.grid: np.ndarray = data["grid"]
        self.origin = tuple(float(v) for v in data["origin"])
        self.cell_size = float(data["cell_size"])
        self.h, self.w = self.grid.shape

    def is_blocked(self, x: float, y: float) -> bool:
        ix = round((x - self.origin[0]) / self.cell_size)
        iy = round((y - self.origin[1]) / self.cell_size)
        if ix < 0 or iy < 0 or ix >= self.w or iy >= self.h:
            return False  # outside recorded map = unknown, not a failure
        return bool(self.grid[iy, ix])


class TestCollisionSmooth:
    def test_no_collision_on_waypoint_drive(self) -> None:
        from dimos.core.blueprints import autoconnect
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.nav_msgs.Odometry import Odometry
        from dimos.navigation.smart_nav.main import smart_nav
        from dimos.simulation.unity.module import UnityBridgeModule

        assert COSTMAP_PATH.exists(), f"costmap fixture missing: {COSTMAP_PATH}"
        occ = Occupancy(COSTMAP_PATH)
        print(
            f"[collision-smooth] costmap loaded: {occ.h}x{occ.w} cells, "
            f"cell={occ.cell_size}m, origin={occ.origin}"
        )

        paths_dir = Path(__file__).resolve().parents[3] / "data" / "smart_nav_paths"
        if paths_dir.exists():
            for f in paths_dir.iterdir():
                f.unlink(missing_ok=True)

        blueprint = (
            autoconnect(
                UnityBridgeModule.blueprint(
                    unity_binary="",
                    unity_scene="home_building_1",
                    vehicle_height=1.24,
                ),
                smart_nav(
                    use_simple_planner=True,
                    use_smooth_local_planner=True,
                    terrain_analysis={
                        "obstacle_height_threshold": 0.1,
                        "ground_height_threshold": 0.05,
                        "max_relative_z": 0.3,
                        "min_relative_z": -1.5,
                    },
                    smooth_local_planner={
                        "max_curvature": 1.2,
                        "arc_length": 3.0,
                        "obstacle_range": 3.5,
                        "robot_radius": 0.35,
                        "obstacle_height_threshold": 0.1,
                        "max_relative_z": 0.3,
                        "min_relative_z": -1.5,
                        "curvature_ema_alpha": 0.3,
                        "publish_rate": 20.0,
                        "publish_length": 3.0,
                    },
                    path_follower={
                        "max_speed": 2.0,
                        "autonomy_speed": 2.0,
                        "max_acceleration": 4.0,
                        "slow_down_distance_threshold": 0.5,
                        "omni_dir_goal_threshold": 0.5,
                        "two_way_drive": False,
                    },
                    simple_planner={
                        "cell_size": 0.3,
                        "obstacle_height_threshold": 0.15,
                        "inflation_radius": 0.7,
                        "lookahead_distance": 2.0,
                        "replan_rate": 5.0,
                        "stuck_seconds": 4.0,
                        "stuck_shrink_factor": 0.5,
                    },
                ),
            )
            .remappings(
                [
                    (UnityBridgeModule, "terrain_map", "terrain_map_ext"),
                ]
            )
            .global_config(n_workers=8, robot_model="unitree_g1", simulation=True)
        )

        coordinator = blueprint.build()

        lock = threading.Lock()
        odom_count = 0
        robot_x = 0.0
        robot_y = 0.0

        lcm_url = os.environ.get("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0")
        lc = lcmlib.LCM(lcm_url)

        def _odom_handler(channel: str, data: bytes) -> None:
            nonlocal odom_count, robot_x, robot_y
            msg = Odometry.lcm_decode(data)
            with lock:
                odom_count += 1
                robot_x = msg.x
                robot_y = msg.y

        lc.subscribe(ODOM_TOPIC, _odom_handler)

        lcm_running = True

        def _lcm_loop() -> None:
            while lcm_running:
                try:
                    lc.handle_timeout(100)
                except Exception:
                    pass

        lcm_thread = threading.Thread(target=_lcm_loop, daemon=True)
        lcm_thread.start()

        try:
            coordinator.start()
            print("[collision-smooth] Blueprint started, waiting for odom…")
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                with lock:
                    if odom_count > 0:
                        break
                time.sleep(0.5)
            with lock:
                assert odom_count > 0, "No odometry received after 60s — sim not running?"

            print(f"[collision-smooth] Warming up for {WARMUP_SEC}s…")
            time.sleep(WARMUP_SEC)

            with lock:
                sx, sy = robot_x, robot_y
            print(
                f"[collision-smooth] Start ({sx:.2f}, {sy:.2f}) → goal "
                f"({GOAL[0]}, {GOAL[1]})  budget={GOAL_TIMEOUT}s"
            )

            goal_msg = PointStamped(x=GOAL[0], y=GOAL[1], z=GOAL[2], ts=time.time(), frame_id="map")
            lc.publish(GOAL_TOPIC, goal_msg.lcm_encode())
            print("[collision-smooth] Goal published.")

            collisions: list[tuple[float, float, float]] = []
            t0 = time.monotonic()
            last_print = t0
            reached = False
            while True:
                with lock:
                    cx, cy = robot_x, robot_y
                dist = math.hypot(cx - GOAL[0], cy - GOAL[1])
                elapsed = time.monotonic() - t0

                if elapsed >= SETTLE_SEC and occ.is_blocked(cx, cy):
                    collisions.append((elapsed, cx, cy))

                if time.monotonic() - last_print >= 5.0:
                    print(
                        f"[collision-smooth] t={elapsed:.0f}s  "
                        f"pos=({cx:.2f},{cy:.2f})  dist={dist:.2f}m  "
                        f"collisions={len(collisions)}"
                    )
                    last_print = time.monotonic()

                if dist <= GOAL_THRESHOLD:
                    reached = True
                    print(f"[collision-smooth] ✓ reached in {elapsed:.1f}s (dist={dist:.2f}m)")
                    break
                if elapsed >= GOAL_TIMEOUT:
                    print(
                        f"[collision-smooth] ✗ not reached after {elapsed:.1f}s (dist={dist:.2f}m)"
                    )
                    break
                time.sleep(0.1)

        finally:
            print("[collision-smooth] Stopping blueprint…")
            lcm_running = False
            lcm_thread.join(timeout=3)
            coordinator.stop()
            print("[collision-smooth] Done.")

        if collisions:
            head = "\n  ".join(f"t={t:.1f}s at ({x:.2f}, {y:.2f})" for t, x, y in collisions[:10])
            pytest.fail(
                f"Robot entered {len(collisions)} blocked cell(s). First samples:\n  {head}"
            )
        assert reached, "Robot did not reach the goal within the timeout"
