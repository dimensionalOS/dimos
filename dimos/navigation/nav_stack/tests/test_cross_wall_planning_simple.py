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

"""E2E integration test: cross-wall planning using SimplePlanner.

Mirrors ``test_cross_wall_planning.py`` but swaps FarPlanner for
SimplePlanner (grid A*). Same blueprint, same waypoint sequence, same
success thresholds — this is the apples-to-apples comparison to see
whether the simple planner can route through doorways.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import threading
import time

import lcm as lcmlib
import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.nav_stack.main import create_nav_stack
from dimos.simulation.unity.module import UnityBridgeModule

os.environ.setdefault("DISPLAY", ":1")

ODOM_TOPIC = "/odometry#nav_msgs.Odometry"
GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"

# Waypoint definitions: (name, x, y, z, timeout_sec, reach_threshold_m)
WAYPOINTS = [
    ("p0", -0.3, 2.5, 0.0, 30, 1.5),
    ("p1", 11.2, -1.8, 0.0, 120, 2.0),
    ("p2", 3.3, -4.9, 0.0, 120, 2.0),
    ("p3", 7.0, -5.0, 0.0, 120, 2.0),
    ("p4", 11.3, -5.6, 0.0, 120, 2.0),
    ("p4→p1", 11.2, -1.8, 0.0, 180, 2.0),
]

WARMUP_SEC = 15.0


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


pytestmark = [pytest.mark.slow]


class TestCrossWallPlanningSimple:
    """E2E: cross-wall routing with SimplePlanner (A* on 2D costmap)."""

    def test_cross_wall_sequence_simple(self) -> None:
        paths_dir = (
            Path(__file__).resolve().parents[3]
            / "data"
            / "unitree_g1_local_planner_precomputed_paths"
        )
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
                create_nav_stack(
                    use_simple_planner=True,
                    terrain_analysis={
                        "obstacle_height_threshold": 0.1,
                        "ground_height_threshold": 0.05,
                        "max_relative_z": 0.3,
                        "min_relative_z": -1.5,
                    },
                    local_planner={
                        "max_speed": 2.0,
                        "autonomy_speed": 2.0,
                        "obstacle_height_threshold": 0.1,
                        "max_relative_z": 0.3,
                        "min_relative_z": -1.5,
                        "freeze_ang": 180.0,
                        "two_way_drive": False,
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
                        # Tighten stuck-detection for the test so doorways
                        # that the wider inflation blocks get opened up
                        # within a few seconds of non-progress.
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

        coordinator = ModuleCoordinator.build(blueprint)

        lock = threading.Lock()
        odom_count = 0
        robot_x = 0.0
        robot_y = 0.0
        robot_z = 0.0
        max_z = 0.0
        # If the robot's z ever exceeds this, it has gone through the
        # ceiling / climbed on top of geometry — navigation is broken.
        # The sim's terrain-z estimate drifts ~0.3 m near walls (wall
        # points within the 0.5 m terrain sampling radius pull the ground
        # estimate upward), so this must tolerate vehicle_height (1.24 m)
        # + terrain drift while still catching through-the-roof failures
        # (roof is at ~3 m+).
        MAX_ALLOWED_Z = 2.0

        lcm_url = os.environ.get("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0")
        lc = lcmlib.LCM(lcm_url)

        def _odom_handler(channel: str, data: bytes) -> None:
            nonlocal odom_count, robot_x, robot_y, robot_z, max_z
            msg = Odometry.lcm_decode(data)
            with lock:
                odom_count += 1
                robot_x = msg.x
                robot_y = msg.y
                robot_z = msg.pose.position.z
                if robot_z > max_z:
                    max_z = robot_z

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
            print("[test-simple] Blueprint started, waiting for odom…")

            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                with lock:
                    if odom_count > 0:
                        break
                time.sleep(0.5)

            with lock:
                assert odom_count > 0, "No odometry received after 60s — sim not running?"

            print(f"[test-simple] Odom online. Robot at ({robot_x:.2f}, {robot_y:.2f})")

            print(f"[test-simple] Warming up for {WARMUP_SEC}s…")
            time.sleep(WARMUP_SEC)
            with lock:
                print(
                    f"[test-simple] Warmup complete. odom_count={odom_count}, "
                    f"pos=({robot_x:.2f}, {robot_y:.2f})"
                )

            for name, gx, gy, gz, timeout_sec, threshold in WAYPOINTS:
                with lock:
                    sx, sy = robot_x, robot_y

                print(
                    f"\n[test-simple] === {name}: goal ({gx}, {gy}) | "
                    f"robot ({sx:.2f}, {sy:.2f}) | "
                    f"dist={_distance(sx, sy, gx, gy):.2f}m | "
                    f"budget={timeout_sec}s ==="
                )

                goal = PointStamped(x=gx, y=gy, z=gz, ts=time.time(), frame_id="map")
                lc.publish(GOAL_TOPIC, goal.lcm_encode())
                print(f"[test-simple] Goal published for {name}")

                t0 = time.monotonic()
                reached = False
                last_print = t0
                cx, cy = sx, sy
                dist = _distance(cx, cy, gx, gy)
                while True:
                    with lock:
                        cx, cy = robot_x, robot_y
                        cz = robot_z
                        cur_max_z = max_z

                    assert cz <= MAX_ALLOWED_Z, (
                        f"{name}: robot z={cz:.2f}m exceeded {MAX_ALLOWED_Z}m — "
                        f"robot went through the ceiling. "
                        f"pos=({cx:.2f}, {cy:.2f}, {cz:.2f}), max_z={cur_max_z:.2f}m"
                    )

                    dist = _distance(cx, cy, gx, gy)
                    now = time.monotonic()
                    elapsed = now - t0

                    if now - last_print >= 5.0:
                        print(
                            f"[test-simple]   {name}: {elapsed:.0f}s/{timeout_sec}s | "
                            f"pos ({cx:.2f}, {cy:.2f}, z={cz:.2f}) | dist={dist:.2f}m"
                        )
                        last_print = now

                    if dist <= threshold:
                        reached = True
                        print(
                            f"[test-simple] ✓ {name}: reached in {elapsed:.1f}s "
                            f"(dist={dist:.2f}m ≤ {threshold}m)"
                        )
                        break

                    if elapsed >= timeout_sec:
                        print(
                            f"[test-simple] ✗ {name}: NOT reached after {elapsed:.1f}s "
                            f"(dist={dist:.2f}m > {threshold}m)"
                        )
                        break

                    time.sleep(0.1)

                assert reached, (
                    f"{name}: robot did not reach ({gx}, {gy}) within {timeout_sec}s. "
                    f"Final pos=({cx:.2f}, {cy:.2f}), dist={dist:.2f}m"
                )

            # Final guard: the robot should never have gone above the
            # allowed height at any point during the entire test run.
            with lock:
                final_max_z = max_z
            assert final_max_z <= MAX_ALLOWED_Z, (
                f"Robot z peaked at {final_max_z:.2f}m during the run "
                f"(limit {MAX_ALLOWED_Z}m) — went through the ceiling"
            )

        finally:
            print("\n[test-simple] Stopping blueprint…")
            lcm_running = False
            lcm_thread.join(timeout=3)
            coordinator.stop()
            print("[test-simple] Done.")
