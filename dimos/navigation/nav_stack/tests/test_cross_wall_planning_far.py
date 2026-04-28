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

"""E2E integration test: cross-wall planning through Unity sim.

Verifies that the FAR planner routes through doorways instead of through walls.
Uses the full navigation stack (same blueprint as unitree_g1_nav_sim) and
tracks the robot position via odometry to verify goal-reaching.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import threading
import time

import lcm as lcmlib
import pytest

# create_nav_stack pulls in PGO which requires gtsam — skip the whole module
# if it isn't installed.
pytest.importorskip("gtsam")

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.protocol.service.lcmservice import _DEFAULT_LCM_URL
from dimos.robot.unitree.g1.g1_rerun import g1_static_robot
from dimos.simulation.unity.module import UnityBridgeModule
from dimos.utils.logging_config import setup_logger
from dimos.visualization.vis_module import vis_module

logger = setup_logger()


@pytest.fixture
def display_env():
    """Set DISPLAY for the test, restore the prior value on teardown."""
    prior = os.environ.get("DISPLAY")
    os.environ.setdefault("DISPLAY", ":1")
    yield
    if prior is None:
        os.environ.pop("DISPLAY", None)
    else:
        os.environ["DISPLAY"] = prior


ODOM_TOPIC = "/odometry#nav_msgs.Odometry"
GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"

# Waypoint definitions: (name, x, y, z, timeout_sec, reach_threshold_m)
WAYPOINTS = [
    ("p0", -0.3, 2.5, 0.0, 30, 1.5),
    ("p1", 11.2, -1.8, 0.0, 120, 2.0),
    ("p2", 3.3, -4.9, 0.0, 120, 2.0),
    ("p3", 7.0, -5.0, 0.0, 120, 2.0),  # Through doorway into right room
    ("p4", 11.3, -5.6, 0.0, 120, 2.0),  # Deep in right room
    ("p4→p1", 11.2, -1.8, 0.0, 180, 2.0),  # CRITICAL: cross-wall back
]

WARMUP_SEC = 15.0  # seconds to let nav stack build terrain + visibility graph


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


pytestmark = [pytest.mark.slow]


class TestCrossWallPlanning:
    def test_cross_wall_sequence(self, display_env):
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
                    far_planner={
                        "sensor_range": 15.0,
                        "is_static_env": True,
                        "converge_dist": 1.5,
                    },
                ),
                vis_module(
                    viewer_backend=global_config.viewer,
                    rerun_config=nav_stack_rerun_config(
                        {
                            "blueprint": UnityBridgeModule.rerun_blueprint,
                            "visual_override": {
                                "world/camera_info": UnityBridgeModule.rerun_suppress_camera_info,
                            },
                            "static": {
                                "world/color_image": UnityBridgeModule.rerun_static_pinhole,
                                "world/tf/robot": g1_static_robot,
                            },
                        }
                    ),
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

        lcm = lcmlib.LCM(_DEFAULT_LCM_URL)

        def _odom_handler(channel: str, data: bytes) -> None:
            nonlocal odom_count, robot_x, robot_y
            msg = Odometry.lcm_decode(data)
            with lock:
                odom_count += 1
                robot_x = msg.x
                robot_y = msg.y

        lcm.subscribe(ODOM_TOPIC, _odom_handler)

        # LCM receive thread
        lcm_stop = threading.Event()

        def _lcm_loop() -> None:
            while not lcm_stop.is_set():
                try:
                    lcm.handle_timeout(100)
                except Exception:
                    pass

        lcm_thread = threading.Thread(target=_lcm_loop, daemon=True)
        lcm_thread.start()

        try:
            logger.info("[test] Blueprint started, waiting for odom…")

            # Wait for first odom (sim is up)
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                with lock:
                    if odom_count > 0:
                        break
                time.sleep(0.5)

            with lock:
                assert odom_count > 0, "No odometry received after 60s — sim not running?"

            logger.info(f"[test] Odom online. Robot at ({robot_x:.2f}, {robot_y:.2f})")

            # Let the nav stack warm up (terrain analysis, PGO, FAR visibility graph)
            logger.info(f"[test] Warming up for {WARMUP_SEC}s…")
            time.sleep(WARMUP_SEC)

            for name, gx, gy, gz, timeout_sec, threshold in WAYPOINTS:
                with lock:
                    sx, sy = robot_x, robot_y

                logger.info(
                    f"[test] === {name}: goal ({gx}, {gy}) | "
                    f"robot ({sx:.2f}, {sy:.2f}) | "
                    f"dist={_distance(sx, sy, gx, gy):.2f}m | "
                    f"budget={timeout_sec}s ==="
                )

                goal = PointStamped(x=gx, y=gy, z=gz, ts=time.time(), frame_id="map")
                lcm.publish(GOAL_TOPIC, goal.lcm_encode())

                t0 = time.monotonic()
                reached = False
                cx, cy = sx, sy
                dist = _distance(cx, cy, gx, gy)
                while True:
                    with lock:
                        cx, cy = robot_x, robot_y
                    dist = _distance(cx, cy, gx, gy)
                    elapsed = time.monotonic() - t0
                    if dist <= threshold:
                        reached = True
                        break
                    if elapsed >= timeout_sec:
                        break
                    time.sleep(0.1)

                assert reached, (
                    f"{name}: robot did not reach ({gx}, {gy}) within {timeout_sec}s. "
                    f"Final pos=({cx:.2f}, {cy:.2f}), dist={dist:.2f}m"
                )

        finally:
            lcm_stop.set()
            lcm_thread.join(timeout=3)
            assert not lcm_thread.is_alive(), "LCM loop thread didn't exit cleanly"
            coordinator.stop()
