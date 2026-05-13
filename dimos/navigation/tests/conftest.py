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

"""Shared infrastructure for nav_stack cross-wall planning E2E tests.

The full stack drives the robot via /clicked_point (PointStamped) goals and
we verify reach by polling odometry — a different goal-mechanism than the
shared `follow_points` fixture in dimos/e2e_tests/conftest.py (which uses
/goal_request + /goal_reached). That's why these tests don't reuse it.
"""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time

import lcm as lcmlib

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.protocol.service.lcmservice import _DEFAULT_LCM_URL
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


ODOM_TOPIC = "/odometry#nav_msgs.Odometry"
GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"

# (name, x, y, z, timeout_sec, reach_threshold_m)
CROSS_WALL_WAYPOINTS: list[tuple[str, float, float, float, float, float]] = [
    ("p0", -0.3, 2.5, 0.0, 30, 1.5),
    ("p1", 11.2, -1.8, 0.0, 120, 2.0),
    ("p2", 3.3, -4.9, 0.0, 120, 2.0),
    ("p3", 7.0, -5.0, 0.0, 120, 2.0),  # Through doorway into right room
    ("p4", 11.3, -5.6, 0.0, 120, 2.0),  # Deep in right room
    ("p4→p1", 11.2, -1.8, 0.0, 180, 2.0),  # CRITICAL: cross-wall back
]

# Seconds for nav stack to build terrain + visibility graph before goals fly.
WARMUP_SEC = 15.0


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def _clear_precomputed_paths() -> None:
    paths_dir = (
        Path(__file__).resolve().parents[3] / "data" / "unitree_g1_local_planner_precomputed_paths"
    )
    if paths_dir.exists():
        for f in paths_dir.iterdir():
            f.unlink(missing_ok=True)


# Shared nav-stack configs for cross-wall E2E tests.  Both the FAR and
# Simple planner tests use the same terrain/local/path_follower tuning so
# results are apples-to-apples.
CROSS_WALL_TERRAIN_ANALYSIS = {
    "obstacle_height_threshold": 0.1,
    "ground_height_threshold": 0.05,
    "max_relative_z": 0.3,
    "min_relative_z": -1.5,
}

CROSS_WALL_LOCAL_PLANNER = {
    "max_speed": 2.0,
    "autonomy_speed": 2.0,
    "obstacle_height_threshold": 0.1,
    "max_relative_z": 0.3,
    "min_relative_z": -1.5,
    "freeze_ang": 180.0,
    "two_way_drive": False,
}

CROSS_WALL_PATH_FOLLOWER = {
    "max_speed": 2.0,
    "autonomy_speed": 2.0,
    "max_acceleration": 4.0,
    "slow_down_distance_threshold": 0.5,
    "omni_dir_goal_threshold": 0.5,
    "two_way_drive": False,
}


def run_cross_wall_test(
    blueprint: Blueprint,
    *,
    label: str,
    max_z: float | None = None,
    waypoints: list[tuple[str, float, float, float, float, float]] | None = None,
) -> None:
    """Build the coordinator, drive the cross-wall waypoint sequence, tear down.

    Args:
        blueprint: A fully configured Blueprint (Unity sim + nav stack + remappings).
        label: Short tag used in log lines so far/simple runs are distinguishable.
        max_z: If set, asserts the robot's z never exceeds this (catches the
            robot climbing geometry / passing through the ceiling).
        waypoints: Optional waypoint sequence to drive.  Defaults to
            ``CROSS_WALL_WAYPOINTS`` (Unity ``home_building_1`` scene).
            Provide a scene-specific list for non-Unity sims.
    """
    waypoints = waypoints if waypoints is not None else CROSS_WALL_WAYPOINTS
    _clear_precomputed_paths()

    coordinator = ModuleCoordinator.build(blueprint)

    lock = threading.Lock()
    odom_count = 0
    robot_x = 0.0
    robot_y = 0.0
    robot_z = 0.0
    max_z_seen = 0.0

    lcm = lcmlib.LCM(_DEFAULT_LCM_URL)

    def _odom_handler(_channel: str, data: bytes) -> None:
        nonlocal odom_count, robot_x, robot_y, robot_z, max_z_seen
        msg = Odometry.lcm_decode(data)
        with lock:
            odom_count += 1
            robot_x = msg.x
            robot_y = msg.y
            robot_z = msg.pose.position.z
            if robot_z > max_z_seen:
                max_z_seen = robot_z

    subscription = lcm.subscribe(ODOM_TOPIC, _odom_handler)

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
        logger.info(f"[{label}] Blueprint started, waiting for odom…")

        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            with lock:
                if odom_count > 0:
                    break
            time.sleep(0.5)

        with lock:
            assert odom_count > 0, "No odometry received after 60s — sim not running?"
            x0, y0 = robot_x, robot_y

        logger.info(f"[{label}] Odom online. Robot at ({x0:.2f}, {y0:.2f})")
        logger.info(f"[{label}] Warming up for {WARMUP_SEC}s…")
        time.sleep(WARMUP_SEC)

        for name, gx, gy, gz, timeout_sec, threshold in waypoints:
            with lock:
                sx, sy = robot_x, robot_y

            logger.info(
                f"[{label}] === {name}: goal ({gx}, {gy}) | "
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
                    cz = robot_z
                    cur_max_z = max_z_seen

                if max_z is not None:
                    assert cz <= max_z, (
                        f"{name}: robot z={cz:.2f}m exceeded {max_z}m — "
                        f"robot went through the ceiling. "
                        f"pos=({cx:.2f}, {cy:.2f}, {cz:.2f}), max_z={cur_max_z:.2f}m"
                    )

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

        if max_z is not None:
            with lock:
                final_max_z = max_z_seen
            assert final_max_z <= max_z, (
                f"Robot z peaked at {final_max_z:.2f}m during the run "
                f"(limit {max_z}m) — went through the ceiling"
            )

    finally:
        lcm_stop.set()
        lcm_thread.join(timeout=3)
        assert not lcm_thread.is_alive(), "LCM loop thread didn't exit cleanly"
        lcm.unsubscribe(subscription)
        coordinator.stop()
