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

import asyncio
import math
from pathlib import Path
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

# Seconds to wait for the first odometry message after the blueprint starts.
ODOM_WAIT_SEC = 60.0

# Seconds between odometry polls while waiting for the robot to reach a goal.
GOAL_POLL_INTERVAL_SEC = 0.1


def _distance(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
    return math.sqrt((from_x - to_x) ** 2 + (from_y - to_y) ** 2)


def _clear_precomputed_paths() -> None:
    paths_dir = (
        Path(__file__).resolve().parents[3] / "data" / "unitree_g1_local_planner_precomputed_paths"
    )
    if paths_dir.exists():
        for path in paths_dir.iterdir():
            path.unlink(missing_ok=True)


async def _run_cross_wall_test(blueprint: Blueprint, *, label: str, max_z: float | None) -> None:
    _clear_precomputed_paths()

    coordinator = ModuleCoordinator.build(blueprint)

    odom_count = 0
    robot_x = 0.0
    robot_y = 0.0
    robot_z = 0.0
    max_z_seen = 0.0

    lcm = lcmlib.LCM(_DEFAULT_LCM_URL)

    def _odom_handler(_channel: str, data: bytes) -> None:
        nonlocal odom_count, robot_x, robot_y, robot_z, max_z_seen
        msg = Odometry.lcm_decode(data)
        odom_count += 1
        robot_x = msg.x
        robot_y = msg.y
        robot_z = msg.pose.position.z
        if robot_z > max_z_seen:
            max_z_seen = robot_z

    subscription = lcm.subscribe(ODOM_TOPIC, _odom_handler)

    loop = asyncio.get_running_loop()
    lcm_fd = lcm.fileno()

    def _on_lcm_readable() -> None:
        try:
            lcm.handle()
        except Exception:
            logger.exception("LCM handle failed; removing reader to stop further polling")
            loop.remove_reader(lcm_fd)

    loop.add_reader(lcm_fd, _on_lcm_readable)

    try:
        logger.info(f"[{label}] Blueprint started, waiting for odom…")

        deadline = time.monotonic() + ODOM_WAIT_SEC
        while time.monotonic() < deadline and odom_count == 0:
            await asyncio.sleep(0.5)

        assert odom_count > 0, f"No odometry received after {ODOM_WAIT_SEC}s — sim not running?"
        initial_x, initial_y = robot_x, robot_y

        logger.info(f"[{label}] Odom online. Robot at ({initial_x:.2f}, {initial_y:.2f})")
        logger.info(f"[{label}] Warming up for {WARMUP_SEC}s…")
        await asyncio.sleep(WARMUP_SEC)

        for name, goal_x, goal_y, goal_z, timeout_sec, threshold in CROSS_WALL_WAYPOINTS:
            start_x, start_y = robot_x, robot_y

            logger.info(
                f"[{label}] === {name}: goal ({goal_x}, {goal_y}) | "
                f"robot ({start_x:.2f}, {start_y:.2f}) | "
                f"dist={_distance(start_x, start_y, goal_x, goal_y):.2f}m | "
                f"budget={timeout_sec}s ==="
            )

            goal = PointStamped(x=goal_x, y=goal_y, z=goal_z, ts=time.time(), frame_id="map")
            lcm.publish(GOAL_TOPIC, goal.lcm_encode())

            start_time = time.monotonic()
            reached = False
            current_x, current_y = start_x, start_y
            distance = _distance(current_x, current_y, goal_x, goal_y)
            while True:
                current_x, current_y = robot_x, robot_y
                current_z = robot_z
                current_max_z = max_z_seen

                if max_z is not None:
                    assert current_z <= max_z, (
                        f"{name}: robot z={current_z:.2f}m exceeded {max_z}m — "
                        f"robot went through the ceiling. "
                        f"pos=({current_x:.2f}, {current_y:.2f}, {current_z:.2f}), "
                        f"max_z={current_max_z:.2f}m"
                    )

                distance = _distance(current_x, current_y, goal_x, goal_y)
                elapsed = time.monotonic() - start_time
                if distance <= threshold:
                    reached = True
                    break
                if elapsed >= timeout_sec:
                    break
                await asyncio.sleep(GOAL_POLL_INTERVAL_SEC)

            assert reached, (
                f"{name}: robot did not reach ({goal_x}, {goal_y}) within {timeout_sec}s. "
                f"Final pos=({current_x:.2f}, {current_y:.2f}), dist={distance:.2f}m"
            )

        if max_z is not None:
            assert max_z_seen <= max_z, (
                f"Robot z peaked at {max_z_seen:.2f}m during the run "
                f"(limit {max_z}m) — went through the ceiling"
            )

    finally:
        loop.remove_reader(lcm_fd)
        lcm.unsubscribe(subscription)
        coordinator.stop()


def run_cross_wall_test(blueprint: Blueprint, *, label: str, max_z: float | None = None) -> None:
    """Build the coordinator, drive the cross-wall waypoint sequence, tear down."""
    asyncio.run(_run_cross_wall_test(blueprint, label=label, max_z=max_z))
