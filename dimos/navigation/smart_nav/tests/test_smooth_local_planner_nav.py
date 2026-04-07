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

"""E2E integration test: cross-wall nav with SmoothLocalPlanner.

Mirrors ``test_cross_wall_planning_simple.py`` but swaps the native
C++ LocalPlanner for the Python SmoothLocalPlanner and additionally
asserts a bound on commanded yaw-rate sign-flips (a smoothness proxy).
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import threading
import time

import lcm as lcmlib
import pytest

os.environ.setdefault("DISPLAY", ":1")

ODOM_TOPIC = "/odometry#nav_msgs.Odometry"
GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"
CMD_VEL_TOPIC = "/nav_cmd_vel#geometry_msgs.Twist"

# Same waypoints as test_cross_wall_planning_simple.py
WAYPOINTS = [
    ("p0", -0.3, 2.5, 0.0, 30, 1.5),
    ("p1", 11.2, -1.8, 0.0, 120, 2.0),
    ("p2", 3.3, -4.9, 0.0, 120, 2.0),
    ("p3", 7.0, -5.0, 0.0, 120, 2.0),
    ("p4", 11.3, -5.6, 0.0, 120, 2.0),
    ("p4→p1", 11.2, -1.8, 0.0, 180, 2.0),
]

WARMUP_SEC = 15.0

# Smoothness threshold: fewer than this many yaw-rate sign-flips per second.
MAX_SIGN_FLIPS_PER_SEC = 2.0
# Only count sign-flips that cross this magnitude in either direction.
SIGN_FLIP_WZ_THRESHOLD = 0.1


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


pytestmark = [pytest.mark.slow]


class TestSmoothLocalPlannerNav:
    """E2E: cross-wall routing with SmoothLocalPlanner driving nav_cmd_vel."""

    def test_cross_wall_sequence_smooth(self) -> None:
        from dimos.core.blueprints import autoconnect
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.msgs.nav_msgs.Odometry import Odometry
        from dimos.navigation.smart_nav.main import smart_nav
        from dimos.simulation.unity.module import UnityBridgeModule

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
        # Yaw-rate sample buffer — reset per leg
        wz_samples: list[float] = []

        lcm_url = os.environ.get("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0")
        lc = lcmlib.LCM(lcm_url)

        def _odom_handler(channel: str, data: bytes) -> None:
            nonlocal odom_count, robot_x, robot_y
            msg = Odometry.lcm_decode(data)
            with lock:
                odom_count += 1
                robot_x = msg.x
                robot_y = msg.y

        def _cmd_vel_handler(channel: str, data: bytes) -> None:
            msg = Twist.lcm_decode(data)
            with lock:
                wz_samples.append(float(msg.angular.z))

        lc.subscribe(ODOM_TOPIC, _odom_handler)
        lc.subscribe(CMD_VEL_TOPIC, _cmd_vel_handler)

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
            print("[test-smooth] Blueprint started, waiting for odom…")

            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                with lock:
                    if odom_count > 0:
                        break
                time.sleep(0.5)

            with lock:
                assert odom_count > 0, "No odometry received after 60s — sim not running?"

            print(f"[test-smooth] Odom online. Robot at ({robot_x:.2f}, {robot_y:.2f})")

            print(f"[test-smooth] Warming up for {WARMUP_SEC}s…")
            time.sleep(WARMUP_SEC)
            with lock:
                print(
                    f"[test-smooth] Warmup complete. odom_count={odom_count}, "
                    f"pos=({robot_x:.2f}, {robot_y:.2f})"
                )

            for name, gx, gy, gz, timeout_sec, threshold in WAYPOINTS:
                with lock:
                    sx, sy = robot_x, robot_y
                    wz_samples.clear()

                print(
                    f"\n[test-smooth] === {name}: goal ({gx}, {gy}) | "
                    f"robot ({sx:.2f}, {sy:.2f}) | "
                    f"dist={_distance(sx, sy, gx, gy):.2f}m | "
                    f"budget={timeout_sec}s ==="
                )

                goal = PointStamped(x=gx, y=gy, z=gz, ts=time.time(), frame_id="map")
                lc.publish(GOAL_TOPIC, goal.lcm_encode())
                print(f"[test-smooth] Goal published for {name}")

                t0 = time.monotonic()
                reached = False
                last_print = t0
                cx, cy = sx, sy
                dist = _distance(cx, cy, gx, gy)
                while True:
                    with lock:
                        cx, cy = robot_x, robot_y

                    dist = _distance(cx, cy, gx, gy)
                    now = time.monotonic()
                    elapsed = now - t0

                    if now - last_print >= 5.0:
                        print(
                            f"[test-smooth]   {name}: {elapsed:.0f}s/{timeout_sec}s | "
                            f"pos ({cx:.2f}, {cy:.2f}) | dist={dist:.2f}m"
                        )
                        last_print = now

                    if dist <= threshold:
                        reached = True
                        print(
                            f"[test-smooth] ✓ {name}: reached in {elapsed:.1f}s "
                            f"(dist={dist:.2f}m ≤ {threshold}m)"
                        )
                        break

                    if elapsed >= timeout_sec:
                        print(
                            f"[test-smooth] ✗ {name}: NOT reached after {elapsed:.1f}s "
                            f"(dist={dist:.2f}m > {threshold}m)"
                        )
                        break

                    time.sleep(0.1)

                leg_duration = max(time.monotonic() - t0, 1e-6)
                with lock:
                    samples = list(wz_samples)
                flips = _count_sign_flips(samples, SIGN_FLIP_WZ_THRESHOLD)
                flips_per_sec = flips / leg_duration
                print(
                    f"[test-smooth] {name}: wz samples={len(samples)}, "
                    f"sign-flips={flips}, rate={flips_per_sec:.2f}/s "
                    f"(limit {MAX_SIGN_FLIPS_PER_SEC:.1f}/s)"
                )

                assert reached, (
                    f"{name}: robot did not reach ({gx}, {gy}) within {timeout_sec}s. "
                    f"Final pos=({cx:.2f}, {cy:.2f}), dist={dist:.2f}m"
                )
                # Only check smoothness on non-trivial legs (>5 s) so we
                # don't trip on short reorientation pivots at spawn.
                if leg_duration >= 5.0 and len(samples) >= 20:
                    assert flips_per_sec < MAX_SIGN_FLIPS_PER_SEC, (
                        f"{name}: commanded yaw-rate flipped sign {flips} times in "
                        f"{leg_duration:.1f}s ({flips_per_sec:.2f}/s ≥ "
                        f"{MAX_SIGN_FLIPS_PER_SEC:.1f}/s)"
                    )

        finally:
            print("\n[test-smooth] Stopping blueprint…")
            lcm_running = False
            lcm_thread.join(timeout=3)
            coordinator.stop()
            print("[test-smooth] Done.")


def _count_sign_flips(samples: list[float], threshold: float) -> int:
    """Count sign-flips in a yaw-rate sample sequence.

    A flip is counted when the signal crosses between ``< -threshold``
    and ``> +threshold``. Small samples around zero don't reset the
    running sign, so slow drift through zero doesn't count.
    """
    sign = 0  # -1, 0, +1
    flips = 0
    for v in samples:
        if v > threshold:
            new = 1
        elif v < -threshold:
            new = -1
        else:
            continue
        if sign != 0 and new != sign:
            flips += 1
        sign = new
    return flips
