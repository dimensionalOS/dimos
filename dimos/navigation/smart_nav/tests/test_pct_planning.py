"""E2E integration test: PCT planner navigation in Unity sim.

Verifies that the PCT (point cloud tomography) planner can:
1. Build a tomogram from the accumulated explored_areas cloud
2. Plan a 3D path across floors to a goal
3. Publish lookahead waypoints that the local planner follows
4. Actually drive the robot to each goal in sequence

Modeled after test_cross_wall_planning.py but swaps FAR → PCT via the
`use_pct_planner` flag on `smart_nav(...)`.

Run:
    DISPLAY=:1 uv run pytest dimos/navigation/smart_nav/tests/test_pct_planning.py -v -s -m slow
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

# Waypoint definitions from plan.md:
#   (name, x, y, z, timeout_sec, reach_threshold_m)
WAYPOINTS = [
    ("p0", -0.3, 2.5, 0.0, 30, 1.5),  # open corridor speed test
    ("p1", 3.3, -4.9, 0.0, 120, 2.0),  # navigate toward doorway area
    ("p2", 11.3, -5.6, 0.0, 120, 2.0),  # into right room
    ("p2→p0", -0.3, 2.5, 0.0, 180, 2.0),  # CRITICAL: cross-area return
]

# PCT needs time to receive its first explored_areas cloud and build a
# tomogram before the first plan can be computed.
WARMUP_SEC = 20.0


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


pytestmark = [pytest.mark.slow]


class TestPCTPlanning:
    """E2E integration test: PCT planner waypoint following through Unity sim."""

    def test_pct_navigation_sequence(self) -> None:
        from dimos.core.coordination.blueprints import autoconnect
        from dimos.core.coordination.module_coordinator import ModuleCoordinator
        from dimos.core.global_config import global_config
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.nav_msgs.Odometry import Odometry
        from dimos.navigation.smart_nav.main import smart_nav, smart_nav_rerun_config
        from dimos.robot.unitree.g1.blueprints.navigation.g1_rerun import (
            g1_static_robot,
        )
        from dimos.simulation.unity.module import UnityBridgeModule
        from dimos.visualization.vis_module import vis_module

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
                    use_pct_planner=True,
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
                    pct_planner={
                        "resolution": 0.075,
                        "slice_dh": 0.4,
                        "slope_max": 0.45,
                        "step_max": 0.5,
                        "lookahead_distance": 1.25,
                        "cost_barrier": 100.0,
                        "kernel_size": 11,
                    },
                ),
                vis_module(
                    viewer_backend=global_config.viewer,
                    rerun_config=smart_nav_rerun_config(
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
            print("[test] Blueprint started, waiting for odom…")

            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                with lock:
                    if odom_count > 0:
                        break
                time.sleep(0.5)

            with lock:
                assert odom_count > 0, "No odometry received after 60s — sim not running?"

            print(f"[test] Odom online. Robot at ({robot_x:.2f}, {robot_y:.2f})")

            print(f"[test] Warming up for {WARMUP_SEC}s (PCT builds initial tomogram)…")
            time.sleep(WARMUP_SEC)
            with lock:
                print(
                    f"[test] Warmup complete. odom_count={odom_count}, "
                    f"pos=({robot_x:.2f}, {robot_y:.2f})"
                )

            for name, gx, gy, gz, timeout_sec, threshold in WAYPOINTS:
                with lock:
                    sx, sy = robot_x, robot_y

                print(
                    f"\n[test] === {name}: goal ({gx}, {gy}) | "
                    f"robot ({sx:.2f}, {sy:.2f}) | "
                    f"dist={_distance(sx, sy, gx, gy):.2f}m | "
                    f"budget={timeout_sec}s ==="
                )

                goal = PointStamped(x=gx, y=gy, z=gz, ts=time.time(), frame_id="map")
                lc.publish(GOAL_TOPIC, goal.lcm_encode())
                print(f"[test] Goal published for {name}")

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
                            f"[test]   {name}: {elapsed:.0f}s/{timeout_sec}s | "
                            f"pos ({cx:.2f}, {cy:.2f}) | dist={dist:.2f}m"
                        )
                        last_print = now

                    if dist <= threshold:
                        reached = True
                        print(
                            f"[test] PCT {name}: reached in {elapsed:.1f}s "
                            f"(dist={dist:.2f}m <= {threshold}m)"
                        )
                        break

                    if elapsed >= timeout_sec:
                        print(
                            f"[test] PCT {name}: NOT reached after {elapsed:.1f}s "
                            f"(dist={dist:.2f}m > {threshold}m)"
                        )
                        break

                    time.sleep(0.1)

                assert reached, (
                    f"{name}: robot did not reach ({gx}, {gy}) within {timeout_sec}s. "
                    f"Final pos=({cx:.2f}, {cy:.2f}), dist={dist:.2f}m"
                )

        finally:
            print("\n[test] Stopping blueprint…")
            lcm_running = False
            lcm_thread.join(timeout=3)
            coordinator.stop()
            print("[test] Done.")
