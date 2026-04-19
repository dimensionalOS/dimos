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

"""E2E integration test: cross-wall planning through DimSim.

Uses the DimSim browser-based sim (headless, CPU rendering) with the Go2 nav
blueprint. Tests progressive navigation complexity:

  Phase 1 — Open corridor:  Straight-line drive, no obstacles.
  Phase 2 — Furniture:      Navigate around dining table/chairs.
  Phase 3 — Wall explore:   Drive along a wall to map both sides.
  Phase 4 — Cross-wall:     Goal on the other side of a wall — planner
                             must route through a doorway, not through
                             the wall.

The test verifies that the planner correctly routes around walls by
checking that the robot reaches each waypoint within the timeout. If the
planner tries to drive through a wall, Rapier collision physics will
block it and it will never reach the goal.

Run::

    DIMSIM_LOCAL=1 pytest dimos/e2e_tests/test_dimsim_cross_wall.py -v -s

Requires: dimsim CLI installed (or DIMSIM_LOCAL=1 with ~/repos/DimSim).
"""

from __future__ import annotations

import math
import os
import signal
import socket
import subprocess
import threading
import time

import lcm as lcmlib
import pytest

os.environ.setdefault("DISPLAY", ":1")

ODOM_TOPIC = "/odom#geometry_msgs.PoseStamped"
GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"
BRIDGE_PORT = 8090

# DimSim "apt" scene waypoints (ROS Z-up frame).
#
# Robot starts near ROS (2.6, 1.9) in the living/hallway area.
# Three.js Y-up → ROS Z-up: ROS_x = Three_z, ROS_y = Three_x.
#
# Scene layout (ROS frame):
#   - Living room:  x ≈ 1-2,   y ≈ 3-5   (sofa, coffee table)
#   - Dining area:  x ≈ 3-4,   y ≈ -1-0  (table, chairs)
#   - Kitchen:      x ≈ 0-4,   y ≈ -4--6 (cabinets, appliances)
#   - Bedroom:      x ≈ -2--4, y ≈ -4--6 (bed, wardrobe)
#   - Bathroom:     x ≈ -4--1, y ≈ 2-5   (bathtub, toilet)
#   - Interior walls divide kitchen/bedroom from living/dining
#
# (name, x, y, z, timeout_sec, reach_threshold_m)
#
# Robot spawns at ~(3.0, 2.0) in the living room.
WAYPOINTS = [
    # Phase 1: Open corridor — straight drive, no obstacles nearby
    ("p0_open", 6.0, 2.0, 0.0, 45, 2.0),

    # Phase 2: Furniture — navigate into the hallway/dining area past furniture
    ("p1_furniture", 2.0, 0.0, 0.0, 90, 2.0),

    # Phase 3: Explore wall — drive to map the divider wall from hallway side
    ("p2_wall_side_a", -1.0, 0.0, 0.0, 90, 2.0),
    # Then explore from the kitchen/bedroom side
    ("p3_wall_side_b", -1.0, -3.0, 0.0, 90, 2.0),

    # Phase 4: Cross-wall — goal in bedroom, on the other side of the wall.
    # From the kitchen side, the greedy path to this point crosses through
    # the bedroom wall. The planner must route through a doorway.
    ("p4_cross_wall", -3.0, -2.0, 0.0, 120, 2.5),
]

WARMUP_SEC = 20.0  # Let nav stack build initial voxel map
ODOM_WAIT_SEC = 180.0  # Rapier snapshot init can take 2-3 min on CPU rendering


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def _force_kill_port(port: int) -> None:
    """Kill any process listening on the given port."""
    try:
        result = subprocess.run(
            ["lsof", "-ti", f":{port}"],
            capture_output=True, text=True, timeout=5,
        )
        for pid in result.stdout.strip().split():
            if pid:
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except (ProcessLookupError, ValueError):
                    pass
    except Exception:
        pass


def _wait_for_port(port: int, timeout: float = 120) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", port), timeout=2):
                return True
        except OSError:
            time.sleep(1)
    return False


def _wait_for_port_free(port: int, timeout: float = 30) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", port), timeout=1):
                time.sleep(1)
        except OSError:
            return True
    return False


def _kill_headless_chrome() -> None:
    """Kill any leftover headless Chrome processes from DimSim."""
    try:
        result = subprocess.run(
            ["pgrep", "-f", "chrome.*headless.*dimsim\\|chrome.*headless.*playwright"],
            capture_output=True, text=True, timeout=5,
        )
        for pid in result.stdout.strip().split():
            if pid:
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except (ProcessLookupError, ValueError):
                    pass
    except Exception:
        pass


pytestmark = [pytest.mark.slow]


class TestDimSimCrossWall:
    """E2E integration test: cross-wall routing through DimSim."""

    def test_cross_wall_sequence(self) -> None:
        from dimos.core.coordination.module_coordinator import ModuleCoordinator
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_dimsim import (
            unitree_go2_dimsim,
        )

        # -- Cleanup from previous runs --------------------------------------
        _force_kill_port(BRIDGE_PORT)
        _kill_headless_chrome()
        assert _wait_for_port_free(BRIDGE_PORT, timeout=10), (
            f"Port {BRIDGE_PORT} still in use"
        )

        # -- Build blueprint --------------------------------------------------
        coordinator = ModuleCoordinator.build(unitree_go2_dimsim)

        # -- Odom tracking via LCM -------------------------------------------
        lock = threading.Lock()
        odom_count = 0
        robot_x = 0.0
        robot_y = 0.0

        lcm_url = os.environ.get(
            "LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0"
        )
        lc = lcmlib.LCM(lcm_url)

        def _odom_handler(channel: str, data: bytes) -> None:
            nonlocal odom_count, robot_x, robot_y
            msg = PoseStamped.lcm_decode(data)
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

            # Wait for first odom (sim is up + Rapier snapshot received).
            # CPU rendering (SwiftShader) can take 2-3 min for scene load.
            deadline = time.monotonic() + ODOM_WAIT_SEC
            while time.monotonic() < deadline:
                with lock:
                    if odom_count > 0:
                        break
                time.sleep(0.5)

            with lock:
                assert odom_count > 0, (
                    f"No odometry received after {ODOM_WAIT_SEC}s — DimSim not running?"
                )

            print(f"[test] Odom online. Robot at ({robot_x:.2f}, {robot_y:.2f})")

            # Let the nav stack warm up (build voxel map from lidar)
            print(f"[test] Warming up for {WARMUP_SEC}s…")
            time.sleep(WARMUP_SEC)
            with lock:
                print(
                    f"[test] Warmup complete. odom_count={odom_count}, "
                    f"pos=({robot_x:.2f}, {robot_y:.2f})"
                )

            # -- Navigate waypoint sequence -----------------------------------
            for name, gx, gy, gz, timeout_sec, threshold in WAYPOINTS:
                with lock:
                    sx, sy = robot_x, robot_y

                print(
                    f"\n[test] === {name}: goal ({gx}, {gy}) | "
                    f"robot ({sx:.2f}, {sy:.2f}) | "
                    f"dist={_distance(sx, sy, gx, gy):.2f}m | "
                    f"budget={timeout_sec}s ==="
                )

                # Publish goal
                goal = PointStamped(
                    x=gx, y=gy, z=gz, ts=time.time(), frame_id="map"
                )
                lc.publish(GOAL_TOPIC, goal.lcm_encode())
                print(f"[test] Goal published for {name}")

                # Wait for robot to reach goal or timeout
                t0 = time.monotonic()
                reached = False
                last_print = t0
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
                            f"[test] ✓ {name}: reached in {elapsed:.1f}s "
                            f"(dist={dist:.2f}m ≤ {threshold}m)"
                        )
                        break

                    if elapsed >= timeout_sec:
                        print(
                            f"[test] ✗ {name}: NOT reached after {elapsed:.1f}s "
                            f"(dist={dist:.2f}m > {threshold}m)"
                        )
                        break

                    time.sleep(0.1)

                assert reached, (
                    f"{name}: robot did not reach ({gx}, {gy}) within "
                    f"{timeout_sec}s. Final pos=({cx:.2f}, {cy:.2f}), "
                    f"dist={dist:.2f}m"
                )

        finally:
            print("\n[test] Stopping blueprint…")
            lcm_running = False
            lcm_thread.join(timeout=3)
            coordinator.stop()
            # Clean up headless Chrome
            _kill_headless_chrome()
            _force_kill_port(BRIDGE_PORT)
            print("[test] Done.")
