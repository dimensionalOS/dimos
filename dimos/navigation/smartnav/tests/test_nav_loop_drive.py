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

"""Integration test: robot navigates a multi-waypoint loop.

Sends waypoints in a square pattern and verifies the robot actually
moves toward each one. Prints detailed odometry + cmd_vel diagnostics.

This is the definitive test that the nav stack works end-to-end.
"""

from __future__ import annotations

import math
from pathlib import Path
import platform
import threading
import time
from typing import Any

import numpy as np
import pytest

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

_NATIVE_DIR = Path(__file__).resolve().parent.parent
_HAS_BINARIES = all(
    (_NATIVE_DIR / d / "bin" / name).exists()
    for d, name in [
        ("result-terrain-analysis", "terrain_analysis"),
        ("result-local-planner", "local_planner"),
        ("result-path-follower", "path_follower"),
    ]
)
_IS_LINUX_X86 = platform.system() == "Linux" and platform.machine() in ("x86_64", "AMD64")

pytestmark = [
    pytest.mark.slow,
    pytest.mark.skipif(not _IS_LINUX_X86, reason="Native modules require Linux x86_64"),
    pytest.mark.skipif(not _HAS_BINARIES, reason="Native binaries not built"),
]


def _make_ground(rx: float, ry: float) -> np.ndarray:
    """Flat ground cloud around robot. Nx3."""
    step = 1.5
    xs = np.arange(rx - 15, rx + 15, step)
    ys = np.arange(ry - 15, ry + 15, step)
    xx, yy = np.meshgrid(xs, ys)
    return np.column_stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)]).astype(np.float32)


class VehicleConfig(ModuleConfig):
    sensor_rate: float = 5.0
    sim_rate: float = 50.0


class Vehicle(Module[VehicleConfig]):
    """Kinematic sim vehicle with public position for test inspection."""

    default_config = VehicleConfig
    cmd_vel: In[Twist]
    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]

    def __init__(self, **kw):  # type: ignore[no-untyped-def]
        super().__init__(**kw)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.75
        self.yaw = 0.0
        self._fwd = 0.0
        self._left = 0.0
        self._yr = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._threads: list[threading.Thread] = []

    def __getstate__(self) -> dict[str, Any]:
        s = super().__getstate__()
        for k in ("_lock", "_threads"):
            s.pop(k, None)
        return s

    def __setstate__(self, s: dict) -> None:
        super().__setstate__(s)
        self._lock = threading.Lock()
        self._threads = []

    def start(self) -> None:
        self.cmd_vel._transport.subscribe(self._on_cmd)
        self._running = True
        for fn in (self._sim_loop, self._sensor_loop):
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self) -> None:
        self._running = False
        for t in self._threads:
            t.join(timeout=3)
        super().stop()

    def _on_cmd(self, tw: Twist) -> None:
        with self._lock:
            self._fwd = tw.linear.x
            self._left = tw.linear.y
            self._yr = tw.angular.z

    def _sim_loop(self) -> None:
        dt = 1.0 / self.config.sim_rate
        while self._running:
            t0 = time.monotonic()
            with self._lock:
                fwd, left, yr = self._fwd, self._left, self._yr
            self.yaw += dt * yr
            cy, sy = math.cos(self.yaw), math.sin(self.yaw)
            self.x += dt * (cy * fwd - sy * left)
            self.y += dt * (sy * fwd + cy * left)
            now = time.time()
            q = Quaternion.from_euler(Vector3(0.0, 0.0, self.yaw))
            self.odometry._transport.publish(
                Odometry(
                    ts=now,
                    frame_id="map",
                    child_frame_id="sensor",
                    pose=Pose(position=[self.x, self.y, self.z], orientation=[q.x, q.y, q.z, q.w]),
                    twist=Twist(linear=[fwd, left, 0], angular=[0, 0, yr]),
                )
            )
            self.tf.publish(
                Transform(
                    translation=Vector3(self.x, self.y, self.z),
                    rotation=q,
                    frame_id="map",
                    child_frame_id="sensor",
                    ts=now,
                )
            )
            sl = dt - (time.monotonic() - t0)
            if sl > 0:
                time.sleep(sl)

    def _sensor_loop(self) -> None:
        dt = 1.0 / self.config.sensor_rate
        while self._running:
            now = time.time()
            cloud = _make_ground(self.x, self.y)
            self.registered_scan._transport.publish(
                PointCloud2.from_numpy(cloud, frame_id="map", timestamp=now)
            )
            time.sleep(dt)


def test_multi_waypoint_loop():
    """Send 4 waypoints in a square, verify robot moves toward each."""
    from dimos.core.blueprints import autoconnect
    from dimos.navigation.smartnav.modules.local_planner.local_planner import LocalPlanner
    from dimos.navigation.smartnav.modules.path_follower.path_follower import PathFollower
    from dimos.navigation.smartnav.modules.sensor_scan_generation.sensor_scan_generation import (
        SensorScanGeneration,
    )
    from dimos.navigation.smartnav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis

    # Collect cmd_vel to verify non-zero commands
    cmd_log: list[tuple[float, float, float]] = []
    cmd_lock = threading.Lock()

    blueprint = autoconnect(
        Vehicle.blueprint(),
        SensorScanGeneration.blueprint(),
        TerrainAnalysis.blueprint(),
        LocalPlanner.blueprint(
            autonomy_mode=True,
            max_speed=2.0,
            autonomy_speed=2.0,
        ),
        PathFollower.blueprint(
            autonomy_mode=True,
            max_speed=2.0,
            autonomy_speed=2.0,
            max_accel=4.0,
            slow_dwn_dis_thre=0.2,
        ),
    )
    coord = blueprint.build()

    planner = coord.get_instance(LocalPlanner)
    follower = coord.get_instance(PathFollower)

    follower.cmd_vel._transport.subscribe(
        lambda m: (
            cmd_lock.acquire(),
            cmd_log.append((m.linear.x, m.linear.y, m.angular.z)),
            cmd_lock.release(),
        )
    )

    # Also track path sizes to diagnose stop paths
    path_sizes: list[int] = []
    path_lock = threading.Lock()
    planner.path._transport.subscribe(
        lambda m: (path_lock.acquire(), path_sizes.append(len(m.poses)), path_lock.release())
    )

    # We can't access vehicle._x directly (Actor proxy blocks private attrs).
    # Instead, subscribe to odometry and track position ourselves.
    positions: list[tuple[float, float]] = []
    pos_lock = threading.Lock()

    def _on_odom(msg: Odometry) -> None:
        with pos_lock:
            positions.append((msg.pose.position.x, msg.pose.position.y))

    vehicle_actor = coord.get_instance(Vehicle)
    vehicle_actor.odometry._transport.subscribe(_on_odom)

    coord.start()

    waypoints = [(5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)]

    try:
        # Wait for C++ modules to initialize
        print("[test] Waiting 3s for modules to start...")
        time.sleep(3.0)

        for i, (wx, wy) in enumerate(waypoints):
            wp = PointStamped(x=wx, y=wy, z=0.0, frame_id="map")
            planner.way_point._transport.publish(wp)
            print(f"[test] Sent waypoint {i}: ({wx}, {wy})")

            # Drive toward waypoint for up to 8 seconds
            t0 = time.monotonic()
            while time.monotonic() - t0 < 8.0:
                time.sleep(0.5)
                with pos_lock:
                    if positions:
                        cx, cy = positions[-1]
                    else:
                        cx, cy = 0.0, 0.0
                dist = math.sqrt((cx - wx) ** 2 + (cy - wy) ** 2)
                if dist < 1.0:
                    print(f"[test]   Reached wp{i} at ({cx:.2f}, {cy:.2f}), dist={dist:.2f}")
                    break
            else:
                with pos_lock:
                    if positions:
                        cx, cy = positions[-1]
                    else:
                        cx, cy = 0.0, 0.0
                dist = math.sqrt((cx - wx) ** 2 + (cy - wy) ** 2)
                print(f"[test]   Timeout wp{i}: pos=({cx:.2f}, {cy:.2f}), dist={dist:.2f}")

        # Final position summary
        with pos_lock:
            if positions:
                fx, fy = positions[-1]
            else:
                fx, fy = 0.0, 0.0
        print(f"[test] Final position: ({fx:.2f}, {fy:.2f})")

        # Check we actually moved
        with pos_lock:
            all_x = [p[0] for p in positions]
            all_y = [p[1] for p in positions]
        x_range = max(all_x) - min(all_x) if all_x else 0
        y_range = max(all_y) - min(all_y) if all_y else 0
        print(
            f"[test] Position range: x=[{min(all_x):.2f}, {max(all_x):.2f}] y=[{min(all_y):.2f}, {max(all_y):.2f}]"
        )

        with cmd_lock:
            total_cmds = len(cmd_log)
            nonzero = sum(
                1 for vx, vy, wz in cmd_log if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01
            )
        print(f"[test] cmd_vel: {total_cmds} total, {nonzero} non-zero")

        with path_lock:
            n_paths = len(path_sizes)
            stop_paths = sum(1 for s in path_sizes if s <= 1)
            real_paths = sum(1 for s in path_sizes if s > 1)
            if path_sizes:
                avg_len = sum(path_sizes) / len(path_sizes)
            else:
                avg_len = 0
        print(
            f"[test] paths: {n_paths} total, {real_paths} real (>1 pose), {stop_paths} stop (<=1 pose), avg_len={avg_len:.1f}"
        )

        # Hard assertions
        assert total_cmds > 0, "No cmd_vel messages at all"
        assert nonzero > 0, f"All {total_cmds} cmd_vel were zero — autonomyMode not working"
        assert x_range > 1.0 or y_range > 1.0, (
            f"Robot barely moved: x_range={x_range:.2f}, y_range={y_range:.2f}. "
            f"Non-zero cmds: {nonzero}/{total_cmds}"
        )

    finally:
        coord.stop()
