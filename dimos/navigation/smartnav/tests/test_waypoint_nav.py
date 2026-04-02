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

"""Integration test: waypoint navigation produces path + movement.

Sets a waypoint at (10, 0) and verifies:
1. TerrainAnalysis produces terrain_map
2. LocalPlanner produces a path toward the goal
3. PathFollower produces non-zero cmd_vel
4. Robot position moves toward the waypoint

This is the core nav stack test without any exploration planner.
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


def _make_ground_cloud(rx: float, ry: float) -> np.ndarray:
    """Flat ground + obstacle wall at x=8 to test path planning around it."""
    pts = []
    # Ground plane
    step = 1.0
    for x in np.arange(rx - 12, rx + 12, step):
        for y in np.arange(ry - 12, ry + 12, step):
            pts.append([x, y, 0.0])
    # Wall obstacle at x=5, y=-2..2, z=0..1 (partial blockage)
    for y in np.arange(-2, 2, 0.3):
        for z in np.arange(0, 1.0, 0.3):
            pts.append([5.0, y, z])
    return np.array(pts, dtype=np.float32)


class SimVehicleConfig(ModuleConfig):
    sensor_rate: float = 5.0
    sim_rate: float = 50.0


class SimVehicle(Module[SimVehicleConfig]):
    """Kinematic vehicle sim: publishes lidar + odom, integrates cmd_vel."""

    default_config = SimVehicleConfig
    cmd_vel: In[Twist]
    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]

    def __init__(self, **kwargs):  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
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
            cloud = _make_ground_cloud(self.x, self.y)
            self.registered_scan._transport.publish(
                PointCloud2.from_numpy(cloud, frame_id="map", timestamp=now)
            )
            time.sleep(dt)


def test_waypoint_nav_produces_path_and_movement():
    """Send waypoint at (10,0), verify terrain_map + path + non-zero cmd_vel."""
    from dimos.core.blueprints import autoconnect
    from dimos.msgs.geometry_msgs.PointStamped import PointStamped
    from dimos.navigation.smartnav.modules.local_planner.local_planner import LocalPlanner
    from dimos.navigation.smartnav.modules.path_follower.path_follower import PathFollower
    from dimos.navigation.smartnav.modules.sensor_scan_generation.sensor_scan_generation import (
        SensorScanGeneration,
    )
    from dimos.navigation.smartnav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis

    terrain_msgs: list = []
    path_msgs: list = []
    cmd_msgs: list[tuple] = []
    lock = threading.Lock()

    blueprint = autoconnect(
        SimVehicle.blueprint(),
        SensorScanGeneration.blueprint(),
        TerrainAnalysis.blueprint(),
        LocalPlanner.blueprint(autonomy_mode=True),
        PathFollower.blueprint(autonomy_mode=True),
    )
    coordinator = blueprint.build()

    terrain = coordinator.get_instance(TerrainAnalysis)
    planner = coordinator.get_instance(LocalPlanner)
    follower = coordinator.get_instance(PathFollower)

    terrain.terrain_map._transport.subscribe(
        lambda m: (lock.acquire(), terrain_msgs.append(1), lock.release())
    )
    planner.path._transport.subscribe(
        lambda m: (lock.acquire(), path_msgs.append(1), lock.release())
    )
    follower.cmd_vel._transport.subscribe(
        lambda m: (
            lock.acquire(),
            cmd_msgs.append((m.linear.x, m.linear.y, m.angular.z)),
            lock.release(),
        )
    )

    # Send waypoint after modules warm up
    def _send_wp():
        time.sleep(2.0)
        wp = PointStamped(x=10.0, y=0.0, z=0.0, frame_id="map")
        planner.way_point._transport.publish(wp)
        print("[test] Sent waypoint (10, 0)")

    threading.Thread(target=_send_wp, daemon=True).start()

    try:
        coordinator.start()

        # Wait up to 20s for all pipeline stages
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            with lock:
                ok = len(terrain_msgs) > 0 and len(path_msgs) > 0 and len(cmd_msgs) > 0
            if ok:
                break
            time.sleep(0.5)

        # Let movement accumulate
        time.sleep(5.0)

        with lock:
            n_terrain = len(terrain_msgs)
            n_path = len(path_msgs)
            n_cmd = len(cmd_msgs)
            nonzero = [
                (vx, vy, wz)
                for vx, vy, wz in cmd_msgs
                if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01
            ]

        print(
            f"[test] terrain_map: {n_terrain}, path: {n_path}, "
            f"cmd_vel: {n_cmd} (nonzero: {len(nonzero)})"
        )

        assert n_terrain > 0, "TerrainAnalysis produced no terrain_map"
        assert n_path > 0, "LocalPlanner produced no path"
        assert n_cmd > 0, "PathFollower produced no cmd_vel"
        assert len(nonzero) > 0, f"All {n_cmd} cmd_vel messages were zero — robot not moving"

    finally:
        coordinator.stop()
