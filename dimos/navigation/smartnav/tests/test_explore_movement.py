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

"""Integration test: verify exploration planner produces movement.

Validates the complete explore pipeline:
    [MockVehicle] → registered_scan + odometry
        → [SensorScanGeneration] → sensor_scan
        → [TerrainAnalysis] → terrain_map
        → [TarePlanner] → way_point (exploration waypoints)
        → [LocalPlanner] → path (autonomyMode=true)
        → [PathFollower] → cmd_vel
        → [MockVehicle] (tracks position changes)

Requires built C++ native binaries (nix build).
"""

from __future__ import annotations

from dataclasses import dataclass, field
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
_REQUIRED_BINARIES = [
    ("result-terrain-analysis", "terrain_analysis"),
    ("result-local-planner", "local_planner"),
    ("result-path-follower", "path_follower"),
    ("result-tare-planner", "tare_planner"),
]
_HAS_BINARIES = all((_NATIVE_DIR / d / "bin" / name).exists() for d, name in _REQUIRED_BINARIES)
_IS_LINUX_X86 = platform.system() == "Linux" and platform.machine() in ("x86_64", "AMD64")

pytestmark = [
    pytest.mark.slow,
    pytest.mark.skipif(not _IS_LINUX_X86, reason="Native modules require Linux x86_64"),
    pytest.mark.skipif(
        not _HAS_BINARIES,
        reason="Native binaries not built (run: cd smartnav/native && nix build)",
    ),
]


# Helpers (must be at module level for pickling)


def _make_room_cloud(
    robot_x: float,
    robot_y: float,
    room_size: float = 20.0,
    wall_height: float = 2.5,
    ground_z: float = 0.0,
    density: float = 0.3,
) -> np.ndarray:
    """Generate a room point cloud: flat ground + walls on 4 sides.

    Returns Nx3 array [x, y, z] (PointCloud2.from_numpy expects Nx3).
    """
    pts = []

    step = 1.0 / density
    half = room_size / 2
    xs = np.arange(robot_x - half, robot_x + half, step)
    ys = np.arange(robot_y - half, robot_y + half, step)
    xx, yy = np.meshgrid(xs, ys)
    ground = np.column_stack(
        [
            xx.ravel(),
            yy.ravel(),
            np.full(xx.size, ground_z),
        ]
    )
    pts.append(ground)

    wall_step = 0.5
    for wall_x in [robot_x - half, robot_x + half]:
        wy = np.arange(robot_y - half, robot_y + half, wall_step)
        wz = np.arange(ground_z, ground_z + wall_height, wall_step)
        wyy, wzz = np.meshgrid(wy, wz)
        wall = np.column_stack(
            [
                np.full(wyy.size, wall_x),
                wyy.ravel(),
                wzz.ravel(),
            ]
        )
        pts.append(wall)

    for wall_y in [robot_y - half, robot_y + half]:
        wx = np.arange(robot_x - half, robot_x + half, wall_step)
        wz = np.arange(ground_z, ground_z + wall_height, wall_step)
        wxx, wzz = np.meshgrid(wx, wz)
        wall = np.column_stack(
            [
                wxx.ravel(),
                np.full(wxx.size, wall_y),
                wzz.ravel(),
            ]
        )
        pts.append(wall)

    return np.concatenate(pts, axis=0).astype(np.float32)


class MockVehicleConfig(ModuleConfig):
    rate: float = 10.0
    sim_rate: float = 50.0


class MockVehicle(Module[MockVehicleConfig]):
    """Publishes sensor data and integrates cmd_vel for position tracking."""

    default_config = MockVehicleConfig

    cmd_vel: In[Twist]
    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]

    def __init__(self, **kwargs):  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._x = 0.0
        self._y = 0.0
        self._z = 0.75
        self._yaw = 0.0
        self._fwd = 0.0
        self._left = 0.0
        self._yaw_rate = 0.0
        self._cmd_lock = threading.Lock()
        self._running = False
        self._sensor_thread: threading.Thread | None = None
        self._sim_thread: threading.Thread | None = None

    def __getstate__(self) -> dict[str, Any]:
        state = super().__getstate__()
        state.pop("_cmd_lock", None)
        state.pop("_sensor_thread", None)
        state.pop("_sim_thread", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._cmd_lock = threading.Lock()
        self._sensor_thread = None
        self._sim_thread = None

    def start(self) -> None:
        self.cmd_vel._transport.subscribe(self._on_cmd_vel)
        self._running = True
        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._sim_thread.start()
        self._sensor_thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self._sensor_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._sim_thread:
            self._sim_thread.join(timeout=3.0)
        if self._sensor_thread:
            self._sensor_thread.join(timeout=3.0)
        super().stop()

    def _on_cmd_vel(self, twist: Twist) -> None:
        with self._cmd_lock:
            self._fwd = twist.linear.x
            self._left = twist.linear.y
            self._yaw_rate = twist.angular.z

    def _sim_loop(self) -> None:
        dt = 1.0 / self.config.sim_rate
        while self._running:
            t0 = time.monotonic()
            with self._cmd_lock:
                fwd, left, yr = self._fwd, self._left, self._yaw_rate

            self._yaw += dt * yr
            cy, sy = math.cos(self._yaw), math.sin(self._yaw)
            self._x += dt * (cy * fwd - sy * left)
            self._y += dt * (sy * fwd + cy * left)

            now = time.time()
            quat = Quaternion.from_euler(Vector3(0.0, 0.0, self._yaw))
            self.odometry._transport.publish(
                Odometry(
                    ts=now,
                    frame_id="map",
                    child_frame_id="sensor",
                    pose=Pose(
                        position=[self._x, self._y, self._z],
                        orientation=[quat.x, quat.y, quat.z, quat.w],
                    ),
                    twist=Twist(
                        linear=[fwd, left, 0.0],
                        angular=[0.0, 0.0, yr],
                    ),
                )
            )
            self.tf.publish(
                Transform(
                    translation=Vector3(self._x, self._y, self._z),
                    rotation=quat,
                    frame_id="map",
                    child_frame_id="sensor",
                    ts=now,
                ),
            )

            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    def _sensor_loop(self) -> None:
        dt = 1.0 / self.config.rate
        while self._running:
            now = time.time()
            cloud_data = _make_room_cloud(self._x, self._y)
            self.registered_scan._transport.publish(
                PointCloud2.from_numpy(cloud_data, frame_id="map", timestamp=now)
            )
            time.sleep(dt)


@dataclass
class Collector:
    """Thread-safe message collector."""

    waypoints: list = field(default_factory=list)
    paths: list = field(default_factory=list)
    cmd_vels: list = field(default_factory=list)
    terrain_maps: list = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock)


# Test


def test_explore_produces_movement():
    """End-to-end: TARE planner drives robot movement via full pipeline."""
    from dimos.core.blueprints import autoconnect
    from dimos.msgs.geometry_msgs.PointStamped import PointStamped
    from dimos.msgs.nav_msgs.Path import Path as NavPath
    from dimos.navigation.smartnav.modules.local_planner.local_planner import LocalPlanner
    from dimos.navigation.smartnav.modules.path_follower.path_follower import PathFollower
    from dimos.navigation.smartnav.modules.sensor_scan_generation.sensor_scan_generation import (
        SensorScanGeneration,
    )
    from dimos.navigation.smartnav.modules.tare_planner.tare_planner import TarePlanner
    from dimos.navigation.smartnav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis

    collector = Collector()

    blueprint = autoconnect(
        MockVehicle.blueprint(),
        SensorScanGeneration.blueprint(),
        TerrainAnalysis.blueprint(),
        LocalPlanner.blueprint(autonomy_mode=True),
        PathFollower.blueprint(autonomy_mode=True),
        TarePlanner.blueprint(),
    )

    coordinator = blueprint.build()

    # Subscribe to outputs
    tare = coordinator.get_instance(TarePlanner)
    planner = coordinator.get_instance(LocalPlanner)
    follower = coordinator.get_instance(PathFollower)
    coordinator.get_instance(MockVehicle)
    terrain = coordinator.get_instance(TerrainAnalysis)

    def _on_wp(msg: PointStamped) -> None:
        with collector.lock:
            collector.waypoints.append((msg.x, msg.y, msg.z))

    def _on_terrain(msg: PointCloud2) -> None:
        with collector.lock:
            collector.terrain_maps.append(True)

    def _on_path(msg: NavPath) -> None:
        with collector.lock:
            collector.paths.append(msg)

    def _on_cmd(msg: Twist) -> None:
        with collector.lock:
            collector.cmd_vels.append((msg.linear.x, msg.linear.y, msg.angular.z))

    tare.way_point._transport.subscribe(_on_wp)
    planner.path._transport.subscribe(_on_path)
    follower.cmd_vel._transport.subscribe(_on_cmd)
    terrain.terrain_map._transport.subscribe(_on_terrain)

    try:
        coordinator.start()

        # Wait for pipeline outputs — TARE needs several scan cycles
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            with collector.lock:
                has_terrain = len(collector.terrain_maps) > 0
                has_waypoints = len(collector.waypoints) > 0
                has_paths = len(collector.paths) > 0
                has_cmds = len(collector.cmd_vels) > 0
            if has_terrain and has_waypoints and has_paths and has_cmds:
                break
            time.sleep(0.5)

        # Let movement accumulate
        time.sleep(5.0)

        # -- Assertions --
        with collector.lock:
            assert len(collector.terrain_maps) > 0, "TerrainAnalysis never produced terrain_map"

            assert len(collector.waypoints) > 0, "TarePlanner never produced a waypoint"

            assert len(collector.paths) > 0, (
                "LocalPlanner never produced a path — check that autonomyMode=true is being passed"
            )

            nonzero_cmds = [
                (vx, vy, wz)
                for vx, vy, wz in collector.cmd_vels
                if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01
            ]
            assert len(nonzero_cmds) > 0, (
                f"PathFollower produced {len(collector.cmd_vels)} cmd_vels "
                f"but ALL were zero — robot is not moving"
            )

    finally:
        coordinator.stop()
