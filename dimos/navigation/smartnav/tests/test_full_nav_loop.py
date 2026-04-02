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

"""Integration test: full navigation closed loop.

Verifies that synthetic lidar + odometry data flows through the entire
SmartNav pipeline and produces autonomous navigation output:

    [MockSensor] → registered_scan + odometry
        → [SensorScanGeneration] → sensor_scan
        → [TerrainAnalysis] → terrain_map
        → [LocalPlanner] → path
        → [PathFollower] → cmd_vel

Requires built C++ native binaries (nix build).
"""

from __future__ import annotations

from pathlib import Path
import platform
import threading
import time
from typing import Any

import numpy as np
import pytest

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Skip conditions
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


def _make_flat_ground_cloud() -> np.ndarray:
    """Nx3 flat ground cloud around origin."""
    step = 2.0
    xs = np.arange(-10, 10, step)
    ys = np.arange(-10, 10, step)
    xx, yy = np.meshgrid(xs, ys)
    return np.column_stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)]).astype(np.float32)


class MockSensorConfig(ModuleConfig):
    rate: float = 5.0


class MockSensor(Module[MockSensorConfig]):
    """Publishes synthetic lidar + odometry at fixed rate."""

    default_config = MockSensorConfig
    registered_scan: Out[PointCloud2]
    odometry: Out[Odometry]

    def __init__(self, **kwargs):  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._running = False
        self._thread: threading.Thread | None = None

    def __getstate__(self) -> dict[str, Any]:
        state = super().__getstate__()
        state.pop("_thread", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._thread = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        super().stop()

    def _loop(self) -> None:
        dt = 1.0 / self.config.rate
        while self._running:
            now = time.time()
            self.registered_scan._transport.publish(
                PointCloud2.from_numpy(_make_flat_ground_cloud(), frame_id="map", timestamp=now)
            )
            quat = Quaternion(0.0, 0.0, 0.0, 1.0)
            self.odometry._transport.publish(
                Odometry(
                    ts=now,
                    frame_id="map",
                    child_frame_id="sensor",
                    pose=Pose(
                        position=[0.0, 0.0, 0.75],
                        orientation=[quat.x, quat.y, quat.z, quat.w],
                    ),
                    twist=Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0]),
                )
            )
            self.tf.publish(
                Transform(
                    translation=Vector3(0.0, 0.0, 0.75),
                    rotation=quat,
                    frame_id="map",
                    child_frame_id="sensor",
                    ts=now,
                ),
            )
            time.sleep(dt)


def test_full_nav_closed_loop():
    """End-to-end: synthetic data -> terrain_map + path + cmd_vel produced."""
    from dimos.core.blueprints import autoconnect
    from dimos.msgs.geometry_msgs.PointStamped import PointStamped
    from dimos.navigation.smartnav.modules.local_planner.local_planner import LocalPlanner
    from dimos.navigation.smartnav.modules.path_follower.path_follower import PathFollower
    from dimos.navigation.smartnav.modules.sensor_scan_generation.sensor_scan_generation import (
        SensorScanGeneration,
    )
    from dimos.navigation.smartnav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis

    terrain_maps: list = []
    paths: list = []
    cmd_vels: list = []
    lock = threading.Lock()

    blueprint = autoconnect(
        MockSensor.blueprint(),
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
        lambda m: (lock.acquire(), terrain_maps.append(m), lock.release())
    )
    planner.path._transport.subscribe(lambda m: (lock.acquire(), paths.append(m), lock.release()))
    follower.cmd_vel._transport.subscribe(
        lambda m: (lock.acquire(), cmd_vels.append(m), lock.release())
    )

    # Send waypoint after warmup
    def _send_waypoint() -> None:
        time.sleep(3.0)
        lp = coordinator.get_instance(LocalPlanner)
        wp = PointStamped(x=5.0, y=0.0, z=0.0, frame_id="map")
        lp.way_point._transport.publish(wp)

    wp_thread = threading.Thread(target=_send_waypoint, daemon=True)
    wp_thread.start()

    try:
        coordinator.start()

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            with lock:
                done = len(terrain_maps) > 0 and len(paths) > 0 and len(cmd_vels) > 0
            if done:
                break
            time.sleep(0.5)

        with lock:
            assert len(terrain_maps) > 0, "TerrainAnalysis produced no terrain_map"
            assert len(paths) > 0, "LocalPlanner produced no path"
            assert len(cmd_vels) > 0, "PathFollower produced no cmd_vel"
    finally:
        coordinator.stop()
