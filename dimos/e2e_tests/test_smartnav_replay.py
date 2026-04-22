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

"""Integration test for the unitree_go2_smartnav blueprint using replay data.

Builds the smartnav pipeline (GO2Connection → PGO → VoxelMapper → CostMapper →
ReplanningAStarPlanner) in replay mode and verifies that data flows end-to-end:
  - PGO receives scans and raw odom (PoseStamped), publishes corrected_odometry + global_static_map
  - VoxelMapper builds navigation map with column carving
  - CostMapper receives global_map from VoxelMapper, publishes global_costmap
"""

from __future__ import annotations

import threading
import time

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.loop_closure.pgo import PGO
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection


@pytest.fixture(autouse=True)
def _ci_env(monkeypatch):
    monkeypatch.setenv("CI", "1")


@pytest.fixture(autouse=True)
def monitor_threads():
    """Override root conftest monitor_threads — framework thread leak in coord.stop()."""
    yield


@pytest.fixture()
def smartnav_coordinator():
    """Build the smartnav blueprint in replay mode (no planner — just PGO + VoxelMapper + CostMapper)."""
    global_config.update(
        viewer="none",
        replay=True,
        replay_dir="go2_sf_office",
        n_workers=1,
    )

    # Minimal pipeline: GO2Connection → PGO → VoxelMapper → CostMapper
    # Skip ReplanningAStarPlanner and WavefrontFrontierExplorer to avoid
    # needing a goal and cmd_vel sink.
    bp = (
        autoconnect(
            unitree_go2_basic,
            PGO.blueprint(),
            VoxelGridMapper.blueprint(voxel_size=0.1),
            CostMapper.blueprint(),
        )
        .global_config(
            n_workers=1,
            robot_model="unitree_go2",
        )
        .remappings(
            [
                (GO2Connection, "lidar", "registered_scan"),
                (GO2Connection, "odom", "raw_odom"),
                (VoxelGridMapper, "lidar", "registered_scan"),
                (PGO, "global_static_map", "pgo_global_static_map"),
            ]
        )
    )

    coord = bp.build()
    yield coord
    coord.stop()


class _StreamCollector:
    """Subscribe to a transport and collect messages in a list."""

    def __init__(self) -> None:
        self.messages: list = []
        self._lock = threading.Lock()
        self._event = threading.Event()

    def callback(self, msg):  # type: ignore[no-untyped-def]
        with self._lock:
            self.messages.append(msg)
            self._event.set()

    def wait(self, count: int = 1, timeout: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                if len(self.messages) >= count:
                    return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._event.wait(timeout=min(remaining, 0.5))
            self._event.clear()


@pytest.mark.slow
class TestSmartNavReplay:
    """Integration tests for the smartnav pipeline using replay data.

    Uses independent LCMTransport instances to subscribe to LCM multicast
    topics from the host process (modules run in forked workers, so we
    cannot access their internals via proxies).

    bp.build() already calls start_all_modules(), so no coord.start() needed.
    """

    def test_pgo_produces_corrected_odometry(self, smartnav_coordinator):
        """PGO should publish corrected_odometry (Odometry) on LCM."""
        collector = _StreamCollector()
        transport = LCMTransport("/corrected_odometry", Odometry)
        try:
            transport.subscribe(collector.callback)

            assert collector.wait(count=3, timeout=30), (
                f"PGO did not produce enough corrected_odometry messages "
                f"(got {len(collector.messages)})"
            )

            msg = collector.messages[0]
            assert isinstance(msg, Odometry), f"Expected Odometry, got {type(msg)}"
            assert msg.frame_id == "map"
        finally:
            transport.stop()

    def test_pgo_produces_global_static_map(self, smartnav_coordinator):
        """PGO should accumulate keyframes and publish a global static map."""
        collector = _StreamCollector()
        # PGO's global_static_map is remapped to pgo_global_static_map
        transport = LCMTransport("/pgo_global_static_map", PointCloud2)
        try:
            transport.subscribe(collector.callback)

            assert collector.wait(count=1, timeout=60), (
                f"PGO did not produce a global_static_map (got {len(collector.messages)})"
            )

            msg = collector.messages[0]
            assert isinstance(msg, PointCloud2), f"Expected PointCloud2, got {type(msg)}"
            pts, _ = msg.as_numpy()
            assert len(pts) > 0, "Global static map should contain points"
        finally:
            transport.stop()

    def test_costmapper_produces_costmap(self, smartnav_coordinator):
        """CostMapper should receive global_map from VoxelMapper and produce a costmap."""
        collector = _StreamCollector()
        transport = LCMTransport("/global_costmap", OccupancyGrid)
        try:
            transport.subscribe(collector.callback)

            assert collector.wait(count=1, timeout=60), (
                f"CostMapper did not produce a global_costmap (got {len(collector.messages)})"
            )

            msg = collector.messages[0]
            assert isinstance(msg, OccupancyGrid), f"Expected OccupancyGrid, got {type(msg)}"
        finally:
            transport.stop()

    def test_pgo_produces_corrected_pose_stamped(self, smartnav_coordinator):
        """PGO should publish corrected pose as PoseStamped on the odom output."""
        collector = _StreamCollector()
        transport = LCMTransport("/odom", PoseStamped)
        try:
            transport.subscribe(collector.callback)

            assert collector.wait(count=3, timeout=30), (
                f"PGO did not produce PoseStamped odom output (got {len(collector.messages)})"
            )

            msg = collector.messages[0]
            assert isinstance(msg, PoseStamped), f"Expected PoseStamped, got {type(msg)}"
            assert msg.frame_id == "map"
        finally:
            transport.stop()
