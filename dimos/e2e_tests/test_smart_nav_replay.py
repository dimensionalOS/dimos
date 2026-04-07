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

"""Integration test for the unitree_go2_smart_nav blueprint using replay data.

Builds the smart_nav pipeline (GO2Connection → OdomAdapter → PGO → CostMapper →
ReplanningAStarPlanner) in replay mode and verifies that data flows end-to-end:
  - PGO receives scans and odom, publishes corrected_odometry + global_map
  - CostMapper receives global_map, publishes global_costmap
"""

from __future__ import annotations

import threading
import time

import pytest

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.costmapper import CostMapper
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.smart_nav.modules.odom_adapter.odom_adapter import OdomAdapter
from dimos.navigation.smart_nav.modules.pgo.pgo import PGO
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection


@pytest.fixture(autouse=True)
def _ci_env(monkeypatch):
    monkeypatch.setenv("CI", "1")


@pytest.fixture()
def smart_nav_coordinator():
    """Build the smart_nav blueprint in replay mode (no planner — just PGO + CostMapper)."""
    global_config.update(
        viewer="none",
        replay=True,
        replay_dir="go2_sf_office",
        n_workers=1,
    )

    # Minimal pipeline: GO2Connection → OdomAdapter → PGO → CostMapper
    # Skip ReplanningAStarPlanner and WavefrontFrontierExplorer to avoid
    # needing a goal and cmd_vel sink.
    bp = (
        autoconnect(
            unitree_go2_basic,
            PGO.blueprint(),
            OdomAdapter.blueprint(),
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
    """Integration tests for the smart_nav pipeline using replay data."""

    def test_pgo_produces_corrected_odometry(self, smart_nav_coordinator):
        """PGO should receive odom+scans via OdomAdapter and publish corrected_odometry."""
        coord = smart_nav_coordinator

        # Find the PGO module instance
        pgo_mod = None
        for mod in coord.all_modules:
            if isinstance(mod, PGO):
                pgo_mod = mod
                break
        assert pgo_mod is not None, "PGO module not found in coordinator"

        # Subscribe to corrected_odometry output
        collector = _StreamCollector()
        pgo_mod.corrected_odometry._transport.subscribe(collector.callback)

        # Start the system — replay data flows automatically
        coord.start()

        # Wait for PGO to produce at least 3 corrected odometry messages
        assert collector.wait(count=3, timeout=30), (
            f"PGO did not produce enough corrected_odometry messages "
            f"(got {len(collector.messages)})"
        )

        # Verify the messages are Odometry with reasonable values
        msg = collector.messages[0]
        assert isinstance(msg, Odometry), f"Expected Odometry, got {type(msg)}"
        assert msg.frame_id == "map"

    def test_pgo_produces_global_map(self, smart_nav_coordinator):
        """PGO should accumulate keyframes and publish a global map."""
        coord = smart_nav_coordinator

        pgo_mod = None
        for mod in coord.all_modules:
            if isinstance(mod, PGO):
                pgo_mod = mod
                break
        assert pgo_mod is not None

        collector = _StreamCollector()
        pgo_mod.global_map._transport.subscribe(collector.callback)

        coord.start()

        # Global map publishes less frequently — wait longer
        assert collector.wait(count=1, timeout=60), (
            f"PGO did not produce a global_map (got {len(collector.messages)})"
        )

        msg = collector.messages[0]
        assert isinstance(msg, PointCloud2), f"Expected PointCloud2, got {type(msg)}"
        pts, _ = msg.as_numpy()
        assert len(pts) > 0, "Global map should contain points"

    def test_costmapper_produces_costmap(self, smart_nav_coordinator):
        """CostMapper should receive global_map from PGO and produce a costmap."""
        coord = smart_nav_coordinator

        from dimos.mapping.costmapper import CostMapper

        cm_mod = None
        for mod in coord.all_modules:
            if isinstance(mod, CostMapper):
                cm_mod = mod
                break
        assert cm_mod is not None, "CostMapper module not found in coordinator"

        collector = _StreamCollector()
        cm_mod.global_costmap._transport.subscribe(collector.callback)

        coord.start()

        assert collector.wait(count=1, timeout=60), (
            f"CostMapper did not produce a global_costmap (got {len(collector.messages)})"
        )

        msg = collector.messages[0]
        assert isinstance(msg, OccupancyGrid), f"Expected OccupancyGrid, got {type(msg)}"

    def test_odom_adapter_converts_bidirectionally(self, smart_nav_coordinator):
        """OdomAdapter should convert PoseStamped→Odometry and Odometry→PoseStamped."""
        coord = smart_nav_coordinator

        from dimos.navigation.smart_nav.modules.odom_adapter.odom_adapter import OdomAdapter

        adapter = None
        for mod in coord.all_modules:
            if isinstance(mod, OdomAdapter):
                adapter = mod
                break
        assert adapter is not None, "OdomAdapter not found in coordinator"

        # Collect outputs from both directions
        odom_out = _StreamCollector()
        ps_out = _StreamCollector()
        adapter.odometry._transport.subscribe(odom_out.callback)
        adapter.odom._transport.subscribe(ps_out.callback)

        coord.start()

        # OdomAdapter.odometry (PoseStamped→Odometry) should fire from replay odom
        assert odom_out.wait(count=3, timeout=30), (
            f"OdomAdapter did not produce Odometry output (got {len(odom_out.messages)})"
        )
        assert isinstance(odom_out.messages[0], Odometry)

        # OdomAdapter.odom (Odometry→PoseStamped) fires when PGO publishes corrected_odometry
        assert ps_out.wait(count=1, timeout=30), (
            f"OdomAdapter did not produce PoseStamped output (got {len(ps_out.messages)})"
        )
        assert isinstance(ps_out.messages[0], PoseStamped)
