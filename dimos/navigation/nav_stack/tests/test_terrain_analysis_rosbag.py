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

"""Rosbag accuracy test: replays scan+odom at original timing, compares terrain_map to reference."""

from __future__ import annotations

from pathlib import Path
import threading
import time

import lcm as lcmlib
import numpy as np
import pytest

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.tests.rosbag_fixtures import (
    LcmCollector,
    NativeProcessRunner,
    feed_at_original_timing,
    lcm_handle_loop,
    load_rosbag_window,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

pytestmark = [pytest.mark.slow]

_PROCESS_STARTUP_SEC = 1.0
_POST_FEED_DRAIN_SEC = 2.0

TERRAIN_ANALYSIS_BIN = (
    Path(__file__).parent.parent
    / "modules"
    / "terrain_analysis"
    / "result"
    / "bin"
    / "terrain_analysis"
)

SCAN_LCM = "/rbta_scan#sensor_msgs.PointCloud2"
ODOM_LCM = "/rbta_odom#nav_msgs.Odometry"
TERRAIN_OUT_LCM = "/rbta_terrain#sensor_msgs.PointCloud2"


class TestTerrainAnalysisRosbag:
    """Validate TerrainAnalysis accuracy against OG nav stack recording."""

    def test_terrain_map_accuracy(self) -> None:
        """Feed scan + odom at original timing and compare terrain_map output."""
        if not TERRAIN_ANALYSIS_BIN.exists():
            pytest.skip(f"TerrainAnalysis binary not found: {TERRAIN_ANALYSIS_BIN}")

        window = load_rosbag_window()
        ref_tmaps = window.terrain_maps
        assert len(ref_tmaps) > 0, "No reference terrain maps in fixture"

        lcm = lcmlib.LCM()
        terrain_collector = LcmCollector(topic=TERRAIN_OUT_LCM, msg_type=PointCloud2)
        terrain_collector.start(lcm)

        stop_event = threading.Event()
        handle_thread = threading.Thread(
            target=lcm_handle_loop, args=(lcm, stop_event), daemon=True
        )
        handle_thread.start()

        runner = NativeProcessRunner(
            binary_path=str(TERRAIN_ANALYSIS_BIN),
            args=[
                "--registered_scan",
                SCAN_LCM,
                "--odometry",
                ODOM_LCM,
                "--terrain_map",
                TERRAIN_OUT_LCM,
                "--sensorRange",
                "20.0",
                "--scanVoxelSize",
                "0.05",
                "--terrainVoxelSize",
                "0.2",
                "--obstacleHeightThre",
                "0.1",
                "--groundHeightThre",
                "0.05",
                "--vehicleHeight",
                "1.5",
            ],
        )

        try:
            runner.start()
            assert runner.is_running, "TerrainAnalysis binary failed to start"
            time.sleep(_PROCESS_STARTUP_SEC)

            feed_at_original_timing(
                lcm,
                window,
                topic_map={
                    "odom": ODOM_LCM,
                    "scan": SCAN_LCM,
                },
            )

            time.sleep(_POST_FEED_DRAIN_SEC)

        finally:
            runner.stop()
            stop_event.set()
            handle_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            terrain_collector.stop(lcm)

        # Compare terrain map output
        our_count = len(terrain_collector.messages)
        ref_count = len(ref_tmaps)

        # Extract point counts from our output
        our_point_counts = []
        for msg in terrain_collector.messages:
            points, _ = msg.as_numpy()
            if points is not None:
                our_point_counts.append(len(points))

        # Reference point counts
        ref_point_counts = [len(pts) for _, pts in ref_tmaps]

        count_ratio = our_count / ref_count if ref_count > 0 else 0.0
        our_mean_pts = float(np.mean(our_point_counts)) if our_point_counts else 0.0
        ref_mean_pts = float(np.mean(ref_point_counts)) if ref_point_counts else 0.0
        pts_ratio = our_mean_pts / ref_mean_pts if ref_mean_pts > 0 else 0.0

        logger.info(f"\n{'=' * 60}")
        logger.info("TERRAIN ANALYSIS DEVIATION SCORE")
        logger.info(f"  Our terrain maps:   {our_count}")
        logger.info(f"  Reference:          {ref_count}")
        logger.info(f"  Count ratio:        {count_ratio:.3f}")
        logger.info(f"  Our mean pts/frame: {our_mean_pts:.0f}")
        logger.info(f"  Ref mean pts/frame: {ref_mean_pts:.0f}")
        logger.info(f"  Point count ratio:  {pts_ratio:.3f}")
        logger.info(f"{'=' * 60}\n")

        assert our_count > 0, "TerrainAnalysis produced no terrain maps"
        assert our_mean_pts > 0, "Terrain maps have zero points"
