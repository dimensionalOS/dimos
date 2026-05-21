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

"""Rosbag accuracy test: replays scan+odom at original timing, validates PGO outputs."""

from __future__ import annotations

import time
from typing import Any

import numpy as np
import pytest
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO
from dimos.navigation.nav_stack.tests.rosbag_fixtures import (
    RosbagScanOdomPlaybackModule,
    load_rosbag_window,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

pytestmark = [pytest.mark.self_hosted, pytest.mark.skipif_no_nix, pytest.mark.skipif_macos_bug]

POST_FEED_DRAIN_SEC = 3.0
POLL_INTERVAL_SEC = 0.25


class PgoOutputCollectorModule(Module):
    """Captures PGO's corrected_odometry, global_map, and corrected_tf streams."""

    corrected_odometry: In[Odometry]
    global_map: In[PointCloud2]
    corrected_tf: In[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._corrected_positions: list[list[float]] = []
        self._global_map_point_counts: list[int] = []
        self._last_tf_translation: list[float] = [0.0, 0.0, 0.0]
        self._tf_count: int = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.corrected_odometry.subscribe(self._on_corrected_odometry))
        )
        self.register_disposable(Disposable(self.global_map.subscribe(self._on_global_map)))
        self.register_disposable(Disposable(self.corrected_tf.subscribe(self._on_corrected_tf)))

    def _on_corrected_odometry(self, message: Odometry) -> None:
        self._corrected_positions.append(
            [message.pose.position.x, message.pose.position.y, message.pose.position.z]
        )

    def _on_global_map(self, message: PointCloud2) -> None:
        points, _ = message.as_numpy()
        if points is not None:
            self._global_map_point_counts.append(len(points))

    def _on_corrected_tf(self, message: Odometry) -> None:
        self._last_tf_translation = [
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ]
        self._tf_count += 1

    @rpc
    def corrected_positions(self) -> list[list[float]]:
        return list(self._corrected_positions)

    @rpc
    def global_map_point_counts(self) -> list[int]:
        return list(self._global_map_point_counts)

    @rpc
    def tf_count(self) -> int:
        return self._tf_count

    @rpc
    def last_tf_translation(self) -> list[float]:
        return list(self._last_tf_translation)


class TestPGORosbag:
    """Validate PGO native module accuracy against OG nav stack recording."""

    def test_pgo_corrected_odometry(self) -> None:
        """Feed scan + odom at original timing and validate PGO outputs.

        Checks:
        - PGO produces corrected odometry messages
        - Corrected odometry tracks the input trajectory (no wild divergence)
        - Global map is published with non-zero points
        - TF corrections are published
        """
        window = load_rosbag_window()
        assert len(window.scans) > 0, "No scans in rosbag fixture"
        assert len(window.odom) > 0, "No odometry in rosbag fixture"

        playback_blueprint = RosbagScanOdomPlaybackModule.blueprint()
        # Config params matching pgo_unity_sim.yaml.
        pgo_blueprint = PGO.blueprint(
            key_pose_delta_trans=0.5,
            loop_search_radius=1.0,
            loop_time_thresh=60.0,
            loop_score_thresh=0.15,
            loop_submap_half_range=5,
            submap_resolution=0.1,
            min_loop_detect_duration=5.0,
            global_map_voxel_size=0.1,
            global_map_publish_rate=1.0,
            unregister_input=True,
        )
        collector_blueprint = PgoOutputCollectorModule.blueprint()

        blueprint = autoconnect(playback_blueprint, pgo_blueprint, collector_blueprint)
        coordinator = ModuleCoordinator.build(blueprint)
        try:
            playback = coordinator.get_instance(RosbagScanOdomPlaybackModule)
            collector = coordinator.get_instance(PgoOutputCollectorModule)
            while not playback.is_finished():
                time.sleep(POLL_INTERVAL_SEC)
            time.sleep(POST_FEED_DRAIN_SEC)
            corrected_positions = np.array(collector.corrected_positions())
            global_map_point_counts = collector.global_map_point_counts()
            tf_count = collector.tf_count()
            last_tf_translation = collector.last_tf_translation()
        finally:
            coordinator.stop()

        # -- Analysis --
        corrected_count = len(corrected_positions)
        global_map_count = len(global_map_point_counts)

        logger.info(f"\n{'=' * 60}")
        logger.info("PGO NATIVE ROSBAG DEVIATION SCORE")
        logger.info(f"  Input scans:            {len(window.scans)}")
        logger.info(f"  Input odom messages:     {len(window.odom)}")
        logger.info(f"  Corrected odom outputs:  {corrected_count}")
        logger.info(f"  Global map outputs:      {global_map_count}")
        logger.info(f"  TF outputs:              {tf_count}")

        assert corrected_count > 0, "PGO produced no corrected odometry"
        assert global_map_count > 0, "PGO produced no global map messages"
        assert tf_count > 0, "PGO produced no TF messages"

        input_positions = window.odom[:, 1:4]

        # Corrected trajectory should be spatially close to input (no loop closures
        # expected in 60s recording, so correction should be near-identity).
        corrected_centroid = corrected_positions.mean(axis=0)
        input_centroid = input_positions.mean(axis=0)
        centroid_error = float(np.linalg.norm(corrected_centroid - input_centroid))

        # PGO shouldn't collapse the trajectory to a point or explode it.
        corrected_extent = corrected_positions.max(axis=0) - corrected_positions.min(axis=0)
        input_extent = input_positions.max(axis=0) - input_positions.min(axis=0)
        extent_ratio_xy = float(
            np.linalg.norm(corrected_extent[:2]) / max(np.linalg.norm(input_extent[:2]), 1e-6)
        )

        mean_map_points = (
            float(np.mean(global_map_point_counts)) if global_map_point_counts else 0.0
        )
        last_map_points = global_map_point_counts[-1] if global_map_point_counts else 0

        # TF should be near-identity for a short recording without loop closures.
        tf_translation_norm = float(np.linalg.norm(last_tf_translation))

        logger.info(f"  Centroid error:           {centroid_error:.3f} m")
        logger.info(f"  Extent ratio (XY):        {extent_ratio_xy:.3f}")
        logger.info(f"  Mean global map points:   {mean_map_points:.0f}")
        logger.info(f"  Last global map points:   {last_map_points}")
        logger.info(f"  Final TF translation:     {tf_translation_norm:.4f} m")
        logger.info(f"{'=' * 60}\n")

        assert centroid_error < 5.0, (
            f"Corrected trajectory centroid too far from input: {centroid_error:.3f} m"
        )
        assert extent_ratio_xy > 0.5, (
            f"Corrected trajectory collapsed (extent ratio {extent_ratio_xy:.3f})"
        )
        assert extent_ratio_xy < 2.0, (
            f"Corrected trajectory exploded (extent ratio {extent_ratio_xy:.3f})"
        )
        assert last_map_points > 0, "Final global map has zero points"
        assert mean_map_points > 100, f"Global map too sparse: mean {mean_map_points:.0f} points"
