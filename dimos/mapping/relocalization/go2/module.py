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

"""Go2 relocalization: its own strategy, plus the premap merged into the live map."""

import time

import numpy as np
from reactivex import combine_latest, operators as ops

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.mapping.relocalization.go2.relocalize import relocalize
from dimos.mapping.relocalization.module import Config, RelocalizationModule
from dimos.mapping.voxels.grid import VoxelGrid
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


class Go2Config(Config):
    reloc_interval: float = 2.0
    # The L1 needs a well grown global map before walls outweigh noise
    min_local_points: int = 50_000
    # Wall-only fine inlier ratio, not open3d's own fitness
    fitness_threshold: float = 0.45
    use_carving: bool = True


class Go2Relocalization(RelocalizationModule):
    """L1 strategy: match the accumulated global map, score on walls with the floor cut off."""

    config: Go2Config
    global_map: In[PointCloud2]
    merged_map: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        if self.premap is None:
            return
        self.register_disposable(
            backpressure(
                self.global_map.observable().pipe(  # type: ignore[no-untyped-call]
                    ops.throttle_first(self.config.reloc_interval),
                    ops.filter(self._has_enough_points),
                )
            ).subscribe(self._relocalize)
        )
        self.register_disposable(
            backpressure(
                combine_latest(
                    self.global_map.observable(),  # type: ignore[no-untyped-call]
                    self.fixes,
                )
            ).subscribe(self._on_merge_input)
        )

    def _has_enough_points(self, msg: PointCloud2) -> bool:
        return len(msg) >= self.config.min_local_points

    def _relocalize(self, msg: PointCloud2) -> None:
        assert self.premap is not None
        # Checked here, not upstream, so an attempt queued behind the fix is dropped
        if not self.keep_relocalizing():
            return
        t0 = time.monotonic()
        try:
            T, fitness = relocalize(self.premap.pointcloud, msg.pointcloud)
        except Exception:
            logger.exception("relocalize() failed")
            return
        dt = time.monotonic() - t0
        if fitness < self.config.fitness_threshold:
            logger.info(
                f"relocalize go2: refused fitness={fitness:.3f} < {self.config.fitness_threshold} "
                f"after {dt:.1f}s n_pts={len(msg)}"
            )
            return
        logger.info(f"relocalize go2: fitness={fitness:.3f} time_cost={dt:.1f}s n_pts={len(msg)}")
        # T places the live map into the premap; the fix is its inverse.
        placement = Transform.from_matrix(
            np.asarray(T), frame_id=self.config.map_frame, child_frame_id=self.config.world_frame
        )
        self.submit(placement.inverse(), "go2")

    def _on_merge_input(self, pair: tuple[PointCloud2, Transform]) -> None:
        local, tf = pair
        assert self.premap is not None
        premap_in_world = self.premap.transform(tf)
        if self.config.use_carving:
            grid = VoxelGrid(carve_columns=True, frame_id=local.frame_id, show_startup_log=False)
            try:
                grid.add_frame(premap_in_world)
                grid.add_frame(local)
                self.merged_map.publish(grid.get_global_pointcloud2())
            finally:
                grid.dispose()
        else:
            self.merged_map.publish(local + premap_in_world)
