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

"""Publish the latest mapping snapshot as one planning-world obstacle."""

from __future__ import annotations

import asyncio
from collections.abc import Sequence
from typing import Protocol

import numpy as np
from pydantic import Field

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.manipulation.planning.spec.validation import MAX_OCTREE_POINTS
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

GLOBAL_MAP_OBSTACLE_ID = "mapping/global-voxel-map"


class VoxelObstacleSpec(Spec, Protocol):
    """The planning-world side of a mapping snapshot."""

    def set_voxel_obstacle(
        self,
        name: str,
        points: Sequence[Sequence[float]],
        resolution: float,
        frame_id: str = "world",
    ) -> bool: ...


class GlobalMapObstacleBridgeConfig(ModuleConfig):
    resolution: float = Field(default=0.05, gt=0.0)
    planning_frame: str = Field(default="world", min_length=1)
    max_points: int = Field(default=MAX_OCTREE_POINTS, gt=0)


class GlobalMapObstacleBridge(Module):
    """Reconcile complete mapper snapshots through a stable obstacle ID."""

    config: GlobalMapObstacleBridgeConfig  # type: ignore[assignment]
    global_map: In[PointCloud2]
    _planning: VoxelObstacleSpec

    async def handle_global_map(self, cloud: PointCloud2) -> None:
        """Apply this snapshot, once.

        The mapper republishes a complete map, so a failed snapshot is already
        superseded by the time it could be retried. The module dispatcher is a
        single-slot latest mailbox: retrying here would hold the newer map out
        of the planner and then install a stale one on success.
        """
        try:
            await asyncio.to_thread(self._reconcile, cloud)
        except ValueError:
            logger.exception("Rejected invalid global map at stamp %.6f", cloud.ts)
        except Exception:
            logger.exception(
                "Dropped global map at stamp %.6f; the next snapshot supersedes it",
                cloud.ts,
            )

    def _reconcile(self, cloud: PointCloud2) -> None:
        config = self.bridge_config
        if cloud.frame_id != config.planning_frame:
            raise ValueError(
                f"Global map frame '{cloud.frame_id}' does not match planning frame "
                f"'{config.planning_frame}'"
            )
        points = np.asarray(cloud.points_f32(), dtype=np.float64)
        if len(points) and not np.isfinite(points).all():
            raise ValueError("Global map contains non-finite points")
        if len(points) > config.max_points:
            # Every point crosses worker RPC as a pickled tuple. Dropping the
            # snapshot keeps the last good map in the planner rather than
            # stalling the pipe on one that will not fit.
            raise ValueError(
                f"Global map has {len(points)} points, over the max_points limit of "
                f"{config.max_points}. Shrink the mapped region or coarsen voxel_size."
            )

        # tolist() drops to C rather than building a tuple per point in python,
        # and this runs at map cadence.
        self._planning.set_voxel_obstacle(
            GLOBAL_MAP_OBSTACLE_ID,
            points.tolist(),
            config.resolution,
            config.planning_frame,
        )

    @property
    def bridge_config(self) -> GlobalMapObstacleBridgeConfig:
        return self.config  # type: ignore[return-value]


global_map_obstacle_bridge = GlobalMapObstacleBridge.blueprint
