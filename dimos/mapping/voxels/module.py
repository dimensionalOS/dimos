# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.voxels.grid import VoxelGrid
from dimos.mapping.voxels.lidar_defaults import LidarConfigName, voxel_size_for_lidar
from dimos.memory2.module import StreamModule
from dimos.memory2.stream import Stream
from dimos.memory2.transform import Transformer
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation

logger = setup_logger()


class VoxelMapTransformer(Transformer[PointCloud2, PointCloud2]):
    """Accumulate PointCloud2 observations into a global voxel map.

    Assumes input clouds are already in world frame.
    All keyword arguments except ``emit_every`` are forwarded to
    :class:`VoxelGrid`.

    Args:
        emit_every: Yield the current accumulated map every *n* frames.
            ``1`` (default) = yield after every frame (live-compatible).
            ``0`` = yield only when upstream exhausts (batch mode).
        **grid_kwargs: Forwarded to ``VoxelGrid()``.
    """

    def __init__(self, *, emit_every: int = 1, **grid_kwargs: Any) -> None:
        self.emit_every = emit_every
        self._grid_kwargs = grid_kwargs

    def _make_obs(
        self, grid: VoxelGrid, last_obs: Observation[PointCloud2], count: int
    ) -> Observation[PointCloud2]:
        # pose=None: the global map is in world frame, per-observation pose is meaningless
        cloud = grid.get_global_pointcloud2()
        n_voxels = grid.size()
        logger.info(f"live map: frames={count} voxels={n_voxels}")
        return last_obs.derive(
            data=cloud,
            pose=None,
            tags={**last_obs.tags, "frame_count": count, "voxel_count": n_voxels},
        )

    def __call__(
        self, upstream: Iterator[Observation[PointCloud2]]
    ) -> Iterator[Observation[PointCloud2]]:
        grid = VoxelGrid(**self._grid_kwargs)
        try:
            last_obs: Observation[PointCloud2] | None = None
            count = 0

            for obs in upstream:
                grid.add_frame(obs.data)
                last_obs = obs
                count += 1

                if self.emit_every > 0 and count % self.emit_every == 0:
                    yield self._make_obs(grid, last_obs, count)

            # Yield on exhaustion: always in batch mode, or if there are un-emitted frames
            if last_obs is not None and (self.emit_every == 0 or count % self.emit_every != 0):
                yield self._make_obs(grid, last_obs, count)
        finally:
            grid.dispose()


class VoxelGridMapperConfig(ModuleConfig):
    """Configuration for VoxelGridMapper."""

    # None → inherit GlobalConfig.lidar_config (``default`` / ``mid360``).
    lidar_config: LidarConfigName | None = None
    # None → derive from lidar_config (0.05 default, 0.03 for mid360).
    voxel_size: float | None = None
    block_count: int = 2_000_000
    device: str = "CUDA:0"
    carve_columns: bool = True
    frame_id: str = "world"
    emit_every: int = 1

    def resolved_voxel_size(self) -> float:
        if self.voxel_size is not None:
            return self.voxel_size
        lidar = self.lidar_config if self.lidar_config is not None else self.g.lidar_config
        return voxel_size_for_lidar(lidar)


class VoxelGridMapper(StreamModule[PointCloud2, PointCloud2]):
    """Accumulate lidar point clouds into a global voxel map."""

    config: VoxelGridMapperConfig

    def pipeline(self, stream: Stream[PointCloud2]) -> Stream[PointCloud2]:
        cfg = self.config.model_dump(
            include=set(VoxelGridMapperConfig.model_fields) - set(ModuleConfig.model_fields)
        )
        cfg.pop("lidar_config", None)
        cfg["voxel_size"] = self.config.resolved_voxel_size()
        logger.info(
            f"VoxelGridMapper voxel_size={cfg['voxel_size']} "
            f"(lidar_config={self.config.lidar_config or self.config.g.lidar_config!r})"
        )
        return stream.transform(VoxelMapTransformer(**cfg))

    lidar: In[PointCloud2]
    global_map: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()
