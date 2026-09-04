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

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.memory.transform import Transformer
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory.type.observation import Observation

logger = setup_logger()


class RayTraceMap(Transformer[PointCloud2, PointCloud2]):
    """Accumulate lidar into a voxel map with raycast clearing.

    Each cloud is sensor-frame and registered into the world by its odometry
    pose. The instance owns its mapper, so callers can act on it between
    pulls, and a reused instance continues the same map.
    """

    def __init__(
        self,
        *,
        voxel_size: float = 0.08,
        max_range: float = 30.0,
        emit_every: int = 1,
        **mapper_kwargs: Any,
    ) -> None:
        if emit_every < 0:
            raise ValueError(f"emit_every must be >= 0, got {emit_every}")
        self.emit_every = emit_every
        # emit_every=1 turns on frame batching. This transformer consumes it
        # with take_local_bounds on its own cadence.
        self.mapper = VoxelRayMapper(
            voxel_size=voxel_size,
            max_range=max_range,
            emit_every=1,
            **mapper_kwargs,
        )

    def _make_obs(
        self,
        mapper: VoxelRayMapper,
        last_obs: Observation[PointCloud2],
        count: int,
    ) -> Observation[PointCloud2]:
        import open3d as o3d  # type: ignore[import-untyped]
        import open3d.core as o3c  # type: ignore[import-untyped]

        tags = {**last_obs.tags, "frame_count": count}
        cx, cy, radius, z_min, z_max = mapper.take_local_bounds()
        positions = mapper.local_map((cx, cy, 0.0), radius, z_min, z_max)
        tags["region_bounds"] = (cx, cy, radius, z_min, z_max)
        pcd = o3d.t.geometry.PointCloud()
        pcd.point["positions"] = o3c.Tensor.from_numpy(positions)
        cloud = PointCloud2(pointcloud=pcd, frame_id="world", ts=last_obs.ts)
        return last_obs.derive(data=cloud, tags=tags)

    def __call__(
        self,
        upstream: Iterator[Observation[PointCloud2]],
    ) -> Iterator[Observation[PointCloud2]]:
        mapper = self.mapper
        last_obs: Observation[PointCloud2] | None = None
        count = 0

        for obs in upstream:
            if obs.pose_tuple is None:
                logger.warning("RayTraceMap: obs %s has no pose. dropped a cloud", obs.id)
                continue
            x, y, z, qx, qy, qz, qw = obs.pose_tuple
            mapper.add_frame(obs.data.points_f32(), (x, y, z), (qx, qy, qz, qw))
            last_obs = obs
            count += 1

            if self.emit_every > 0 and count % self.emit_every == 0:
                yield self._make_obs(mapper, last_obs, count)

        if last_obs is not None and (self.emit_every == 0 or count % self.emit_every != 0):
            yield self._make_obs(mapper, last_obs, count)
