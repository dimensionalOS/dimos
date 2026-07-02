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

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper, local_bounds
from dimos.memory2.transform import Transformer
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from numpy.typing import NDArray

    from dimos.memory2.type.observation import Observation

logger = setup_logger()

DEFAULT_SHADOW_DEPTH = 0.2


class RayTraceMap(Transformer[PointCloud2, PointCloud2]):
    """Accumulate lidar into a voxel map with raycast clearing.

    Each cloud is sensor-frame and registered into the world by its odometry pose.
    """

    def __init__(
        self,
        *,
        voxel_size: float = 0.1,
        max_range: float = 30.0,
        emit_every: int = 1,
        region_percentile: float = 95.0,
        **mapper_kwargs: Any,
    ) -> None:
        if emit_every < 0:
            raise ValueError(f"emit_every must be >= 0, got {emit_every}")
        self.voxel_size = voxel_size
        self.max_range = max_range
        self.emit_every = emit_every
        self.region_percentile = region_percentile
        self._mapper_kwargs = mapper_kwargs

    def _local_bounds(
        self,
        batch_points: list[NDArray[np.float32]],
        batch_origins: list[tuple[float, float, float]],
        last_obs: Observation[PointCloud2],
    ) -> tuple[float, float, float, float, float]:
        """Robot-centered cylinder sized to a percentile of the observed points.

        An empty batch yields a zero-radius region at the robot.
        """
        if not batch_origins:
            pose = last_obs.pose_tuple
            assert pose is not None, "poseless obs are skipped upstream"
            rx, ry, rz = pose[:3]
            return rx, ry, 0.0, rz, rz

        points = np.concatenate(batch_points, axis=0)
        origins = np.asarray(batch_origins, dtype=np.float32)
        margin = self._mapper_kwargs.get("shadow_depth", DEFAULT_SHADOW_DEPTH) + self.voxel_size
        return local_bounds(points, origins, self.region_percentile, margin)

    def _make_obs(
        self,
        mapper: VoxelRayMapper,
        last_obs: Observation[PointCloud2],
        count: int,
        batch_points: list[NDArray[np.float32]],
        batch_origins: list[tuple[float, float, float]],
    ) -> Observation[PointCloud2]:
        tags = {**last_obs.tags, "frame_count": count}
        cx, cy, radius, z_min, z_max = self._local_bounds(batch_points, batch_origins, last_obs)
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
        mapper = VoxelRayMapper(
            voxel_size=self.voxel_size, max_range=self.max_range, **self._mapper_kwargs
        )
        last_obs: Observation[PointCloud2] | None = None
        count = 0
        batch_points: list[NDArray[np.float32]] = []
        batch_origins: list[tuple[float, float, float]] = []

        for obs in upstream:
            if obs.pose_tuple is None:
                logger.debug("RayTraceMap: obs %s has no pose; skipping", obs.id)
                continue
            x, y, z, qx, qy, qz, qw = obs.pose_tuple
            # Sensor-frame cloud: register into the world by the odom pose.
            # Apply it to the f32 array directly to skip an Open3D float64 round-trip.
            mat = Transform(
                translation=Vector3(x, y, z), rotation=Quaternion(qx, qy, qz, qw)
            ).to_matrix()
            rot = mat[:3, :3].astype(np.float32)
            trans = mat[:3, 3].astype(np.float32)
            pts = obs.data.points_f32() @ rot.T + trans
            mapper.add_frame(pts, (x, y, z))
            if pts.size:
                batch_points.append(pts)
                batch_origins.append((x, y, z))
            last_obs = obs
            count += 1

            if self.emit_every > 0 and count % self.emit_every == 0:
                yield self._make_obs(mapper, last_obs, count, batch_points, batch_origins)
                batch_points = []
                batch_origins = []

        if last_obs is not None and (self.emit_every == 0 or count % self.emit_every != 0):
            yield self._make_obs(mapper, last_obs, count, batch_points, batch_origins)
