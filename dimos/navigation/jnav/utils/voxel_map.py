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

"""Force a point cloud onto a discrete voxel grid for occupancy comparisons.

A ``VoxelMap`` is just the set of occupied integer voxel keys at a fixed voxel
size. ``diff`` (symmetric-difference count, lower == better aligned) and the raw
occupied ``count`` are the building blocks for map-quality scores: drift smears a
map into more, mis-aligned voxels; a good loop closure collapses it back.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class VoxelMap:
    voxel_size: float
    keys: frozenset[tuple[int, int, int]]

    @classmethod
    def from_points(cls, points: np.ndarray, voxel_size: float) -> VoxelMap:
        if points.shape[0] == 0:
            return cls(voxel_size=voxel_size, keys=frozenset())
        indices = np.floor(np.asarray(points)[:, :3] / voxel_size).astype(np.int64)
        unique = np.unique(indices, axis=0)
        return cls(voxel_size=voxel_size, keys=frozenset(map(tuple, unique)))

    @classmethod
    def from_pointcloud(cls, cloud: PointCloud2, voxel_size: float) -> VoxelMap:
        return cls.from_points(cloud.points_f32(), voxel_size)

    @property
    def count(self) -> int:
        """Number of occupied voxels."""
        return len(self.keys)

    def __len__(self) -> int:
        return len(self.keys)

    def diff(self, other: VoxelMap) -> int:
        """Symmetric-difference count: voxels occupied in exactly one of the two."""
        return len(self.keys ^ other.keys)

    def intersection(self, other: VoxelMap) -> int:
        return len(self.keys & other.keys)

    def iou(self, other: VoxelMap) -> float:
        union = len(self.keys | other.keys)
        return len(self.keys & other.keys) / union if union else 0.0
