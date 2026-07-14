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

import numpy as np
from numpy.typing import NDArray

class VoxelRayMapper:
    """Voxel map with raycast clearing of dynamic objects."""

    def __init__(
        self,
        *,
        voxel_size: float,
        max_range: float,
        ray_subsample: int = 1,
        shadow_depth: float = 0.1,
        grace_depth: float = 0.2,
        min_health: int = -1,
        max_health: int = 5,
        graze_cos: float = 0.7,
        support_min: int = 4,
    ) -> None: ...
    @property
    def voxel_size(self) -> float: ...
    @property
    def shadow_depth(self) -> float: ...
    def add_frame(
        self,
        points: NDArray[np.float32],
        origin: tuple[float, float, float],
    ) -> None:
        """Update the map with a frame of lidar points. Shape (N, 3) float32."""
        ...

    def global_map(self) -> NDArray[np.float32]:
        """Return the centers of all healthy voxels as (M, 3) float32."""
        ...

    def global_map_normals(self) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
        """Return healthy voxel centers and their surface normals, both (M, 3) float32.

        Matching order. The normal is the zero vector where the voxel has no plane.
        """
        ...

    def local_map(
        self,
        origin: tuple[float, float, float],
        radius: float,
        z_min: float,
        z_max: float,
    ) -> NDArray[np.float32]:
        """Return healthy voxels inside the cylinder around origin as (M, 3) float32."""
        ...

    def voxel_count(self) -> int:
        """Number of healthy voxels currently in the map."""
        ...

    def clear(self) -> None:
        """Reset the map to empty."""
        ...

    def __len__(self) -> int: ...
    def __repr__(self) -> str: ...

def local_bounds(
    points: NDArray[np.float32],
    origins: NDArray[np.float32],
    percentile: float,
    margin: float,
) -> tuple[float, float, float, float, float]:
    """Local region a batch of frames observed, as (cx, cy, radius, z_min, z_max).

    Non-finite points are ignored.
    """
    ...

__all__ = ["VoxelRayMapper", "local_bounds"]
