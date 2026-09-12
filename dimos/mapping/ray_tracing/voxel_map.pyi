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
        fine_divisor: int = 3,
        ray_subsample: int = 1,
        shadow_depth: float = 0.1,
        grace_depth: float = 0.2,
        min_health: int = -1,
        max_health: int = 5,
        graze_cos: float = 0.7,
        support_min: int = 4,
        region_percentile: float = 95.0,
        worker_threads: int = 4,
        emit_every: int = 0,
    ) -> None: ...
    @property
    def voxel_size(self) -> float: ...
    @property
    def shadow_depth(self) -> float: ...
    def add_frame(
        self,
        points: NDArray[np.float32],
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
    ) -> None:
        """Register a sensor-frame cloud by the pose and fold it into the map.

        Points are (N, 3) float32. Orientation is an (x, y, z, w) quaternion.
        """
        ...

    def add_frame_world(
        self,
        points: NDArray[np.float32],
        origin: tuple[float, float, float],
    ) -> None:
        """Fold an already world-frame cloud into the map, raycasting from origin."""
        ...

    def seed_points(self, points: NDArray[np.float32]) -> int:
        """Bulk-seed a world-frame map cloud, creating only absent voxels.

        Points are (N, 3) float32. Returns how many voxels were created.
        """
        ...

    def registered_points(self) -> NDArray[np.float32]:
        """Return the last frame's registered points as (N, 3) float32."""
        ...

    def take_local_bounds(self) -> tuple[float, float, float, float, float]:
        """Cylinder over the frames batched since the last call.

        Returns (cx, cy, radius, z_min, z_max) and consumes the batch.
        Frames batch only when emit_every is nonzero.
        """
        ...

    def global_map(self) -> NDArray[np.float32]:
        """Return the centers of all healthy voxels as (M, 3) float32."""
        ...

    def full_map(self) -> NDArray[np.float32]:
        """Return the support-gated snapshot of the whole map as (M, 3) float32."""
        ...

    def global_map_normals(self) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
        """Return healthy voxel centers and their surface normals, both (M, 3) float32.

        Matching order. The normal is the zero vector where the voxel has no plane.
        """
        ...

    def global_map_normal_fits(
        self,
    ) -> tuple[NDArray[np.float32], NDArray[np.float32], NDArray[np.float32]]:
        """global_map_normals with freshly recomputed fits plus smallest eigenvalues (M,).

        Whole-map refit cost. Visualization only.
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

    def local_map_fine(
        self,
        origin: tuple[float, float, float],
        radius: float,
        z_min: float,
        z_max: float,
    ) -> NDArray[np.float32]:
        """Return fine-cell centers inside the cylinder as (M, 3) float32.

        Raises ValueError when fine_divisor is not set.
        """
        ...

    def voxel_count(self) -> int:
        """Number of healthy voxels currently in the map."""
        ...

    def clear(self) -> None:
        """Reset the map to empty."""
        ...

    def __len__(self) -> int: ...
    def __repr__(self) -> str: ...

__all__ = ["VoxelRayMapper"]
