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

class MLSPlanner:
    """Multi-level surface path planner over a voxelized global map."""

    def __init__(
        self,
        *,
        voxel_size: float,
        robot_height: float,
        max_overhead_m: float = 2.0,
        surface_closing_radius: float = 0.3,
        node_spacing_m: float = 1.0,
        wall_clearance_m: float = 0.1,
        wall_buffer_m: float = 0.75,
        wall_buffer_weight: float = 100.0,
        step_threshold_m: float = 0.16,
        max_surface_slope_deg: float = 45.0,
        step_penalty_weight: float = 4.0,
        worker_threads: int = 4,
    ) -> None: ...
    def update_global_map(self, points: NDArray[np.float32]) -> None:
        """Voxelize the map and rebuild surfaces, nodes, and edges.

        Points are (N, 3) float32 voxel centers, or (N, 7) with each voxel's
        plane as VoxelRayMapper.local_surfaces emits it.
        """
        ...

    def update_region(
        self,
        points: NDArray[np.float32],
        origin: tuple[float, float],
        radius: float,
        z_min: float,
        z_max: float,
        sensor_z: float,
    ) -> None:
        """Replace the cylindrical region with a local map slice and rebuild.

        Points are (N, 3) or (N, 7) as for update_global_map. z_max is capped at
        sensor_z + max_overhead_m.
        """
        ...

    def update_seed_region(
        self,
        points: NDArray[np.float32],
        origin: tuple[float, float],
        radius: float,
        z_min: float,
        z_max: float,
    ) -> None:
        """Apply one region of a seeded map through the region pipeline.

        Points are (N, 3) or (N, 7) as for update_global_map. The z band is the
        premap's own, uncapped.
        """
        ...

    def surface_map(self) -> NDArray[np.float32]:
        """Standable surface cells as (M, 3) float32 column centers at their surface height."""
        ...

    def surface_clearance_map(self) -> NDArray[np.float32]:
        """Surface cells as (M, 4) float32 rows of [x, y, z, clearance].

        Clearance is the horizontal distance to the nearest untraversable edge.
        Unreached cells report +inf.
        """
        ...

    def nodes(self) -> NDArray[np.float32]:
        """Graph node positions as (K, 3) float32."""
        ...

    def node_edges(self) -> NDArray[np.float32]:
        """Edge segments as (E, 7) float32 rows of [x0, y0, z0, x1, y1, z1, cost]."""
        ...

    def plan(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> NDArray[np.float32] | None:
        """Plan a path between start and goal. Returns (W, 3) float32, or None if unreachable."""
        ...

    def voxel_count(self) -> int:
        """Number of occupied voxels in the current map."""
        ...

    def voxel_map(self) -> NDArray[np.float32]:
        """Accumulated occupied voxel centers as (N, 3) float32, for visualization."""
        ...

    def clear(self) -> None:
        """Drop the graph and buffered state."""
        ...

    def __repr__(self) -> str: ...

__all__ = ["MLSPlanner"]
