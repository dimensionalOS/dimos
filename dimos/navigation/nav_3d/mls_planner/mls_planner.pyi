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
        surface_dilation_passes: int = 3,
        surface_erosion_passes: int = 3,
        node_spacing_m: float = 1.0,
        node_wall_buffer_m: float = 0.3,
        node_step_threshold_m: float = 0.25,
    ) -> None: ...
    def update_global_map(self, points: NDArray[np.float32]) -> None:
        """Voxelize the map and rebuild surfaces, nodes, and edges. Shape (N, 3) float32."""
        ...

    def surface_map(self) -> NDArray[np.float32]:
        """Standable surface cells as (M, 3) float32 centers."""
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

    def last_timings(self) -> dict[str, float]:
        """Per-substep wall-clock ms of the last update_global_map.

        Keys are voxelize_ms, surfaces_ms, and graph_ms.
        """
        ...

    def clear(self) -> None:
        """Drop the graph and buffered state."""
        ...

    def __repr__(self) -> str: ...

__all__ = ["MLSPlanner"]
