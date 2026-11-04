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

from typing import Any

import numpy as np
from numpy.typing import NDArray

# pose_graph and loop_closure_event record layouts, as numpy dtype fields.
POSE_GRAPH_DTYPE: list[tuple[str, str]]
LOOP_CLOSURE_EVENT_DTYPE: list[tuple[str, str]]

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
        step_penalty_weight: float = 4.0,
        worker_threads: int = 4,
        voxel_times: bool = False,
        reform_rekey_radius_m: float = 3.0,
        require_full_route: bool = False,
        max_region_columns: int = 10_000_000,
        max_closing_box_cells: int = 250_000_000,
    ) -> None: ...
    def update_global_map(self, points: NDArray[np.float32], stamp: float | None = None) -> None:
        """Voxelize the map and rebuild surfaces, nodes, and edges. Shape (N, 3) float32."""
        ...

    def update_region(
        self,
        points: NDArray[np.float32],
        origin: tuple[float, float],
        radius: float,
        z_min: float,
        z_max: float,
        sensor_z: float,
        stamp: float | None = None,
    ) -> None:
        """Replace the cylindrical region with a local map slice and rebuild.

        Points are (N, 3) float32. z_max is capped at sensor_z + max_overhead_m.
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

        Points are (N, 3) float32. The z band is the premap's own, uncapped.
        """
        ...

    def surface_map(self) -> NDArray[np.float32]:
        """Standable surface cells as (M, 3) float32 centers."""
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

    def add_keyframes(self, records: bytes) -> int:
        """Take keyframes from raw pose_graph records; returns how many are known."""
        ...

    def apply_loop_closure(
        self,
        records: bytes,
        held: list[tuple[float, float, float]] = ...,
        blend_sigma_m: float = 0.0,
        blend_time_sigma_s: float = 0.0,
        min_shift_m: float = 0.04,
        thin_cell_m: float = 0.0,
        reach_sigmas: float = 3.0,
    ) -> dict[str, Any] | None:
        """Warp the planner through raw loop_closure_event records; None when no keyframe moved past min_shift_m."""
        ...

    def __repr__(self) -> str: ...

__all__ = ["LOOP_CLOSURE_EVENT_DTYPE", "POSE_GRAPH_DTYPE", "MLSPlanner"]
