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

import numpy as np
from scipy import ndimage  # type: ignore[import-untyped]

from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid


def smooth_occupied(
    occupancy_grid: OccupancyGrid, min_neighbor_fraction: float = 0.4
) -> OccupancyGrid:
    """Smooth occupied zones by removing unsupported protrusions.

    Removes occupied cells that don't have sufficient neighboring occupied
    cells.

    Args:
        occupancy_grid: Input occupancy grid
        min_neighbor_fraction: Minimum fraction of 8-connected neighbors
            that must be occupied for a cell to remain occupied.
    Returns:
        New OccupancyGrid with smoothed occupied zones
    """
    grid_array = occupancy_grid.grid
    occupied_mask = grid_array >= CostValues.OCCUPIED

    # Count occupied neighbors for each cell (8-connectivity).
    kernel = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], dtype=np.uint8)
    neighbor_count = ndimage.convolve(
        occupied_mask.astype(np.uint8), kernel, mode="constant", cval=0
    )

    # Remove cells with too few occupied neighbors.
    min_neighbors = int(np.ceil(8 * min_neighbor_fraction))
    unsupported = occupied_mask & (neighbor_count < min_neighbors)

    result_grid = grid_array.copy()
    result_grid[unsupported] = CostValues.FREE

    return OccupancyGrid(
        grid=result_grid,
        resolution=occupancy_grid.resolution,
        origin=occupancy_grid.origin,
        frame_id=occupancy_grid.frame_id,
        ts=occupancy_grid.ts,
    )


def overlay_occupied(base: OccupancyGrid, overlay: OccupancyGrid) -> OccupancyGrid:
    """Overlay occupied zones from one grid onto another.

    Marks cells as occupied in the base grid wherever they are occupied
    in the overlay grid.

    Args:
        base: The base occupancy grid
        overlay: The grid whose occupied zones will be overlaid onto base
    Returns:
        New OccupancyGrid with combined occupied zones
    """
    if base.grid.shape != overlay.grid.shape:
        raise ValueError(
            f"Grid shapes must match: base {base.grid.shape} vs overlay {overlay.grid.shape}"
        )

    result_grid = base.grid.copy()
    overlay_occupied_mask = overlay.grid >= CostValues.OCCUPIED
    result_grid[overlay_occupied_mask] = CostValues.OCCUPIED

    return OccupancyGrid(
        grid=result_grid,
        resolution=base.resolution,
        origin=base.origin,
        frame_id=base.frame_id,
        ts=base.ts,
    )


def update_confirmation_counts(
    counts: np.ndarray, observation: OccupancyGrid, max_abs_count: int = 100
) -> np.ndarray:
    """Update signed per-cell confidence counts from a new observation.

    Positive values mean confidence toward occupied, negative toward free.
    Unknown cells in the observation do not change confidence.
    """
    if counts.shape != observation.grid.shape:
        raise ValueError(
            f"Counts shape must match observation: {counts.shape} vs {observation.grid.shape}"
        )

    updated = counts.astype(np.int32, copy=True)
    occupied_mask = observation.grid >= CostValues.OCCUPIED
    free_mask = observation.grid == CostValues.FREE

    updated[occupied_mask] += 1
    updated[free_mask] -= 1

    if max_abs_count <= 0:
        raise ValueError("max_abs_count must be positive")

    np.clip(updated, -max_abs_count, max_abs_count, out=updated)
    return updated.astype(np.int16)


def update_structural_map(
    structural_map: OccupancyGrid,
    counts: np.ndarray,
    write_threshold: int,
    clear_threshold: int,
    ts: float,
) -> OccupancyGrid:
    """Apply confidence thresholds to mutate a structural map."""
    if structural_map.grid.shape != counts.shape:
        raise ValueError(
            f"Structural map shape must match counts: {structural_map.grid.shape} vs {counts.shape}"
        )

    if clear_threshold > write_threshold:
        raise ValueError(
            f"clear_threshold ({clear_threshold}) must be <= write_threshold ({write_threshold})"
        )

    result_grid = structural_map.grid.copy()
    result_grid[counts >= write_threshold] = CostValues.OCCUPIED
    result_grid[counts <= clear_threshold] = CostValues.FREE

    return OccupancyGrid(
        grid=result_grid,
        resolution=structural_map.resolution,
        origin=structural_map.origin,
        frame_id=structural_map.frame_id,
        ts=ts,
    )


def fuse_planning_map(
    structural_map: OccupancyGrid, live_map: OccupancyGrid | None, ts: float
) -> OccupancyGrid:
    """Compose planning map from structural map plus recent live occupancy."""
    if live_map is None:
        return OccupancyGrid(
            grid=structural_map.grid.copy(),
            resolution=structural_map.resolution,
            origin=structural_map.origin,
            frame_id=structural_map.frame_id,
            ts=ts,
        )

    if structural_map.grid.shape != live_map.grid.shape:
        raise ValueError(
            f"Grid shapes must match: structural {structural_map.grid.shape} vs live {live_map.grid.shape}"
        )

    fused = overlay_occupied(structural_map, live_map)
    return OccupancyGrid(
        grid=fused.grid,
        resolution=fused.resolution,
        origin=fused.origin,
        frame_id=fused.frame_id,
        ts=ts,
    )
