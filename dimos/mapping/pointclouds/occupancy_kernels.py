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

"""numba kernels for the occupancy grids; imported lazily because numba is a 0.3 s import."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from numba import njit, prange  # type: ignore[import-untyped]
import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray


@njit(cache=True)  # type: ignore[untyped-decorator]
def height_map_kernel(
    points: NDArray[np.floating[Any]],
    min_height_map: NDArray[np.floating[Any]],
    max_height_map: NDArray[np.floating[Any]],
    min_x: float,
    min_y: float,
    inv_res: float,
    width: int,
    height: int,
) -> None:
    """Build min/max height maps from points (faster than np.fmax/fmin.at)."""
    n = points.shape[0]
    for i in range(n):
        x = points[i, 0]
        y = points[i, 1]
        z = points[i, 2]

        gx = int((x - min_x) * inv_res + 0.5)
        gy = int((y - min_y) * inv_res + 0.5)

        if 0 <= gx < width and 0 <= gy < height:
            cur_min = min_height_map[gy, gx]
            cur_max = max_height_map[gy, gx]
            # NaN comparisons are always False, so first point sets the value
            if z < cur_min or cur_min != cur_min:  # cur_min != cur_min checks for NaN
                min_height_map[gy, gx] = z
            if z > cur_max or cur_max != cur_max:
                max_height_map[gy, gx] = z


@njit(cache=True, parallel=True)  # type: ignore[untyped-decorator]
def simple_occupancy_kernel(
    points: NDArray[np.floating[Any]],
    grid: NDArray[np.signedinteger[Any]],
    min_x: float,
    min_y: float,
    inv_res: float,
    width: int,
    height: int,
    min_height: float,
    max_height: float,
) -> None:
    """Numba-accelerated kernel for simple_occupancy grid population."""
    n = points.shape[0]
    # Pass 1: Mark ground as free
    for i in prange(n):
        x = points[i, 0]
        y = points[i, 1]
        z = points[i, 2]
        if z < min_height:
            gx = int((x - min_x) * inv_res + 0.5)
            gy = int((y - min_y) * inv_res + 0.5)
            if 0 <= gx < width and 0 <= gy < height:
                grid[gy, gx] = 0

    # Pass 2: Mark obstacles (overwrites ground)
    for i in prange(n):
        x = points[i, 0]
        y = points[i, 1]
        z = points[i, 2]
        if min_height <= z <= max_height:
            gx = int((x - min_x) * inv_res + 0.5)
            gy = int((y - min_y) * inv_res + 0.5)
            if 0 <= gx < width and 0 <= gy < height:
                grid[gy, gx] = 100
