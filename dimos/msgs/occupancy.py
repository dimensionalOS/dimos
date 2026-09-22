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

"""Array operations on generated occupancy grids."""

import math

from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.nav_msgs.msg import OccupancyGrid
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry import pose_matrix


def occupancy_view(msg: OccupancyGrid) -> NDArray[np.int8]:
    """Borrow a read-only row-major occupancy grid, retaining its owner."""
    if len(msg.data) != msg.info.height * msg.info.width:
        raise ValueError("occupancy data length does not match dimensions")
    return msg.data.view().reshape(msg.info.height, msg.info.width)


def block_max_reduce(cells: NDArray[np.int8], factor: int) -> NDArray[np.int8]:
    """Coarsen an occupancy grid by taking the max of factor x factor blocks.

    Block maximum (not mean) so coarsening never erases an obstacle; a block
    is unknown (-1) only when every cell in it is unknown. Trailing remainder
    rows/cols are trimmed, keeping row 0/col 0 - the origin corner - so the
    grid origin stays valid. Grids thinner than `factor` pass through.
    """
    h, w = cells.shape[:2]
    new_h, new_w = h // factor, w // factor
    if new_h == 0 or new_w == 0:
        return cells
    trimmed = cells[: new_h * factor, : new_w * factor]
    blocks = trimmed.reshape(new_h, factor, new_w, factor)
    # Sink unknown below every known value for the max, then map it back.
    as_int = blocks.astype(np.int16)
    known = np.where(as_int < 0, -1000, as_int)
    reduced = known.max(axis=(1, 3))
    reduced[reduced == -1000] = -1
    result: NDArray[np.int8] = reduced.astype(np.int8)
    return result


def occupancy_extent(message: OccupancyGrid) -> tuple[float, float]:
    """Return physical width and height, rejecting an invalid grid resolution."""
    info = message.info
    if not math.isfinite(info.resolution) or info.resolution <= 0:
        raise ValueError("OccupancyGrid resolution must be finite and positive")
    return info.width * info.resolution, info.height * info.resolution


def world_to_grid(message: OccupancyGrid, point: Point) -> tuple[float, float]:
    """Convert a world point to continuous grid coordinates, including origin rotation."""
    occupancy_extent(message)
    matrix = pose_matrix(message.info.origin)
    local = matrix[:3, :3].T @ (np.array([point.x, point.y, point.z]) - matrix[:3, 3])
    if not np.isfinite(local).all():
        raise ValueError("grid coordinates must be finite")
    return float(local[0] / message.info.resolution), float(local[1] / message.info.resolution)


def grid_to_world(message: OccupancyGrid, coordinates: tuple[float, float]) -> Point:
    """Convert continuous grid coordinates to the grid plane in world coordinates."""
    occupancy_extent(message)
    x, y = coordinates
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("grid coordinates must be finite")
    result = pose_matrix(message.info.origin) @ np.array(
        [x * message.info.resolution, y * message.info.resolution, 0.0, 1.0]
    )
    return Point(x=float(result[0]), y=float(result[1]), z=float(result[2]))
