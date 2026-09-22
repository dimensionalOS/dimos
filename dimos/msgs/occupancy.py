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

from dimos_generated.nav_msgs.msg import OccupancyGrid
import numpy as np
from numpy.typing import NDArray


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
