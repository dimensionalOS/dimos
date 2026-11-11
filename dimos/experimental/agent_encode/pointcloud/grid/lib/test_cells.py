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
import pytest

from dimos.experimental.agent_encode.pointcloud.constants import MAX_GRID_CELLS
from dimos.experimental.agent_encode.pointcloud.grid.lib.cells import cover


def inside(points, origin, shape, cell_m):
    ij = np.floor((points[:, :2] - np.asarray(origin)) / cell_m)
    return bool(((ij >= 0) & (ij < np.asarray(shape))).all())


@pytest.mark.parametrize(
    "points",
    [
        [[0, 0, 0], [10, 5, 0]],
        [[-10, -5, 0], [0, 0, 0]],
        [[0.3, -0.7, 0], [0.3, 9.9, 0]],
        [[100, 100, 0]],
        [[-0.32, 3.97, 0], [6.03, 10.32, 0]],
    ],
)
def test_cover_includes_extrema_on_cells_aligned_to_the_cell_size(points):
    points = np.asarray(points, dtype=float)

    origin, shape, cell_m, limited = cover(points, 0.1)

    assert not limited and cell_m == 0.1
    assert inside(points, origin, shape, cell_m)
    assert np.allclose(np.asarray(origin) / 0.1, np.round(np.asarray(origin) / 0.1))
    _, shifted_shape, shifted_cell_m, _ = cover(points + np.array([50, 0, 0]), 0.1)
    assert shifted_cell_m == cell_m and shifted_shape == shape


def test_cover_grows_cells_only_for_the_grid_memory_limit():
    corners = np.array([[0, 0, 0], [300, 300, 0]], dtype=float)

    origin, shape, cell_m, limited = cover(corners, 0.1)

    assert limited and cell_m > 0.1
    assert shape[0] * shape[1] <= MAX_GRID_CELLS
    assert inside(corners, origin, shape, cell_m)
