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

from dimos.mapping.voxels.grid import VoxelGrid
from dimos.messages.sensor_msgs.msg import PointCloud2


@pytest.fixture
def grid(request):
    grid = VoxelGrid(voxel_size=1.0, device="CPU:0", carve_columns=request.param)
    try:
        yield grid
    finally:
        grid.dispose()


@pytest.mark.parametrize(
    "grid, expected",
    [
        pytest.param(
            True,
            {(0.5, 0.5, 2.5), (2.5, 0.5, 0.5)},
            id="carving-replaces-touched-column",
        ),
        pytest.param(
            False,
            {(0.5, 0.5, 0.5), (0.5, 0.5, 1.5), (0.5, 0.5, 2.5), (2.5, 0.5, 0.5)},
            id="additive-keeps-old-voxels",
        ),
    ],
    indirect=["grid"],
)
def test_new_frame_preserves_untouched_columns(grid, expected):
    grid.add_frame(
        PointCloud2.from_numpy(
            np.array([[0.5, 0.5, 0.5], [0.5, 0.5, 1.5], [2.5, 0.5, 0.5]], dtype=np.float32)
        )
    )
    grid.add_frame(PointCloud2.from_numpy(np.array([[0.5, 0.5, 2.5]], dtype=np.float32)))

    points = grid.get_global_pointcloud2().points_f32()
    assert {tuple(point) for point in points} == expected
