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

import math

from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np
import pytest

from dimos.msgs.occupancy import occupancy_extent, occupancy_view


def test_occupancy_cells_keep_ros_row_order_and_do_not_alias_message():
    message = OccupancyGrid(
        info=MapMetaData(width=3, height=2, resolution=0.1), data=[-1, 0, 100, 25, 50, 75]
    )
    decoded = OccupancyGrid.decode(message.encode())
    values = occupancy_view(decoded)
    np.testing.assert_array_equal(values, [[-1, 0, 100], [25, 50, 75]])
    with pytest.raises(ValueError, match="read-only"):
        values[0, 0] = 0
    copied = values.copy()
    copied[0, 0] = 0
    assert decoded.data[0] == -1


def test_occupancy_rejects_dimensions_that_do_not_match_payload():
    message = OccupancyGrid(info=MapMetaData(width=2, height=2, resolution=1.0), data=[0])
    with pytest.raises(ValueError, match="data length"):
        occupancy_view(message)


@pytest.mark.parametrize("resolution", [0.0, -0.1, math.nan, math.inf])
def test_occupancy_rejects_invalid_resolution_for_nonempty_grid(resolution):
    message = OccupancyGrid(info=MapMetaData(width=1, height=1, resolution=resolution), data=[0])
    with pytest.raises(ValueError, match="resolution"):
        occupancy_extent(message)


def test_empty_default_occupancy_grid_has_no_cells():
    assert occupancy_view(OccupancyGrid()).shape == (0, 0)
