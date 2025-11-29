# Copyright 2025 Dimensional Inc.
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
from open3d.geometry import PointCloud  # type: ignore[import-untyped]
import pytest

from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.global_planner.astar import astar
from dimos.utils.data import get_data


@pytest.fixture
def costmap() -> PointCloud:
    return OccupancyGrid(np.load(get_data("occupancy_simple.npy"))).gradient(max_distance=1.5)


@pytest.mark.parametrize(
    "mode,expected_image",
    [
        ("general", "astar_general.png"),
        ("min_cost", "astar_min_cost.png"),
    ],
)
def test_astar(costmap, mode, expected_image) -> None:
    start = Vector3(4.0, 2.0, 0)
    goal = Vector3(6.15, 10.0)
    expected = Image.from_file(get_data(expected_image))

    path = astar(mode, costmap, goal, start)
    actual = visualize_occupancy_grid(costmap, "rainbow", path)

    np.testing.assert_array_equal(actual.data, expected.data)
