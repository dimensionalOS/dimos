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
import pytest

from dimos.msgs.geometry_msgs import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.navigation.bt_navigator.goal_validator import find_safe_goal
from dimos.utils.data import get_data


@pytest.fixture
def costmap() -> OccupancyGrid:
    return OccupancyGrid(np.load(get_data("occupancy_simple.npy")))


@pytest.mark.parametrize(
    "input_pos,expected_pos",
    [
        # Identical.
        ((6.15, 10.0), (6.15, 10.0)),
        # Very slightly off.
        ((6.0, 10.0), (6.05, 10.0)),
        # Don't pick a spot that's the closest, but is actually on the other side of the wall.
        ((5.0, 9.0), (5.85, 9.6)),
    ],
)
def test_find_safe_goal(costmap, input_pos, expected_pos) -> None:
    goal = Vector3(input_pos[0], input_pos[1], 0.0)

    safe_goal = find_safe_goal(
        costmap,
        goal,
        algorithm="bfs_contiguous",
        cost_threshold=CostValues.OCCUPIED,
        min_clearance=0.3,
        max_search_distance=5.0,
        connectivity_check_radius=0,
    )

    assert safe_goal == Vector3(expected_pos[0], expected_pos[1], 0.0)
