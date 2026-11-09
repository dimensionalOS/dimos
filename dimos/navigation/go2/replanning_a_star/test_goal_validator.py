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
import pytest

from dimos.message_utils import occupancy as occupancy_utils, time as message_time
from dimos.message_utils.occupancy import CostValues
from dimos.messages.geometry_msgs.msg import Vector3
from dimos.messages.nav_msgs.msg import OccupancyGrid
from dimos.messages.std_msgs.msg import Header
from dimos.navigation.go2.replanning_a_star.goal_validator import find_safe_goal
from dimos.utils.data import get_data


@pytest.fixture
def costmap() -> OccupancyGrid:
    return occupancy_utils.from_array(
        np.asarray(np.load(get_data("occupancy_simple.npy")), dtype=np.int8),
        header=Header(stamp=message_time.time_from_seconds(0.0), frame_id=""),
        resolution=0.05,
        origin=None,
    )


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

    np.testing.assert_allclose(
        [safe_goal.x, safe_goal.y, safe_goal.z], [expected_pos[0], expected_pos[1], 0.0]
    )
