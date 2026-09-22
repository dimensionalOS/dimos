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

from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np
import pytest

from dimos.navigation.replanning_a_star.goal_validator import find_safe_goal
from dimos.utils.data import get_data


@pytest.fixture
def costmap() -> OccupancyGrid:
    cells = np.load(get_data("occupancy_simple.npy"))
    return OccupancyGrid(
        info=MapMetaData(
            width=cells.shape[1],
            height=cells.shape[0],
            resolution=0.05,
            origin=Pose(orientation=Quaternion(w=1)),
        ),
        data=cells.astype(np.int8).ravel(),
    )


@pytest.mark.parametrize(
    "input_pos,expected_pos",
    [
        # The requested point falls in the preceding cell at ROS float32 resolution.
        ((6.15, 10.0), (6.10, 9.95)),
        # Very slightly off.
        ((6.0, 10.0), (6.05, 9.95)),
        # Don't pick a spot that's the closest, but is actually on the other side of the wall.
        ((5.0, 9.0), (5.85, 9.6)),
    ],
)
def test_find_safe_goal(costmap, input_pos, expected_pos) -> None:
    goal = Point(x=input_pos[0], y=input_pos[1])

    safe_goal = find_safe_goal(
        costmap,
        goal,
        algorithm="bfs_contiguous",
        cost_threshold=100,
        min_clearance=0.3,
        max_search_distance=5.0,
        connectivity_check_radius=0,
    )

    assert safe_goal is not None
    assert (safe_goal.x, safe_goal.y, safe_goal.z) == pytest.approx((*expected_pos, 0), abs=1e-6)
