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

from dimos.message_utils.occupancy import occupancy_from_array
from dimos.message_utils.time import time_from_nanoseconds
from dimos.messages.geometry_msgs.msg import Point
from dimos.messages.std_msgs.msg import Header
from dimos.navigation.go2.replanning_a_star.global_planner import GlobalPlanner


@pytest.fixture
def planner():
    value = GlobalPlanner(robot_width=0.3, robot_rotation_diameter=0.6)
    yield value
    value.stop()


def test_find_wide_path_with_start_inside_inflation(planner) -> None:
    """A wall observed at the last moment can be so close that its inflation
    covers the robot's own cell (the robot drove there before the costmap
    caught up). Planning must still find a way out instead of failing."""

    resolution = 0.05
    grid = np.zeros((60, 60), dtype=np.int8)
    grid[20:40, 30] = 100  # wall at x=1.5m spanning y=1.0..2.0m
    costmap = occupancy_from_array(
        grid,
        resolution=resolution,
        header=Header(frame_id="world", stamp=time_from_nanoseconds(1700000000123456789)),
    )

    planner.handle_global_costmap(costmap)

    # 7 cm in front of the wall: within the inflation radius
    # (robot_width * 1.1 / 2 = 0.165m), so the start cell is engulfed.
    robot = Point(x=1.43, y=1.5, z=0)
    # On the other side of the wall; the path must round a wall end.
    goal = Point(x=2.75, y=1.5, z=0)

    path = planner._find_wide_path(goal, robot)

    assert path is not None
    assert len(path.poses) > 0

    assert path.header == costmap.header
