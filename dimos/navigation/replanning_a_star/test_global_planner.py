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

from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.core.global_config import GlobalConfig
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner


def test_find_wide_path_with_start_inside_inflation() -> None:
    """A wall observed at the last moment can be so close that its inflation
    covers the robot's own cell (the robot drove there before the costmap
    caught up). Planning must still find a way out instead of failing."""

    resolution = 0.05
    grid = np.zeros((60, 60), dtype=np.int8)
    grid[20:40, 30] = 100  # wall at x=1.5m spanning y=1.0..2.0m
    costmap = OccupancyGrid(
        header=Header(frame_id="world"),
        info=MapMetaData(
            width=60, height=60, resolution=resolution, origin=Pose(orientation=Quaternion(w=1))
        ),
        data=grid.ravel(),
    )

    planner = GlobalPlanner(GlobalConfig())
    planner.handle_global_costmap(costmap)

    # 7 cm in front of the wall: within the inflation radius
    # (robot_width * 1.1 / 2 = 0.165m), so the start cell is engulfed.
    robot = Point(x=1.43, y=1.5)
    # On the other side of the wall; the path must round a wall end.
    goal = Point(x=2.75, y=1.5)

    path = planner._find_wide_path(goal, robot)

    assert path is not None
    assert len(path.poses) > 0
