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

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner


@pytest.fixture
def planner_without_motion(mocker):
    planner = GlobalPlanner(GlobalConfig())
    start_planning = mocker.patch.object(planner._local_planner, "start_planning")
    try:
        yield planner, start_planning
    finally:
        planner.stop()


@pytest.mark.parametrize(
    "remaining_m, reached", [(0.0, True), (0.19, True), (0.21, False), (1.1, False)]
)
def test_local_arrival_requires_reaching_requested_position(
    planner_without_motion, remaining_m, reached
):
    planner, _ = planner_without_motion
    planner.handle_global_costmap(
        OccupancyGrid(grid=np.zeros((60, 60), dtype=np.int8), resolution=0.1, origin=Pose())
    )
    planner.handle_odom(PoseStamped(position=[1.0, 2.0, 0.0]))
    planner.handle_goal_request(PoseStamped(position=[3.0, 2.0, 0.0]))
    outcomes = []

    with planner.goal_reached.subscribe(lambda msg: outcomes.append(msg.data)):
        planner.handle_odom(PoseStamped(position=[3.0 - remaining_m, 2.0, 0.0]))
        planner._handle_stop_message("arrived")

    assert planner.is_goal_reached() is reached
    assert outcomes == [reached]


def test_arrival_at_snapped_endpoint_fails_original_goal_without_replanning(planner_without_motion):
    planner, start_planning = planner_without_motion
    grid = np.zeros((60, 60), dtype=np.int8)
    grid[10:30, 30] = 100
    planner.handle_global_costmap(OccupancyGrid(grid=grid, resolution=0.1, origin=Pose()))
    planner.handle_odom(PoseStamped(position=[1.0, 2.0, 0.0]))
    requested_goal = PoseStamped(position=[3.0, 2.0, 0.0])
    planner.handle_goal_request(requested_goal)
    planned_path = start_planning.call_args.args[0]
    endpoint = planned_path.poses[-1]
    assert endpoint.position.distance(requested_goal.position) > planner._goal_tolerance
    outcomes = []

    with planner.goal_reached.subscribe(lambda msg: outcomes.append(msg.data)):
        planner.handle_odom(endpoint)
        planner._handle_stop_message("arrived")

    assert planner.is_goal_reached() is False
    assert outcomes == [False]
    start_planning.assert_called_once()


def test_find_wide_path_with_start_inside_inflation() -> None:
    """A wall observed at the last moment can be so close that its inflation
    covers the robot's own cell (the robot drove there before the costmap
    caught up). Planning must still find a way out instead of failing."""

    resolution = 0.05
    grid = np.zeros((60, 60), dtype=np.int8)
    grid[20:40, 30] = 100  # wall at x=1.5m spanning y=1.0..2.0m
    costmap = OccupancyGrid(grid=grid, resolution=resolution, origin=Pose(), frame_id="world")

    planner = GlobalPlanner(GlobalConfig())
    planner.handle_global_costmap(costmap)

    # 7 cm in front of the wall: within the inflation radius
    # (robot_width * 1.1 / 2 = 0.165m), so the start cell is engulfed.
    robot = Vector3(1.43, 1.5, 0)
    # On the other side of the wall; the path must round a wall end.
    goal = Vector3(2.75, 1.5, 0)

    path = planner._find_wide_path(goal, robot)

    assert path is not None
    assert len(path.poses) > 0
