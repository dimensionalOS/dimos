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

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.go2.replanning_a_star.global_planner import GlobalPlanner


@pytest.fixture
def unmapped_planner(mocker):
    planner = GlobalPlanner(robot_width=0.3, robot_rotation_diameter=0.6)
    # Exercise real path generation without starting a velocity-producing thread.
    mocker.patch.object(planner._local_planner, "start_planning")
    yield planner
    planner.stop()


@pytest.fixture
def planner(unmapped_planner):
    planner = unmapped_planner
    planner.handle_global_costmap(
        OccupancyGrid(
            grid=np.zeros((30, 30), dtype=np.int8),
            resolution=0.2,
            origin=Pose(-2, -2, 0),
            frame_id="odom",
        )
    )
    return planner


def test_goal_before_the_first_map_is_rejected(unmapped_planner):
    unmapped_planner.handle_odom(PoseStamped(frame_id="odom", position=[0, 0, 0]))
    paths = []
    with unmapped_planner.path.subscribe(paths.append):
        accepted = unmapped_planner.handle_goal_request(
            PoseStamped(frame_id="odom", position=[1, 0, 0])
        )

    assert not accepted
    assert paths == []


def test_goal_in_an_unrelated_frame_is_rejected(planner):
    planner.handle_odom(PoseStamped(frame_id="odom", position=[0, 0, 0]))
    paths = []
    with planner.path.subscribe(paths.append):
        accepted = planner.handle_goal_request(PoseStamped(frame_id="world", position=[1, 0, 0]))

    assert not accepted
    assert paths == []


def test_unrelated_pose_cannot_replace_navigation_odometry(planner):
    planner.handle_odom(PoseStamped(frame_id="odom", position=[0, 0, 0]))
    planner.handle_odom(PoseStamped(frame_id="world", position=[99, 99, 0]))
    paths = []
    with planner.path.subscribe(paths.append):
        accepted = planner.handle_goal_request(PoseStamped(frame_id="odom", position=[1, 0, 0]))
        planner.cancel_goal()

    assert accepted
    route = next(path for path in paths if path.poses)
    assert route.poses[0].position.distance(Vector3()) < 0.3
    assert route.poses[-1].position.distance(Vector3(1, 0, 0)) < 0.3
    assert paths[-1].poses == []
    assert {path.frame_id for path in paths} == {"odom"}


def test_changing_map_frames_cancels_the_old_goal(planner):
    planner.handle_odom(PoseStamped(frame_id="odom", position=[0, 0, 0]))
    planner.handle_goal_request(PoseStamped(frame_id="odom", position=[1, 0, 0]))
    paths = []
    with planner.path.subscribe(paths.append):
        planner.handle_global_costmap(
            OccupancyGrid(
                grid=np.zeros((30, 30), dtype=np.int8),
                resolution=0.2,
                origin=Pose(-2, -2, 0),
                frame_id="new_map",
            )
        )

    assert len(paths) == 1
    assert paths[0].frame_id == "new_map" and paths[0].poses == []
    assert not planner.handle_goal_request(PoseStamped(frame_id="odom", position=[1, 0, 0]))


def test_find_wide_path_with_start_inside_inflation() -> None:
    """A wall observed at the last moment can be so close that its inflation
    covers the robot's own cell (the robot drove there before the costmap
    caught up). Planning must still find a way out instead of failing."""

    resolution = 0.05
    grid = np.zeros((60, 60), dtype=np.int8)
    grid[20:40, 30] = 100  # wall at x=1.5m spanning y=1.0..2.0m
    costmap = OccupancyGrid(grid=grid, resolution=resolution, origin=Pose(), frame_id="world")

    planner = GlobalPlanner(robot_width=0.3, robot_rotation_diameter=0.6)
    planner.handle_global_costmap(costmap)

    # 7 cm in front of the wall: within the inflation radius
    # (robot_width * 1.1 / 2 = 0.165m), so the start cell is engulfed.
    robot = Vector3(1.43, 1.5, 0)
    # On the other side of the wall; the path must round a wall end.
    goal = Vector3(2.75, 1.5, 0)

    path = planner._find_wide_path(goal, robot)

    assert path is not None
    assert len(path.poses) > 0
