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

from itertools import count
import math

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.base import NavigationState
from dimos.navigation.go2.replanning_a_star import global_planner
from dimos.navigation.go2.replanning_a_star.global_planner import GlobalPlanner
from dimos.navigation.go2.replanning_a_star.local_planner import LocalPlanner, PlannerState


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


def test_safe_goal_shift_ignores_the_height_of_the_goal(mocker: MockerFixture) -> None:
    """A goal built from odometry sits at the height of the robot's body. Snapping
    it to a costmap cell moves it a few centimetres, which deserves no warning."""

    warning = mocker.patch.object(global_planner.logger, "warning")
    grid = np.zeros((60, 60), dtype=np.int8)
    costmap = OccupancyGrid(grid=grid, resolution=0.05, origin=Pose(), frame_id="world")
    planner = GlobalPlanner(robot_width=0.3, robot_rotation_diameter=0.6)
    planner.handle_global_costmap(costmap)

    safe_goal = planner._find_safe_goal(Vector3(1.52, 1.52, 0.29))

    assert safe_goal is not None
    assert (round(safe_goal.x, 2), round(safe_goal.y, 2)) == (1.5, 1.5)
    warning.assert_not_called()


def test_replan_near_the_goal_ignores_the_height_of_the_robot() -> None:
    """Within the replan tolerance of the goal the robot has arrived, however far
    its body is above a goal on the ground."""

    planner = GlobalPlanner(robot_width=0.3, robot_rotation_diameter=0.6)
    planner.handle_odom(PoseStamped(position=[1.0, 1.0, 0.52]))
    planner._current_goal = PoseStamped(position=[1.3, 1.0, 0.0])

    planner._replan_path()

    assert planner.is_goal_reached()


@pytest.fixture
def planner():
    planner = GlobalPlanner(robot_width=0.3, robot_rotation_diameter=0.6)
    yield planner
    planner.stop()


@pytest.fixture
def mapped_planner(planner: GlobalPlanner, mocker: MockerFixture) -> GlobalPlanner:
    planner.handle_global_costmap(
        OccupancyGrid(
            grid=np.zeros((50, 50), dtype=np.int8),
            resolution=0.1,
            origin=Pose(),
            frame_id="world",
        )
    )
    # Exercise path planning without starting the local controller thread.
    mocker.patch.object(LocalPlanner, "start_planning")
    return planner


@pytest.mark.parametrize("distance", [0.0, 0.3], ids=["in_place", "near_goal"])
@pytest.mark.parametrize(
    "robot_degrees, goal_degrees, arrived",
    [(-15, -179, False), (179, -179, True), (-179, 179, True)],
)
def test_replan_near_goal_checks_heading(
    mapped_planner: GlobalPlanner,
    distance: float,
    robot_degrees: float,
    goal_degrees: float,
    arrived: bool,
) -> None:
    planner = mapped_planner
    planner.handle_odom(
        PoseStamped(
            position=[1.0, 1.0, 0.3],
            orientation=Quaternion.from_euler(Vector3(0, 0, math.radians(robot_degrees))),
        )
    )
    planner.handle_goal_request(
        PoseStamped(
            position=[1.0 + distance, 1.0, 0.0],
            orientation=Quaternion.from_euler(Vector3(0, 0, math.radians(goal_degrees))),
        )
    )
    outcomes: list[bool] = []

    with planner.goal_reached.subscribe(lambda msg: outcomes.append(msg.data)):
        planner._replan_path()

    assert planner.is_goal_reached() == arrived
    assert outcomes == [True] * int(arrived)
    assert planner._replan_limiter.get_attempt() == int(not arrived)


@pytest.mark.parametrize("state", ["initial_rotation", "final_rotation", "path_following"])
def test_positional_stuck_check_only_applies_to_path_following(
    mapped_planner: GlobalPlanner, mocker: MockerFixture, state: PlannerState
) -> None:
    planner = mapped_planner
    planner.handle_odom(PoseStamped(position=[1.0, 1.0, 0.3]))
    planner.handle_goal_request(
        PoseStamped(
            position=[1.0, 1.0, 0.3], orientation=Quaternion.from_euler(Vector3(0, 0, math.pi))
        )
    )
    mocker.patch.object(planner._local_planner, "get_unique_state", return_value=(state, 1))
    clock = mocker.patch.object(global_planner, "time")
    clock.perf_counter.side_effect = count(0, 9)
    polls = iter([False, False, True])

    def monitor_tick(timeout: float) -> bool:
        # Two checks in the same state, separated by more than the stuck window.
        planner.handle_odom(
            PoseStamped(
                position=[1.0, 1.0, 0.3],
                orientation=Quaternion.from_euler(Vector3(0, 0, math.radians(15))),
            )
        )
        if next(polls):
            planner._stop_planner.set()
        return False

    mocker.patch.object(planner._replan_event, "wait", side_effect=monitor_tick)
    replan = mocker.spy(planner, "_replan_path")

    planner._thread_entrypoint()

    assert not planner.is_goal_reached()
    assert replan.call_count == int(state == "path_following")


def test_first_goal_does_not_log_cancellation_but_explicit_cancel_does(planner, mocker):
    log = mocker.patch.object(global_planner.logger, "info")
    goal = PoseStamped(position=[1.0, 2.0, 0.0])

    planner.handle_goal_request(goal)

    log.assert_called_once_with("Got new goal", goal=str(goal))

    planner.cancel_goal()

    log.assert_called_with("Cancelling goal.", but_will_try_again=False, arrived=False, reason=None)


@pytest.mark.parametrize("distance", [0.3, 2.0], ids=["near", "far"])
def test_goal_before_first_costmap_fails_and_can_be_retried(
    planner: GlobalPlanner, mocker: MockerFixture, distance: float
) -> None:
    start_planning = mocker.patch.object(LocalPlanner, "start_planning")
    warning = mocker.patch.object(global_planner.logger, "warning")
    planner.handle_odom(PoseStamped(position=[1.0, 1.0, 0.3]))
    goal = PoseStamped(position=[1.0 + distance, 1.0, 0.3], frame_id="world")
    outcomes: list[bool] = []

    with planner.goal_reached.subscribe(lambda msg: outcomes.append(msg.data)):
        planner.handle_goal_request(goal)

        assert outcomes == [False]
        assert planner.get_state() == NavigationState.IDLE
        assert not planner.is_goal_reached()
        reason = "Missing global costmap: check that lidar data is arriving and mapping is running"
        assert planner.get_goal_cancel_reason() == reason
        warning.assert_called_once_with("Cannot handle goal request.", reason=reason)
        start_planning.assert_not_called()
        # The failed goal must be cleared, not left pending until a map arrives.
        planner.cancel_goal()
        assert outcomes == [False]

        planner.handle_global_costmap(
            OccupancyGrid(
                grid=np.zeros((50, 50), dtype=np.int8),
                resolution=0.1,
                origin=Pose(),
                frame_id="world",
            )
        )
        start_planning.assert_not_called()

        planner.handle_goal_request(goal)

        assert planner.get_goal_cancel_reason() is None
        start_planning.assert_called_once()
        path = start_planning.call_args.args[0]
        assert path.poses[-1].x == pytest.approx(goal.x)
        assert path.poses[-1].y == pytest.approx(goal.y)
        assert outcomes == [False]
