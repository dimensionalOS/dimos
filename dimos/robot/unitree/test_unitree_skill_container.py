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

from collections.abc import Iterator
from itertools import count

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.go2.replanning_a_star import global_planner
from dimos.navigation.go2.replanning_a_star.local_planner import LocalPlanner, StopMessage
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree import unitree_skill_container
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer


@pytest.fixture
def navigation() -> Iterator[ReplanningAStarPlanner]:
    module = ReplanningAStarPlanner()
    yield module
    module.stop()


@pytest.fixture
def skill(
    navigation: ReplanningAStarPlanner, mocker: MockerFixture
) -> Iterator[UnitreeSkillContainer]:
    module = UnitreeSkillContainer()
    mocker.patch.object(module, "_navigation", navigation, create=True)
    yield module
    module.stop()


def test_move_to_before_first_costmap_returns_failure(
    skill: UnitreeSkillContainer, navigation: ReplanningAStarPlanner, mocker: MockerFixture
) -> None:
    pose = Transform(translation=Vector3(1.0, 1.0, 0.3), child_frame_id="base_link")
    navigation._planner.handle_odom(pose.to_pose())
    tfbuffer = mocker.patch.object(
        UnitreeSkillContainer, "tfbuffer", new_callable=mocker.PropertyMock
    )
    tfbuffer.return_value.get.return_value = pose
    # Advance the skill's polling clock without sleeping or mocking navigation results.
    clock = mocker.patch.object(unitree_skill_container, "time")
    clock.monotonic.side_effect = count()

    result = skill.move_to(x=0.3, relative=True)

    assert result == (
        "Navigation was cancelled or failed. "
        "Robot is at x=1.00 y=1.00 heading=0deg; goal was x=1.30 y=1.00 heading=0deg."
    )


@pytest.mark.parametrize(
    "replanning_enabled, stop_message, attempts, reason",
    [
        (False, "error", 1, "Local planner error: replanning is disabled"),
        (True, "obstacle_found", 7, "Obstacle found: replan attempts exhausted"),
    ],
)
def test_move_to_reports_failed_turn_reason(
    skill: UnitreeSkillContainer,
    navigation: ReplanningAStarPlanner,
    mocker: MockerFixture,
    replanning_enabled: bool,
    stop_message: StopMessage,
    attempts: int,
    reason: str,
) -> None:
    planner = navigation._planner
    pose = Transform(translation=Vector3(1.0, 1.0, 0.3), child_frame_id="base_link")
    planner.handle_odom(pose.to_pose())
    planner.handle_global_costmap(
        OccupancyGrid(grid=np.zeros((50, 50), dtype=np.int8), resolution=0.1, origin=Pose())
    )
    navigation.set_replanning_enabled(replanning_enabled)
    mocker.patch.object(LocalPlanner, "start_planning")
    tfbuffer = mocker.patch.object(
        UnitreeSkillContainer, "tfbuffer", new_callable=mocker.PropertyMock
    )
    tfbuffer.return_value.get.return_value = pose
    clock = mocker.patch.object(unitree_skill_container, "time")
    clock.monotonic.side_effect = count()
    log = mocker.spy(global_planner.logger, "info")
    set_goal = navigation.set_goal

    def fail_turn(goal: PoseStamped) -> bool:
        set_goal(goal)
        # Exercise both immediate cancellation and exhaustion of real retries.
        for _ in range(attempts):
            planner._handle_stop_message(stop_message)
        return True

    mocker.patch.object(navigation, "set_goal", side_effect=fail_turn)

    result = skill.move_to(degrees=180, relative=True)

    assert result == (
        f"Navigation failed: {reason}. "
        "Robot is at x=1.00 y=1.00 heading=0deg; goal was x=1.00 y=1.00 heading=180deg."
    )
    assert not navigation.is_goal_reached()
    log.assert_any_call("Cancelling goal.", but_will_try_again=False, arrived=False, reason=reason)

    # A new request must not inherit the previous failure reason.
    planner.handle_goal_request(PoseStamped(position=[2.0, 1.0, 0.3]))
    assert navigation.get_goal_cancel_reason() is None
