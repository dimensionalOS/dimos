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

import pytest
from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
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
