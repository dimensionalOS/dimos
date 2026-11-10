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

"""The standard manipulation API's gripper routing on the R1 Pro's model.

The module runs unstarted: the world monitor and the control coordinator are
mocks, so what is checked is the routing itself — which planning group owns a
gripper, which task a command lands on, and what is refused before any task
is invoked. The task side of the chain is covered end to end in
``test_r1pro_coordinator.py``.
"""

from __future__ import annotations

from collections.abc import Iterator
import math
from unittest.mock import MagicMock

import pytest

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import CommandStatus
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.planners.config import RRTConnectPlannerConfig
from dimos.robot.galaxea.r1pro.config import R1PRO_GRIPPER_TASKS, make_r1pro_model_config

GROUPS = [
    PlanningGroup("left_arm", ("j1",), base_link="base_link", tip_link="left_gripper_link"),
    PlanningGroup("right_arm", ("j2",), base_link="base_link", tip_link="right_gripper_link"),
    PlanningGroup("torso", ("j3", "j4"), base_link="base_link"),
]


def _group(group_id: str) -> PlanningGroup:
    for group in GROUPS:
        if group.id == group_id:
            return group
    raise KeyError(group_id)


@pytest.fixture
def module() -> Iterator[ManipulationModule]:
    mod = ManipulationModule(
        model=make_r1pro_model_config(),
        gripper_tasks=R1PRO_GRIPPER_TASKS,
        world_backend="drake",
        planner=RRTConnectPlannerConfig(),
        visualization={"backend": "none"},
    )
    world_monitor = MagicMock()
    world_monitor.planning_groups.list.return_value = GROUPS
    world_monitor.planning_groups.get.side_effect = _group
    mod._world_monitor = world_monitor
    mod._control_coordinator = MagicMock()
    mod._control_coordinator.task_invoke.return_value = True
    try:
        yield mod
    finally:
        mod.stop()


def test_each_arm_group_routes_to_its_own_gripper_task(module: ManipulationModule) -> None:
    assert module.set_gripper_position(0.5, "left_arm").status is CommandStatus.SUCCEEDED
    module._control_coordinator.task_invoke.assert_called_once_with(
        "r1pro_left_gripper", "set_normalized", {"values": [0.5]}
    )

    module._control_coordinator.task_invoke.reset_mock()
    assert module.set_gripper_position(0.25, "right_arm").status is CommandStatus.SUCCEEDED
    module._control_coordinator.task_invoke.assert_called_once_with(
        "r1pro_right_gripper", "set_normalized", {"values": [0.25]}
    )


def test_an_unscoped_command_is_rejected_when_two_groups_have_grippers(
    module: ManipulationModule,
) -> None:
    result = module.set_gripper_position(0.5)
    assert result.status is CommandStatus.REJECTED
    module._control_coordinator.task_invoke.assert_not_called()


def test_the_torso_declares_no_gripper(module: ManipulationModule) -> None:
    result = module.set_gripper_position(0.5, "torso")
    assert result.status is CommandStatus.REJECTED
    module._control_coordinator.task_invoke.assert_not_called()


def test_an_unknown_group_is_rejected(module: ManipulationModule) -> None:
    result = module.set_gripper_position(0.5, "nobody")
    assert result.status is CommandStatus.REJECTED
    module._control_coordinator.task_invoke.assert_not_called()


@pytest.mark.parametrize("position", [math.nan, math.inf, -0.1, 1.1])
def test_an_illegal_opening_is_rejected_before_any_task(
    module: ManipulationModule, position: float
) -> None:
    result = module.set_gripper_position(position, "left_arm")
    assert result.status is CommandStatus.REJECTED
    module._control_coordinator.task_invoke.assert_not_called()


def test_capabilities_follow_the_binding(module: ManipulationModule) -> None:
    infos = {info.id: info for info in module.list_planning_groups()}
    assert infos["left_arm"].has_gripper is True
    assert infos["right_arm"].has_gripper is True
    assert infos["torso"].has_gripper is False


def test_the_readback_comes_from_the_groups_own_task(module: ManipulationModule) -> None:
    module._control_coordinator.task_invoke.return_value = [0.4]
    assert module._get_group_gripper_position(_group("left_arm")) == pytest.approx(0.4)
    module._control_coordinator.task_invoke.assert_called_once_with(
        "r1pro_left_gripper", "get_normalized", {}
    )

    # A group without a gripper reads back nothing and asks no task.
    module._control_coordinator.task_invoke.reset_mock()
    assert module._get_group_gripper_position(_group("torso")) is None
    module._control_coordinator.task_invoke.assert_not_called()
