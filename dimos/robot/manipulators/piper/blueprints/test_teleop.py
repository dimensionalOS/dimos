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

from typing import Any, cast, get_type_hints

import pytest

from dimos.control.connection.connection_module import ConnectionModule
from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.contract.limits import limits
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig, TaskConfig
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.robot.manipulators.piper.blueprints.teleop import (
    coordinator_teleop_piper,
    keyboard_teleop_piper,
)
from dimos.robot.manipulators.piper.connection import PiperConnection


def _coordinator_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    kwargs = next(
        cast("dict[str, Any]", atom.kwargs)
        for atom in blueprint.blueprints
        if isinstance(atom.module, type) and issubclass(atom.module, ControlCoordinator)
    )
    return cast("list[TaskConfig]", kwargs["tasks"])


@pytest.mark.parametrize(
    ("choices", "connection", "overrides"),
    [
        ({}, PiperConnection, {"piperconnection": {"can_port": "can0"}}),
        ({"arm": "mock"}, MockJointConnection, {}),
    ],
)
def test_arm_choice_runs_the_real_arm_or_a_mock_with_its_joints(
    choices: dict[str, str], connection: type[ConnectionModule], overrides: dict[str, Any]
) -> None:
    blueprint = coordinator_teleop_piper.solidify(choices=choices)
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides=overrides)
    coordinator = next(
        atom for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    )
    (arm,) = [atom for atom in blueprint.blueprints if issubclass(atom.module, ConnectionModule)]
    declared = ControlCoordinatorConfig.model_validate(parsed.module_kwargs(coordinator.name))
    driver = get_type_hints(connection)["config"].model_validate(parsed.module_kwargs(arm.name))

    assert arm.module is connection
    assert limits(driver) == limits(declared.connections[0])
    assert list(limits(driver)) == [
        *(f"arm/joint{i}/position" for i in range(1, 7)),
        "arm/gripper/position",
    ]
    assert blueprint.remapping_map[(arm.name, "position_command")] == "arm_position_command"


@pytest.mark.parametrize(
    "blueprint", [keyboard_teleop_piper.solidify(), coordinator_teleop_piper.solidify()]
)
def test_trajectory_accepts_gripper_and_gripper_has_dedicated_task(
    blueprint: Blueprint,
) -> None:
    tasks = _coordinator_tasks(blueprint)
    trajectory = next(task for task in tasks if task.type == "trajectory")
    gripper = next(task for task in tasks if task.type == "gripper")

    assert "arm/gripper" in trajectory.joint_names
    assert (gripper.name, gripper.joint_names) == ("arm_gripper", ["arm/gripper"])
