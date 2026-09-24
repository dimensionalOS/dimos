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

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import pytest

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.robot.manipulators.openarm.blueprints import mini_teleop
from dimos.robot.manipulators.openarm.blueprints.teleop import (
    OpenArmTeleopCoordinator,
    _OpenArmManipulationModule,
)
from dimos.robot.manipulators.openarm.config import openarm_urdf_joints
from dimos.teleop.openarm_mini.feetech import OPENARM_MINI_DEFAULT_BAUDRATE
from dimos.teleop.openarm_mini.teleop_module import (
    OpenArmMiniTeleopModule,
    OpenArmMiniTeleopModuleConfig,
)


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _module_types(blueprint: Blueprint) -> list[type]:
    return [atom.module for atom in blueprint.blueprints]


def _teleop_config_after_cli_override(
    blueprint: Blueprint,
    overrides: Sequence[str],
) -> OpenArmMiniTeleopModuleConfig:
    parsed = BlueprintConfigParser(blueprint).parse(overrides, environ={})
    module_kwargs = _module_kwargs(blueprint, OpenArmMiniTeleopModule).copy()
    module_kwargs.update(parsed.module_kwargs(OpenArmMiniTeleopModule.name))
    return OpenArmMiniTeleopModuleConfig(**module_kwargs)


@pytest.mark.parametrize(
    ("blueprint", "enabled_sides"),
    [
        pytest.param(mini_teleop.teleop_openarm_mini_left, ("left",), id="left"),
        pytest.param(mini_teleop.teleop_openarm_mini_right, ("right",), id="right"),
        pytest.param(mini_teleop.teleop_openarm_mini, ("left", "right"), id="dual"),
    ],
)
def test_openarm_mini_blueprints_stream_leader_joints_into_trajectory_task(
    blueprint: Blueprint,
    enabled_sides: tuple[str, ...],
) -> None:
    assert _module_types(blueprint) == [
        OpenArmMiniTeleopModule,
        OpenArmTeleopCoordinator,
        _OpenArmManipulationModule,
    ]

    teleop_config = OpenArmMiniTeleopModuleConfig(
        **_module_kwargs(blueprint, OpenArmMiniTeleopModule)
    )
    assert teleop_config.enabled_sides == enabled_sides

    coordinator_kwargs = _module_kwargs(blueprint, OpenArmTeleopCoordinator)
    assert coordinator_kwargs["instance_name"] == "ControlCoordinator"
    tasks = coordinator_kwargs["tasks"]
    assert len(tasks) == 1
    task = tasks[0]
    assert isinstance(task, TaskConfig)
    assert task.name == JOINT_TRAJECTORY_TASK_NAME
    assert task.type == "trajectory"

    expected_joints = [joint for side in enabled_sides for joint in openarm_urdf_joints(side)]
    assert task.joint_names == expected_joints
    assert list(task.params["velocity_limits"]) == expected_joints
    for side in enabled_sides:
        assert task.joint_names[task.joint_names.index(f"openarm_{side}_joint1") :][:7] == list(
            teleop_config.target_joint_names(side)
        )

    manipulation_kwargs = _module_kwargs(blueprint, _OpenArmManipulationModule)
    assert manipulation_kwargs["visualization"] == {"backend": "viser"}


def test_right_openarm_mini_cli_port_override_preserves_right_side_default() -> None:
    config = _teleop_config_after_cli_override(
        mini_teleop.teleop_openarm_mini_right,
        ["--openarmminiteleopmodule.port-right=/dev/ttyACM0"],
    )

    assert config.enabled_sides == ("right",)
    assert config.port_right == "/dev/ttyACM0"
    assert config.connection_baudrate() == OPENARM_MINI_DEFAULT_BAUDRATE


def test_dual_openarm_mini_cli_overrides_reach_leader_ports_and_can_ports() -> None:
    parsed = BlueprintConfigParser(mini_teleop.teleop_openarm_mini).parse(
        [
            "--openarmminiteleopmodule.port-left=/dev/ttyACM0",
            "--openarmminiteleopmodule.port-right=/dev/ttyACM1",
            "--controlcoordinator.left-can-port=can0",
            "--controlcoordinator.right-can-port=can1",
        ],
        environ={},
    )

    teleop_kwargs = _module_kwargs(mini_teleop.teleop_openarm_mini, OpenArmMiniTeleopModule).copy()
    teleop_kwargs.update(parsed.module_kwargs(OpenArmMiniTeleopModule.name))
    config = OpenArmMiniTeleopModuleConfig(**teleop_kwargs)
    assert config.enabled_sides == ("left", "right")
    assert config.port_left == "/dev/ttyACM0"
    assert config.port_right == "/dev/ttyACM1"

    coordinator_kwargs = parsed.module_kwargs("ControlCoordinator")
    assert coordinator_kwargs["left_can_port"] == "can0"
    assert coordinator_kwargs["right_can_port"] == "can1"
