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

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import _is_name_unique
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

_BOTH_ARMS = [*openarm_urdf_joints("left"), *openarm_urdf_joints("right")]


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


def _trajectory_task_of(blueprint: Blueprint) -> TaskConfig:
    (task,) = _module_kwargs(blueprint, OpenArmTeleopCoordinator)["tasks"]
    assert isinstance(task, TaskConfig)
    return task


def test_combined_blueprint_streams_leader_joints_into_one_trajectory_task() -> None:
    blueprint = mini_teleop.teleop_openarm_mini
    assert _module_types(blueprint) == [
        OpenArmMiniTeleopModule,
        OpenArmTeleopCoordinator,
        _OpenArmManipulationModule,
    ]
    assert _is_name_unique(blueprint, "joint_command")
    coordinator_kwargs = _module_kwargs(blueprint, OpenArmTeleopCoordinator)
    assert coordinator_kwargs["instance_name"] == "ControlCoordinator"
    task = _trajectory_task_of(blueprint)
    assert task.name == JOINT_TRAJECTORY_TASK_NAME
    assert task.type == "trajectory"
    assert task.joint_names == _BOTH_ARMS
    assert list(task.params["velocity_limits"]) == _BOTH_ARMS
    assert _module_kwargs(blueprint, _OpenArmManipulationModule)["visualization"] == {
        "backend": "viser"
    }


def test_leader_ports_select_the_sides() -> None:
    right_only = _teleop_config_after_cli_override(
        mini_teleop.teleop_openarm_mini_leader,
        ["--openarmminiteleopmodule.port-right=/dev/ttyACM0"],
    )
    assert right_only.sides() == ("right",)
    assert right_only.port_right == "/dev/ttyACM0"
    assert right_only.connection_baudrate() == OPENARM_MINI_DEFAULT_BAUDRATE

    both = _teleop_config_after_cli_override(
        mini_teleop.teleop_openarm_mini,
        [
            "--openarmminiteleopmodule.port-left=/dev/ttyACM0",
            "--openarmminiteleopmodule.port-right=/dev/ttyACM1",
        ],
    )
    assert both.sides() == ("left", "right")

    explicit = _teleop_config_after_cli_override(
        mini_teleop.teleop_openarm_mini_leader,
        [
            "--openarmminiteleopmodule.port-left=/dev/ttyACM0",
            "--openarmminiteleopmodule.port-right=/dev/ttyACM1",
            '--openarmminiteleopmodule.enabled-sides=["left"]',
        ],
    )
    assert explicit.sides() == ("left",)


def test_leader_blueprint_joins_the_robot_bus_as_a_client() -> None:
    blueprint = mini_teleop.teleop_openarm_mini_leader
    assert _module_types(blueprint) == [OpenArmMiniTeleopModule]
    assert _is_name_unique(blueprint, "joint_command")
    assert blueprint.global_config_overrides["serve_coordinator_rpc"] is False
    assert blueprint.global_config_overrides["zenoh_mode"] == "client"


def test_follower_blueprint_consumes_joint_command_for_both_arms() -> None:
    blueprint = mini_teleop.teleop_openarm_mini_follower
    assert _module_types(blueprint) == [OpenArmTeleopCoordinator, _OpenArmManipulationModule]
    assert _is_name_unique(blueprint, "joint_command")
    assert _trajectory_task_of(blueprint).joint_names == _BOTH_ARMS
    assert blueprint.global_config_overrides["zenoh_connect"] == mini_teleop.OPENARM_ROUTER
    assert mini_teleop.OPENARM_ROUTER == "tcp/127.0.0.1:7447"
    visualization = _module_kwargs(blueprint, _OpenArmManipulationModule)["visualization"]
    assert visualization.host == "0.0.0.0"


def test_follower_accepts_can_port_overrides() -> None:
    parsed = BlueprintConfigParser(mini_teleop.teleop_openarm_mini_follower).parse(
        ["--controlcoordinator.left-can-port=can1", "--controlcoordinator.right-can-port=can2"],
        environ={},
    )
    kwargs = parsed.module_kwargs("ControlCoordinator")
    assert kwargs["left_can_port"] == "can1"
    assert kwargs["right_can_port"] == "can2"
