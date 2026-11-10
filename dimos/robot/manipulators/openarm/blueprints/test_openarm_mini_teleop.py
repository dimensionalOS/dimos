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
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoCoordinator
from dimos.robot.manipulators.openarm.blueprints import mini_teleop
from dimos.robot.manipulators.openarm.config import (
    openarm_bimanual_model_config,
    openarm_urdf_joints,
)
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.teleop.openarm_mini.feetech import OPENARM_MINI_DEFAULT_BAUDRATE
from dimos.teleop.openarm_mini.teleop_module import (
    OpenArmMiniTeleopModule,
    OpenArmMiniTeleopModuleConfig,
)

_MODEL = openarm_bimanual_model_config(robot="arm")
_BOTH_ARMS = list(_MODEL.joint_names)
# The connection config requires both CAN selectors, so parsing a follower needs them.
_CAN_PORTS = ["--left-can-port=can1", "--right-can-port=can2"]


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _module_types(blueprint: Blueprint) -> list[type]:
    return [atom.module for atom in blueprint.blueprints]


def _teleop_config_after_cli_override(
    blueprint: Blueprint,
    overrides: Sequence[str],
) -> OpenArmMiniTeleopModuleConfig:
    has_follower = OpenArmConnection in _module_types(blueprint)
    parsed = BlueprintConfigParser(blueprint).parse(
        [*overrides, *(_CAN_PORTS if has_follower else [])], environ={}
    )
    module_kwargs = _module_kwargs(blueprint, OpenArmMiniTeleopModule).copy()
    module_kwargs.update(parsed.module_kwargs(OpenArmMiniTeleopModule.name))
    return OpenArmMiniTeleopModuleConfig(**module_kwargs)


def _tasks_of(blueprint: Blueprint) -> list[TaskConfig]:
    tasks = _module_kwargs(blueprint, DamiaoCoordinator)["tasks"]
    assert all(isinstance(task, TaskConfig) for task in tasks)
    return list(tasks)


def _trajectory_task_of(blueprint: Blueprint) -> TaskConfig:
    (task,) = [task for task in _tasks_of(blueprint) if task.type == "trajectory"]
    return task


def _assert_gripper_tasks(blueprint: Blueprint) -> None:
    grippers = [task for task in _tasks_of(blueprint) if task.type == "gripper"]
    assert [
        (task.name, task.joint_names, task.params["open_at"], task.stream_bind) for task in grippers
    ] == [
        (
            "arm_left_gripper",
            ["arm/openarm_left_finger_joint1"],
            "upper",
            {"gripper_command": "left_gripper_command"},
        ),
        (
            "arm_right_gripper",
            ["arm/openarm_right_finger_joint1"],
            "lower",
            {"gripper_command": "right_gripper_command"},
        ),
    ]
    assert all(task.priority == 20 for task in grippers)
    assert _module_kwargs(blueprint, ManipulationModule)["gripper_tasks"] == {
        "left_arm": "arm_left_gripper",
        "right_arm": "arm_right_gripper",
    }


def _assert_follower_wiring(blueprint: Blueprint) -> None:
    coordinator_kwargs = _module_kwargs(blueprint, DamiaoCoordinator)
    assert coordinator_kwargs["instance_name"] == "ControlCoordinator"
    assert coordinator_kwargs["model"] is mini_teleop._MODEL.model
    assert _module_kwargs(blueprint, OpenArmConnection)["source"] == "arm"
    assert _module_kwargs(blueprint, ManipulationModule)["model"] is mini_teleop._MODEL
    assert dict(blueprint.remapping_map) == {
        (OpenArmConnection.name, "position_command"): "arm_position_command",
        ("ControlCoordinator", "arm_joint_state"): "joint_state",
    }


def test_combined_blueprint_streams_leader_joints_into_one_trajectory_task() -> None:
    blueprint = mini_teleop.teleop_openarm_mini
    assert _module_types(blueprint) == [
        OpenArmMiniTeleopModule,
        DamiaoCoordinator,
        OpenArmConnection,
        ManipulationModule,
    ]
    assert _is_name_unique(blueprint, "joint_command")
    _assert_follower_wiring(blueprint)
    task = _trajectory_task_of(blueprint)
    assert task.name == JOINT_TRAJECTORY_TASK_NAME
    assert task.joint_names == _BOTH_ARMS
    assert list(task.params["velocity_limits"]) == _BOTH_ARMS
    assert task.params["velocity_limits"]["arm/openarm_left_joint1"] == 3.0
    assert task.params["velocity_limits"]["arm/openarm_right_joint7"] == 6.0
    _assert_gripper_tasks(blueprint)
    assert _module_kwargs(blueprint, ManipulationModule)["visualization"] == {"backend": "viser"}


def test_leader_targets_the_connection_joint_names() -> None:
    for blueprint in (mini_teleop.teleop_openarm_mini, mini_teleop.teleop_openarm_mini_leader):
        targets = _module_kwargs(blueprint, OpenArmMiniTeleopModule)["target_joint_names_by_side"]
        for side in ("left", "right"):
            assert targets[side] == tuple(f"arm/{joint}" for joint in openarm_urdf_joints(side))
        config = _teleop_config_after_cli_override(
            blueprint, ["--openarmminiteleopmodule.port-left=/dev/ttyACM0"]
        )
        assert config.target_joint_names("left") == targets["left"]


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
    assert _module_types(blueprint) == [DamiaoCoordinator, OpenArmConnection, ManipulationModule]
    assert _is_name_unique(blueprint, "joint_command")
    _assert_follower_wiring(blueprint)
    assert _trajectory_task_of(blueprint).joint_names == _BOTH_ARMS
    _assert_gripper_tasks(blueprint)
    assert blueprint.global_config_overrides["zenoh_connect"] == mini_teleop.OPENARM_ROUTER
    assert mini_teleop.OPENARM_ROUTER == "tcp/127.0.0.1:7447"
    visualization = _module_kwargs(blueprint, ManipulationModule)["visualization"]
    assert visualization.host == "0.0.0.0"


def test_follower_accepts_can_port_overrides() -> None:
    parsed = BlueprintConfigParser(mini_teleop.teleop_openarm_mini_follower).parse(
        _CAN_PORTS, environ={}
    )
    kwargs = parsed.module_kwargs(OpenArmConnection.name)
    assert kwargs["left_can_port"] == "can1"
    assert kwargs["right_can_port"] == "can2"


def test_joycon_blueprints_add_the_joycon_module_beside_the_leader() -> None:
    from dimos.teleop.joycon.joycon_module import JoyConTeleopModule

    combined = mini_teleop.teleop_openarm_mini_joycon
    assert _module_types(combined) == [
        OpenArmMiniTeleopModule,
        JoyConTeleopModule,
        DamiaoCoordinator,
        OpenArmConnection,
        ManipulationModule,
    ]
    assert _is_name_unique(combined, "joint_command")
    assert _is_name_unique(combined, "teleop_buttons")
    assert _is_name_unique(combined, "left_gripper_command")
    _assert_gripper_tasks(combined)

    operator = mini_teleop.teleop_openarm_mini_joycon_leader
    assert _module_types(operator) == [OpenArmMiniTeleopModule, JoyConTeleopModule]
    assert operator.global_config_overrides["serve_coordinator_rpc"] is False
    assert operator.global_config_overrides["zenoh_mode"] == "client"
