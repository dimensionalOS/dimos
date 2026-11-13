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
import xml.etree.ElementTree as ET

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import _is_name_unique
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.openarm.blueprints import mini_teleop
from dimos.robot.manipulators.openarm.config import (
    openarm_bimanual_model_config,
    openarm_urdf_joints,
)
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.teleop.leader.feetech import FEETECH_DEFAULT_BAUDRATE
from dimos.teleop.leader.module import LeaderTeleopModule, LeaderTeleopModuleConfig

LEADER_JOINT_COMMAND = "leader_joint_command"
_MODEL = openarm_bimanual_model_config(robot="arm")
_BOTH_ARMS = list(_MODEL.joint_names)
# The connection config requires both CAN selectors, so parsing a follower needs them.
_CAN_PORTS = ["--left-can-port=can1", "--right-can-port=can2"]
_LEADER = LeaderTeleopModule.name


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _module_types(blueprint: Blueprint) -> list[type]:
    return [atom.module for atom in blueprint.blueprints]


def _leader_config_after_cli_override(
    blueprint: Blueprint, overrides: Sequence[str]
) -> LeaderTeleopModuleConfig:
    has_follower = OpenArmConnection in _module_types(blueprint)
    parsed = BlueprintConfigParser(blueprint).parse(
        [*overrides, *(_CAN_PORTS if has_follower else [])], environ={}
    )
    module_kwargs = _module_kwargs(blueprint, LeaderTeleopModule).copy()
    module_kwargs.update(parsed.module_kwargs(_LEADER))
    return LeaderTeleopModuleConfig(**module_kwargs)


def _tasks_of(blueprint: Blueprint) -> list[TaskConfig]:
    tasks = _module_kwargs(blueprint, mini_teleop.OpenArmLeaderCoordinator)["tasks"]
    assert all(isinstance(task, TaskConfig) for task in tasks)
    return list(tasks)


def _assert_leader_task(blueprint: Blueprint) -> None:
    (task,) = [task for task in _tasks_of(blueprint) if task.type == "trajectory"]
    assert task.name == JOINT_TRAJECTORY_TASK_NAME
    assert task.priority == 10
    assert task.stream_bind == {"joint_command": LEADER_JOINT_COMMAND}
    assert task.joint_names == _BOTH_ARMS
    assert list(task.params["velocity_limits"]) == _BOTH_ARMS
    assert task.params["velocity_limits"]["arm/openarm_left_joint1"] == 3.0
    assert task.params["velocity_limits"]["arm/openarm_right_joint7"] == 6.0


def _assert_gripper_tasks(blueprint: Blueprint) -> None:
    grippers = [task for task in _tasks_of(blueprint) if task.type == "gripper"]
    assert [
        (task.name, task.joint_names, task.params["open_at"], task.stream_bind) for task in grippers
    ] == [
        (
            "left_arm_gripper",
            ["arm/openarm_left_finger_joint1"],
            "upper",
            {"gripper_command": "left_gripper_command"},
        ),
        (
            "right_arm_gripper",
            ["arm/openarm_right_finger_joint1"],
            "lower",
            {"gripper_command": "right_gripper_command"},
        ),
    ]
    assert all(task.priority == 20 for task in grippers)
    assert _module_kwargs(blueprint, ManipulationModule)["gripper_tasks"] == {
        "left_arm": "left_arm_gripper",
        "right_arm": "right_arm_gripper",
    }


def _assert_follower_wiring(blueprint: Blueprint) -> None:
    coordinator_kwargs = _module_kwargs(blueprint, mini_teleop.OpenArmLeaderCoordinator)
    assert coordinator_kwargs["instance_name"] == "ControlCoordinator"
    assert coordinator_kwargs["model"] is mini_teleop._MODEL.model
    assert _module_kwargs(blueprint, OpenArmConnection)["source"] == "arm"
    assert _module_kwargs(blueprint, ManipulationModule)["model"] is mini_teleop._MODEL
    assert dict(blueprint.remapping_map) == {
        (OpenArmConnection.name, "position_command"): "arm_position_command",
        ("ControlCoordinator", "arm_joint_state"): "joint_state",
    }


def _urdf_limits() -> dict[str, tuple[float, float]]:
    limits = {}
    for joint in ET.fromstring(_MODEL.model.load().xml).iter("joint"):
        limit = joint.find("limit")
        if limit is not None:
            limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
    return limits


def test_mappings_drive_each_side_joint_for_joint_inside_the_urdf_limits() -> None:
    urdf_limits = _urdf_limits()
    for side in ("left", "right"):
        mapping = mini_teleop.MAPPINGS[side]
        assert mapping.joint_names() == tuple(f"arm/{joint}" for joint in openarm_urdf_joints(side))
        assert mapping.motor_names() == (*[f"joint_{i}" for i in range(1, 8)], "gripper")
        for joint in mapping.joints:
            lower, upper = urdf_limits[joint.joint]
            assert lower <= joint.lower < joint.upper <= upper, joint.joint
            assert (joint.scale, joint.offset) == (1.0, 0.0)
        assert mapping.gripper is not None
        assert mapping.gripper.travel_radians == 1.0
    assert mini_teleop.MAPPINGS["left"].joints[3].lower == 0.0


def test_combined_blueprint_streams_leader_targets_into_the_leader_task() -> None:
    blueprint = mini_teleop.teleop_openarm_mini
    assert _module_types(blueprint) == [
        LeaderTeleopModule,
        mini_teleop.OpenArmLeaderCoordinator,
        OpenArmConnection,
        ManipulationModule,
    ]
    assert _is_name_unique(blueprint, LEADER_JOINT_COMMAND)
    _assert_follower_wiring(blueprint)
    _assert_leader_task(blueprint)
    _assert_gripper_tasks(blueprint)
    assert _module_kwargs(blueprint, ManipulationModule)["visualization"] == {"backend": "viser"}


def test_leader_ports_select_the_sides() -> None:
    right_only = _leader_config_after_cli_override(
        mini_teleop.teleop_openarm_mini_leader, [f"--{_LEADER}.port-right=/dev/ttyACM0"]
    )
    assert right_only.sides() == ("right",)
    assert right_only.port_right == "/dev/ttyACM0"
    assert right_only.baudrate == FEETECH_DEFAULT_BAUDRATE
    assert right_only.command_filter_cutoff_hz == 6.0
    assert right_only.tick_period_s == 0.01
    assert right_only.mappings["right"] == mini_teleop.MAPPINGS["right"]

    both = _leader_config_after_cli_override(
        mini_teleop.teleop_openarm_mini,
        [f"--{_LEADER}.port-left=/dev/ttyACM0", f"--{_LEADER}.port-right=/dev/ttyACM1"],
    )
    assert both.sides() == ("left", "right")

    explicit = _leader_config_after_cli_override(
        mini_teleop.teleop_openarm_mini_leader,
        [
            f"--{_LEADER}.port-left=/dev/ttyACM0",
            f"--{_LEADER}.port-right=/dev/ttyACM1",
            f'--{_LEADER}.enabled-sides=["left"]',
        ],
    )
    assert explicit.sides() == ("left",)


def test_leader_blueprint_joins_the_robot_bus_as_a_client() -> None:
    blueprint = mini_teleop.teleop_openarm_mini_leader
    assert _module_types(blueprint) == [LeaderTeleopModule]
    assert blueprint.global_config_overrides["serve_coordinator_rpc"] is False
    assert blueprint.global_config_overrides["zenoh_mode"] == "client"


def test_follower_blueprint_reads_the_leader_port_for_both_arms() -> None:
    blueprint = mini_teleop.teleop_openarm_mini_follower
    assert _module_types(blueprint) == [
        mini_teleop.OpenArmLeaderCoordinator,
        OpenArmConnection,
        ManipulationModule,
    ]
    assert _is_name_unique(blueprint, LEADER_JOINT_COMMAND)
    _assert_follower_wiring(blueprint)
    _assert_leader_task(blueprint)
    _assert_gripper_tasks(blueprint)
    assert blueprint.global_config_overrides["zenoh_connect"] == mini_teleop.OPENARM_ROUTER
    assert mini_teleop.OPENARM_ROUTER == "tcp/127.0.0.1:7447"
    assert _module_kwargs(blueprint, ManipulationModule)["visualization"].host == "0.0.0.0"


def test_follower_accepts_can_port_overrides() -> None:
    parsed = BlueprintConfigParser(mini_teleop.teleop_openarm_mini_follower).parse(
        _CAN_PORTS, environ={}
    )
    kwargs = parsed.module_kwargs(OpenArmConnection.name)
    assert kwargs["left_can_port"] == "can1"
    assert kwargs["right_can_port"] == "can2"
