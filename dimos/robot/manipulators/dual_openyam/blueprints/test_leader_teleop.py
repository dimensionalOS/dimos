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
from typing import Any

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import _is_name_unique
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.damiao.damiao_blueprint import DamiaoCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints import leader_teleop
from dimos.robot.manipulators.dual_openyam.config import dual_openyam_model_config
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.spec.utils import spec_structural_compliance
from dimos.teleop.yam_leader.mapping import yam_follower_joint_names
from dimos.teleop.yam_leader.teleop_module import (
    YamLeaderTeleopModule,
    YamLeaderTeleopModuleConfig,
)

_JOINTS = list(dual_openyam_model_config(robot="arm").joint_names)
# The connection has no mock fallback, so every parse needs both selectors.
_CAN_PORTS = ["--left-can-port=follower_l", "--right-can-port=follower_r"]


def _atom(blueprint: Blueprint, module_type: type) -> Any:
    return next(atom for atom in blueprint.blueprints if atom.module is module_type)


def _types(blueprint: Blueprint) -> list[type]:
    return [atom.module for atom in blueprint.blueprints]


def _tasks(blueprint: Blueprint) -> list[TaskConfig]:
    tasks = _atom(blueprint, DamiaoCoordinator).kwargs["tasks"]
    assert all(isinstance(task, TaskConfig) for task in tasks)
    return list(tasks)


def test_one_machine_blueprint_streams_both_arms_and_grippers() -> None:
    blueprint = leader_teleop.teleop_dual_openyam_leader
    assert _types(blueprint) == [
        YamLeaderTeleopModule,
        DamiaoCoordinator,
        DualOpenYamConnection,
        ManipulationModule,
    ]
    assert _atom(blueprint, DamiaoCoordinator).kwargs["instance_name"] == "ControlCoordinator"
    for stream in ("joint_command", "left_gripper_command", "right_gripper_command"):
        assert _is_name_unique(blueprint, stream)

    trajectory, left_gripper, right_gripper = _tasks(blueprint)
    assert (trajectory.name, trajectory.type) == (JOINT_TRAJECTORY_TASK_NAME, "trajectory")
    assert trajectory.joint_names == _JOINTS
    limits = trajectory.params["velocity_limits"]
    assert limits == leader_teleop.dual_openyam_leader_velocity_limits()
    assert set(limits) == set(_JOINTS)
    assert limits["arm/left_joint1"] == leader_teleop.DUAL_OPENYAM_LEADER_VELOCITY_RAD_S
    assert limits["arm/right_joint6"] == leader_teleop.DUAL_OPENYAM_LEADER_WRIST_VELOCITY_RAD_S
    assert trajectory.priority < left_gripper.priority
    assert (left_gripper.type, left_gripper.joint_names) == ("gripper", ["arm/left_gripper"])
    assert left_gripper.stream_bind == {"gripper_command": "left_gripper_command"}
    assert right_gripper.joint_names == ["arm/right_gripper"]
    assert right_gripper.stream_bind == {"gripper_command": "right_gripper_command"}
    assert _atom(blueprint, ManipulationModule).kwargs["visualization"].backend == "viser"


def test_leader_targets_carry_the_coordinator_joint_names() -> None:
    trajectory = _tasks(leader_teleop.teleop_dual_openyam_leader)[0]
    assert [*yam_follower_joint_names("left"), *yam_follower_joint_names("right")] == list(
        trajectory.joint_names
    )


def test_leader_ports_select_the_sides_from_the_command_line() -> None:
    parsed = BlueprintConfigParser(leader_teleop.teleop_dual_openyam_leader).parse(
        ["--yamleaderteleopmodule.port-right=/dev/ttyACM1", *_CAN_PORTS], environ={}
    )
    config = YamLeaderTeleopModuleConfig(**parsed.module_kwargs(YamLeaderTeleopModule.name))
    assert config.sides() == ("right",)
    assert config.port_right == "/dev/ttyACM1"


def test_can_ports_reach_the_connection_from_the_command_line() -> None:
    blueprint = leader_teleop.teleop_dual_openyam_leader
    parsed = BlueprintConfigParser(blueprint).parse(_CAN_PORTS, environ={})
    kwargs = parsed.module_kwargs(_atom(blueprint, DualOpenYamConnection).name)
    assert (kwargs["left_can_port"], kwargs["right_can_port"]) == ("follower_l", "follower_r")


def test_operator_blueprint_joins_the_robot_bus_as_a_client() -> None:
    blueprint = leader_teleop.teleop_dual_openyam_leader_operator
    assert _types(blueprint) == [YamLeaderTeleopModule]
    assert blueprint.global_config_overrides["zenoh_mode"] == "client"
    assert blueprint.global_config_overrides["serve_coordinator_rpc"] is False


def test_follower_blueprint_dials_the_router() -> None:
    blueprint = leader_teleop.teleop_dual_openyam_leader_follower
    assert _types(blueprint) == [DamiaoCoordinator, DualOpenYamConnection, ManipulationModule]
    assert blueprint.global_config_overrides["zenoh_connect"] == leader_teleop.DUAL_OPENYAM_ROUTER
    assert _tasks(blueprint) == _tasks(leader_teleop.teleop_dual_openyam_leader)


def test_leader_links_to_the_coordinator_through_an_optional_control_spec() -> None:
    blueprint = leader_teleop.teleop_dual_openyam_leader
    leader = next(a for a in blueprint.active_blueprints if a.module is YamLeaderTeleopModule)
    (ref,) = [ref for ref in leader.module_refs if ref.name == "_control"]
    assert ref.optional is True
    assert spec_structural_compliance(DamiaoCoordinator, ref.spec)
    assert not spec_structural_compliance(ManipulationModule, ref.spec)
    operator = next(
        a
        for a in leader_teleop.teleop_dual_openyam_leader_operator.active_blueprints
        if a.module is YamLeaderTeleopModule
    )
    assert any(ref.name == "_control" and ref.optional for ref in operator.module_refs)
