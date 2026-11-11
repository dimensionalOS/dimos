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

import pytest

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import _is_name_unique
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.manipulators.dual_openyam.blueprints import leader_teleop
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_PER_ARM,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.teleop.leader.calibration import LEADER_SIDES
from dimos.teleop.leader.coordinator import (
    LEADER_JOINT_COMMAND,
    LEADER_TASK_PRIORITY,
    LeaderCoordinator,
)
from dimos.teleop.leader.module import LeaderTeleopModule, LeaderTeleopModuleConfig

_MODEL = dual_openyam_model_config(robot="arm")
# The connection has no mock fallback, so every parse needs both selectors.
_CAN_PORTS = ["--left-can-port=follower_l", "--right-can-port=follower_r"]


def _atom(blueprint: Blueprint, module_type: type) -> Any:
    return next(atom for atom in blueprint.blueprints if atom.module is module_type)


def _tasks(blueprint: Blueprint) -> dict[str, TaskConfig]:
    tasks = _atom(blueprint, LeaderCoordinator).kwargs["tasks"]
    assert all(isinstance(task, TaskConfig) for task in tasks)
    return {task.name: task for task in tasks}


def test_mappings_drive_every_planning_model_joint_in_order() -> None:
    assert leader_teleop.DUAL_OPENYAM_LEADER_JOINTS == list(_MODEL.joint_names)
    for side, group in zip(LEADER_SIDES, _MODEL.planning_groups, strict=True):
        mapping = leader_teleop.MAPPINGS[side]
        assert mapping.joint_names() == tuple(group.joint_names)
        assert mapping.motor_names() == tuple(f"joint_{index}" for index in range(1, 8))
        assert [(joint.scale, joint.offset) for joint in mapping.joints] == [(1.0, 0.0)] * 6
        assert [(joint.lower, joint.upper) for joint in mapping.joints] == list(
            leader_teleop.DUAL_OPENYAM_JOINT_LIMITS
        )
        assert mapping.gripper is not None
        assert mapping.gripper.motor == "joint_7"
        assert mapping.gripper.travel_radians == pytest.approx(1.5)


@pytest.mark.self_hosted
def test_mapping_limits_match_the_urdf() -> None:
    loaded = _MODEL.model.load()
    for mapping in leader_teleop.MAPPINGS.values():
        for joint_mapping in mapping.joints:
            joint = loaded.get_joint(joint_mapping.joint)
            assert joint is not None
            assert joint.lower == pytest.approx(joint_mapping.lower, abs=1e-4)
            assert joint.upper == pytest.approx(joint_mapping.upper, abs=1e-4)


def test_teleop_blueprint_gives_the_leader_its_own_lane_above_the_planner() -> None:
    blueprint = leader_teleop.dual_openyam_leader_teleop
    assert [atom.module for atom in blueprint.blueprints] == [
        LeaderTeleopModule,
        LeaderCoordinator,
        DualOpenYamConnection,
        ManipulationModule,
    ]
    assert _atom(blueprint, LeaderCoordinator).kwargs["instance_name"] == "ControlCoordinator"
    for stream in (LEADER_JOINT_COMMAND, "left_gripper_command", "right_gripper_command"):
        assert _is_name_unique(blueprint, stream)

    tasks = _tasks(blueprint)
    assert list(tasks) == [JOINT_TRAJECTORY_TASK_NAME, "left_arm_gripper", "right_arm_gripper"]
    leader = tasks[JOINT_TRAJECTORY_TASK_NAME]
    assert leader.type == "trajectory"
    assert leader.joint_names == list(_MODEL.joint_names)
    assert leader.stream_bind == {"joint_command": LEADER_JOINT_COMMAND}
    assert leader.priority == LEADER_TASK_PRIORITY
    limits = leader.params["velocity_limits"]
    assert limits == leader_teleop.dual_openyam_leader_velocity_limits()
    assert set(limits) == set(_MODEL.joint_names)
    assert limits["arm/left_joint1"] == leader_teleop.DUAL_OPENYAM_LEADER_VELOCITY_RAD_S
    assert limits["arm/right_joint6"] == leader_teleop.DUAL_OPENYAM_LEADER_WRIST_VELOCITY_RAD_S
    for side in ("left", "right"):
        gripper = tasks[f"{side}_arm_gripper"]
        assert (gripper.type, gripper.joint_names) == ("gripper", [f"arm/{side}_gripper"])
        assert gripper.params == {"open_at": "upper"}
        assert gripper.stream_bind == {"gripper_command": f"{side}_gripper_command"}
    leader_module = _atom(blueprint, LeaderTeleopModule)
    assert leader_module.kwargs["mappings"] == leader_teleop.MAPPINGS
    assert leader_module.kwargs["command_filter_cutoff_hz"] == pytest.approx(6.0)
    assert leader_module.kwargs["tick_period_s"] == pytest.approx(0.01)
    assert "reader" not in leader_module.kwargs
    assert _atom(blueprint, ManipulationModule).kwargs["visualization"].backend == "viser"


def test_leader_ports_select_the_sides_from_the_command_line() -> None:
    blueprint = leader_teleop.dual_openyam_leader_teleop
    parsed = BlueprintConfigParser(blueprint).parse(
        ["--leaderteleopmodule.port-right=/dev/ttyACM1", *_CAN_PORTS], environ={}
    )
    config = LeaderTeleopModuleConfig(**parsed.module_kwargs(LeaderTeleopModule.name))
    assert config.sides() == ("right",)
    assert config.port_right == "/dev/ttyACM1"
    assert config.mappings == leader_teleop.MAPPINGS
    kwargs = parsed.module_kwargs(_atom(blueprint, DualOpenYamConnection).name)
    assert (kwargs["left_can_port"], kwargs["right_can_port"]) == ("follower_l", "follower_r")


@pytest.mark.self_hosted
def test_simulation_mocks_the_leaders_and_the_can_buses(mocker: Any) -> None:
    mocker.patch.object(global_config, "simulation", "mujoco")
    leader = leader_teleop.dual_openyam_leader_module()
    (atom,) = leader.blueprints
    assert atom.kwargs["reader"] == "mock"
    assert list(atom.kwargs["mock_positions"].values()) == DUAL_OPENYAM_HOME_PER_ARM
    config = LeaderTeleopModuleConfig(**atom.kwargs)
    assert config.sides() == ("left", "right")

    control = leader_teleop.dual_openyam_leader_control()
    assert not any(issubclass(atom.module, DualOpenYamConnection) for atom in control.blueprints)
    mock = _atom(control, MockJointConnection)
    assert mock.kwargs["source"] == "arm"
    assert control.remapping_map[(mock.name, "position_command")] == "arm_position_command"


def test_blueprint_is_registered() -> None:
    assert all_blueprints["dual-openyam-leader-teleop"].endswith(
        "leader_teleop:dual_openyam_leader_teleop"
    )
