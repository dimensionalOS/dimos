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

from dimos.control.components import HardwareType
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.manipulation.planning.spec.validation import validate_robot_model_config
from dimos.robot.manipulators.openyam.blueprints.basic import (
    coordinator_openyam,
    openyam_planner_coordinator,
)
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    keyboard_teleop_openyam,
    keyboard_teleop_openyam_planner,
    teleop_quest_openyam,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_ARM_JOINTS,
    OPENYAM_DOF,
    OPENYAM_GRIPPER_JOINT,
    OPENYAM_HARDWARE_ID,
    OPENYAM_HOME_JOINTS,
    OPENYAM_JOINTS,
    OPENYAM_MODEL_PATH,
    make_openyam_model_config,
    openyam_hardware,
)
from dimos.robot.manipulators.openyam.teleop_ik import OpenYamPinkPoseTargetSolver
from dimos.teleop.quest.quest_extensions import ArmTeleopModule


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(
        atom.kwargs for atom in blueprint.blueprints if issubclass(atom.module, module_type)
    )


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return next(
        atom.kwargs for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    )


def test_make_openyam_model_config_uses_canonical_arm_joints() -> None:
    config = make_openyam_model_config()

    assert config.model.source_path is OPENYAM_MODEL_PATH
    assert config.joint_names == OPENYAM_ARM_JOINTS
    assert config.base_link == "base"
    assert config.planning_groups[0].tip_link == "gripper_tip"
    assert config.gripper_hardware_id == "arm"
    assert config.home_joints == OPENYAM_HOME_JOINTS
    assert config.velocity_limits == [2.0] * OPENYAM_DOF
    assert config.max_velocity == 2.0


@pytest.mark.self_hosted
def test_openyam_model_contains_canonical_arm_joints() -> None:
    config = make_openyam_model_config()
    model = validate_robot_model_config(config)

    assert [joint.name for joint in model.joints if joint.name in config.joint_names] == (
        OPENYAM_ARM_JOINTS
    )


def test_make_openyam_model_config_preserves_explicit_home() -> None:
    configured_home = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    config = make_openyam_model_config(home_joints=configured_home)

    assert config.home_joints == configured_home


def test_quest_teleop_matches_dual_openyam_response_tuning() -> None:
    tasks = _coordinator_kwargs(teleop_quest_openyam)["tasks"]
    teleop = next(task for task in tasks if task.type == "teleop_ik")

    assert teleop.params["pink"].gain == 1.0
    assert teleop.params["solver_type"] is OpenYamPinkPoseTargetSolver
    assert teleop.params["max_joint_velocity_rad_s"] == 2.0
    assert teleop.params["joint_command_filter_cutoff_hz"] == 30.0


def test_openyam_hardware_physical_mode_returns_one_whole_body(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "")
    monkeypatch.setattr(global_config, "can_port", "can1")

    hardware = openyam_hardware()

    assert (hardware.hardware_id, hardware.hardware_type, hardware.adapter_type) == (
        OPENYAM_HARDWARE_ID,
        HardwareType.WHOLE_BODY,
        "openyam_damiao",
    )
    assert hardware.adapter_kwargs["runtime_config"].bus_devices == {"openyam": "can1"}


def test_openyam_hardware_without_can_port_uses_platform_default(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "")
    monkeypatch.setattr(global_config, "can_port", None)

    hardware = openyam_hardware()

    assert hardware.adapter_kwargs["runtime_config"].bus_devices == {}


def test_openyam_hardware_simulation_mode_returns_generic_whole_body_mock(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")

    hardware = openyam_hardware()

    assert hardware.adapter_type == "mock_whole_body"
    limits = hardware.limits
    assert limits is not None
    assert limits.position_lower == [*([None] * OPENYAM_DOF), 0.0]
    assert limits.position_upper == [*([None] * OPENYAM_DOF), 1.0]


def test_quest_teleop_module_accepts_blueprint_config() -> None:
    kwargs = _module_kwargs(teleop_quest_openyam, ArmTeleopModule)

    module = ArmTeleopModule(**kwargs)
    module.stop()


@pytest.mark.parametrize(
    "blueprint",
    [
        coordinator_openyam,
        openyam_planner_coordinator,
    ],
)
def test_openyam_basic_trajectory_accepts_all_hardware_joints(blueprint: Blueprint) -> None:
    kwargs = _coordinator_kwargs(blueprint)

    assert kwargs["hardware"][0].joints == OPENYAM_JOINTS
    trajectory = next(task for task in kwargs["tasks"] if task.type == "trajectory")
    gripper = next(task for task in kwargs["tasks"] if task.type == "gripper")

    assert trajectory.joint_names == OPENYAM_JOINTS
    assert (gripper.name, gripper.joint_names) == (
        "openyam_gripper",
        [OPENYAM_GRIPPER_JOINT],
    )


@pytest.mark.parametrize(
    "blueprint",
    [
        keyboard_teleop_openyam,
        keyboard_teleop_openyam_planner,
    ],
)
def test_openyam_teleop_blueprints_expose_arm_and_gripper_control(
    blueprint: Blueprint,
) -> None:
    kwargs = _coordinator_kwargs(blueprint)
    claimed_joints = [task.joint_names for task in kwargs["tasks"]]

    assert kwargs["hardware"][0].joints == OPENYAM_JOINTS
    assert OPENYAM_ARM_JOINTS in claimed_joints
    assert all(
        joints in (OPENYAM_ARM_JOINTS, [OPENYAM_GRIPPER_JOINT], OPENYAM_JOINTS)
        for joints in claimed_joints
    )


def test_keyboard_teleop_gripper_control_is_independent() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam)["tasks"]
    gripper = next(task for task in tasks if task.name == "openyam_gripper")

    assert gripper.joint_names == [OPENYAM_GRIPPER_JOINT]
    assert gripper.type == "gripper"


def test_keyboard_teleop_openyam_planner_trajectory_has_priority_over_eef_task() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam_planner)["tasks"]
    trajectory = next(task for task in tasks if task.type == "trajectory")
    eef_twist = next(task for task in tasks if task.type == "eef_twist")

    assert trajectory.joint_names == OPENYAM_JOINTS
    assert trajectory.priority == 20
    assert eef_twist.priority == 10


def test_keyboard_teleop_openyam_gripper_task_has_no_extra_params() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam)["tasks"]
    gripper = next(task for task in tasks if task.name == "openyam_gripper")

    assert gripper.joint_names == [OPENYAM_GRIPPER_JOINT]
    assert gripper.params == {}


def test_quest_teleop_routes_pose_and_gripper_to_separate_tasks() -> None:
    tasks = _coordinator_kwargs(teleop_quest_openyam)["tasks"]
    teleop = next(task for task in tasks if task.type == "teleop_ik")
    gripper = next(task for task in tasks if task.type == "gripper")

    assert teleop.params["bindings"] == [{"hand": "right", "target_frame": "gripper_tip"}]
    assert gripper.joint_names == [OPENYAM_GRIPPER_JOINT]
    assert gripper.stream_bind == {"gripper_command": "right_gripper_command"}
