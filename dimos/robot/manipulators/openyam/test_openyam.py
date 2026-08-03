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
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.robot.manipulators.openyam.blueprints.basic import (
    coordinator_openyam,
    openyam_planner_coordinator,
)
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    keyboard_teleop_openyam,
    keyboard_teleop_openyam_planner,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_ARM_JOINTS,
    OPENYAM_DOF,
    OPENYAM_GRAVITY_MODEL_PATH,
    OPENYAM_GRIPPER_JOINT,
    OPENYAM_HARDWARE_ID,
    OPENYAM_JOINTS,
    OPENYAM_PACKAGE_PATHS,
    make_openyam_model_config,
    openyam_hardware,
)


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return _module_kwargs(blueprint, ControlCoordinator)


def test_make_openyam_model_config_default_name_maps_only_six_arm_joints() -> None:
    config = make_openyam_model_config(name="arm")

    assert config.joint_names == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    assert config.joint_name_mapping == {
        "arm/joint1": "joint1",
        "arm/joint2": "joint2",
        "arm/joint3": "joint3",
        "arm/joint4": "joint4",
        "arm/joint5": "joint5",
        "arm/joint6": "joint6",
    }
    assert config.base_link == "base"
    assert config.end_effector_link == "gripper_tip"
    assert list(config.package_paths) == list(OPENYAM_PACKAGE_PATHS)
    assert config.gripper_hardware_id is None


def test_openyam_hardware_physical_mode_returns_one_whole_body(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "")
    monkeypatch.setattr(global_config, "can_port", "can1")

    hardware = openyam_hardware()

    assert hardware.hardware_id == OPENYAM_HARDWARE_ID
    assert hardware.hardware_type is HardwareType.WHOLE_BODY
    assert hardware.adapter_type == "openyam_damiao"
    assert hardware.joints == OPENYAM_JOINTS
    assert hardware.gripper_joints == []
    assert hardware.wb_config is not None
    assert hardware.wb_config.kp == (80.0, 80.0, 80.0, 10.0, 10.0, 10.0, 0.0)
    assert hardware.wb_config.kd == (5.0, 5.0, 5.0, 1.5, 1.5, 1.5, 0.0)
    runtime = hardware.adapter_kwargs["runtime_config"]
    assert isinstance(runtime, DamiaoRuntimeConfig)
    assert runtime.bus_addresses == {"openyam": "can1"}
    assert runtime.gravity_comp is True


def test_openyam_hardware_simulation_mode_returns_generic_whole_body_mock(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")

    hardware = openyam_hardware()

    assert hardware.adapter_type == "mock_whole_body"
    assert hardware.adapter_kwargs == {}
    assert hardware.joints == OPENYAM_JOINTS


def test_openyam_planner_coordinator_trajectory_claims_only_arm_joints() -> None:
    hardware = _coordinator_kwargs(openyam_planner_coordinator)["hardware"][0]
    trajectory = _coordinator_kwargs(openyam_planner_coordinator)["tasks"][0]

    assert hardware.joints == OPENYAM_JOINTS
    assert trajectory.type == "trajectory"
    assert trajectory.joint_names == OPENYAM_ARM_JOINTS
    assert OPENYAM_GRIPPER_JOINT not in trajectory.joint_names


def test_openyam_planner_coordinator_model_uses_arm_planning_group() -> None:
    kwargs = _module_kwargs(openyam_planner_coordinator, ManipulationModule)
    config = ManipulationModuleConfig(**kwargs).robots[0]

    assert config.name == "arm"
    assert config.joint_names == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def test_keyboard_teleop_openyam_planner_eef_task_controls_only_arm() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam_planner)["tasks"]
    eef_twist = next(task for task in tasks if task.type == "eef_twist")

    assert eef_twist.joint_names == OPENYAM_ARM_JOINTS
    assert eef_twist.params == {
        "model_path": OPENYAM_GRAVITY_MODEL_PATH,
        "ee_joint_id": OPENYAM_DOF,
    }


def test_keyboard_teleop_openyam_planner_gripper_task_is_independent_and_idle() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam_planner)["tasks"]
    gripper = next(task for task in tasks if task.name == "servo_gripper")

    assert gripper.joint_names == [OPENYAM_GRIPPER_JOINT]
    assert gripper.params == {"timeout": 0.0}


def test_keyboard_teleop_openyam_planner_trajectory_has_priority_over_eef_task() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam_planner)["tasks"]
    trajectory = next(task for task in tasks if task.type == "trajectory")
    eef_twist = next(task for task in tasks if task.type == "eef_twist")

    assert trajectory.joint_names == OPENYAM_ARM_JOINTS
    assert trajectory.priority == 20
    assert eef_twist.priority == 10


def test_coordinator_openyam_arm_task_claims_six_of_seven_registered_joints() -> None:
    kwargs = _coordinator_kwargs(coordinator_openyam)

    assert kwargs["hardware"][0].joints == OPENYAM_JOINTS
    assert kwargs["tasks"][0].joint_names == OPENYAM_ARM_JOINTS


def test_keyboard_teleop_openyam_eef_task_controls_only_arm() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam)["tasks"]
    eef_twist = next(task for task in tasks if task.type == "eef_twist")

    assert eef_twist.joint_names == OPENYAM_ARM_JOINTS


def test_keyboard_teleop_openyam_gripper_task_has_no_default_position() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam)["tasks"]
    gripper = next(task for task in tasks if task.name == "servo_gripper")

    assert gripper.joint_names == [OPENYAM_GRIPPER_JOINT]
    assert "default_positions" not in gripper.params


def test_keyboard_teleop_openyam_visualization_uses_viser_backend() -> None:
    visualization = _module_kwargs(keyboard_teleop_openyam, ManipulationModule)["visualization"]

    assert visualization == {"backend": "viser"}
