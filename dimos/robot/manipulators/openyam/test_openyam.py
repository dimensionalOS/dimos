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
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return _module_kwargs(blueprint, ControlCoordinator)


def test_openyam_model_config_maps_only_six_arm_joints() -> None:
    config = make_openyam_model_config(name="arm")

    assert config.joint_names == [f"joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert config.joint_name_mapping == {
        f"arm/joint{i}": f"joint{i}" for i in range(1, OPENYAM_DOF + 1)
    }
    assert config.base_link == "base"
    assert config.end_effector_link == "gripper_tip"
    assert list(config.package_paths) == list(OPENYAM_PACKAGE_PATHS)
    assert config.gripper_hardware_id is None


def test_openyam_physical_hardware_is_one_whole_body(monkeypatch: Any) -> None:
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
    assert runtime.bus_addresses == {"can": "can1"}
    assert runtime.gravity_comp is True


def test_openyam_simulation_uses_generic_whole_body_mock(monkeypatch: Any) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")

    hardware = openyam_hardware()

    assert hardware.adapter_type == "mock_whole_body"
    assert hardware.adapter_kwargs == {}
    assert hardware.joints == OPENYAM_JOINTS


def test_openyam_planner_blueprint_keeps_gripper_out_of_trajectory() -> None:
    blueprint = openyam_planner_coordinator
    kwargs = _module_kwargs(blueprint, ManipulationModule)
    config = ManipulationModuleConfig(**kwargs).robots[0]
    hardware = _coordinator_kwargs(blueprint)["hardware"][0]
    trajectory = _coordinator_kwargs(blueprint)["tasks"][0]

    assert config.name == "arm"
    assert config.joint_names == [f"joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert hardware.joints == OPENYAM_JOINTS
    assert trajectory.type == "trajectory"
    assert trajectory.joint_names == OPENYAM_ARM_JOINTS
    assert OPENYAM_GRIPPER_JOINT not in trajectory.joint_names
    assert all(atom.module is not KeyboardTeleopModule for atom in blueprint.blueprints)


def test_openyam_keyboard_planner_has_independent_idle_gripper_task() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam_planner)["tasks"]
    trajectory = next(task for task in tasks if task.type == "trajectory")
    eef_twist = next(task for task in tasks if task.type == "eef_twist")
    gripper = next(task for task in tasks if task.name == "servo_gripper")

    assert trajectory.joint_names == OPENYAM_ARM_JOINTS
    assert trajectory.priority == 20
    assert eef_twist.joint_names == OPENYAM_ARM_JOINTS
    assert eef_twist.params == {
        "model_path": OPENYAM_GRAVITY_MODEL_PATH,
        "ee_joint_id": OPENYAM_DOF,
    }
    assert gripper.joint_names == [OPENYAM_GRIPPER_JOINT]
    assert gripper.params == {"timeout": 0.0}


def test_openyam_coordinator_registers_all_joints_but_arm_task_claims_six() -> None:
    kwargs = _coordinator_kwargs(coordinator_openyam)

    assert kwargs["hardware"][0].joints == OPENYAM_JOINTS
    assert kwargs["tasks"][0].joint_names == OPENYAM_ARM_JOINTS


def test_openyam_teleop_uses_separate_arm_and_gripper_tasks() -> None:
    tasks = _coordinator_kwargs(keyboard_teleop_openyam)["tasks"]
    eef_twist = next(task for task in tasks if task.type == "eef_twist")
    gripper = next(task for task in tasks if task.name == "servo_gripper")

    assert eef_twist.joint_names == OPENYAM_ARM_JOINTS
    assert gripper.joint_names == [OPENYAM_GRIPPER_JOINT]
    assert "default_positions" not in gripper.params
    assert _module_kwargs(keyboard_teleop_openyam, ManipulationModule)["visualization"] == {
        "backend": "viser"
    }
