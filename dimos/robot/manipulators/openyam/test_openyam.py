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

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.hardware.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.manipulators.mock.adapter import MockAdapter
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
    OPENYAM_DOF,
    OPENYAM_GRAVITY_MODEL_PATH,
    OPENYAM_PACKAGE_PATHS,
    make_openyam_hardware,
    make_openyam_model_config,
    openyam_hardware,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return _module_kwargs(blueprint, ControlCoordinator)


def test_openyam_model_config_has_expected_links_and_mapping() -> None:
    config = make_openyam_model_config(name="arm")

    assert config.joint_names == [f"joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert config.joint_name_mapping == {
        f"arm/joint{i}": f"joint{i}" for i in range(1, OPENYAM_DOF + 1)
    }
    assert config.base_link == "base"
    assert config.end_effector_link == "gripper_tip"
    assert list(config.package_paths) == list(OPENYAM_PACKAGE_PATHS)
    assert config.gripper_hardware_id == "arm"


def test_openyam_mock_hardware_has_gripper() -> None:
    hardware = make_openyam_hardware("arm")

    assert hardware.adapter_type == "mock"
    assert hardware.joints == [f"arm/joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert hardware.gripper_joints == ["arm/gripper"]


def test_openyam_physical_hardware_uses_registered_damiao_adapter(monkeypatch: Any) -> None:
    monkeypatch.setattr(global_config, "simulation", "")
    monkeypatch.setattr(global_config, "can_port", "can1")

    hardware = openyam_hardware("arm")

    assert hardware.adapter_type == "openyam_damiao"
    assert hardware.address == "can1"
    runtime_config = hardware.adapter_kwargs["runtime_config"]
    assert isinstance(runtime_config, DamiaoRuntimeConfig)
    assert runtime_config.gravity_model_path == OPENYAM_GRAVITY_MODEL_PATH
    assert runtime_config.gravity_comp is True
    assert len(hardware.joints) == OPENYAM_DOF
    assert hardware.gripper_joints == []
    assert "initial_positions" not in hardware.adapter_kwargs

    direct = make_openyam_hardware(
        "arm",
        adapter_type="openyam_damiao",
        home_joints=[0.1] * OPENYAM_DOF,
    )
    assert "initial_positions" not in direct.adapter_kwargs


def test_openyam_simulation_hardware_remains_mock(monkeypatch: Any) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")

    hardware = openyam_hardware("arm")

    assert hardware.adapter_type == "mock"
    assert hardware.address is None


def test_openyam_mock_adapter_set_get_behavior() -> None:
    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    adapter = MockAdapter(dof=OPENYAM_DOF, initial_positions=positions)

    assert adapter.read_joint_positions() == positions
    updated_positions = [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6]
    assert adapter.write_joint_positions(updated_positions)
    assert adapter.read_joint_positions() == updated_positions
    assert adapter.write_gripper_position(0.25)
    assert adapter.read_gripper_position() == 0.25


def test_openyam_planner_blueprint_preserves_model_config() -> None:
    blueprint = openyam_planner_coordinator
    kwargs = _module_kwargs(blueprint, ManipulationModule)
    config = ManipulationModuleConfig(**kwargs).robots[0]

    assert config.name == "arm"
    assert config.joint_names == [f"joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert config.end_effector_link == "gripper_tip"
    assert config.gripper_hardware_id == "arm"
    tasks = _coordinator_kwargs(blueprint)["tasks"]
    assert len(tasks) == 1
    trajectory = tasks[0]
    assert trajectory.type == "trajectory"
    assert trajectory.joint_names == [f"arm/joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert trajectory.priority == 10
    assert all(atom.module is not KeyboardTeleopModule for atom in blueprint.blueprints)


def test_openyam_keyboard_planner_blueprint_combines_teleop_and_trajectory() -> None:
    blueprint = keyboard_teleop_openyam_planner
    tasks = _coordinator_kwargs(blueprint)["tasks"]
    trajectory = next(task for task in tasks if task.type == "trajectory")
    eef_twist = next(task for task in tasks if task.type == "eef_twist")

    assert trajectory.joint_names == [f"arm/joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert trajectory.priority == 20
    assert eef_twist.joint_names == trajectory.joint_names
    assert eef_twist.params["ee_joint_id"] == OPENYAM_DOF
    assert eef_twist.params["model_path"] == OPENYAM_GRAVITY_MODEL_PATH
    assert eef_twist.priority == 10
    assert _module_kwargs(blueprint, KeyboardTeleopModule) == {}


def test_openyam_coordinator_blueprint_uses_six_arm_joints() -> None:
    blueprint = coordinator_openyam
    kwargs = _coordinator_kwargs(blueprint)
    assert len(kwargs["hardware"]) == 1
    assert len(kwargs["hardware"][0].joints) == OPENYAM_DOF
    assert kwargs["tasks"][0].joint_names == kwargs["hardware"][0].joints


def test_openyam_teleop_blueprint_constructs_with_eef_twist() -> None:
    blueprint = keyboard_teleop_openyam
    task = next(
        task for task in _coordinator_kwargs(blueprint)["tasks"] if task.type == "eef_twist"
    )

    assert task.joint_names == [f"arm/joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert task.params["ee_joint_id"] == OPENYAM_DOF
    assert task.params["model_path"] == OPENYAM_GRAVITY_MODEL_PATH
    assert _module_kwargs(blueprint, ManipulationModule)["visualization"] == {"backend": "viser"}
