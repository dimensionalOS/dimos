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
from dimos.hardware.manipulators.mock.adapter import MockAdapter
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.robot.manipulators.openyam.blueprints.basic import (
    coordinator_openyam,
    openyam_planner_coordinator,
)
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    keyboard_teleop_openyam,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_DOF,
    OPENYAM_PACKAGE_PATHS,
    make_openyam_hardware,
    make_openyam_model_config,
)


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return _module_kwargs(blueprint, ControlCoordinator)


def test_openyam_model_config_has_expected_links_and_mapping() -> None:
    config = make_openyam_model_config(name="arm")

    assert config.joint_names == [f"yam_joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert config.joint_name_mapping == {
        f"arm/joint{i}": f"yam_joint{i}" for i in range(1, OPENYAM_DOF + 1)
    }
    assert config.base_link == "yam_base_link"
    assert config.end_effector_link == "yam_hand_tcp"
    assert list(config.package_paths) == list(OPENYAM_PACKAGE_PATHS)
    assert config.gripper_hardware_id == "arm"


def test_openyam_mock_hardware_has_gripper() -> None:
    hardware = make_openyam_hardware("arm")

    assert hardware.adapter_type == "mock"
    assert hardware.joints == [f"arm/joint{i}" for i in range(1, OPENYAM_DOF + 1)]
    assert hardware.gripper_joints == ["arm/gripper"]


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

    assert config == make_openyam_model_config(name="arm")
    task = _coordinator_kwargs(blueprint)["tasks"][0]
    assert task.type == "trajectory"
    assert task.joint_names == [f"arm/joint{i}" for i in range(1, OPENYAM_DOF + 1)]


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
    assert _module_kwargs(blueprint, ManipulationModule)["visualization"] == {"backend": "viser"}
