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

from pathlib import Path
from typing import Any

import pytest

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.visualization.config import (
    NoManipulationVisualizationConfig,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.get_all_blueprints import get_blueprint_by_name
from dimos.robot.manipulators.a1z.blueprints.teleop import keyboard_teleop_a1z
from dimos.robot.manipulators.a750.blueprints.teleop import keyboard_teleop_a750
from dimos.robot.manipulators.common.blueprints import eef_twist_task, planner
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.openarm.blueprints.teleop import (
    keyboard_teleop_openarm,
    keyboard_teleop_openarm_mock,
)
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    keyboard_teleop_openyam,
)
from dimos.robot.manipulators.piper.blueprints.teleop import (
    coordinator_teleop_piper,
    keyboard_teleop_piper,
)
from dimos.robot.manipulators.xarm.blueprints.basic import (
    dual_xarm6_planner,
    dual_xarm6_planner_coordinator,
    xarm6_planner_only,
    xarm7_planner_coordinator,
)
from dimos.robot.manipulators.xarm.blueprints.teleop import (
    keyboard_teleop_xarm6,
    keyboard_teleop_xarm7,
)
from dimos.robot.manipulators.xarm.config import (
    make_xarm7_model_config,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
    make_xarm_hardware,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModuleConfig
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.teleop.quest.blueprints import teleop_quest_piper
from dimos.teleop.quest.quest_extensions import ArmTeleopModule


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _manipulation_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return _module_kwargs(blueprint, ManipulationModule)


def _manipulation_config(blueprint: Blueprint) -> ManipulationModuleConfig:
    return ManipulationModuleConfig(**_manipulation_kwargs(blueprint))


def _coordinator_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    return _module_kwargs(blueprint, ControlCoordinator)["tasks"]


def test_quest_piper_teleop_routes_to_declarative_teleop_task() -> None:
    arm_kwargs = _module_kwargs(teleop_quest_piper, ArmTeleopModule)
    assert arm_kwargs["task_names"] == {"left": "teleop_piper"}
    assert "coordinator_cartesian_command" in teleop_quest_piper.remapping_map.values()


def test_piper_teleop_blueprints_declare_viser_manipulation() -> None:
    for blueprint in (keyboard_teleop_piper, coordinator_teleop_piper):
        kwargs = _manipulation_kwargs(blueprint)
        assert kwargs["robots"][0].coordinator_task_name == "traj_arm"
        assert kwargs["visualization"] == {"backend": "viser"}


def test_quest_piper_composes_planner_with_trajectory_coordinator() -> None:
    assert _module_kwargs(coordinator_teleop_piper, ControlCoordinator)
    assert _module_kwargs(coordinator_teleop_piper, ManipulationModule)
    coordinator_planner = next(
        atom for atom in coordinator_teleop_piper.blueprints if atom.module is ManipulationModule
    )
    quest_planners = [
        atom for atom in teleop_quest_piper.blueprints if atom.module is ManipulationModule
    ]
    assert quest_planners == [coordinator_planner]


def test_piper_teleop_declares_teleop_task() -> None:
    tasks = _coordinator_tasks(coordinator_teleop_piper)
    assert [(task.name, task.type) for task in tasks] == [
        ("teleop_piper", "teleop_ik"),
        ("traj_arm", "trajectory"),
    ]


def test_piper_keyboard_declares_high_priority_gripper_servo() -> None:
    tasks = _coordinator_tasks(keyboard_teleop_piper)
    servo = next(task for task in tasks if task.name == "servo_gripper")
    trajectory = next(task for task in tasks if task.name == "traj_arm")
    assert servo.type == "servo"
    assert servo.joint_names == ["arm/gripper"]
    assert servo.priority > next(task.priority for task in tasks if task.type == "eef_twist")
    assert trajectory.type == "trajectory"


def test_planner_helper_defaults_to_no_visualization() -> None:
    blueprint = planner(robots=[make_xarm7_model_config(name="arm", add_gripper=True)])

    kwargs = _manipulation_kwargs(blueprint)
    config = ManipulationModuleConfig(**kwargs)

    assert "visualization" not in kwargs
    assert isinstance(config.visualization, NoManipulationVisualizationConfig)


def test_planner_helper_preserves_explicit_visualization() -> None:
    blueprint = planner(
        robots=[make_xarm7_model_config(name="arm", add_gripper=True)],
        visualization={"backend": "meshcat"},
    )

    assert _manipulation_kwargs(blueprint)["visualization"] == {"backend": "meshcat"}


def test_xarm_planner_blueprints_default_to_no_visualization() -> None:
    for blueprint in (xarm6_planner_only, dual_xarm6_planner, xarm7_planner_coordinator):
        config = _manipulation_config(blueprint)

        assert isinstance(config.visualization, NoManipulationVisualizationConfig)


def test_dual_xarm6_planner_coordinator_blueprints_preserve_visualization_backends() -> None:
    assert get_blueprint_by_name("dual-xarm6-planner-coordinator") is dual_xarm6_planner_coordinator

    config = _manipulation_config(dual_xarm6_planner_coordinator)
    coordinator_kwargs = next(
        atom.kwargs
        for atom in dual_xarm6_planner_coordinator.blueprints
        if atom.module is ControlCoordinator
    )

    assert isinstance(config.visualization, ViserVisualizationConfig)
    assert [robot.name for robot in config.robots] == ["left_arm", "right_arm"]
    assert [hardware.hardware_id for hardware in coordinator_kwargs["hardware"]] == [
        "left_arm",
        "right_arm",
    ]
    assert [task.name for task in coordinator_kwargs["tasks"]] == [
        "traj_left_arm",
        "traj_right_arm",
    ]


def test_xarm_perception_sim_uses_aligned_camera_frame() -> None:
    sim_robot = make_xarm7_sim_robot_config()
    sim_config = MujocoSimModuleConfig(
        **make_xarm7_sim_module_kwargs("test-xarm7-scene.xml"),
    )

    assert sim_robot.xacro_args["attach_rpy"] == "0 0 0"
    assert sim_config.base_frame_id == "link7"
    assert sim_config.reset_joint_positions == sim_robot.home_joints


def test_eef_twist_task_helper_uses_hardware_joints_and_default_name() -> None:
    hardware = make_xarm_hardware("arm", 6, adapter_type="mock")

    task = eef_twist_task(hardware, model_path=Path("fake.urdf"), ee_joint_id=6)

    assert task.name == EEF_TWIST_TASK_NAME
    assert task.type == "eef_twist"
    assert task.joint_names == hardware.joints
    assert task.params == {"model_path": Path("fake.urdf"), "ee_joint_id": 6}


@pytest.mark.parametrize(
    "blueprint",
    [
        pytest.param(keyboard_teleop_xarm6, id="xarm6"),
        pytest.param(keyboard_teleop_xarm7, id="xarm7"),
        pytest.param(keyboard_teleop_piper, id="piper"),
        pytest.param(keyboard_teleop_openarm_mock, id="openarm-mock"),
        pytest.param(keyboard_teleop_openarm, id="openarm"),
        pytest.param(keyboard_teleop_openyam, id="openyam"),
        pytest.param(keyboard_teleop_a750, id="a750"),
        pytest.param(keyboard_teleop_a1z, id="a1z"),
    ],
)
def test_manipulator_keyboard_blueprint_uses_eef_twist_and_light_keyboard_kwargs(
    blueprint: Blueprint,
) -> None:
    keyboard_kwargs = _module_kwargs(blueprint, KeyboardTeleopModule)
    coordinator_tasks = _coordinator_tasks(blueprint)
    eef_twist_tasks = [task for task in coordinator_tasks if task.type == "eef_twist"]

    assert keyboard_kwargs == {}
    assert [task.name for task in eef_twist_tasks] == [EEF_TWIST_TASK_NAME]
    assert all(task.type != "cartesian_ik" for task in coordinator_tasks)
