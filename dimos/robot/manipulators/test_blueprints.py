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

"""Piper teleop blueprint composition.

The xarm/planner-helper cases that used to live here were dropped on main
along with the blueprints they covered (`xarm6_planner_only`,
`dual_xarm6_planner`); only the Piper teleop coverage is kept.
"""

from typing import Any

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.piper.blueprints.teleop import (
    coordinator_teleop_piper,
    keyboard_teleop_piper,
)
from dimos.teleop.quest.blueprints import teleop_quest_piper
from dimos.teleop.quest.quest_extensions import ArmTeleopModule


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    """Kwargs of the atom for *module_type*, preferring an exact class match.

    Blueprints may declare a subclass to add ports - Piper's teleop
    coordinator is a ControlCoordinator subclass - so fall back to a
    subclass match rather than requiring class identity.
    """
    atoms = blueprint.blueprints
    exact = (atom for atom in atoms if atom.module is module_type)
    derived = (
        atom
        for atom in atoms
        if isinstance(atom.module, type) and issubclass(atom.module, module_type)
    )
    return next(atom.kwargs for atom in (*exact, *derived))


def _manipulation_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return _module_kwargs(blueprint, ManipulationModule)


def _coordinator_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    return _module_kwargs(blueprint, ControlCoordinator)["tasks"]


def test_quest_piper_teleop_routes_to_declarative_teleop_task() -> None:
    arm_kwargs = _module_kwargs(teleop_quest_piper, ArmTeleopModule)
    assert arm_kwargs["task_names"] == {"left": "teleop_piper"}
    assert "coordinator_cartesian_command" in teleop_quest_piper.remapping_map.values()


def test_piper_teleop_blueprints_declare_viser_manipulation() -> None:
    for blueprint in (keyboard_teleop_piper, coordinator_teleop_piper):
        kwargs = _manipulation_kwargs(blueprint)
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
