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

from typing import Any, cast

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.robot.manipulators.a750.blueprints.teleop import keyboard_teleop_a750


def _coordinator_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    kwargs: dict[str, Any] = next(
        atom.kwargs
        for atom in blueprint.blueprints
        if isinstance(atom.module, type) and issubclass(atom.module, ControlCoordinator)
    )
    return cast("list[TaskConfig]", kwargs["tasks"])


def test_keyboard_teleop_a750_wires_joint_trajectory_execution() -> None:
    tasks = _coordinator_tasks(keyboard_teleop_a750)

    trajectory = next(task for task in tasks if task.name == "joint_trajectory")
    gripper = next(task for task in tasks if task.type == "gripper")

    assert trajectory.type == "trajectory"
    assert trajectory.joint_names == [
        *(f"arm/joint{i}" for i in range(1, 7)),
        "arm/finger",
    ]
    assert (gripper.name, gripper.joint_names) == ("arm_gripper", ["arm/finger"])
