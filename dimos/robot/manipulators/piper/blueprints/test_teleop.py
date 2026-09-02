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

import pytest

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.robot.manipulators.piper.blueprints.teleop import (
    coordinator_teleop_piper,
    keyboard_teleop_piper,
)


def _coordinator_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    kwargs = next(
        cast("dict[str, Any]", atom.kwargs)
        for atom in blueprint.blueprints
        if isinstance(atom.module, type) and issubclass(atom.module, ControlCoordinator)
    )
    return cast("list[TaskConfig]", kwargs["tasks"])


@pytest.mark.parametrize("blueprint", [keyboard_teleop_piper, coordinator_teleop_piper])
def test_trajectory_accepts_gripper_and_gripper_has_dedicated_task(
    blueprint: Blueprint,
) -> None:
    tasks = _coordinator_tasks(blueprint)
    trajectory = next(task for task in tasks if task.type == "trajectory")
    gripper = next(task for task in tasks if task.type == "gripper")

    assert "arm/gripper" in trajectory.joint_names
    assert (gripper.name, gripper.joint_names) == ("arm_gripper", ["arm/gripper"])
