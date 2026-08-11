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
        if atom.module is ControlCoordinator
    )
    return cast("list[TaskConfig]", kwargs["tasks"])


@pytest.mark.parametrize("blueprint", [keyboard_teleop_piper, coordinator_teleop_piper])
def test_gripper_joint_has_one_configured_owner(blueprint: Blueprint) -> None:
    owners = [task for task in _coordinator_tasks(blueprint) if "arm/gripper" in task.joint_names]

    assert [(task.name, task.type) for task in owners] == [("arm_gripper", "gripper")]
