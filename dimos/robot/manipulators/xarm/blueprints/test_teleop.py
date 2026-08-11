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

import pytest

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.xarm.blueprints.teleop import (
    coordinator_teleop_xarm6,
    coordinator_teleop_xarm7,
    keyboard_teleop_xarm6,
    keyboard_teleop_xarm7,
)

_GRIPPER_BLUEPRINTS = [
    keyboard_teleop_xarm6,
    keyboard_teleop_xarm7,
    coordinator_teleop_xarm6,
    coordinator_teleop_xarm7,
]


def _coordinator_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    kwargs = next(atom.kwargs for atom in blueprint.blueprints if atom.module is ControlCoordinator)
    return kwargs["tasks"]


@pytest.mark.parametrize("blueprint", [keyboard_teleop_xarm6, keyboard_teleop_xarm7])
def test_keyboard_teleop_uses_roboplan_compatible_visualization(blueprint: Blueprint) -> None:
    manipulation = next(atom for atom in blueprint.blueprints if atom.module is ManipulationModule)
    config = ManipulationModuleConfig.model_validate(manipulation.kwargs)

    assert config.world_backend == "roboplan"
    assert isinstance(config.visualization, ViserVisualizationConfig)
    assert config.visualization.requires_world_visualization is False


@pytest.mark.parametrize("blueprint", _GRIPPER_BLUEPRINTS)
def test_gripper_joint_has_one_configured_owner(blueprint: Blueprint) -> None:
    owners = [task for task in _coordinator_tasks(blueprint) if "arm/gripper" in task.joint_names]

    assert [(task.name, task.type) for task in owners] == [("arm_gripper", "gripper")]
