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

import platform
import sys
from typing import cast

import pytest

# The tests extra excludes yourdfpy on Linux ARM because embreex has no wheel.
if sys.platform == "linux" and platform.machine() == "aarch64":
    pytest.importorskip(
        "yourdfpy", reason="yourdfpy is unavailable in the Linux ARM test environment"
    )

from dimos.control.connection_source import ConnectionSource
from dimos.control.contract.limits import limits
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig, TaskConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.gripper_task.gripper_task import GripperControlTask
from dimos.control.tasks.registry import control_task_registry
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.xarm.blueprints.basic import xarm7_planner_coordinator
from dimos.robot.manipulators.xarm.blueprints.grasp import xarm_grasp
from dimos.robot.manipulators.xarm.blueprints.simulation import xarm_perception_sim
from dimos.robot.manipulators.xarm.blueprints.teleop import (
    coordinator_teleop_xarm6,
    coordinator_teleop_xarm7,
    keyboard_teleop_xarm6,
    keyboard_teleop_xarm7,
    xarm6_teleop,
    xarm7_teleop,
)
from dimos.robot.manipulators.xarm.connection import XArmConfig, XArmConnection
from dimos.robot.urdf_manager import UrdfManager

_GRIPPER_BLUEPRINTS = [
    keyboard_teleop_xarm6,
    keyboard_teleop_xarm7,
    coordinator_teleop_xarm6,
    coordinator_teleop_xarm7,
    xarm7_planner_coordinator,
    xarm_perception_sim,
]


def _coordinator_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    kwargs = next(
        atom.kwargs
        for atom in blueprint.blueprints
        if isinstance(atom.module, type) and issubclass(atom.module, ControlCoordinator)
    )
    return cast("list[TaskConfig]", kwargs["tasks"])


@pytest.mark.parametrize(
    "blueprint",
    [
        keyboard_teleop_xarm6,
        keyboard_teleop_xarm7,
        xarm_grasp,
        xarm_perception_sim,
    ],
)
def test_xarm_roboplan_blueprints_use_compatible_visualization(blueprint: Blueprint) -> None:
    manipulation = next(
        atom for atom in blueprint.blueprints if issubclass(atom.module, ManipulationModule)
    )
    config = ManipulationModuleConfig.model_validate(manipulation.kwargs)

    assert config.world_backend == "roboplan"
    assert isinstance(config.visualization, ViserVisualizationConfig)
    assert config.visualization.requires_world_visualization is False


@pytest.mark.parametrize("blueprint", _GRIPPER_BLUEPRINTS)
def test_gripper_has_dedicated_task(blueprint: Blueprint) -> None:
    tasks = _coordinator_tasks(blueprint)
    gripper = next(task for task in tasks if task.type == "gripper")

    assert (gripper.name, gripper.joint_names) == ("arm_gripper", ["arm/gripper"])


@pytest.mark.parametrize("blueprint", [coordinator_teleop_xarm6, coordinator_teleop_xarm7])
def test_trajectory_accepts_gripper(blueprint: Blueprint) -> None:
    trajectory = next(task for task in _coordinator_tasks(blueprint) if task.type == "trajectory")

    assert "arm/gripper" in trajectory.joint_names


@pytest.mark.parametrize("blueprint", [xarm6_teleop, xarm7_teleop])
def test_connection_blueprint_shares_the_contract_declaration(blueprint: Blueprint) -> None:
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={}, overrides={"xarmconnection": {"ip": "192.0.2.1"}}
    )
    coordinator = next(
        atom for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    )
    connection = next(atom for atom in blueprint.blueprints if atom.module is XArmConnection)
    config = ControlCoordinatorConfig.model_validate(parsed.module_kwargs(coordinator.name))
    driver = XArmConfig.model_validate(parsed.module_kwargs(connection.name))
    assert len(config.connections) == 1
    assert limits(config.connections[0]) == limits(driver)
    assert config.connections[0].model == driver.model

    assert "model" not in ControlCoordinatorConfig.model_fields
    assert any(atom.module is UrdfManager for atom in blueprint.blueprints)
    gripper_cfg = next(task for task in _coordinator_tasks(blueprint) if task.type == "gripper")
    assert gripper_cfg.joint_names == ["arm/drive_joint"]
    assert gripper_cfg.stream_bind == {"gripper_command": "right_gripper_command"}
    source = ConnectionSource(
        config.connections[0], ["position_command", "velocity_command"], ["joint_state"], 1.0
    )
    gripper = control_task_registry.create("gripper", gripper_cfg, connections={"arm": source})
    assert isinstance(gripper, GripperControlTask)
    state = CoordinatorState(joints=JointStateSnapshot(), t_now=0.0)
    for opening, target in [(1.0, 0.0), (0.0, 0.85)]:
        assert gripper.set_normalized([opening])
        output = gripper.compute(state)
        assert output is not None
        assert output.positions == pytest.approx([target])
