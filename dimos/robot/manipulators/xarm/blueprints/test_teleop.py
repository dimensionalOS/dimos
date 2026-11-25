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
import runpy
import sys
from types import ModuleType
from typing import Any, cast, get_type_hints

import pytest

# The tests extra excludes yourdfpy on Linux ARM because embreex has no wheel.
if sys.platform == "linux" and platform.machine() == "aarch64":
    pytest.importorskip(
        "yourdfpy", reason="yourdfpy is unavailable in the Linux ARM test environment"
    )

from dimos.control.connection.connection_module import ConnectionModule
from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.connection_source import ConnectionSource
from dimos.control.contract.limits import limits, movable_joints
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig, TaskConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.gripper_task.gripper_task import GripperControlTask
from dimos.control.tasks.registry import control_task_registry
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.xarm import config as xarm_config
from dimos.robot.manipulators.xarm.blueprints import basic, teleop
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
from dimos.robot.usd_manager import UsdManager
from dimos.simulation.engines.mujoco_arm_connection import MujocoArmConnection
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

_GRIPPER_BLUEPRINTS = [
    keyboard_teleop_xarm6.solidify(),
    keyboard_teleop_xarm7.solidify(),
    coordinator_teleop_xarm6.solidify(),
    coordinator_teleop_xarm7.solidify(),
    xarm7_planner_coordinator.solidify(),
    xarm_perception_sim.solidify(),
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
        keyboard_teleop_xarm6.solidify(),
        keyboard_teleop_xarm7.solidify(),
        xarm_grasp.solidify(),
        xarm_perception_sim.solidify(),
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

    assert (gripper.name, gripper.joint_names) == ("arm_gripper", ["arm/drive_joint"])


@pytest.mark.parametrize(
    "blueprint",
    [
        coordinator_teleop_xarm6.solidify(),
        coordinator_teleop_xarm7.solidify(),
        xarm7_planner_coordinator.solidify(),
    ],
)
def test_trajectory_accepts_gripper(blueprint: Blueprint) -> None:
    trajectory = next(task for task in _coordinator_tasks(blueprint) if task.type == "trajectory")

    assert "arm/drive_joint" in trajectory.joint_names


# The blueprints that run their arm and its gripper in MuJoCo under --simulation.
_SIM_BLUEPRINTS: dict[str, ModuleType] = {
    "keyboard_teleop_xarm6": teleop,
    "keyboard_teleop_xarm7": teleop,
    "coordinator_teleop_xarm6": teleop,
    "coordinator_teleop_xarm7": teleop,
    "coordinator_xarm6": basic,
}


def _under_simulation(module: ModuleType, monkeypatch: pytest.MonkeyPatch) -> dict[str, Any]:
    """What the module defines when it is built under ``--simulation``.

    Blueprints are built at import, so this runs the module's source again and
    leaves the imported module as it is. Plain strings stand in for the scenes,
    because building a sim blueprint from an ``LfsPath`` downloads the scene.
    """
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    monkeypatch.setattr(basic, "SIMULATED", True)
    monkeypatch.setattr(xarm_config, "XARM6_SIM_PATH", "xarm6/scene.xml")
    monkeypatch.setattr(xarm_config, "XARM7_SIM_PATH", "xarm7/scene.xml")
    # teleop builds its connections with the imported basic, which already holds the LfsPaths.
    monkeypatch.setattr(
        basic, "XARM_SIM_SCENES", {"xarm6": "xarm6/scene.xml", "xarm7": "xarm7/scene.xml"}
    )
    assert module.__file__ is not None
    return runpy.run_path(module.__file__)


@pytest.mark.parametrize("name", _SIM_BLUEPRINTS)
def test_simulation_runs_the_arm_and_gripper_in_mujoco(
    name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    blueprint = autoconnect(_under_simulation(_SIM_BLUEPRINTS[name], monkeypatch)[name]).solidify()
    connection = next(atom for atom in blueprint.blueprints if atom.module is MujocoArmConnection)
    sim = next(atom for atom in blueprint.blueprints if atom.module is MujocoSimModule)

    assert not any(atom.module is XArmConnection for atom in blueprint.blueprints)
    assert connection.kwargs["gripper_joint"] == "arm/drive_joint"
    # The sim finds the gripper only in a scene joint past its dof.
    assert sim.kwargs["dof"] == len(movable_joints(connection.kwargs["model"])) - 1


@pytest.mark.parametrize(
    ("choices", "connection", "overrides"),
    [
        ({}, XArmConnection, {"xarmconnection": {"ip": "192.0.2.1"}}),
        ({"arm": "mock"}, MockJointConnection, {}),
    ],
)
def test_arm_choice_runs_the_real_arm_or_a_mock_with_its_joints(
    choices: dict[str, str], connection: type[ConnectionModule], overrides: dict[str, Any]
) -> None:
    blueprint = keyboard_teleop_xarm7.solidify(choices=choices)
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides=overrides)
    coordinator = next(
        atom for atom in blueprint.blueprints if issubclass(atom.module, ControlCoordinator)
    )
    (arm,) = [atom for atom in blueprint.blueprints if issubclass(atom.module, ConnectionModule)]
    declared = ControlCoordinatorConfig.model_validate(parsed.module_kwargs(coordinator.name))
    driver = get_type_hints(connection)["config"].model_validate(parsed.module_kwargs(arm.name))

    assert arm.module is connection
    assert limits(driver) == limits(declared.connections[0])
    assert list(limits(driver)) == [
        *(f"arm/joint{i}/position" for i in range(1, 8)),
        "arm/drive_joint/position",
    ]
    ports = ("position_command", "velocity_command", "joint_state")
    assert [blueprint.remapping_map[(arm.name, port)] for port in ports] == [
        f"arm_{port}" for port in ports
    ]


@pytest.mark.parametrize("blueprint", [xarm6_teleop.solidify(), xarm7_teleop.solidify()])
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
    assert any(atom.module is UsdManager for atom in blueprint.blueprints)
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
