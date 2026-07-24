# Copyright 2025-2026 Dimensional Inc.
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

"""Focused tests for manipulation planning wiring."""

from __future__ import annotations

from collections.abc import Callable, Generator
from pathlib import Path
import sys
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.factory import (
    create_kinematics,
    create_planner,
    create_planning_stack,
    create_world,
    validate_backend_combination,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.kinematics.config import JacobianKinematicsConfig
from dimos.manipulation.planning.kinematics.jacobian_ik import JacobianIK
from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.protocols import PlannerSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


@pytest.fixture
def make_module() -> Generator[Callable[..., ManipulationModule], None, None]:
    """Build ManipulationModules and stop them on teardown, even on failure."""
    modules: list[ManipulationModule] = []

    def _make(**kwargs: Any) -> ManipulationModule:
        module = ManipulationModule(**kwargs)
        modules.append(module)
        return module

    yield _make
    for module in modules:
        module.stop()


@pytest.fixture
def robot_config() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        joint_names=["joint1", "joint2"],
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint1", "joint2"),
                base_link="base_link",
                tip_link="tcp",
            )
        ],
        coordinator_task_name="traj_arm",
    )


def test_create_world_unknown_backend() -> None:
    with pytest.raises(
        ValueError, match=r"Unknown backend: fake\. Available: \['drake', 'roboplan'\]"
    ):
        create_world(backend="fake")


def test_factory_selects_expected_implementations() -> None:
    assert isinstance(create_planner(name="rrt_connect"), RRTConnectPlanner)
    assert isinstance(create_kinematics(name="jacobian"), JacobianIK)


def test_default_planner_path_does_not_import_roboplan(monkeypatch: pytest.MonkeyPatch) -> None:
    for module_name in list(sys.modules):
        if module_name == "roboplan" or module_name.startswith("roboplan."):
            monkeypatch.delitem(sys.modules, module_name, raising=False)

    create_planner(name="rrt_connect")
    validate_backend_combination()

    assert "roboplan.core" not in sys.modules
    assert "roboplan.rrt" not in sys.modules


def test_validate_backend_combination_rejects_invalid_combinations() -> None:
    with pytest.raises(
        ValueError, match='planner_name="roboplan" requires world_backend="roboplan"'
    ):
        validate_backend_combination(world_backend="drake", planner_name="roboplan")

    with pytest.raises(
        ValueError, match='kinematics_name="drake_optimization" requires world_backend="drake"'
    ):
        validate_backend_combination(world_backend="roboplan", kinematics_name="drake_optimization")


def test_create_planner_uses_roboplan_world_as_native_planner(mocker: MockerFixture) -> None:
    world = mocker.MagicMock(spec=PlannerSpec)

    assert create_planner(name="roboplan", world=world, world_backend="roboplan") is world


def test_create_planner_rejects_roboplan_without_roboplan_world(mocker: MockerFixture) -> None:
    with pytest.raises(
        ValueError, match='planner_name="roboplan" requires world_backend="roboplan"'
    ):
        create_planner(name="roboplan", world=mocker.MagicMock(), world_backend="drake")


def test_create_planning_stack_wires_selected_components(
    mocker: MockerFixture, robot_config: RobotModelConfig
) -> None:
    world = mocker.MagicMock()
    world.add_robot.return_value = "robot-id"

    kinematics = mocker.MagicMock(name="kinematics")
    planner = mocker.MagicMock(name="planner")

    mock_world = mocker.patch(
        "dimos.manipulation.planning.factory.create_world", return_value=world
    )
    mock_kinematics = mocker.patch(
        "dimos.manipulation.planning.factory.create_kinematics",
        return_value=kinematics,
    )
    mock_planner = mocker.patch(
        "dimos.manipulation.planning.factory.create_planner",
        return_value=planner,
    )

    result = create_planning_stack(
        robot_config,
        world_backend="drake",
        planner_name="rrt_connect",
        kinematics_name="jacobian",
    )

    assert result == (world, kinematics, planner, "robot-id")
    mock_world.assert_called_once_with(backend="drake", visualization=None)
    mock_kinematics.assert_called_once_with(config=JacobianKinematicsConfig())
    mock_planner.assert_called_once_with(name="rrt_connect", world=world, world_backend="drake")
    world.add_robot.assert_called_once_with(robot_config)
    world.finalize.assert_called_once()


def test_start_with_no_robots_skips_planning(
    mocker: MockerFixture, make_module: Callable[..., ManipulationModule]
) -> None:
    module = make_module(robots=[])
    create_world_mock = mocker.patch("dimos.manipulation.manipulation_module.create_world")
    create_planning_specs_mock = mocker.patch(
        "dimos.manipulation.manipulation_module.create_planning_specs"
    )

    module._initialize_planning()

    assert module._robots == {}
    assert module._world_monitor is None
    create_world_mock.assert_not_called()
    create_planning_specs_mock.assert_not_called()


def test_start_uses_configured_planner_and_kinematics(
    mocker: MockerFixture,
    robot_config: RobotModelConfig,
    make_module: Callable[..., ManipulationModule],
) -> None:
    module = make_module(robots=[robot_config], kinematics=JacobianKinematicsConfig())
    world = mocker.MagicMock(name="world")
    world_monitor = mocker.MagicMock()
    world_monitor.add_robot.return_value = "robot-id"
    planner = mocker.MagicMock(name="planner")
    kinematics = mocker.MagicMock(name="kinematics")
    planning_specs = mocker.MagicMock(
        world_monitor=world_monitor,
        planner=planner,
        kinematics=kinematics,
    )
    create_world_mock = mocker.patch(
        "dimos.manipulation.manipulation_module.create_world", return_value=world
    )
    create_planning_specs_mock = mocker.patch(
        "dimos.manipulation.manipulation_module.create_planning_specs",
        return_value=planning_specs,
    )

    module._initialize_planning()

    create_world_mock.assert_called_once_with(
        backend="drake", visualization=module.config.visualization
    )
    create_planning_specs_mock.assert_called_once_with(
        world=world,
        world_backend="drake",
        planner_name="rrt_connect",
        kinematics_name=None,
        kinematics=module.config.kinematics,
    )
    assert module._planner is planner
    assert module._kinematics is kinematics
    assert module._robots["arm"][0] == "robot-id"
