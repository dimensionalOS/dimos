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

from pathlib import Path
import sys
from unittest.mock import MagicMock, patch

import pytest

from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.planning.factory import (
    create_kinematics,
    create_planner,
    create_planning_stack,
    create_world,
    validate_backend_combination,
)
from dimos.manipulation.planning.kinematics.config import JacobianKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


@pytest.fixture
def robot_config() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        joint_names=["joint1", "joint2"],
        end_effector_link="tcp",
        coordinator_task_name="traj_arm",
    )


def test_create_world_unknown_backend():
    with pytest.raises(
        ValueError, match=r"Unknown backend: fake\. Available: \['drake', 'roboplan'\]"
    ):
        create_world(backend="fake")


def test_factory_selects_expected_implementations():
    assert create_planner(name="rrt_connect") is not None
    assert create_kinematics(name="jacobian") is not None


def test_default_planner_path_does_not_import_roboplan(monkeypatch):
    for module_name in list(sys.modules):
        if module_name == "roboplan" or module_name.startswith("roboplan."):
            monkeypatch.delitem(sys.modules, module_name, raising=False)

    create_planner(name="rrt_connect")
    validate_backend_combination()

    assert "roboplan.core" not in sys.modules
    assert "roboplan.rrt" not in sys.modules


def test_validate_backend_combination_rejects_invalid_combinations():
    with pytest.raises(
        ValueError, match='planner_name="roboplan" requires world_backend="roboplan"'
    ):
        validate_backend_combination(world_backend="drake", planner_name="roboplan")

    with pytest.raises(
        ValueError, match='kinematics_name="drake_optimization" requires world_backend="drake"'
    ):
        validate_backend_combination(world_backend="roboplan", kinematics_name="drake_optimization")


def test_create_planner_uses_roboplan_world_as_native_planner():
    world = MagicMock()
    world.plan_joint_path = MagicMock()

    assert create_planner(name="roboplan", world=world, world_backend="roboplan") is world


def test_create_planner_rejects_roboplan_without_roboplan_world():
    with pytest.raises(
        ValueError, match='planner_name="roboplan" requires world_backend="roboplan"'
    ):
        create_planner(name="roboplan", world=MagicMock(), world_backend="drake")


def test_create_planning_stack_wires_selected_components(robot_config):
    world = MagicMock()
    world.add_robot.return_value = "robot-id"

    kinematics = MagicMock(name="kinematics")
    planner = MagicMock(name="planner")

    with (
        patch("dimos.manipulation.planning.factory.create_world", return_value=world) as mock_world,
        patch(
            "dimos.manipulation.planning.factory.create_kinematics",
            return_value=kinematics,
        ) as mock_kinematics,
        patch(
            "dimos.manipulation.planning.factory.create_planner",
            return_value=planner,
        ) as mock_planner,
    ):
        result = create_planning_stack(
            robot_config,
            enable_viz=True,
            world_backend="drake",
            planner_name="rrt_connect",
            kinematics_name="jacobian",
        )

    assert result == (world, kinematics, planner, "robot-id")
    mock_world.assert_called_once_with(backend="drake", enable_viz=True)
    mock_kinematics.assert_called_once_with(config=JacobianKinematicsConfig())
    mock_planner.assert_called_once_with(name="rrt_connect", world=world, world_backend="drake")
    world.add_robot.assert_called_once_with(robot_config)
    world.finalize.assert_called_once()


def test_start_with_no_robots_skips_planning(monkeypatch):
    module = ManipulationModule.__new__(ManipulationModule)
    module.config = ManipulationModuleConfig(robots=[], enable_viz=False)
    module._robots = {}
    module._world_monitor = None

    world_monitor = MagicMock()
    monkeypatch.setattr(
        "dimos.manipulation.manipulation_module.WorldMonitor", MagicMock(return_value=world_monitor)
    )
    create_planner_mock = MagicMock()
    create_kinematics_mock = MagicMock()
    monkeypatch.setattr(
        "dimos.manipulation.manipulation_module.create_planner", create_planner_mock
    )
    monkeypatch.setattr(
        "dimos.manipulation.manipulation_module.create_kinematics", create_kinematics_mock
    )

    module._initialize_planning()

    assert module._robots == {}
    assert module._world_monitor is None
    create_planner_mock.assert_not_called()
    create_kinematics_mock.assert_not_called()


def test_start_uses_configured_planner_and_kinematics(monkeypatch, robot_config):
    module = ManipulationModule.__new__(ManipulationModule)
    module.config = ManipulationModuleConfig(robots=[robot_config], enable_viz=False)
    module._robots = {}
    module._world_monitor = None
    module._tf_stop_event = MagicMock()

    world_monitor = MagicMock()
    world_monitor.add_robot.return_value = "robot-id"
    world_monitor.world = MagicMock(name="world")
    world_monitor_cls = MagicMock(return_value=world_monitor)
    monkeypatch.setattr("dimos.manipulation.manipulation_module.WorldMonitor", world_monitor_cls)

    planner = MagicMock(name="planner")
    kinematics = MagicMock(name="kinematics")
    create_planner_mock = MagicMock(return_value=planner)
    create_kinematics_mock = MagicMock(return_value=kinematics)
    monkeypatch.setattr(
        "dimos.manipulation.manipulation_module.create_planner", create_planner_mock
    )
    monkeypatch.setattr(
        "dimos.manipulation.manipulation_module.create_kinematics", create_kinematics_mock
    )

    module._initialize_planning()

    world_monitor_cls.assert_called_once_with(backend="drake", enable_viz=False)
    create_planner_mock.assert_called_once_with(
        name="rrt_connect", world=world_monitor.world, world_backend="drake"
    )
    create_kinematics_mock.assert_called_once_with(config=JacobianKinematicsConfig())
    assert module._planner is planner
    assert module._kinematics is kinematics
