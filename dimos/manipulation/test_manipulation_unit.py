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

"""Unit tests for the ManipulationModule."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.manipulation._test_manipulation_helpers import (
    close_test_runtimes,
    install_runtime,
    make_module as _make_module,
)
from dimos.manipulation.execution_runtime import prepare_generated_plan
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus, PlanningStatus
from dimos.manipulation.planning.spec.models import (
    GeneratedPlan,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


@pytest.fixture
def robot_config():
    """Create a robot config for testing."""
    return RobotModelConfig(
        name="test_arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1", "joint2", "joint3"],
        base_link="link_base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint1", "joint2", "joint3"),
                base_link="link_base",
                tip_link="link_tcp",
            )
        ],
        max_velocity=1.0,
        max_acceleration=2.0,
        coordinator_task_name="traj_arm",
    )


@pytest.fixture(autouse=True)
def _close_test_runtimes() -> Iterator[None]:
    yield
    close_test_runtimes()


@pytest.fixture
def robot_config_with_mapping():
    """Create a robot config with joint name mapping (dual-arm scenario)."""
    return RobotModelConfig(
        name="left_arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1", "joint2", "joint3"],
        base_link="link_base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint1", "joint2", "joint3"),
                base_link="link_base",
                tip_link="link_tcp",
            )
        ],
        joint_name_mapping={
            "left/joint1": "joint1",
            "left/joint2": "joint2",
            "left/joint3": "joint3",
        },
        coordinator_task_name="traj_left",
    )


@pytest.fixture
def simple_trajectory():
    """Create a simple trajectory for testing."""
    return JointTrajectory(
        joint_names=["joint1", "joint2", "joint3"],
        points=[
            TrajectoryPoint(
                positions=[0.0, 0.0, 0.0], velocities=[0.0, 0.0, 0.0], time_from_start=0.0
            ),
            TrajectoryPoint(
                positions=[0.5, 0.5, 0.5], velocities=[0.0, 0.0, 0.0], time_from_start=1.0
            ),
        ],
    )


def _one_joint_config(name: str = "arm") -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path("/path"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["j0"],
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator", joint_names=("j0",), base_link="base_link", tip_link="ee"
            )
        ],
        coordinator_task_name=f"traj_{name}",
    )


def _install_generated_plan(
    module: ManipulationModule,
    config: RobotModelConfig,
    traj_gen: MagicMock,
    *points: list[float],
) -> None:
    """Install a generated plan and enough monitor state to derive robot paths."""
    global_joint_names = [f"{config.name}/{joint}" for joint in config.joint_names]
    module._robots = {config.name: ("robot_id", config, traj_gen)}
    module._world_monitor = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry([config])
    module._world_monitor.get_current_joint_state.return_value = JointState(
        name=config.joint_names,
        position=[0.0 for _ in config.joint_names],
    )
    module._world_monitor.current_global_joint_state.return_value = JointState(
        name=global_joint_names,
        position=[0.0 for _ in config.joint_names],
    )
    generated_plan = GeneratedPlan(
        trajectory=JointTrajectory(
            joint_names=global_joint_names,
            points=[
                TrajectoryPoint(
                    time_from_start=float(index),
                    positions=list(point),
                    velocities=[0.0 for _ in config.joint_names],
                )
                for index, point in enumerate(points)
            ],
        ),
        group_ids=(f"{config.name}/manipulator",),
        status=PlanningStatus.SUCCESS,
        path=[
            JointState(
                name=global_joint_names,
                position=list(point),
            )
            for point in points
        ],
    )
    install_runtime(module, [config])
    token = module._execution_runtime.start_planning()
    prepared = prepare_generated_plan(generated_plan, module._execution_topology)
    assert token is not None
    assert module._execution_runtime.complete_planning(token, prepared).accepted


def _generated_plan_trajectory(joint_names: list[str], *points: list[float]) -> JointTrajectory:
    return JointTrajectory(
        joint_names=joint_names,
        points=[
            TrajectoryPoint(
                time_from_start=float(index),
                positions=list(point),
                velocities=[0.0 for _ in joint_names],
            )
            for index, point in enumerate(points)
        ],
    )


def _make_trajectory(*points: tuple[float, list[float]]) -> JointTrajectory:
    joint_names = [f"j{i}" for i in range(len(points[0][1]))] if points else []
    return JointTrajectory(
        joint_names=joint_names,
        points=[
            TrajectoryPoint(time_from_start=time_from_start, positions=positions)
            for time_from_start, positions in points
        ],
    )


class TestStateMachine:
    """Test state transitions."""


class TestRobotSelection:
    """Test robot selection logic."""

    def test_single_robot_default(self, robot_config):
        """Single robot is used by default."""
        module = _make_module()
        module._robots = {"arm": ("id", robot_config, MagicMock())}

        result = module._get_robot()
        assert result is not None
        assert result[0] == "arm"

    def test_multiple_robots_require_name(self, robot_config):
        """Multiple robots require explicit name."""
        module = _make_module()
        module._robots = {
            "left": ("id1", robot_config, MagicMock()),
            "right": ("id2", robot_config, MagicMock()),
        }

        # No name - fails
        assert module._get_robot() is None

        # With name - works
        result = module._get_robot("left")
        assert result is not None
        assert result[0] == "left"


class PlanningInitializationHarness:
    def __init__(self, mocker: MockerFixture) -> None:
        self.mock_world = MagicMock()
        self.mock_world_monitor = MagicMock(spec=WorldMonitor)
        self.mock_world_monitor.add_robot.return_value = "robot_id"
        self.planning_specs = MagicMock(
            world_monitor=self.mock_world_monitor,
            planner=MagicMock(),
            kinematics=MagicMock(),
        )
        self.mock_planning_specs = mocker.patch(
            "dimos.manipulation.manipulation_module.create_planning_specs",
            return_value=self.planning_specs,
        )
        mocker.patch(
            "dimos.manipulation.manipulation_module.create_world",
            return_value=self.mock_world,
        )
        mocker.patch("dimos.manipulation.manipulation_module.create_manipulation_visualization")
        mocker.patch("dimos.manipulation.manipulation_module.JointTrajectoryGenerator")


@pytest.fixture
def planning_initialization(mocker: MockerFixture) -> PlanningInitializationHarness:
    return PlanningInitializationHarness(mocker)


class TestPlanningInitialization:
    """Test planning backend configuration wiring."""

    def test_default_kinematics_config_uses_pink(self) -> None:
        """Pink IK is the default solver for manipulation modules."""
        config = ManipulationModuleConfig()

        assert isinstance(config.kinematics, PinkKinematicsConfig)

    def test_kinematics_config_is_passed_to_factory(
        self, robot_config, planning_initialization: PlanningInitializationHarness
    ):
        """ManipulationModule config selects the requested IK backend."""
        module = _make_module()
        kinematics = PinkKinematicsConfig(max_iterations=100, dt=0.02)
        module.config = ManipulationModuleConfig(
            robots=[robot_config],
            kinematics=kinematics,
        )

        module._initialize_planning()

        planning_initialization.mock_planning_specs.assert_called_once_with(
            world=planning_initialization.mock_world,
            world_backend="drake",
            planner_name="rrt_connect",
            kinematics_name=None,
            kinematics=kinematics,
        )
        module.stop()

    def test_legacy_kinematics_name_still_selects_backend(
        self, robot_config, planning_initialization: PlanningInitializationHarness
    ):
        """The old kinematics_name field remains a compatibility shim."""
        module = _make_module()
        module.config = ManipulationModuleConfig(
            robots=[robot_config],
            kinematics_name="pink",
        )

        module._initialize_planning()

        planning_initialization.mock_planning_specs.assert_called_once_with(
            world=planning_initialization.mock_world,
            world_backend="drake",
            planner_name="rrt_connect",
            kinematics_name="pink",
            kinematics=module.config.kinematics,
        )
        module.stop()

    def test_nested_kinematics_config_parses_cli_override_shape(self) -> None:
        """Pydantic parses the nested CLI config shape used by -o overrides."""
        config = ManipulationModuleConfig(
            kinematics={
                "backend": "pink",
                "max_iterations": "100",
                "dt": "0.02",
                "posture_cost": "0.0",
            }
        )

        assert isinstance(config.kinematics, PinkKinematicsConfig)
        assert config.kinematics.max_iterations == 100
        assert config.kinematics.dt == 0.02
        assert config.kinematics.posture_cost == 0.0


class TestPlanningGroupApis:
    """Test explicit planning-group API behavior."""

    def test_list_planning_groups_and_robot_info_include_groups(self, robot_config):
        module = _make_module()
        registry = PlanningGroupRegistry([robot_config])
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = registry
        module._init_joints = {}

        groups = module.list_planning_groups()
        info = module.get_robot_info()

        assert [group.id for group in groups] == ["test_arm/manipulator"]
        assert info is not None
        assert info["planning_groups"] == groups
        assert info["end_effector_link"] == "link_tcp"
        assert info["has_joint_name_mapping"] is False

    def test_pose_wrappers_fail_safely_without_unique_pose_group(self, robot_config):
        no_pose_config = RobotModelConfig(
            name="test_arm",
            model_path=robot_config.model_path,
            base_pose=robot_config.base_pose,
            joint_names=robot_config.joint_names,
            base_link=robot_config.base_link,
            planning_groups=[
                PlanningGroupDefinition(
                    name="joint_only",
                    joint_names=("joint1", "joint2", "joint3"),
                    base_link="link_base",
                )
            ],
        )
        module = _make_module()
        module._robots = {"test_arm": ("robot_id", no_pose_config, MagicMock())}
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([no_pose_config])
        module._world_monitor.get_ee_pose.side_effect = ValueError("no pose group")
        module._kinematics = MagicMock()

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())

        assert module.get_ee_pose() is None
        assert module.plan_to_pose(pose) is False
        result = module.inverse_kinematics_single(pose)
        assert result.status == IKStatus.NO_SOLUTION
        assert "no pose-targetable planning group" in result.message

    def test_pose_wrappers_fail_safely_with_multiple_pose_groups(self, robot_config):
        multi_pose_config = RobotModelConfig(
            name="test_arm",
            model_path=robot_config.model_path,
            base_pose=robot_config.base_pose,
            joint_names=robot_config.joint_names,
            base_link=robot_config.base_link,
            planning_groups=[
                PlanningGroupDefinition(
                    name="wrist",
                    joint_names=("joint1", "joint2"),
                    base_link="link_base",
                    tip_link="link_wrist",
                ),
                PlanningGroupDefinition(
                    name="tool",
                    joint_names=("joint1", "joint2", "joint3"),
                    base_link="link_base",
                    tip_link="link_tcp",
                ),
            ],
        )
        module = _make_module()
        module._robots = {"test_arm": ("robot_id", multi_pose_config, MagicMock())}
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([multi_pose_config])
        module._world_monitor.get_ee_pose.side_effect = ValueError("multiple pose groups")
        module._kinematics = MagicMock()

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())

        assert module.get_ee_pose() is None
        assert module.plan_to_pose(pose) is False
        result = module.inverse_kinematics_single(pose)
        assert result.status == IKStatus.NO_SOLUTION
        assert "2 pose-targetable planning groups" in result.message


class TestJointNameTranslation:
    """Test trajectory joint name translation for coordinator."""


class TestExecute:
    """Test coordinator execution."""


class TestRobotModelConfigMapping:
    """Test RobotModelConfig joint name mapping helpers."""

    def test_bidirectional_mapping(self, robot_config_with_mapping):
        """Test URDF <-> coordinator name translation."""
        config = robot_config_with_mapping

        # Coordinator -> URDF
        assert config.get_urdf_joint_name("left/joint1") == "joint1"
        assert config.get_urdf_joint_name("unknown") == "unknown"

        # URDF -> Coordinator
        assert config.get_coordinator_joint_name("joint1") == "left/joint1"
        assert config.get_coordinator_joint_name("unknown") == "unknown"
