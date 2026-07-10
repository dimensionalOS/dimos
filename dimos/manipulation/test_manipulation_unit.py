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

from pathlib import Path
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.manipulation._test_manipulation_helpers import make_module as _make_module
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
    ManipulationState,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus, PlanningStatus
from dimos.manipulation.planning.spec.models import (
    GeneratedPlan,
    IKResult,
    PlanningResult,
    PlanningSceneInfo,
)
from dimos.manipulation.planning.spec.protocols import VisualizationSpec
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
    module._robots = {config.name: ("robot_id", config, traj_gen)}
    module._world_monitor = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry([config])
    module._world_monitor.get_current_joint_state.return_value = JointState(
        name=config.joint_names,
        position=[0.0 for _ in config.joint_names],
    )
    module._last_plan = GeneratedPlan(
        group_ids=(f"{config.name}/manipulator",),
        status=PlanningStatus.SUCCESS,
        path=[
            JointState(
                name=[f"{config.name}/{joint}" for joint in config.joint_names],
                position=list(point),
            )
            for point in points
        ],
    )


class TestStateMachine:
    """Test state transitions."""

    def test_cancel_interrupts_active_work(self):
        """Cancel works for executing motion and in-progress planning."""
        module = _make_module()

        module._state = ManipulationState.IDLE
        assert module.cancel() is False

        module._state = ManipulationState.PLANNING
        assert module.cancel() is True
        assert module._state == ManipulationState.IDLE
        assert module._planning_epoch == 1

        module._state = ManipulationState.EXECUTING
        assert module.cancel() is True
        assert module._state == ManipulationState.IDLE

    def test_cancel_hides_active_plan_preview(self):
        module = _make_module()
        module._state = ManipulationState.EXECUTING
        module._last_plan = GeneratedPlan(group_ids=("arm/manipulator",), path=[])
        module._world_monitor = MagicMock()

        assert module.cancel() is True

        module._world_monitor.hide_preview.assert_called_once_with(("arm/manipulator",))
        module._world_monitor.publish_visualization.assert_called_once_with()

    def test_reset_not_during_execution(self):
        """Reset works in any state except EXECUTING."""
        module = _make_module()

        module._state = ManipulationState.FAULT
        module._error_message = "Error"
        result = module.reset()
        assert result.is_success()
        assert module._state == ManipulationState.IDLE
        assert module._error_message == ""

        module._state = ManipulationState.EXECUTING
        result = module.reset()
        assert not result.is_success()
        assert result.error_code == "INVALID_STATE"

    def test_fail_sets_fault_state(self):
        """_fail helper sets FAULT state and message."""
        module = _make_module()
        module._state = ManipulationState.PLANNING

        result = module._fail("Test error")
        assert result is False
        assert module._state == ManipulationState.FAULT
        assert module._error_message == "Test error"

    def test_begin_planning_state_checks(self, robot_config):
        """_begin_planning only allowed from IDLE or COMPLETED."""
        module = _make_module()
        module._world_monitor = MagicMock()
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}

        # From IDLE - OK
        module._state = ManipulationState.IDLE
        assert module._begin_planning() == ("test_arm", "robot_id")
        assert module._state == ManipulationState.PLANNING

        # From COMPLETED - OK
        module._state = ManipulationState.COMPLETED
        assert module._begin_planning() == ("test_arm", "robot_id")

        # From EXECUTING - Fail
        module._state = ManipulationState.EXECUTING
        assert module._begin_planning() is None


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

    def test_solve_ik_rpc_calls_configured_backend(self, robot_config):
        """solve_ik returns the backend IKResult without path planning."""
        module = _make_module()
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([robot_config])
        current = JointState(name=robot_config.joint_names, position=[0.0, 0.0, 0.0])
        current_global = JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.current_global_joint_state.return_value = current_global
        expected = IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(name=robot_config.joint_names, position=[0.1, 0.2, 0.3]),
            position_error=0.0001,
            orientation_error=0.0002,
            iterations=3,
            message="ok",
        )
        module._kinematics = MagicMock()
        module._kinematics.solve_pose_targets.return_value = expected

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())
        result = module.solve_ik(pose)

        assert result is expected
        assert module._state == ManipulationState.COMPLETED
        assert module._last_plan is None
        module._kinematics.solve_pose_targets.assert_called_once()
        _, kwargs = module._kinematics.solve_pose_targets.call_args
        assert kwargs["world"] is module._world_monitor.world
        assert kwargs["seed"].name == current_global.name
        assert kwargs["seed"].position == current.position
        assert kwargs["check_collision"] is True
        [(group, target_pose)] = kwargs["pose_targets"].items()
        assert group.id == "test_arm/manipulator"
        assert target_pose.frame_id == "world"
        assert target_pose.position.x == 0.45

    def test_solve_ik_rpc_returns_failure_without_joint_state(self, robot_config):
        """solve_ik reports a failed IKResult when no seed state is available."""
        module = _make_module()
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([robot_config])
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=[], position=[]
        )
        module._kinematics = MagicMock()

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())
        result = module.solve_ik(pose)

        assert result.status == IKStatus.NO_SOLUTION
        assert result.message == "No joint state"
        assert module._state == ManipulationState.IDLE
        module._kinematics.solve_pose_targets.assert_not_called()

    def test_solve_ik_rpc_uses_explicit_seed(self, robot_config):
        """solve_ik initializes the backend from an explicit seed when provided."""
        module = _make_module()
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([robot_config])
        explicit_seed = JointState(name=robot_config.joint_names, position=[0.2, 0.1, 0.0])
        expected = IKResult(status=IKStatus.SUCCESS, joint_state=explicit_seed)
        module._kinematics = MagicMock()
        module._kinematics.solve_pose_targets.return_value = expected

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())
        result = module.solve_ik(pose, seed=explicit_seed)

        assert result is expected
        _, kwargs = module._kinematics.solve_pose_targets.call_args
        assert kwargs["seed"] is explicit_seed
        module._world_monitor.current_global_joint_state.assert_not_called()


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

    def test_plan_to_joint_targets_stores_generated_plan_and_legacy_caches(self, robot_config):
        module = _make_module()
        registry = PlanningGroupRegistry([robot_config])
        traj_gen = MagicMock()
        traj_gen.generate.return_value = _make_trajectory(
            (0.0, [0.0, 0.0, 0.0]), (1.0, [0.1, 0.2, 0.3])
        )
        module._robots = {"test_arm": ("robot_id", robot_config, traj_gen)}
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.planning_groups = registry
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=robot_config.joint_names,
            position=[0.0, 0.0, 0.0],
        )
        result_path = [
            JointState(
                name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                position=[0.0, 0.0, 0.0],
            ),
            JointState(
                name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                position=[0.1, 0.2, 0.3],
            ),
        ]
        module._planner = MagicMock()
        module._planner.plan_selected_joint_path.return_value = PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=result_path,
            planning_time=0.01,
            path_length=0.3,
            iterations=4,
            message="ok",
        )

        success = module.plan_to_joint_targets(
            {
                "test_arm/manipulator": JointState(
                    name=robot_config.joint_names,
                    position=[0.1, 0.2, 0.3],
                )
            }
        )

        assert success is True
        assert module._last_plan is not None
        assert module._last_plan.group_ids == ("test_arm/manipulator",)
        assert module._last_plan.path == result_path
        projected = module._project_plan_to_robot_paths(module._last_plan)
        assert projected is not None
        assert projected["test_arm"][1].position == [0.1, 0.2, 0.3]
        assert module._trajectory_from_robot_path("test_arm", projected["test_arm"]) is not None
        module._planner.plan_selected_joint_path.assert_called_once()
        _, kwargs = module._planner.plan_selected_joint_path.call_args
        assert kwargs["selection"].group_ids == ("test_arm/manipulator",)
        assert kwargs["goal"].name == [
            "test_arm/joint1",
            "test_arm/joint2",
            "test_arm/joint3",
        ]

        success = module.plan_to_joint_targets(
            {
                "test_arm/manipulator": JointState(
                    name=robot_config.joint_names,
                    position=[0.1, 0.2, 0.3],
                )
            }
        )

        assert success is True
        assert module._planner.plan_selected_joint_path.call_count == 2

    def test_plan_to_pose_targets_uses_group_ik_and_selected_path(self, robot_config):
        module = _make_module()
        registry = PlanningGroupRegistry([robot_config])
        traj_gen = MagicMock()
        traj_gen.generate.return_value = _make_trajectory(
            (0.0, [0.0, 0.0, 0.0]), (1.0, [0.1, 0.2, 0.3])
        )
        module._robots = {"test_arm": ("robot_id", robot_config, traj_gen)}
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.planning_groups = registry
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=robot_config.joint_names,
            position=[0.0, 0.0, 0.0],
        )
        ik_goal = JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
            position=[0.1, 0.2, 0.3],
        )
        module._kinematics = MagicMock()
        module._kinematics.solve_pose_targets.return_value = IKResult(
            status=IKStatus.SUCCESS,
            joint_state=ik_goal,
        )
        module._planner = MagicMock()
        module._planner.plan_selected_joint_path.return_value = PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=[
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.0, 0.0, 0.0],
                ),
                ik_goal,
            ],
        )
        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())

        success = module.plan_to_pose_targets({"test_arm/manipulator": pose})

        assert success is True
        module._kinematics.solve_pose_targets.assert_called_once()
        _, ik_kwargs = module._kinematics.solve_pose_targets.call_args
        target_groups = list(ik_kwargs["pose_targets"].keys())
        assert [group.id for group in target_groups] == ["test_arm/manipulator"]
        target_pose = ik_kwargs["pose_targets"][target_groups[0]]
        assert target_pose.position.x == 0.45
        assert ik_kwargs["seed"].name == [
            "test_arm/joint1",
            "test_arm/joint2",
            "test_arm/joint3",
        ]
        _, planner_kwargs = module._planner.plan_selected_joint_path.call_args
        assert planner_kwargs["goal"] is ik_goal

    def test_failed_plan_projection_clears_generated_plan(self, robot_config, simple_trajectory):
        module = _make_module()
        registry = PlanningGroupRegistry([robot_config])
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.planning_groups = registry
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.get_current_joint_state.return_value = None
        module._last_plan = GeneratedPlan(
            group_ids=("test_arm/manipulator",),
            status=PlanningStatus.SUCCESS,
            path=[
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.0, 0.0, 0.0],
                ),
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.2, 0.2, 0.2],
                ),
            ],
        )
        module._planner = MagicMock()
        module._planner.plan_selected_joint_path.return_value = PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=[
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.1, 0.2, 0.3],
                )
            ],
        )

        success = module.plan_to_joint_targets(
            {"test_arm/manipulator": JointState(position=[0.1, 0.2, 0.3])}
        )

        assert success is False
        assert module._state == ManipulationState.FAULT
        assert module._last_plan is None
        assert module.has_planned_path() is False

    def test_execute_plan_dispatches_each_affected_robot(self, robot_config, simple_trajectory):
        module = _make_module()
        registry = PlanningGroupRegistry([robot_config])
        traj_gen = MagicMock()
        traj_gen.generate.return_value = simple_trajectory
        module._robots = {"test_arm": ("robot_id", robot_config, traj_gen)}
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = registry
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=robot_config.joint_names,
            position=[0.0, 0.0, 0.0],
        )
        module._last_plan = GeneratedPlan(
            group_ids=("test_arm/manipulator",),
            status=PlanningStatus.SUCCESS,
            path=[
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.0, 0.0, 0.0],
                ),
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.1, 0.2, 0.3],
                ),
            ],
        )
        mock_client = MagicMock()
        mock_client.task_invoke.return_value = True
        module._coordinator_client = mock_client

        assert module.execute_plan() is True

        mock_client.task_invoke.assert_called_once()
        task_name, method_name, payload = mock_client.task_invoke.call_args.args
        assert task_name == "traj_arm"
        assert method_name == "execute"
        trajectory = payload["trajectory"]
        assert trajectory.joint_names == simple_trajectory.joint_names
        assert trajectory.points == simple_trajectory.points
        assert module._state == ManipulationState.COMPLETED

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

    def test_no_mapping_returns_original(self, robot_config, simple_trajectory):
        """Without mapping, trajectory is returned unchanged."""
        module = _make_module()

        result = module._translate_trajectory_to_coordinator(simple_trajectory, robot_config)
        assert result is simple_trajectory  # Same object

    def test_mapping_translates_names(self, robot_config_with_mapping, simple_trajectory):
        """With mapping, joint names are translated."""
        module = _make_module()

        result = module._translate_trajectory_to_coordinator(
            simple_trajectory, robot_config_with_mapping
        )
        assert result.joint_names == ["left/joint1", "left/joint2", "left/joint3"]
        assert len(result.points) == 2  # Points preserved


class TestExecute:
    """Test coordinator execution."""

    def test_execute_requires_trajectory(self, robot_config):
        """Execute fails without planned trajectory."""
        module = _make_module()
        module._robots = {"test_arm": ("id", robot_config, MagicMock())}

        assert module.execute() is False
        assert module._state == ManipulationState.IDLE

    def test_execute_requires_task_name(self):
        """Execute fails without coordinator_task_name."""
        module = _make_module()
        config_no_task = RobotModelConfig(
            name="arm",
            model_path=Path("/path"),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j1"],
            planning_groups=[
                PlanningGroupDefinition(
                    name="manipulator", joint_names=("j1",), base_link="base_link", tip_link="ee"
                )
            ],
        )
        traj_gen = MagicMock()
        traj_gen.generate.return_value = _make_trajectory((0.0, [0.0]), (1.0, [0.1]))
        _install_generated_plan(module, config_no_task, traj_gen, [0.0], [0.1])

        assert module.execute("arm") is False
        assert module._state == ManipulationState.IDLE

    def test_execute_plan_requires_task_name_without_sticking_executing(self, robot_config):
        module = _make_module()
        config_no_task = RobotModelConfig(
            name="test_arm",
            model_path=robot_config.model_path,
            base_pose=robot_config.base_pose,
            joint_names=robot_config.joint_names,
            base_link=robot_config.base_link,
            planning_groups=robot_config.planning_groups,
            gripper_hardware_id="test_gripper",
        )
        traj_gen = MagicMock()
        traj_gen.generate.return_value = _make_trajectory(
            (0.0, [0.0, 0.0, 0.0]), (1.0, [0.1, 0.2, 0.3])
        )
        module._robots = {"test_arm": ("id", config_no_task, traj_gen)}
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([config_no_task])
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=robot_config.joint_names,
            position=[0.0, 0.0, 0.0],
        )
        module._coordinator_client = MagicMock()
        module._last_plan = GeneratedPlan(
            group_ids=("test_arm/manipulator",),
            status=PlanningStatus.SUCCESS,
            path=[
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.0, 0.0, 0.0],
                ),
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.1, 0.2, 0.3],
                ),
            ],
        )

        assert module.execute_plan() is False
        assert module._state == ManipulationState.FAULT

    def test_execute_success(self, robot_config, simple_trajectory):
        """Successful execute calls coordinator via task_invoke."""
        module = _make_module()
        traj_gen = MagicMock()
        traj_gen.generate.return_value = simple_trajectory
        _install_generated_plan(module, robot_config, traj_gen, [0.0, 0.0, 0.0], [0.1, 0.2, 0.3])

        mock_client = MagicMock()
        mock_client.task_invoke.return_value = True
        module._coordinator_client = mock_client

        assert module.execute("test_arm") is True
        assert module._state == ManipulationState.COMPLETED
        mock_client.task_invoke.assert_called_once()
        task_name, method_name, payload = mock_client.task_invoke.call_args.args
        assert task_name == "traj_arm"
        assert method_name == "execute"
        trajectory = payload["trajectory"]
        assert trajectory.joint_names == simple_trajectory.joint_names
        assert trajectory.points == simple_trajectory.points

    def test_execute_rejected(self, robot_config, simple_trajectory):
        """Rejected execution sets FAULT state."""
        module = _make_module()
        traj_gen = MagicMock()
        traj_gen.generate.return_value = simple_trajectory
        _install_generated_plan(module, robot_config, traj_gen, [0.0, 0.0, 0.0], [0.1, 0.2, 0.3])

        mock_client = MagicMock()
        mock_client.task_invoke.return_value = False
        module._coordinator_client = mock_client

        assert module.execute("test_arm") is False
        assert module._state == ManipulationState.FAULT


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


def _make_module_with_monitor(*configs: RobotModelConfig) -> ManipulationModule:
    """Create a ManipulationModule with a mocked world monitor and robots configured."""
    module = _make_module()
    module._world_monitor = MagicMock()
    module._init_joints = {}
    for config in configs:
        robot_id = f"robot_{config.name}"
        module._robots[config.name] = (robot_id, config, MagicMock())
    return module


def _make_joint_state(positions: list[float], name: list[str] | None = None) -> JointState:
    return JointState(name=name or [f"j{i}" for i in range(len(positions))], position=positions)


def _make_path(*points: list[float]) -> list[JointState]:
    return [_make_joint_state(list(point)) for point in points]


def _make_trajectory(*points: tuple[float, list[float]]) -> JointTrajectory:
    joint_names = [f"j{i}" for i in range(len(points[0][1]))] if points else []
    return JointTrajectory(
        joint_names=joint_names,
        points=[
            TrajectoryPoint(time_from_start=time_from_start, positions=positions)
            for time_from_start, positions in points
        ],
    )


def _make_world_monitor_with_viz(viz: VisualizationSpec | None) -> WorldMonitor:
    world = MagicMock()
    return WorldMonitor(
        world=world,
        visualization=viz,
    )


class FakeVisualization:
    def __init__(self) -> None:
        self.close_count = 0
        self.published = False
        self.preview_shown: list[str] = []
        self.preview_hidden: list[str] = []
        self.animations: list[tuple[str, list[JointState], float]] = []
        self.preview_animation_cancellations = 0

    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        pass

    def get_visualization_url(self) -> str | None:
        return "123"

    def publish_visualization(self, ctx: object | None = None) -> None:
        self.published = True

    def show_preview(self, group_ids: list[str]) -> None:
        self.preview_shown.extend(group_ids)

    def hide_preview(self, group_ids: list[str]) -> None:
        self.preview_hidden.extend(group_ids)

    def animate_plan(self, plan: GeneratedPlan, duration: float = 3.0) -> None:
        self.animations.append((plan.group_ids[0], plan.path, duration))

    def cancel_preview_animation(self) -> None:
        self.preview_animation_cancellations += 1

    def close(self) -> None:
        self.close_count += 1


class TestOnJointState:
    """Test _on_joint_state routing, splitting, and init capture."""

    def test_routes_positions_to_monitor(self, robot_config_with_mapping):
        """Joint positions from aggregated message are routed to the correct monitor."""
        module = _make_module_with_monitor(robot_config_with_mapping)

        msg = JointState(
            name=["left/joint1", "left/joint2", "left/joint3"],
            position=[0.1, 0.2, 0.3],
            velocity=[1.0, 2.0, 3.0],
        )
        module._on_joint_state(msg)

        # Verify world_monitor received the sub-message
        module._world_monitor.on_joint_state.assert_called_once()
        call_args = module._world_monitor.on_joint_state.call_args
        sub_msg = call_args[0][0]
        assert sub_msg.position == [0.1, 0.2, 0.3]
        assert sub_msg.velocity == [1.0, 2.0, 3.0]
        assert call_args[1]["robot_id"] == "robot_left_arm"

    def test_skips_robot_with_missing_joints(self, robot_config_with_mapping):
        """Robots whose joints are absent from the message are skipped."""
        module = _make_module_with_monitor(robot_config_with_mapping)

        # Message has none of left_arm's joints
        msg = JointState(
            name=["right/joint1", "right/joint2"],
            position=[0.5, 0.6],
        )
        module._on_joint_state(msg)

        module._world_monitor.on_joint_state.assert_not_called()

    def test_captures_init_joints_on_first_call(self, robot_config_with_mapping):
        """First joint state is stored as init joints; subsequent calls don't overwrite."""
        module = _make_module_with_monitor(robot_config_with_mapping)

        first_msg = JointState(
            name=["left/joint1", "left/joint2", "left/joint3"],
            position=[0.1, 0.2, 0.3],
        )
        module._on_joint_state(first_msg)
        assert "left_arm" in module._init_joints
        assert module._init_joints["left_arm"].position == [0.1, 0.2, 0.3]

        # Second call should NOT overwrite
        second_msg = JointState(
            name=["left/joint1", "left/joint2", "left/joint3"],
            position=[0.9, 0.8, 0.7],
        )
        module._on_joint_state(second_msg)
        assert module._init_joints["left_arm"].position == [0.1, 0.2, 0.3]

    def test_multi_robot_splits_correctly(self):
        """With two robots, each gets only its own joints from the aggregated message."""
        left_config = RobotModelConfig(
            name="left",
            model_path=Path("/path/to/robot.urdf"),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j1", "j2"],
            base_link="base",
            planning_groups=[
                PlanningGroupDefinition(
                    name="manipulator", joint_names=("j1", "j2"), base_link="base", tip_link="ee"
                )
            ],
            joint_name_mapping={"left/j1": "j1", "left/j2": "j2"},
            coordinator_task_name="traj_left",
        )
        right_config = RobotModelConfig(
            name="right",
            model_path=Path("/path/to/robot.urdf"),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j1", "j2"],
            base_link="base",
            planning_groups=[
                PlanningGroupDefinition(
                    name="manipulator", joint_names=("j1", "j2"), base_link="base", tip_link="ee"
                )
            ],
            joint_name_mapping={"right/j1": "j1", "right/j2": "j2"},
            coordinator_task_name="traj_right",
        )
        module = _make_module_with_monitor(left_config, right_config)

        msg = JointState(
            name=["left/j1", "left/j2", "right/j1", "right/j2"],
            position=[1.0, 2.0, 3.0, 4.0],
            velocity=[0.1, 0.2, 0.3, 0.4],
        )
        module._on_joint_state(msg)

        assert module._world_monitor.on_joint_state.call_count == 2

        # Collect calls by robot_id
        calls = {
            call[1]["robot_id"]: call[0][0]
            for call in module._world_monitor.on_joint_state.call_args_list
        }
        assert calls["robot_left"].position == [1.0, 2.0]
        assert calls["robot_right"].position == [3.0, 4.0]
        assert calls["robot_left"].velocity == [0.1, 0.2]
        assert calls["robot_right"].velocity == [0.3, 0.4]

    def test_no_monitor_returns_early(self, robot_config_with_mapping):
        """When world_monitor is None, _on_joint_state returns without error."""
        module = _make_module()
        module._robots = {"left_arm": ("id", robot_config_with_mapping, MagicMock())}
        module._world_monitor = None

        # Should not raise
        msg = JointState(
            name=["left/joint1", "left/joint2", "left/joint3"],
            position=[0.1, 0.2, 0.3],
        )
        module._on_joint_state(msg)


class TestWorldMonitorVisualization:
    def test_visualization_routing_and_stop_all_monitors(self):
        viz = FakeVisualization()
        monitor = _make_world_monitor_with_viz(viz)
        state_monitor = MagicMock()
        obstacle_monitor = MagicMock()
        monitor._state_monitors = {"robot": state_monitor}
        monitor._obstacle_monitor = obstacle_monitor
        monitor._viz_thread = MagicMock()
        monitor._viz_thread.is_alive.return_value = False

        assert monitor.get_visualization_url() == "123"
        monitor.publish_visualization()
        monitor.show_preview(["robot/group"])
        monitor.hide_preview(["robot/group"])
        path = _make_path([1.0], [2.0], [3.0])
        plan = GeneratedPlan(group_ids=("robot/group",), status=PlanningStatus.SUCCESS, path=path)
        monitor.animate_plan(plan, 4.5)
        assert monitor.visualization is viz
        assert viz.published is True
        assert viz.preview_shown == ["robot/group"]
        assert viz.preview_hidden == ["robot/group"]
        assert viz.animations == [("robot/group", path, 4.5)]

        monitor.stop_all_monitors()

        assert viz.close_count == 1
        state_monitor.stop.assert_called_once()
        obstacle_monitor.stop.assert_called_once()

    def test_visualization_none_is_noop(self):
        monitor = _make_world_monitor_with_viz(None)

        assert monitor.get_visualization_url() is None
        monitor.publish_visualization()
        monitor.show_preview(["robot/group"])
        monitor.hide_preview(["robot/group"])
        monitor.animate_plan(GeneratedPlan(group_ids=("robot/group",), path=[]), 1.0)
        monitor.start_visualization_thread()
        assert monitor._viz_thread is None


class TestManipulationPreview:
    def test_clear_planned_path_invalidates_before_dismissing_preview(self):
        module = _make_module()
        plan = GeneratedPlan(group_ids=("arm/manipulator",), path=[])
        module._last_plan = plan
        module._world_monitor = MagicMock()
        plan_during_dismissal: list[GeneratedPlan | None] = []
        module._world_monitor.hide_preview.side_effect = (
            lambda _group_ids: plan_during_dismissal.append(module._last_plan)
        )

        assert module.clear_planned_path() is True

        assert plan_during_dismissal == [None]
        module._world_monitor.hide_preview.assert_called_once_with(("arm/manipulator",))
        module._world_monitor.publish_visualization.assert_called_once_with()
        assert module._last_plan is None

    def test_clear_planned_path_clears_without_a_world_monitor(self):
        module = _make_module()
        module._last_plan = GeneratedPlan(group_ids=("arm/manipulator",), path=[])

        assert module.clear_planned_path() is True
        assert module._last_plan is None

    def test_dismiss_preview_noop_without_monitor(self):
        module = _make_module()

        module._dismiss_preview(["arm/manipulator"])

    def test_dismiss_preview_routes_to_monitor(self):
        module = _make_module()
        module._world_monitor = MagicMock()

        module._dismiss_preview(["arm/manipulator"])

        module._world_monitor.hide_preview.assert_called_once_with(["arm/manipulator"])
        module._world_monitor.publish_visualization.assert_called_once_with()

    def test_preview_routes_one_complete_plan_with_default_duration(self):
        module = _make_module()
        config = _one_joint_config()
        traj_gen = MagicMock()
        _install_generated_plan(module, config, traj_gen, [0.0], [2.0])

        assert module.preview_plan() is True

        module._world_monitor.animate_plan.assert_called_once_with(module._last_plan, 1.0)

    def test_preview_robot_name_validates_affectedness_without_trimming(self):
        module = _make_module()
        left = _one_joint_config("left")
        right = _one_joint_config("right")
        traj_gen = MagicMock()
        module._robots = {
            "left": ("left_id", left, traj_gen),
            "right": ("right_id", right, traj_gen),
        }
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([left, right])
        module._last_plan = GeneratedPlan(
            group_ids=("left/manipulator", "right/manipulator"),
            status=PlanningStatus.SUCCESS,
            path=[
                JointState(name=["left/j0", "right/j0"], position=[0.0, 0.0]),
                JointState(name=["left/j0", "right/j0"], position=[1.0, 1.0]),
            ],
        )

        assert module.preview_plan(duration=2.5, robot_name="left") is True

        module._world_monitor.animate_plan.assert_called_once_with(module._last_plan, 2.5)

    def test_preview_rejects_unaffected_compatibility_robot(self):
        module = _make_module()
        config = _one_joint_config()
        traj_gen = MagicMock()
        _install_generated_plan(module, config, traj_gen, [0.0], [1.0])

        assert module.preview_plan(robot_name="other") is False
        module._world_monitor.animate_plan.assert_not_called()
