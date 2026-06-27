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
import threading
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
    ManipulationState,
)
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus, PlanningStatus
from dimos.manipulation.planning.spec.models import (
    CollisionCheckResult,
    GeneratedPlan,
    IKResult,
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
        end_effector_link="link_tcp",
        base_link="link_base",
        max_velocity=1.0,
        max_acceleration=2.0,
        coordinator_task_name="traj_arm",
    )


@pytest.fixture
def left_robot_config():
    """Create a robot config for a scoped left arm."""
    return RobotModelConfig(
        name="left_arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1", "joint2", "joint3"],
        end_effector_link="link_tcp",
        base_link="link_base",
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


class _ManipulationModuleHarness(ManipulationModule):
    def __init__(self) -> None:
        self._state = ManipulationState.IDLE
        self._lock = threading.Lock()
        self._error_message = ""
        self._planning_epoch = 0
        self._robots = {}
        self._world_monitor = None
        self._planner = None
        self._kinematics = None
        self._coordinator_client = None
        self._last_plan = None
        self.config = MagicMock(planning_timeout=10.0)


def _make_module() -> ManipulationModule:
    """Create a lightweight ManipulationModule harness for behavior tests."""
    return _ManipulationModuleHarness()


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
        assert module._begin_planning() is True
        assert module._state == ManipulationState.PLANNING

        # From COMPLETED - OK
        module._state = ManipulationState.COMPLETED
        assert module._begin_planning() is True

        # From EXECUTING - Fail
        module._state = ManipulationState.EXECUTING
        assert module._begin_planning() is False


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

    def test_inverse_kinematics_single_calls_configured_backend(self, robot_config):
        """inverse_kinematics_single returns the backend IKResult without path planning."""
        module = _make_module_with_monitor(robot_config)
        module._world_monitor.world = MagicMock()
        current = JointState(name=robot_config.joint_names, position=[0.0, 0.0, 0.0])
        module._world_monitor.get_current_joint_state.return_value = current
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
        result = module.inverse_kinematics_single(pose)

        assert result is expected
        module._kinematics.solve_pose_targets.assert_called_once()
        _, kwargs = module._kinematics.solve_pose_targets.call_args
        assert kwargs["world"] is module._world_monitor.world
        assert kwargs["seed"].name == [
            "test_arm/joint1",
            "test_arm/joint2",
            "test_arm/joint3",
        ]
        assert kwargs["seed"].position == current.position
        target_group, target_pose = next(iter(kwargs["pose_targets"].items()))
        assert target_group.id == "test_arm/manipulator"
        assert target_pose.frame_id == "world"
        assert target_pose.position.x == 0.45

    def test_inverse_kinematics_single_returns_failure_without_joint_state(self, robot_config):
        """inverse_kinematics_single reports failure when no seed state is available."""
        module = _make_module_with_monitor(robot_config)
        module._world_monitor.get_current_joint_state.return_value = None
        module._kinematics = MagicMock()

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())
        result = module.inverse_kinematics_single(pose)

        assert result.status == IKStatus.NO_SOLUTION
        assert result.message == "No joint state"
        module._kinematics.solve_pose_targets.assert_not_called()

    def test_inverse_kinematics_single_uses_explicit_seed(self, robot_config):
        """inverse_kinematics_single initializes the backend from an explicit seed."""
        module = _make_module_with_monitor(robot_config)
        module._world_monitor.world = MagicMock()
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=robot_config.joint_names, position=[0.0, 0.0, 0.0]
        )
        explicit_seed = JointState(name=robot_config.joint_names, position=[0.2, 0.1, 0.0])
        expected = IKResult(status=IKStatus.SUCCESS, joint_state=explicit_seed)
        module._kinematics = MagicMock()
        module._kinematics.solve_pose_targets.return_value = expected

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())
        result = module.inverse_kinematics_single(pose, seed=explicit_seed)

        assert result is expected
        _, kwargs = module._kinematics.solve_pose_targets.call_args
        assert kwargs["seed"] is explicit_seed
        module._world_monitor.get_current_joint_state.assert_not_called()

    def test_forward_kinematics_accepts_extra_global_joints_and_requires_group_joints(
        self, robot_config
    ):
        """forward_kinematics is group-centric and ignores non-group target joints."""
        module = _make_module_with_monitor(robot_config)
        group = _make_global_group("test_arm", "wrist", ["joint1"])
        module._world_monitor.planning_groups = _FakePlanningGroups([group])
        module._world_monitor.get_state_monitor.return_value = MagicMock(
            is_state_stale=lambda max_age: False
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=["joint1", "joint2", "joint3"], position=[0.0, 2.0, 3.0]
        )
        pose = PoseStamped(position=Vector3(x=1.0), orientation=Quaternion())
        module._world_monitor.get_group_ee_pose.return_value = pose

        result = module.forward_kinematics(
            "test_arm/wrist",
            JointState(name=["test_arm/joint1", "test_arm/joint2"], position=[1.0, 9.0]),
        )

        assert result.status == "VALID"
        assert result.pose is pose
        resolved_state = module._world_monitor.get_group_ee_pose.call_args.args[1]
        assert resolved_state.name == ["joint1", "joint2", "joint3"]
        assert resolved_state.position == [1.0, 9.0, 3.0]

        missing = module.forward_kinematics(
            "test_arm/wrist", JointState(name=["test_arm/joint2"], position=[9.0])
        )
        assert missing.status == "INVALID"
        assert "missing group joints" in missing.message


class TestExecute:
    """Test coordinator execution."""

    def test_execute_requires_trajectory(self, robot_config):
        """Execute fails without planned trajectory."""
        module = _make_module()
        module._robots = {"test_arm": ("id", robot_config, MagicMock())}

        assert module.execute() is False

    def test_execute_requires_task_name(self):
        """Execute fails without coordinator_task_name."""
        config_no_task = RobotModelConfig(
            name="arm",
            model_path=Path("/path"),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j1"],
            end_effector_link="ee",
        )
        module = _make_module_with_monitor(config_no_task)
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [_make_global_group("arm", "manipulator", ["j1"])]
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=["j1"], position=[0.0]
        )
        module._last_plan = GeneratedPlan(
            group_ids=("arm/manipulator",),
            path=[JointState(name=["arm/j1"], position=[1.0])],
            status=PlanningStatus.SUCCESS,
        )

        assert module.execute() is False

    def test_execute_success(self, robot_config, simple_trajectory):
        """Successful execute calls coordinator via task_invoke."""
        module = _make_module_with_monitor(robot_config)
        generator = MagicMock()
        generator.generate.return_value = simple_trajectory
        module._robots = {"test_arm": ("id", robot_config, generator)}
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [_make_global_group("test_arm", "manipulator", ["joint1", "joint2", "joint3"])]
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=["joint1", "joint2", "joint3"], position=[0.0, 0.0, 0.0]
        )
        module._last_plan = GeneratedPlan(
            group_ids=("test_arm/manipulator",),
            path=[
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.0, 0.0, 0.0],
                ),
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.5, 0.5, 0.5],
                ),
            ],
            status=PlanningStatus.SUCCESS,
        )

        mock_client = MagicMock()
        mock_client.task_invoke.return_value = True
        module._coordinator_client = mock_client

        assert module.execute() is True
        assert module._state == ManipulationState.COMPLETED
        mock_client.task_invoke.assert_called_once()
        assert mock_client.task_invoke.call_args.args[:2] == ("traj_arm", "execute")
        trajectory = mock_client.task_invoke.call_args.args[2]["trajectory"]
        assert trajectory.joint_names == [
            "test_arm/joint1",
            "test_arm/joint2",
            "test_arm/joint3",
        ]
        assert trajectory.points == simple_trajectory.points

    def test_execute_rejected(self, robot_config, simple_trajectory):
        """Rejected execution sets FAULT state."""
        module = _make_module_with_monitor(robot_config)
        generator = MagicMock()
        generator.generate.return_value = simple_trajectory
        module._robots = {"test_arm": ("id", robot_config, generator)}
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [_make_global_group("test_arm", "manipulator", ["joint1", "joint2", "joint3"])]
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=["joint1", "joint2", "joint3"], position=[0.0, 0.0, 0.0]
        )
        module._last_plan = GeneratedPlan(
            group_ids=("test_arm/manipulator",),
            path=[
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.0, 0.0, 0.0],
                ),
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.5, 0.5, 0.5],
                ),
            ],
            status=PlanningStatus.SUCCESS,
        )

        mock_client = MagicMock()
        mock_client.task_invoke.return_value = False
        module._coordinator_client = mock_client

        assert module.execute() is False
        assert module._state == ManipulationState.FAULT

    def test_execute_times_out_when_coordinator_rpc_does_not_respond(
        self, robot_config, simple_trajectory
    ):
        """Coordinator RPC timeout fails execution instead of hanging silently."""
        module = _make_module_with_monitor(robot_config)
        module.config.coordinator_rpc_timeout = 0.01
        generator = MagicMock()
        generator.generate.return_value = simple_trajectory
        module._robots = {"test_arm": ("id", robot_config, generator)}
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [_make_global_group("test_arm", "manipulator", ["joint1", "joint2", "joint3"])]
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=["joint1", "joint2", "joint3"], position=[0.0, 0.0, 0.0]
        )
        module._last_plan = GeneratedPlan(
            group_ids=("test_arm/manipulator",),
            path=[
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.0, 0.0, 0.0],
                ),
                JointState(
                    name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                    position=[0.5, 0.5, 0.5],
                ),
            ],
            status=PlanningStatus.SUCCESS,
        )
        mock_client = MagicMock()
        mock_client.remote_name = "ControlCoordinator"
        mock_client._unsub_fns = []
        mock_client.rpc.call_sync.side_effect = TimeoutError("no response")
        module._coordinator_client = mock_client

        assert module.execute() is False

        assert module._state == ManipulationState.FAULT
        assert "timed out" in module._error_message
        mock_client.rpc.call_sync.assert_called_once()
        mock_client.task_invoke.assert_not_called()


def _make_module_with_monitor(*configs: RobotModelConfig) -> ManipulationModule:
    """Create a ManipulationModule with a mocked world monitor and robots configured."""
    module = _make_module()
    module._world_monitor = MagicMock()
    module._init_joints = {}
    for config in configs:
        robot_id = f"robot_{config.name}"
        module._robots[config.name] = (robot_id, config, MagicMock())
    module._world_monitor.planning_groups = _FakePlanningGroups(
        [
            _make_global_group(config.name, "manipulator", list(config.joint_names))
            for config in configs
        ]
    )
    module._world_monitor.is_state_valid.return_value = True

    def current_global_joint_state(max_age: float = 1.0) -> JointState | None:
        del max_age
        names: list[str] = []
        positions: list[float] = []
        for config in configs:
            current = module._world_monitor.get_current_joint_state(f"robot_{config.name}")
            if current is None:
                return None
            current_by_name = dict(zip(current.name, current.position, strict=True))
            for local_name in config.joint_names:
                if local_name not in current_by_name:
                    return None
                names.append(f"{config.name}/{local_name}")
                positions.append(float(current_by_name[local_name]))
        return JointState(name=names, position=positions)

    def check_collision(target_joints: JointState, max_age: float = 1.0) -> CollisionCheckResult:
        del target_joints, max_age
        collision_free = bool(module._world_monitor.is_state_valid.return_value)
        return CollisionCheckResult(
            status="VALID" if collision_free else "COLLISION",
            collision_free=collision_free,
            message="Target is collision-free" if collision_free else "Target is in collision",
        )

    module._world_monitor.current_global_joint_state.side_effect = current_global_joint_state
    module._world_monitor.check_collision.side_effect = check_collision
    return module


def _make_joint_state(positions: list[float], name: list[str] | None = None) -> JointState:
    return JointState(name=name or [f"j{i}" for i in range(len(positions))], position=positions)


def _make_robot_config(
    name: str,
    joints: list[str],
    task_name: str,
) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=joints,
        end_effector_link="ee",
        base_link="base",
        coordinator_task_name=task_name,
    )


def _make_global_group(robot_name: str, group_name: str, joints: list[str]) -> PlanningGroup:
    return PlanningGroup(
        id=f"{robot_name}/{group_name}",
        robot_name=robot_name,
        group_name=group_name,
        joint_names=tuple(f"{robot_name}/{joint}" for joint in joints),
        local_joint_names=tuple(joints),
        base_link="base",
        tip_link="ee",
    )


class _FakePlanningGroups:
    def __init__(self, groups: list[PlanningGroup]) -> None:
        self._groups = {group.id: group for group in groups}

    def get(self, group_id: str) -> PlanningGroup:
        return self._groups[group_id]

    def select(self, group_ids: tuple[str, ...]) -> PlanningGroupSelection:
        return PlanningGroupSelection.from_groups(
            tuple(self._groups[group_id] for group_id in group_ids)
        )

    def groups_for_robot(self, robot_name: str) -> tuple[PlanningGroup, ...]:
        return tuple(group for group in self._groups.values() if group.robot_name == robot_name)

    def default_group_id_for_robot(self, robot_name: str) -> str | None:
        group_id = f"{robot_name}/manipulator"
        return group_id if group_id in self._groups else None

    def primary_pose_group_id_for_robot(self, robot_name: str) -> str | None:
        for group in self.groups_for_robot(robot_name):
            if group.has_pose_target:
                return group.id
        return None


def _make_generated_plan(group_ids: tuple[str, ...], *points: list[float]) -> GeneratedPlan:
    return GeneratedPlan(
        group_ids=group_ids,
        path=[
            JointState(
                name=["left/j1", "left/j2", "right/j1"],
                position=list(point),
            )
            for point in points
        ],
        status=PlanningStatus.SUCCESS,
    )


def _trajectory_generator() -> MagicMock:
    generator = MagicMock()
    generator.generate.side_effect = lambda positions: JointTrajectory(
        joint_names=[],
        points=[
            TrajectoryPoint(time_from_start=float(index), positions=list(position))
            for index, position in enumerate(positions)
        ],
    )
    return generator


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
        self.preview_shown: list[tuple[str, ...]] = []
        self.preview_hidden: list[tuple[str, ...]] = []
        self.animations: list[tuple[GeneratedPlan, float]] = []

    def initialize_scene(self, scene: PlanningSceneInfo) -> None:
        pass

    def get_visualization_url(self) -> str | None:
        return "123"

    def publish_visualization(self, ctx: object | None = None) -> None:
        self.published = True

    def show_preview(self, group_ids: Sequence[str]) -> None:
        self.preview_shown.append(tuple(group_ids))

    def hide_preview(self, group_ids: Sequence[str]) -> None:
        self.preview_hidden.append(tuple(group_ids))

    def animate_plan(self, plan: GeneratedPlan, duration: float = 3.0) -> None:
        self.animations.append((plan, duration))

    def close(self) -> None:
        self.close_count += 1


class TestOnJointState:
    """Test _on_joint_state routing, splitting, and init capture."""

    def test_routes_positions_to_monitor(self, left_robot_config):
        """Joint positions from aggregated message are routed to the correct monitor."""
        module = _make_module_with_monitor(left_robot_config)

        msg = JointState(
            name=["left_arm/joint1", "left_arm/joint2", "left_arm/joint3"],
            position=[0.1, 0.2, 0.3],
            velocity=[1.0, 2.0, 3.0],
        )
        module._on_joint_state(msg)

        # Verify world_monitor received the sub-message
        module._world_monitor.on_joint_state.assert_called_once()
        call_args = module._world_monitor.on_joint_state.call_args
        sub_msg = call_args[0][0]
        assert sub_msg.name == ["joint1", "joint2", "joint3"]
        assert sub_msg.position == [0.1, 0.2, 0.3]
        assert sub_msg.velocity == [1.0, 2.0, 3.0]
        assert call_args[1]["robot_id"] == "robot_left_arm"

    def test_skips_robot_with_missing_joints(self, left_robot_config):
        """Robots whose joints are absent from the message are skipped."""
        module = _make_module_with_monitor(left_robot_config)

        # Message has none of left_arm's joints
        msg = JointState(
            name=["right/joint1", "right/joint2"],
            position=[0.5, 0.6],
        )
        module._on_joint_state(msg)

        module._world_monitor.on_joint_state.assert_not_called()

    def test_captures_init_joints_on_first_call(self, left_robot_config):
        """First joint state is stored as init joints; subsequent calls don't overwrite."""
        module = _make_module_with_monitor(left_robot_config)

        first_msg = JointState(
            name=["left_arm/joint1", "left_arm/joint2", "left_arm/joint3"],
            position=[0.1, 0.2, 0.3],
        )
        module._on_joint_state(first_msg)
        assert "left_arm" in module._init_joints
        assert module._init_joints["left_arm"].position == [0.1, 0.2, 0.3]

        # Second call should NOT overwrite
        second_msg = JointState(
            name=["left_arm/joint1", "left_arm/joint2", "left_arm/joint3"],
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
            end_effector_link="ee",
            base_link="base",
            coordinator_task_name="traj_left",
        )
        right_config = RobotModelConfig(
            name="right",
            model_path=Path("/path/to/robot.urdf"),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j1", "j2"],
            end_effector_link="ee",
            base_link="base",
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

    def test_no_monitor_returns_early(self, left_robot_config):
        """When world_monitor is None, _on_joint_state returns without error."""
        module = _make_module()
        module._robots = {"left_arm": ("id", left_robot_config, MagicMock())}
        module._world_monitor = None

        # Should not raise
        msg = JointState(
            name=["left_arm/joint1", "left_arm/joint2", "left_arm/joint3"],
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
        group_ids = ("robot/manipulator",)
        plan = GeneratedPlan(
            group_ids=group_ids,
            path=[JointState(name=["robot/j1"], position=[0.0])],
            status=PlanningStatus.SUCCESS,
        )
        monitor.show_preview(group_ids)
        monitor.hide_preview(group_ids)
        monitor.animate_plan(plan, 4.5)
        assert monitor.visualization is viz
        assert viz.published is True
        assert viz.preview_shown == [group_ids]
        assert viz.preview_hidden == [group_ids]
        assert viz.animations == [(plan, 4.5)]

        monitor.stop_all_monitors()

        assert viz.close_count == 1
        state_monitor.stop.assert_called_once()
        obstacle_monitor.stop.assert_called_once()

    def test_visualization_none_is_noop(self):
        monitor = _make_world_monitor_with_viz(None)

        assert monitor.get_visualization_url() is None
        monitor.publish_visualization()
        plan = GeneratedPlan(
            group_ids=("robot/manipulator",),
            path=[JointState(name=["robot/j1"], position=[0.0])],
            status=PlanningStatus.SUCCESS,
        )
        monitor.show_preview(("robot/manipulator",))
        monitor.hide_preview(("robot/manipulator",))
        monitor.animate_plan(plan, 1.0)
        monitor.start_visualization_thread()
        assert monitor._viz_thread is None


class TestManipulationPreview:
    def test_dismiss_preview_noop_without_monitor(self):
        module = _make_module()

        module._dismiss_preview(("arm/manipulator",))

    def test_dismiss_preview_routes_to_monitor(self):
        module = _make_module()
        module._world_monitor = MagicMock()

        group_ids = ("arm/manipulator",)
        module._dismiss_preview(group_ids)

        module._world_monitor.hide_preview.assert_called_once_with(group_ids)
        module._world_monitor.publish_visualization.assert_called_once_with()

    def test_preview_plan_uses_safe_default_duration(self):
        module = _make_module()
        module._world_monitor = MagicMock()
        module._last_plan = GeneratedPlan(
            group_ids=("arm/manipulator",),
            path=[JointState(name=["arm/j1"], position=[0.0])],
            status=PlanningStatus.SUCCESS,
        )

        assert module.preview_plan() is True

        module._world_monitor.animate_plan.assert_called_once_with(module._last_plan, 1.0)

    def test_preview_plan_explicit_duration_overrides_default(self):
        module = _make_module()
        module._world_monitor = MagicMock()
        module._last_plan = GeneratedPlan(
            group_ids=("arm/manipulator",),
            path=[JointState(name=["arm/j1"], position=[0.0])],
            status=PlanningStatus.SUCCESS,
        )

        assert module.preview_plan(duration=1.5) is True

        module._world_monitor.animate_plan.assert_called_once_with(module._last_plan, 1.5)

    def test_preview_plan_respects_robot_filter(self):
        module = _make_module()
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [_make_global_group("arm", "manipulator", ["j1"])]
        )
        module._last_plan = GeneratedPlan(
            group_ids=("arm/manipulator",),
            path=[JointState(name=["arm/j1"], position=[0.0])],
            status=PlanningStatus.SUCCESS,
        )

        assert module.preview_plan(robot_name="arm") is True

        module._world_monitor.animate_plan.assert_called_once_with(module._last_plan, 1.0)

    def test_preview_plan_rejects_unaffected_robot_filter(self):
        module = _make_module()
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [_make_global_group("arm", "manipulator", ["j1"])]
        )
        module._last_plan = GeneratedPlan(
            group_ids=("arm/manipulator",),
            path=[JointState(name=["arm/j1"], position=[0.0])],
            status=PlanningStatus.SUCCESS,
        )

        assert module.preview_plan(robot_name="other") is False

        module._world_monitor.animate_plan.assert_not_called()

    def test_preview_plan_returns_false_for_missing_inputs(self):
        module = _make_module()

        assert module.preview_plan() is False

        module._world_monitor = MagicMock()
        assert module.preview_plan() is False


class TestGeneratedPlanProjection:
    def test_selected_joint_state_accepts_local_current_state_names(self):
        config = _make_robot_config("left", ["j1", "j2"], "task")
        module = _make_module_with_monitor(config)
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [_make_global_group("left", "arm", ["j1", "j2"])]
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=["j1", "j2"], position=[1.0, 2.0]
        )

        selected = module._selected_joint_state(("left/arm",))

        assert selected is not None
        assert selected.name == ["left/j1", "left/j2"]
        assert selected.position == [1.0, 2.0]

    def test_selected_joint_state_rejects_mixed_current_state_names(self):
        config = _make_robot_config("left", ["j1", "j2"], "task")
        module = _make_module_with_monitor(config)
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [_make_global_group("left", "arm", ["j1", "j2"])]
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=["left/j1", "j2"], position=[1.0, 2.0]
        )

        assert module._selected_joint_state(("left/arm",)) is None

    def test_execute_plan_dispatches_one_trajectory_per_affected_robot(self):
        left_config = _make_robot_config(
            "left",
            ["j1", "j2", "j3"],
            "left_task",
        )
        right_config = _make_robot_config("right", ["j1", "j2"], "right_task")
        module = _make_module_with_monitor(left_config, right_config)
        left_gen = _trajectory_generator()
        right_gen = _trajectory_generator()
        module._robots["left"] = ("robot_left", left_config, left_gen)
        module._robots["right"] = ("robot_right", right_config, right_gen)
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [
                _make_global_group("left", "arm", ["j1", "j2"]),
                _make_global_group("right", "arm", ["j1"]),
            ]
        )
        module._world_monitor.get_current_joint_state.side_effect = [
            JointState(name=["j1", "j2", "j3"], position=[0.0, 0.0, 9.0]),
            JointState(name=["j1", "j2"], position=[0.0, 8.0]),
        ]
        module._coordinator_client = MagicMock()
        module._coordinator_client.task_invoke.return_value = True
        plan = _make_generated_plan(("left/arm", "right/arm"), [1.0, 2.0, 3.0], [4.0, 5.0, 6.0])

        assert module.execute_plan(plan) is True

        assert module._coordinator_client.task_invoke.call_count == 2
        left_call, right_call = module._coordinator_client.task_invoke.call_args_list
        assert left_call.args[0:2] == ("left_task", "execute")
        left_trajectory = left_call.args[2]["trajectory"]
        assert left_trajectory.joint_names == ["left/j1", "left/j2", "left/j3"]
        assert [point.positions for point in left_trajectory.points] == [
            [1.0, 2.0, 9.0],
            [4.0, 5.0, 9.0],
        ]
        assert right_call.args[0:2] == ("right_task", "execute")
        right_trajectory = right_call.args[2]["trajectory"]
        assert right_trajectory.joint_names == ["right/j1", "right/j2"]
        assert [point.positions for point in right_trajectory.points] == [[3.0, 8.0], [6.0, 8.0]]

    def test_execute_plan_holds_non_selected_joints_from_current_state(self):
        config = _make_robot_config("left", ["j1", "j2", "j3"], "task")
        module = _make_module_with_monitor(config)
        generator = _trajectory_generator()
        module._robots["left"] = ("robot_left", config, generator)
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [_make_global_group("left", "arm", ["j2"])]
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=["j1", "j2", "j3"], position=[10.0, 20.0, 30.0]
        )
        module._coordinator_client = MagicMock()
        module._coordinator_client.task_invoke.return_value = True
        plan = GeneratedPlan(
            group_ids=("left/arm",),
            path=[
                JointState(name=["left/j2"], position=[2.0]),
                JointState(name=["left/j2"], position=[3.0]),
            ],
            status=PlanningStatus.SUCCESS,
        )

        assert module.execute_plan(plan) is True

        trajectory = module._coordinator_client.task_invoke.call_args.args[2]["trajectory"]
        assert trajectory.joint_names == ["left/j1", "left/j2", "left/j3"]
        assert [point.positions for point in trajectory.points] == [
            [10.0, 2.0, 30.0],
            [10.0, 3.0, 30.0],
        ]

    def test_execute_plan_rejects_local_waypoint_names(self):
        config = _make_robot_config("left", ["j1", "j2"], "task")
        module = _make_module_with_monitor(config)
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [_make_global_group("left", "arm", ["j1"])]
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=["j1", "j2"], position=[10.0, 20.0]
        )
        module._coordinator_client = MagicMock()
        plan = GeneratedPlan(
            group_ids=("left/arm",),
            path=[JointState(name=["j1"], position=[1.0])],
            status=PlanningStatus.SUCCESS,
        )

        assert module.execute_plan(plan) is False
        module._coordinator_client.task_invoke.assert_not_called()

    def test_preview_plan_with_last_plan_animates_generated_plan(self):
        config = _make_robot_config("left", ["j1", "j2"], "task")
        module = _make_module_with_monitor(config)
        module._world_monitor.planning_groups = _FakePlanningGroups(
            [_make_global_group("left", "arm", ["j1"])]
        )
        module._last_plan = GeneratedPlan(
            group_ids=("left/arm",),
            path=[
                JointState(name=["left/j1"], position=[1.0]),
                JointState(name=["left/j1"], position=[2.0]),
            ],
            status=PlanningStatus.SUCCESS,
        )

        assert module.preview_plan(robot_name="left") is True

        module._world_monitor.animate_plan.assert_called_once_with(module._last_plan, 1.0)

    def test_has_and_clear_planned_path_use_last_plan(self):
        module = _make_module()
        module._last_plan = GeneratedPlan(
            group_ids=("left/arm",),
            path=[JointState(name=["left/j1"], position=[1.0])],
            status=PlanningStatus.SUCCESS,
        )
        assert module.has_planned_path() is True
        assert module.clear_planned_path() is True
        assert module.has_planned_path() is False
        assert module._last_plan is None
