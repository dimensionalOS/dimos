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
from unittest.mock import ANY, MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
    ManipulationState,
)
from dimos.manipulation.manipulation_spec import (
    ExecutionStatus,
    PlanStatus,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus, ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import (
    GeneratedPlan,
    IKResult,
    PlanningResult,
)
from dimos.manipulation.planning.trajectory_generator.config import (
    SimpleTrapezoidParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.simple_parametrizer import (
    SimpleTrapezoidParametrizer,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.robot.assets.model import RobotModel


def _control_coordinator(
    *,
    execute_status: TrajectoryExecutionStatus = TrajectoryExecutionStatus.ACCEPTED,
    cancel_status: TrajectoryCancellationStatus = (TrajectoryCancellationStatus.ALREADY_STOPPED),
) -> MagicMock:
    coordinator = MagicMock(spec=ControlCoordinator)
    coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(execute_status)
    coordinator.cancel_trajectory.return_value = TrajectoryCancellationResult(cancel_status)
    coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.IDLE)
    return coordinator


@pytest.fixture
def robot_config():
    """Create a robot config for testing."""
    return RobotModelConfig(
        name="test_arm",
        model=RobotModel.from_file(Path("/path/to/robot.urdf")),
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
    )


@pytest.fixture
def robot_config_with_mapping():
    """Create a robot config with joint name mapping (dual-arm scenario)."""
    return RobotModelConfig(
        name="left_arm",
        model=RobotModel.from_file(Path("/path/to/robot.urdf")),
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
    )


def _one_joint_config(name: str = "arm") -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model=RobotModel.from_file(Path("/path/robot.urdf")),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["j0"],
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator", joint_names=("j0",), base_link="base_link", tip_link="ee"
            )
        ],
    )


def _install_generated_plan(
    module: ManipulationModule,
    config: RobotModelConfig,
    *points: list[float],
) -> None:
    """Install a generated plan and enough monitor state to derive robot paths."""
    global_joint_names = [f"{config.name}/{joint}" for joint in config.joint_names]
    module._robots = {config.name: ("robot_id", config)}
    module.config = module.config.model_copy(update={"robots": [config]})
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
    module._last_plan = GeneratedPlan(
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
    module._initialize_execution()


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


def _enable_simple_parametrization(module: ManipulationModule) -> None:
    module._trajectory_parametrizer = SimpleTrapezoidParametrizer(
        SimpleTrapezoidParametrizationConfig()
    )


class TestObstacleUpdates:
    def test_complete_update_forwards_new_obstacle_value(self, module_factory) -> None:
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)
        module._world_monitor.update_obstacle.return_value = True
        pose = Pose(
            position=Vector3(1.0, 2.0, 3.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )

        result = module.update_obstacle(
            "moving-shape",
            pose,
            "sphere",
            [0.4],
            color=[0.1, 0.2, 0.3, 0.9],
        )

        assert result is True
        obstacle = module._world_monitor.update_obstacle.call_args.args[0]
        assert obstacle.name == "moving-shape"
        assert obstacle.obstacle_type == ObstacleType.SPHERE
        assert obstacle.dimensions == (0.4,)
        assert obstacle.color == (0.1, 0.2, 0.3, 0.9)
        assert obstacle.pose.position.x == pytest.approx(1.0)

        assert module.update_obstacle("default-color", pose, "box", [1.0, 1.0, 1.0])
        default_obstacle = module._world_monitor.update_obstacle.call_args.args[0]
        assert default_obstacle.color == (0.8, 0.2, 0.2, 0.8)

    def test_pose_update_forwards_only_name_and_pose(self, module_factory) -> None:
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)
        module._world_monitor.update_obstacle_pose.return_value = True
        pose = Pose(
            position=Vector3(4.0, 5.0, 6.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )

        result = module.update_obstacle_pose("moving-shape", pose)

        assert result is True
        name, stamped = module._world_monitor.update_obstacle_pose.call_args.args
        assert name == "moving-shape"
        assert stamped.position.x == pytest.approx(4.0)
        assert stamped.position.y == pytest.approx(5.0)
        assert stamped.position.z == pytest.approx(6.0)

    def test_complete_update_rejects_unknown_shape_before_world_mutation(
        self, module_factory
    ) -> None:
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)

        with pytest.raises(ValueError, match="Unknown obstacle shape"):
            module.update_obstacle("shape", Pose(), "capsule", [1.0])

        module._world_monitor.update_obstacle.assert_not_called()

    def test_complete_update_rejects_incomplete_mesh_and_invalid_color(
        self, module_factory
    ) -> None:
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)

        with pytest.raises(ValueError, match="mesh_path required"):
            module.update_obstacle("mesh", Pose(), "mesh")
        with pytest.raises(ValueError, match="four values"):
            module.update_obstacle("box", Pose(), "box", [1.0, 1.0, 1.0], color=[1.0])

        module._world_monitor.update_obstacle.assert_not_called()

    def test_updates_fail_when_world_monitor_is_unavailable(self, module_factory) -> None:
        module = module_factory()
        module._world_monitor = None

        assert module.update_obstacle("box", Pose(), "box", [1.0, 1.0, 1.0]) is False
        assert module.update_obstacle_pose("box", Pose()) is False


class TestStateMachine:
    """Test state transitions."""

    def test_cancel_interrupts_planning(self, module_factory):
        module = module_factory()

        module._state = ManipulationState.PLANNING
        result = module.cancel()

        assert result.status is ExecutionStatus.ABORTED
        assert module._planning_epoch == 1
        assert module._state is ManipulationState.IDLE

    def test_cancel_discards_pending_plan_and_preview(self, module_factory):
        module = module_factory()
        module._last_plan = GeneratedPlan(
            trajectory=JointTrajectory(), group_ids=("arm/manipulator",), path=[]
        )
        module._world_monitor = MagicMock()

        result = module.cancel()

        assert result.status is ExecutionStatus.NO_EXECUTION
        assert module._last_plan is None
        module._world_monitor.cancel_preview_animation.assert_called_once_with()

    def test_cancel_forwards_to_coordinator(self, module_factory):
        module = module_factory()
        config = _one_joint_config()
        _install_generated_plan(module, config, [0.0], [0.1])
        coordinator = _control_coordinator()
        module._control_coordinator = coordinator
        module._initialize_execution()

        result = module.cancel()

        assert result.status is ExecutionStatus.NO_EXECUTION
        coordinator.cancel_trajectory.assert_called_once_with()

    def test_planning_error_remains_retryable(self, module_factory):
        """Planning errors preserve the message without latching FAULT."""
        module = module_factory()
        module._state = ManipulationState.PLANNING

        result = module._fail("Test error")
        assert result is False
        assert module._state == ManipulationState.IDLE
        assert module._error_message == "Test error"

    def test_group_planning_resolves_speed_per_request(self, module_factory):
        module = module_factory()
        module._world_monitor = MagicMock()

        assert module._begin_group_planning(0.2) == (1, 0.2)
        assert module._state == ManipulationState.PLANNING

        module._state = ManipulationState.COMPLETED
        assert module._begin_group_planning() == (2, module.config.default_speed_scale)

        module._state = ManipulationState.EXECUTING
        assert module._begin_group_planning() is None


class TestRobotSelection:
    """Test robot selection logic."""

    def test_single_robot_default(self, robot_config, module_factory):
        """Single robot is used by default."""
        module = module_factory()
        module._robots = {"arm": ("id", robot_config)}

        result = module._get_robot()
        assert result is not None
        assert result[0] == "arm"

    def test_multiple_robots_require_name(self, robot_config, module_factory):
        """Multiple robots require explicit name."""
        module = module_factory()
        module._robots = {
            "left": ("id1", robot_config),
            "right": ("id2", robot_config),
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
            trajectory_parametrizer=MagicMock(),
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


@pytest.fixture
def planning_initialization(mocker: MockerFixture) -> PlanningInitializationHarness:
    return PlanningInitializationHarness(mocker)


class TestPlanningInitialization:
    """Test planning backend configuration wiring."""

    def test_default_kinematics_config_uses_pink(self) -> None:
        """Pink IK is the default solver for manipulation modules."""
        config = ManipulationModuleConfig()

        assert isinstance(config.kinematics, PinkKinematicsConfig)

    def test_start_eagerly_initializes_planning_and_execution(
        self,
        mocker: MockerFixture,
    ) -> None:
        module = ManipulationModule()
        module.coordinator_joint_state = None
        initialize_planning = mocker.patch.object(module, "_initialize_planning")
        initialize_execution = mocker.patch.object(module, "_initialize_execution")

        with module:
            initialize_execution.assert_called_once_with()
            initialize_planning.assert_called_once_with()

    def test_start_is_idempotent(self, mocker: MockerFixture) -> None:
        module = ManipulationModule()
        initialize_planning = mocker.patch.object(module, "_initialize_planning")
        initialize_execution = mocker.patch.object(module, "_initialize_execution")
        subscribe = mocker.patch.object(module.coordinator_joint_state, "subscribe")

        try:
            module.start()
            module.start()

            initialize_execution.assert_called_once_with()
            initialize_planning.assert_called_once_with()
            subscribe.assert_called_once_with(module._on_joint_state)
        finally:
            module.stop()

    def test_state_is_readable_during_planning_initialization(
        self,
        mocker: MockerFixture,
    ) -> None:
        module = ManipulationModule()
        module._control_coordinator = _control_coordinator()
        module.coordinator_joint_state = None
        observed_status: list[ExecutionStatus] = []

        def observe_state() -> None:
            observed_status.append(module.get_state().execution_status)

        mocker.patch.object(module, "_initialize_planning", side_effect=observe_state)

        try:
            module.start()
            assert observed_status == [ExecutionStatus.IDLE]
        finally:
            module.stop()

    def test_kinematics_config_is_passed_to_factory(
        self,
        robot_config,
        planning_initialization: PlanningInitializationHarness,
        module_factory,
    ):
        """ManipulationModule config selects the requested IK backend."""
        module = module_factory()
        kinematics = PinkKinematicsConfig(max_iterations=100, dt=0.02)
        module.config = ManipulationModuleConfig(
            robots=[robot_config],
            kinematics=kinematics,
        )

        module._initialize_planning()

        planning_initialization.mock_planning_specs.assert_called_once_with(
            world=planning_initialization.mock_world,
            world_backend="roboplan",
            planner=module.config.planner,
            kinematics_name=None,
            kinematics=kinematics,
            trajectory_parametrization=ANY,
        )

    def test_legacy_kinematics_name_still_selects_backend(
        self,
        robot_config,
        planning_initialization: PlanningInitializationHarness,
        module_factory,
    ):
        """The old kinematics_name field remains a compatibility shim."""
        module = module_factory()
        module.config = ManipulationModuleConfig(
            robots=[robot_config],
            kinematics_name="pink",
        )

        module._initialize_planning()

        planning_initialization.mock_planning_specs.assert_called_once_with(
            world=planning_initialization.mock_world,
            world_backend="roboplan",
            planner=module.config.planner,
            kinematics_name="pink",
            kinematics=module.config.kinematics,
            trajectory_parametrization=ANY,
        )

    def test_nested_kinematics_config_parses_cli_override_shape(self) -> None:
        """Pydantic parses the nested shape used by dynamic CLI overrides."""
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

    def test_solve_ik_rpc_calls_configured_backend(self, robot_config, module_factory):
        """solve_ik returns the backend IKResult without path planning."""
        module = module_factory()
        module._robots = {"test_arm": ("robot_id", robot_config)}
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

    def test_solve_ik_rpc_returns_failure_without_joint_state(self, robot_config, module_factory):
        """solve_ik reports a failed IKResult when no seed state is available."""
        module = module_factory()
        module._robots = {"test_arm": ("robot_id", robot_config)}
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

    def test_solve_ik_rpc_accepts_explicit_seed_without_current_state(
        self, robot_config, module_factory
    ):
        """solve_ik succeeds with an explicit seed when no current state is available."""
        module = module_factory()
        module._robots = {"test_arm": ("robot_id", robot_config)}
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([robot_config])
        module._world_monitor.get_current_joint_state.return_value = None
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

    def test_list_planning_groups_exposes_group_capabilities(self, robot_config, module_factory):
        module = module_factory()
        registry = PlanningGroupRegistry([robot_config])
        module._robots = {"test_arm": ("robot_id", robot_config)}
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = registry
        module._init_joints = {}

        groups = module.list_planning_groups()

        assert [group.id for group in groups] == ["test_arm/manipulator"]
        assert groups[0].tip_frame == "link_tcp"
        assert groups[0].joint_names == (
            "test_arm/joint1",
            "test_arm/joint2",
            "test_arm/joint3",
        )

    def test_plan_to_joint_targets_stores_generated_plan_and_legacy_caches(
        self, robot_config, module_factory
    ):
        module = module_factory()
        registry = PlanningGroupRegistry([robot_config])
        module._robots = {"test_arm": ("robot_id", robot_config)}
        _enable_simple_parametrization(module)
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.world.get_robot_ids.return_value = ["robot_id"]
        module._world_monitor.world.get_robot_config.return_value = robot_config
        module._world_monitor.planning_groups = registry
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=robot_config.joint_names,
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
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
        assert module._last_plan.trajectory.points[-1].positions == [0.1, 0.2, 0.3]
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

    def test_plan_to_pose_targets_uses_group_ik_and_selected_path(
        self, robot_config, module_factory
    ):
        module = module_factory()
        registry = PlanningGroupRegistry([robot_config])
        module._robots = {"test_arm": ("robot_id", robot_config)}
        _enable_simple_parametrization(module)
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.world.get_robot_ids.return_value = ["robot_id"]
        module._world_monitor.world.get_robot_config.return_value = robot_config
        module._world_monitor.planning_groups = registry
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.get_current_joint_state.return_value = JointState(
            name=robot_config.joint_names,
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
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

    def test_failed_plan_materialization_clears_generated_plan(self, robot_config, module_factory):
        module = module_factory()
        registry = PlanningGroupRegistry([robot_config])
        module._robots = {"test_arm": ("robot_id", robot_config)}
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.planning_groups = registry
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.get_current_joint_state.return_value = None
        module._last_plan = GeneratedPlan(
            trajectory=_generated_plan_trajectory(
                ["test_arm/joint1", "test_arm/joint2", "test_arm/joint3"],
                [0.0, 0.0, 0.0],
                [0.1, 0.2, 0.3],
            ),
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
        assert module._state == ManipulationState.IDLE
        assert module._last_plan is None
        assert module.has_planned_path() is False

    def test_execute_plan_dispatches_selected_subsets_once_with_shared_clock_and_mapping(
        self, module_factory
    ):
        left = RobotModelConfig(
            name="left",
            model=RobotModel.from_file(Path("/path/robot.urdf")),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j0", "j1"],
            planning_groups=[
                PlanningGroupDefinition(
                    name="wrist", joint_names=("j1",), base_link="base", tip_link="ee"
                )
            ],
            joint_name_mapping={"left_coord_j1": "j1"},
        )
        right = RobotModelConfig(
            name="right",
            model=RobotModel.from_file(Path("/path/robot.urdf")),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["k0", "k1"],
            planning_groups=[
                PlanningGroupDefinition(
                    name="elbow", joint_names=("k0",), base_link="base", tip_link="ee"
                )
            ],
        )
        module = module_factory()
        module._robots = {
            "left": ("left_id", left),
            "right": ("right_id", right),
        }
        module.config = module.config.model_copy(update={"robots": [left, right]})
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([left, right])
        module._last_plan = GeneratedPlan(
            group_ids=("left/wrist", "right/elbow"),
            status=PlanningStatus.SUCCESS,
            path=[
                JointState(name=["left/j1", "right/k0"], position=[0.0, 1.0]),
                JointState(name=["left/j1", "right/k0"], position=[0.5, 1.5]),
            ],
            trajectory=JointTrajectory(
                joint_names=["left/j1", "right/k0"],
                points=[
                    TrajectoryPoint(
                        time_from_start=0.0,
                        positions=[0.0, 1.0],
                        velocities=[0.0, 0.0],
                    ),
                    TrajectoryPoint(
                        time_from_start=2.5,
                        positions=[0.5, 1.5],
                        velocities=[0.2, 0.4],
                    ),
                ],
            ),
        )
        mock_coordinator = _control_coordinator()
        module._control_coordinator = mock_coordinator
        module._initialize_execution()
        module._world_monitor.get_current_joint_state.side_effect = lambda robot_id: {
            "left_id": JointState(name=["j0", "j1"], position=[9.0, 0.0]),
            "right_id": JointState(name=["k0", "k1"], position=[1.0, 9.0]),
        }[robot_id]
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=["left/j1", "right/k0"], position=[0.0, 1.0]
        )

        result = module.execute(blocking=False)

        assert result.status is ExecutionStatus.ACCEPTED
        mock_coordinator.execute_trajectory.assert_called_once()
        payload = mock_coordinator.execute_trajectory.call_args.args[0]
        assert payload.joint_names == ["left_coord_j1", "k0"]
        assert [point.time_from_start for point in payload.points] == [0.0, 2.5]
        assert [point.positions for point in payload.points] == [[0.0, 1.0], [0.5, 1.5]]
        assert [point.velocities for point in payload.points] == [[0.0, 0.0], [0.2, 0.4]]

    def test_pose_wrappers_fail_safely_without_unique_pose_group(
        self, robot_config, module_factory
    ):
        no_pose_config = RobotModelConfig(
            name="test_arm",
            model=robot_config.model,
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
        module = module_factory()
        module._robots = {"test_arm": ("robot_id", no_pose_config)}
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

    def test_pose_wrappers_fail_safely_with_multiple_pose_groups(
        self, robot_config, module_factory
    ):
        multi_pose_config = RobotModelConfig(
            name="test_arm",
            model=robot_config.model,
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
        module = module_factory()
        module._robots = {"test_arm": ("robot_id", multi_pose_config)}
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

    def test_solve_ik_preserves_backend_failure_detail(self, robot_config, module_factory):
        """IK diagnostics include the backend's human-readable failure message."""
        module = module_factory()
        module._robots = {"test_arm": ("robot_id", robot_config)}
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([robot_config])
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=[f"test_arm/{name}" for name in robot_config.joint_names],
            position=[0.0, 0.0, 0.0],
        )
        module._kinematics = MagicMock()
        module._kinematics.solve_pose_targets.return_value = IKResult(
            status=IKStatus.NO_SOLUTION, message="target is outside the workspace"
        )

        result = module.solve_ik(Pose(position=Vector3(), orientation=Quaternion()))

        assert result.status == IKStatus.NO_SOLUTION
        assert module.get_error() == "IK failed: NO_SOLUTION: target is outside the workspace"
        assert module._state == ManipulationState.IDLE


class TestPlanningDiagnostics:
    def test_planner_failure_preserves_detail_and_allows_retry(self, robot_config, module_factory):
        """Planning diagnostics remain visible until the next planning request."""
        module = module_factory()
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry([robot_config])
        module._world_monitor.current_global_joint_state.return_value = JointState(
            name=[f"test_arm/{name}" for name in robot_config.joint_names],
            position=[0.0, 0.0, 0.0],
        )
        module._planner = MagicMock()
        module._planner.plan_selected_joint_path.return_value = PlanningResult(
            status=PlanningStatus.TIMEOUT, message="planner timed out"
        )

        module._robots = {"test_arm": ("robot_id", robot_config)}
        result = module.plan_to_joints(
            {
                "test_arm/manipulator": JointState(
                    name=robot_config.joint_names,
                    position=[1.0, 1.0, 1.0],
                )
            }
        )

        assert result.status is PlanStatus.FAILED
        assert module.get_error() == "Planning failed: TIMEOUT: planner timed out"
        assert module._state == ManipulationState.IDLE

        retry_epoch = module._begin_group_planning()

        assert retry_epoch is not None
        assert module._state == ManipulationState.PLANNING
        assert module.get_error() == ""


class TestExecute:
    """Test coordinator execution."""

    def test_execute_requires_trajectory(self, robot_config, module_factory):
        """Execute fails without planned trajectory."""
        module = module_factory()
        module._robots = {"test_arm": ("id", robot_config)}

        assert module.execute().status is ExecutionStatus.NO_PLAN
        assert module._state == ManipulationState.IDLE


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
