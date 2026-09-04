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
from unittest.mock import ANY, MagicMock, call

import numpy as np
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
    VOXEL_MAP_OBSTACLE_ID,
    ManipulationModule,
    ManipulationModuleConfig,
    ManipulationState,
)
from dimos.manipulation.manipulation_spec import ExecutionStatus, PlanStatus
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus, ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import (
    GeneratedPlan,
    IKResult,
    Obstacle,
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
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
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


def _one_joint_config() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/path")),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["j0"],
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator", joint_names=("j0",), base_link="base_link", tip_link="ee"
            )
        ],
    )


def _bimanual_config() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/path/to/bimanual.urdf")),
        joint_names=["left/j1", "right/j1"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition("left_arm", ("left/j1",), "base", "left/tool"),
            PlanningGroupDefinition("right_arm", ("right/j1",), "base", "right/tool"),
            PlanningGroupDefinition("both_arms", ("left/j1", "right/j1"), "base"),
        ],
        home_joints=[0.0, 0.0],
    )


def _install_generated_plan(
    module: ManipulationModule,
    config: RobotModelConfig,
    *points: list[float],
) -> None:
    """Install a canonical generated plan and current model state."""
    module.config.model = config
    module._world_monitor = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry(config.planning_groups)
    module._world_monitor.current_model_joint_state.return_value = JointState(
        name=config.joint_names,
        position=[0.0 for _ in config.joint_names],
    )
    module._last_plan = GeneratedPlan(
        trajectory=JointTrajectory(
            joint_names=config.joint_names,
            points=[
                TrajectoryPoint(
                    time_from_start=float(index),
                    positions=list(point),
                    velocities=[0.0 for _ in config.joint_names],
                )
                for index, point in enumerate(points)
            ],
        ),
        group_ids=("manipulator",),
        status=PlanningStatus.SUCCESS,
        path=[
            JointState(
                name=config.joint_names,
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


class TestVoxelMap:
    """The mapped workspace arrives on a port, as one replaceable octree."""

    @staticmethod
    def _cloud(points: list[list[float]], frame_id: str = "world") -> PointCloud2:
        return PointCloud2.from_numpy(
            np.asarray(points, dtype=np.float32).reshape((-1, 3)),
            frame_id=frame_id,
            timestamp=1.0,
        )

    def test_a_map_becomes_one_octree_obstacle(self, module_factory) -> None:
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)
        module._world_monitor.update_obstacle.return_value = True

        module._apply_voxel_map(self._cloud([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0]]))

        obstacle = module._world_monitor.update_obstacle.call_args.args[0]
        assert obstacle.name == VOXEL_MAP_OBSTACLE_ID
        assert obstacle.obstacle_type == ObstacleType.OCTREE
        assert obstacle.octree_resolution == module.config.voxel_map_resolution
        np.testing.assert_allclose(obstacle.points, [(0.0, 0.0, 0.0), (0.05, 0.0, 0.0)], atol=1e-7)

    def test_a_later_map_replaces_the_obstacle_rather_than_adding(self, module_factory) -> None:
        # The mapper republishes a complete map, so an accumulating pile of
        # obstacles would wall the arm in with everything it has ever seen.
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)
        module._world_monitor.update_obstacle.return_value = True

        module._apply_voxel_map(self._cloud([[0.0, 0.0, 0.0]]))
        module._apply_voxel_map(self._cloud([[1.0, 0.0, 0.0]]))

        assert module._world_monitor.add_obstacle.call_count == 0
        assert module._world_monitor.update_obstacle.call_count == 2

    def test_the_first_map_is_added_when_there_is_nothing_to_replace(self, module_factory) -> None:
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)
        module._world_monitor.update_obstacle.return_value = False

        module._apply_voxel_map(self._cloud([[0.0, 0.0, 0.0]]))

        assert module._world_monitor.add_obstacle.call_count == 1

    def test_an_empty_map_clears_the_obstacle(self, module_factory) -> None:
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)

        module._apply_voxel_map(self._cloud([]))

        module._world_monitor.remove_obstacle.assert_called_once_with(VOXEL_MAP_OBSTACLE_ID)

    def test_a_map_in_the_wrong_frame_is_dropped_not_reinterpreted(self, module_factory) -> None:
        # The points are metric positions. Registering them through a guessed
        # transform would invent geometry that was never observed.
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)

        module._apply_voxel_map(self._cloud([[0.0, 0.0, 0.0]], frame_id="odom"))

        assert module._world_monitor.update_obstacle.call_count == 0
        assert module._world_monitor.add_obstacle.call_count == 0

    def test_a_non_finite_map_is_dropped(self, module_factory) -> None:
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)

        module._apply_voxel_map(self._cloud([[0.0, 0.0, float("inf")]]))

        assert module._world_monitor.update_obstacle.call_count == 0

    def test_a_rejected_map_leaves_the_previous_one_standing(self, module_factory) -> None:
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)
        module._world_monitor.update_obstacle.side_effect = ValueError("too many cells")

        module._apply_voxel_map(self._cloud([[0.0, 0.0, 0.0]]))

        assert module._world_monitor.add_obstacle.call_count == 0


class TestObstacleUpdates:
    async def test_perception_objects_are_refreshable_and_queryable(self, module_factory) -> None:
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)
        detected = MagicMock()
        pose = PoseStamped(position=Vector3(0.4, 0.1, 0.2))
        obstacle = Obstacle(name="object-1", pose=pose, obstacle_type=ObstacleType.BOX)
        module._world_monitor.refresh_obstacles.return_value = [{"object_id": "object-1"}]
        module._world_monitor.world.get_obstacles.return_value = [obstacle]

        await module.handle_objects([detected])
        count = module.refresh_obstacles()
        obstacles = module.get_obstacles()

        module._world_monitor.on_objects.assert_called_once_with([detected])
        assert count == 1
        assert obstacles == {"object-1": pose}

    def test_scan_from_here_waits_for_osr_then_refreshes(self, module_factory, mocker) -> None:
        module = module_factory()
        module._world_monitor = MagicMock(spec=WorldMonitor)
        module._world_monitor.refresh_obstacles.return_value = [{"object_id": "one"}]
        module._world_monitor.world.get_obstacles.return_value = [
            Obstacle(name="one", pose=PoseStamped(), obstacle_type=ObstacleType.BOX)
        ]
        result_message = MagicMock(detections=[], detections_length=0)
        publish = mocker.patch.object(
            module.scan_requests,
            "publish",
            side_effect=lambda _prompts: module._scan_result_queue.put_nowait(result_message),
        )

        result = module.scan_from_here([" cup ", "bottle"], timeout=1.0)

        publish.assert_called_once_with(["cup", "bottle"])
        module._world_monitor.refresh_obstacles.assert_called_once_with(0.0)
        assert result == {"detected": 0, "refreshed": 1, "total": 1}

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

    def test_cancel_interrupts_active_work(self, module_factory):
        """Cancel works for executing motion and in-progress planning."""
        module = module_factory()

        module._state = ManipulationState.IDLE
        assert module.cancel().status is ExecutionStatus.NO_EXECUTION

        module._state = ManipulationState.PLANNING
        assert module.cancel().status is ExecutionStatus.ABORTED
        assert module._state == ManipulationState.IDLE
        assert module._planning_epoch == 1

        module._state = ManipulationState.EXECUTING
        assert module.cancel().status is ExecutionStatus.NO_EXECUTION
        assert module._state == ManipulationState.IDLE

    def test_cancel_hides_active_plan_preview(self, module_factory):
        module = module_factory()
        module._state = ManipulationState.EXECUTING
        module._last_plan = GeneratedPlan(
            trajectory=JointTrajectory(), group_ids=("manipulator",), path=[]
        )
        module._world_monitor = MagicMock()

        assert module.cancel().status is ExecutionStatus.NO_EXECUTION

        module._world_monitor.cancel_preview_animation.assert_called_once_with()

    def test_cancel_completed_execution_cancels_coordinator_task(self, module_factory):
        module = module_factory()
        config = _one_joint_config()
        _install_generated_plan(module, config, [0.0], [0.1])
        coordinator = _control_coordinator(cancel_status=TrajectoryCancellationStatus.CANCELLED)
        coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.ABORTED)
        module._control_coordinator = coordinator
        module._initialize_execution()

        assert module.execute(blocking=False).status is ExecutionStatus.ACCEPTED
        assert module._state == ManipulationState.EXECUTING

        assert module.cancel().status is ExecutionStatus.ABORTED
        module._control_coordinator.cancel_trajectory.assert_called_once_with()
        assert module._state == ManipulationState.IDLE

    def test_safe_cancel_clears_fault(self, module_factory):
        module = module_factory()

        module._state = ManipulationState.FAULT
        module._error_message = "Error"
        result = module.cancel()
        assert result.status is ExecutionStatus.NO_EXECUTION
        assert module._state == ManipulationState.IDLE
        assert module._error_message == ""

    def test_fail_sets_fault_state(self, module_factory):
        """_fail helper sets FAULT state and message."""
        module = module_factory()
        module._state = ManipulationState.PLANNING

        result = module._fail("Test error")
        assert result is False
        assert module._state == ManipulationState.IDLE
        assert module._error_message == "Test error"


class PlanningInitializationHarness:
    def __init__(self, mocker: MockerFixture) -> None:
        self.mock_world = MagicMock()
        self.mock_world_monitor = MagicMock(spec=WorldMonitor)
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

    def test_start_eagerly_initializes_planning_and_execution(
        self,
        mocker: MockerFixture,
        robot_config,
    ) -> None:
        module = ManipulationModule(model=robot_config)
        module.coordinator_joint_state = None
        module.voxel_map = None
        module.objects = None
        module.scan_results = None
        initialize_planning = mocker.patch.object(module, "_initialize_planning")
        initialize_execution = mocker.patch.object(module, "_initialize_execution")

        with module:
            initialize_planning.assert_called_once_with()
            initialize_execution.assert_called_once_with()

    def test_start_is_idempotent(self, mocker: MockerFixture, robot_config) -> None:
        module = ManipulationModule(model=robot_config)
        module.coordinator_joint_state = None
        module.voxel_map = None
        module.objects = None
        module.scan_results = None
        initialize_planning = mocker.patch.object(module, "_initialize_planning")
        initialize_execution = mocker.patch.object(module, "_initialize_execution")

        try:
            module.start()
            module.start()

            initialize_execution.assert_called_once_with()
            initialize_planning.assert_called_once_with()
        finally:
            module.stop()

    def test_state_is_readable_during_planning_initialization(
        self,
        mocker: MockerFixture,
        robot_config,
    ) -> None:
        module = ManipulationModule(model=robot_config)
        module._control_coordinator = _control_coordinator()
        module.coordinator_joint_state = None
        module.voxel_map = None
        module.objects = None
        module.scan_results = None
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
            model=robot_config,
            kinematics=kinematics,
        )

        ManipulationModule._initialize_planning(module)

        planning_initialization.mock_planning_specs.assert_called_once_with(
            world=planning_initialization.mock_world,
            world_backend="roboplan",
            planner=module.config.planner,
            kinematics=kinematics,
            trajectory_parametrization=ANY,
        )

    def test_nested_kinematics_config_parses_cli_override_shape(self, robot_config) -> None:
        """Pydantic parses the nested shape used by dynamic CLI overrides."""
        config = ManipulationModuleConfig(
            model=robot_config,
            kinematics={
                "backend": "pink",
                "max_iterations": "100",
                "dt": "0.02",
                "posture_cost": "0.0",
            },
        )

        assert isinstance(config.kinematics, PinkKinematicsConfig)
        assert config.kinematics.max_iterations == 100
        assert config.kinematics.dt == 0.02
        assert config.kinematics.posture_cost == 0.0

    def test_solve_ik_rpc_calls_configured_backend(self, robot_config, module_factory):
        """solve_ik returns the backend IKResult without path planning."""
        module = module_factory()
        module.config.model = robot_config
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.world.get_model_config.return_value = robot_config
        module._world_monitor.planning_groups = PlanningGroupRegistry(robot_config.planning_groups)
        current = JointState(name=robot_config.joint_names, position=[0.0, 0.0, 0.0])
        current_model_state = JointState(
            name=["joint1", "joint2", "joint3"],
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.current_model_joint_state.return_value = current_model_state
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
        assert kwargs["seed"].name == current_model_state.name
        assert kwargs["seed"].position == current.position
        assert kwargs["check_collision"] is True
        [(group, target_pose)] = kwargs["pose_targets"].items()
        assert group.id == "manipulator"
        assert target_pose.frame_id == "world"
        assert target_pose.position.x == 0.45

    def test_solve_ik_rpc_returns_failure_without_joint_state(self, robot_config, module_factory):
        """solve_ik reports a failed IKResult when no seed state is available."""
        module = module_factory()
        module.config.model = robot_config
        module.config.model = robot_config
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry(robot_config.planning_groups)
        module._world_monitor.current_model_joint_state.return_value = JointState(
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
        module.config.model = robot_config
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.world.get_model_config.return_value = robot_config
        module._world_monitor.planning_groups = PlanningGroupRegistry(robot_config.planning_groups)
        module._world_monitor.current_model_joint_state.return_value = None
        explicit_seed = JointState(name=robot_config.joint_names, position=[0.2, 0.1, 0.0])
        expected = IKResult(status=IKStatus.SUCCESS, joint_state=explicit_seed)
        module._kinematics = MagicMock()
        module._kinematics.solve_pose_targets.return_value = expected

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())
        result = module.solve_ik(pose, seed=explicit_seed)

        assert result is expected
        _, kwargs = module._kinematics.solve_pose_targets.call_args
        assert kwargs["seed"] is explicit_seed
        module._world_monitor.current_model_joint_state.assert_not_called()


class TestPlanningGroupApis:
    """Test explicit planning-group API behavior."""

    def test_list_planning_groups_and_model_info_include_groups(self, robot_config, module_factory):
        module = module_factory()
        module.config.model = robot_config
        registry = PlanningGroupRegistry(robot_config.planning_groups)
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = registry
        module._init_joints = None

        groups = module.list_planning_groups()
        info = module.get_model_info()

        assert [group.id for group in groups] == ["manipulator"]
        assert info["planning_groups"] == list(groups)
        assert info["planning_groups"][0].tip_frame == "link_tcp"

    def test_generate_joint_plan_stores_generated_plan(self, robot_config, module_factory):
        module = module_factory()
        module.config.model = robot_config
        registry = PlanningGroupRegistry(robot_config.planning_groups)
        _enable_simple_parametrization(module)
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.world.get_model_config.return_value = robot_config
        module._world_monitor.planning_groups = registry
        module._world_monitor.current_model_joint_state.return_value = JointState(
            name=["joint1", "joint2", "joint3"],
            position=[0.0, 0.0, 0.0],
        )
        result_path = [
            JointState(
                name=["joint1", "joint2", "joint3"],
                position=[0.0, 0.0, 0.0],
            ),
            JointState(
                name=["joint1", "joint2", "joint3"],
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

        plan = module.generate_plan_to_joint_targets(
            {
                "manipulator": JointState(
                    name=robot_config.joint_names,
                    position=[0.1, 0.2, 0.3],
                )
            }
        )

        assert plan is not None
        assert module._last_plan is not None
        assert module._last_plan.group_ids == ("manipulator",)
        assert module._last_plan.path == result_path
        assert module._last_plan.trajectory.points[-1].positions == [0.1, 0.2, 0.3]
        module._planner.plan_selected_joint_path.assert_called_once()
        _, kwargs = module._planner.plan_selected_joint_path.call_args
        assert kwargs["selection"].group_ids == ("manipulator",)
        assert kwargs["goal"].name == [
            "joint1",
            "joint2",
            "joint3",
        ]

        plan = module.generate_plan_to_joint_targets(
            {
                "manipulator": JointState(
                    name=robot_config.joint_names,
                    position=[0.1, 0.2, 0.3],
                )
            }
        )

        assert plan is not None
        assert module._planner.plan_selected_joint_path.call_count == 2

    def test_generate_pose_plan_uses_group_ik_and_selected_path(self, robot_config, module_factory):
        module = module_factory()
        module.config.model = robot_config
        registry = PlanningGroupRegistry(robot_config.planning_groups)
        _enable_simple_parametrization(module)
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.world.get_model_config.return_value = robot_config
        module._world_monitor.planning_groups = registry
        module._world_monitor.current_model_joint_state.return_value = JointState(
            name=["joint1", "joint2", "joint3"],
            position=[0.0, 0.0, 0.0],
        )
        ik_goal = JointState(
            name=["joint1", "joint2", "joint3"],
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
                    name=["joint1", "joint2", "joint3"],
                    position=[0.0, 0.0, 0.0],
                ),
                ik_goal,
            ],
        )
        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())

        plan = module.generate_plan_to_pose_targets({"manipulator": pose})

        assert plan is not None
        module._kinematics.solve_pose_targets.assert_called_once()
        _, ik_kwargs = module._kinematics.solve_pose_targets.call_args
        target_groups = list(ik_kwargs["pose_targets"].keys())
        assert [group.id for group in target_groups] == ["manipulator"]
        target_pose = ik_kwargs["pose_targets"][target_groups[0]]
        assert target_pose.position.x == 0.45
        assert ik_kwargs["seed"].name == [
            "joint1",
            "joint2",
            "joint3",
        ]
        _, planner_kwargs = module._planner.plan_selected_joint_path.call_args
        assert planner_kwargs["goal"] is ik_goal

    def test_failed_plan_materialization_clears_generated_plan(self, robot_config, module_factory):
        module = module_factory()
        module.config.model = robot_config
        registry = PlanningGroupRegistry(robot_config.planning_groups)
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.planning_groups = registry
        module._world_monitor.current_model_joint_state.return_value = JointState(
            name=["joint1", "joint2", "joint3"],
            position=[0.0, 0.0, 0.0],
        )
        module._world_monitor.current_model_joint_state.return_value = None
        module._last_plan = GeneratedPlan(
            trajectory=_generated_plan_trajectory(
                ["joint1", "joint2", "joint3"],
                [0.0, 0.0, 0.0],
                [0.1, 0.2, 0.3],
            ),
            group_ids=("manipulator",),
            status=PlanningStatus.SUCCESS,
            path=[
                JointState(
                    name=["joint1", "joint2", "joint3"],
                    position=[0.0, 0.0, 0.0],
                ),
                JointState(
                    name=["joint1", "joint2", "joint3"],
                    position=[0.2, 0.2, 0.2],
                ),
            ],
        )
        module._planner = MagicMock()
        module._planner.plan_selected_joint_path.return_value = PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=[
                JointState(
                    name=["joint1", "joint2", "joint3"],
                    position=[0.1, 0.2, 0.3],
                )
            ],
        )

        plan = module.generate_plan_to_joint_targets(
            {"manipulator": JointState(position=[0.1, 0.2, 0.3])}
        )

        assert plan is None
        assert module._state == ManipulationState.IDLE
        assert module._last_plan is None
        assert module.has_planned_path() is False

    def test_get_state_exposes_selected_group_init_preset(
        self, module_factory, mocker: MockerFixture
    ) -> None:
        model = _bimanual_config()
        module = module_factory()
        module.config.model = model
        module._init_joints = JointState(name=model.joint_names, position=[0.1, -0.1])
        module._world_monitor = MagicMock(spec=WorldMonitor)
        module._world_monitor.planning_groups = PlanningGroupRegistry(model.planning_groups)
        module._world_monitor.current_group_joint_state.return_value = None
        module._world_monitor.get_group_ee_pose.return_value = None

        preset = module.get_state().groups["left_arm"].joint_presets["init"]

        assert preset.name == ["left/j1"]
        assert preset.position == [0.1]

    def test_tf_loop_publishes_every_pose_group_for_bimanual_model(
        self, module_factory, mocker: MockerFixture
    ) -> None:
        model = _bimanual_config()
        module = module_factory()
        module.config.model = model
        module._world_monitor = MagicMock(spec=WorldMonitor)
        module._world_monitor.planning_groups = PlanningGroupRegistry(model.planning_groups)
        module._world_monitor.get_group_ee_pose.side_effect = [
            PoseStamped(position=Vector3(0.4, 0.2, 0.3)),
            PoseStamped(position=Vector3(0.4, -0.2, 0.3)),
        ]
        publish = mocker.patch.object(module.tf, "publish")

        def stop_after_first_iteration(_period: float) -> bool:
            module._tf_stop_event.set()
            return True

        mocker.patch.object(module._tf_stop_event, "wait", side_effect=stop_after_first_iteration)
        module._tf_stop_event.clear()

        module._tf_publish_loop()

        assert module._world_monitor.get_group_ee_pose.call_args_list == [
            call("left_arm"),
            call("right_arm"),
        ]
        publish.assert_called_once()

    def test_pose_wrappers_fail_safely_without_unique_pose_group(
        self, robot_config, module_factory
    ):
        no_pose_config = RobotModelConfig(
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
        module.config.model = no_pose_config
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry(
            no_pose_config.planning_groups
        )
        module._world_monitor.get_ee_pose.side_effect = ValueError("no pose group")
        module._kinematics = MagicMock()

        pose = Pose(position=Vector3(x=0.45, y=0.0, z=0.25), orientation=Quaternion())

        assert module.get_ee_pose() is None
        assert module.plan_to_pose(pose) is False
        result = module.inverse_kinematics_single(pose)
        assert result.status == IKStatus.NO_SOLUTION
        assert "no unique pose-targetable planning group" in result.message

    def test_pose_wrappers_fail_safely_with_multiple_pose_groups(
        self, robot_config, module_factory
    ):
        multi_pose_config = RobotModelConfig(
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
        module.config.model = multi_pose_config
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry(
            multi_pose_config.planning_groups
        )
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
        module.config.model = robot_config
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry(robot_config.planning_groups)
        module._world_monitor.current_model_joint_state.return_value = JointState(
            name=robot_config.joint_names,
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
    def test_planner_failure_preserves_backend_detail(self, robot_config, module_factory):
        """Planning diagnostics include the backend message."""
        module = module_factory()
        module.config.model = robot_config
        module._world_monitor = MagicMock()
        module._world_monitor.planning_groups = PlanningGroupRegistry(robot_config.planning_groups)
        module._world_monitor.current_model_joint_state.return_value = JointState(
            name=robot_config.joint_names,
            position=[0.0, 0.0, 0.0],
        )
        module._planner = MagicMock()
        module._planner.plan_selected_joint_path.return_value = PlanningResult(
            status=PlanningStatus.TIMEOUT, message="planner timed out"
        )
        result = module.plan_to_joints(
            {
                "manipulator": JointState(
                    name=robot_config.joint_names,
                    position=[1.0, 1.0, 1.0],
                )
            }
        )

        assert result.status is PlanStatus.FAILED
        assert module.get_error() == "Planning failed: TIMEOUT: planner timed out"
        assert module._state == ManipulationState.IDLE


class TestExecute:
    """Test coordinator execution."""

    def test_execute_requires_trajectory(self, robot_config, module_factory):
        """Execute fails without planned trajectory."""
        module = module_factory()

        assert module.execute().status is ExecutionStatus.NO_PLAN
        assert module._state == ManipulationState.IDLE
