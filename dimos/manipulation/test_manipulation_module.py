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

"""
Integration tests for ManipulationModule.

These tests verify the full planning stack with Drake backend.
They require Drake to be installed and will be skipped otherwise.
"""

from __future__ import annotations

import importlib.util
from unittest.mock import DEFAULT, MagicMock

import numpy as np
from numpy.typing import NDArray
import pytest

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
    ManipulationState,
)
from dimos.manipulation.manipulation_spec import ExecutionStatus
from dimos.manipulation.planning.planners.config import RRTConnectPlannerConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus, PlanningStatus
from dimos.manipulation.planning.spec.joint_space import (
    CoordinateTopology,
    JointCoordinate,
    JointSpace,
)
from dimos.manipulation.planning.spec.models import IKResult, PlanningResult
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.robot.manipulators.xarm.config import make_xarm7_model_config

pytestmark = pytest.mark.self_hosted


def _drake_available() -> bool:
    return importlib.util.find_spec("pydrake") is not None


def _get_xarm7_config() -> RobotModelConfig:
    return make_xarm7_model_config(add_gripper=False)


@pytest.fixture
def xarm7_config():
    return _get_xarm7_config()


@pytest.fixture
def joint_state_zeros():
    """Create a JointState message with zeros for XArm7."""
    return JointState(
        name=[
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ],
        position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        effort=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )


@pytest.fixture
def module(xarm7_config):
    """Create a started ManipulationModule with ports disabled."""
    coordinator = MagicMock(spec=ControlCoordinator)
    coordinator.get_joint_positions.return_value = {}
    coordinator.execute_result = TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)

    def invoke(task: str, method: str, args: dict | None = None):
        if method == "execute":
            return coordinator.execute_result
        if method == "cancel":
            return TrajectoryCancellationResult(TrajectoryCancellationStatus.ALREADY_STOPPED)
        return DEFAULT

    coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.COMPLETED)
    coordinator.task_invoke.side_effect = invoke
    mod = ManipulationModule(
        model=xarm7_config,
        planning_timeout=10.0,
        world_backend="drake",
        planner=RRTConnectPlannerConfig(),
        visualization={"backend": "none"},
    )
    mod._control_coordinator = coordinator
    mod.coordinator_joint_state = None
    mod.voxel_map = None
    mod.objects = None
    try:
        mod.start()
        yield mod
    finally:
        mod.stop()


def test_plan_to_poses_hands_auxiliary_groups_to_the_pose_planner(xarm7_config):
    """plan_to_poses is the spec's way in; a mobile base named there must reach the planner."""
    module = ManipulationModule(model=xarm7_config, visualization={"backend": "none"})
    module.generate_plan_to_pose_targets = MagicMock(return_value=None)
    pose = PoseStamped(frame_id="world")
    try:
        result = module.plan_to_poses({"manipulator": pose}, auxiliary_groups=["moving_base"])
    finally:
        module.stop()

    assert not result.succeeded
    module.generate_plan_to_pose_targets.assert_called_once_with(
        {"manipulator": pose}, ["moving_base"], speed_scale=None
    )


_JOINT1_UPPER = 3.4907


def _interval_joint_space(names: list[str], upper: float) -> JointSpace:
    return JointSpace(
        tuple(
            JointCoordinate(
                name=name,
                mechanism_type="revolute",
                topology=CoordinateTopology.INTERVAL,
                lower=-upper,
                upper=upper,
                max_velocity=1.0,
                max_acceleration=1.0,
            )
            for name in names
        )
    )


def _module_planning_in(
    joint_space: JointSpace, model: RobotModelConfig, kinematics: MagicMock, planner: MagicMock
) -> ManipulationModule:
    """A module whose world moves in ``joint_space``, solving IK through ``kinematics`` and
    planning through ``planner``, which validates the goal through that joint space the way
    the real planners do."""
    module = ManipulationModule(model=model, visualization={"backend": "none"})
    names = list(joint_space.names)
    world_monitor = MagicMock()
    world_monitor.world.get_prepared_model.return_value.joint_space = joint_space
    world_monitor.planning_groups.select.return_value = MagicMock(joint_names=tuple(names))
    world_monitor.current_model_joint_state.return_value = JointState(
        name=names, position=[0.0] * len(names)
    )
    module._world_monitor = world_monitor
    module._kinematics = kinematics
    module._trajectory_parametrizer = MagicMock()
    module._planner = planner

    def plan(*, start: JointState, goal: JointState, **_: object) -> PlanningResult:
        try:
            joint_space.from_joint_state(goal)
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_GOAL, message=str(exc))
        return PlanningResult(status=PlanningStatus.SUCCESS, path=[start, goal])

    planner.plan_selected_joint_path.side_effect = plan
    return module


@pytest.mark.parametrize(
    ("past_limit", "accepted"),
    [(4.8e-9, True), (1e-3, False)],
    ids=["a rounding error past the limit", "a real violation"],
)
def test_an_ik_solution_a_rounding_error_past_a_limit_is_planned_from_inside_it(
    xarm7_config: RobotModelConfig, past_limit: float, accepted: bool
) -> None:
    """Pink IK tolerates a solution on a limit, and the planner's joint space refuses one any
    distance past it: joint1 = 3.490700004829506 against 3.4907 was "IK solved" and then
    INVALID_GOAL. A solution within 1e-6 past a limit now lands 1e-7 inside it; a real
    violation is still refused."""
    names = ["joint1", "joint2"]
    kinematics = MagicMock()
    kinematics.solve_pose_targets.return_value = IKResult(
        status=IKStatus.SUCCESS,
        joint_state=JointState(name=names, position=[_JOINT1_UPPER + past_limit, 0.5]),
    )
    planner = MagicMock()
    module = _module_planning_in(
        _interval_joint_space(names, _JOINT1_UPPER), xarm7_config, kinematics, planner
    )
    try:
        plan = module.generate_plan_to_pose_targets(
            {"manipulator": Pose()}, auxiliary_groups=["moving_base"]
        )
    finally:
        module.stop()

    goal = planner.plan_selected_joint_path.call_args.kwargs["goal"]
    assert goal.name == names
    if accepted:
        assert plan is not None
        assert goal.position == pytest.approx([_JOINT1_UPPER - 1e-7, 0.5], abs=1e-12)
    else:
        assert plan is None
        assert "INVALID_GOAL" in module._error_message
        assert goal.position == [_JOINT1_UPPER + past_limit, 0.5]


def _module_mapping_into(world_monitor: MagicMock, model: RobotModelConfig) -> ManipulationModule:
    module = ManipulationModule(model=model, visualization={"backend": "none"})
    world_monitor.is_finalized = True
    module._world_monitor = world_monitor
    return module


def _voxel_map(points: NDArray[np.float32]) -> PointCloud2:
    return PointCloud2.from_numpy(points, frame_id="world")


def test_a_voxel_map_identical_to_the_installed_one_is_not_rebuilt(
    xarm7_config: RobotModelConfig,
) -> None:
    """A mapper republishes an unchanged map at its own rate, and rebuilding the octree for
    each copy held the world lock the planner's collision checks wait on. A changed map, and
    the earlier map after a clear, still install; an empty map still clears."""
    world_monitor = MagicMock()
    world_monitor.update_obstacle.return_value = True
    module = _module_mapping_into(world_monitor, xarm7_config)
    points = np.array([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]], dtype=np.float32)
    try:
        module._apply_voxel_map(_voxel_map(points))
        module._apply_voxel_map(_voxel_map(points.copy()))
        assert world_monitor.update_obstacle.call_count == 1

        module._apply_voxel_map(_voxel_map(points + 0.1))
        assert world_monitor.update_obstacle.call_count == 2

        module._apply_voxel_map(_voxel_map(np.zeros((0, 3), dtype=np.float32)))
        world_monitor.remove_obstacle.assert_called_once_with(VOXEL_MAP_OBSTACLE_ID)
        module._apply_voxel_map(_voxel_map(points + 0.1))
        assert world_monitor.update_obstacle.call_count == 3
    finally:
        module.stop()
    obstacle = world_monitor.update_obstacle.call_args.args[0]
    assert obstacle.name == VOXEL_MAP_OBSTACLE_ID


def test_a_voxel_map_the_world_refused_is_tried_again(xarm7_config: RobotModelConfig) -> None:
    """Only an installed map is remembered: a refused one gets another chance."""
    world_monitor = MagicMock()
    world_monitor.update_obstacle.return_value = False
    world_monitor.add_obstacle.return_value = ""
    module = _module_mapping_into(world_monitor, xarm7_config)
    points = np.array([[0.1, 0.2, 0.3]], dtype=np.float32)
    try:
        module._apply_voxel_map(_voxel_map(points))
        module._apply_voxel_map(_voxel_map(points))
    finally:
        module.stop()

    assert world_monitor.update_obstacle.call_count == 2


@pytest.mark.skipif(not _drake_available(), reason="Drake not installed")
class TestManipulationModuleIntegration:
    """Integration tests for ManipulationModule with real Drake backend."""

    def test_module_initialization(self, module):
        """Test module initializes with real Drake world."""
        assert module._state == ManipulationState.IDLE
        assert module._world_monitor is not None
        assert module._planner is not None
        assert module._kinematics is not None
        assert module.get_model_config() == module.config.model

    def test_joint_state_sync(self, module, joint_state_zeros):
        """Test joint state synchronization to Drake world."""
        module._on_joint_state(joint_state_zeros)

        joints = module.get_current_joints()
        assert joints is not None
        assert len(joints) == 7
        assert all(abs(j) < 0.01 for j in joints)

    def test_collision_check(self, module, joint_state_zeros):
        """Test collision checking at a configuration."""
        module._on_joint_state(joint_state_zeros)

        is_free = module.is_collision_free([0.0] * 7)
        assert is_free is True

    def test_plan_to_joints(self, module, joint_state_zeros):
        """Test planning to a joint configuration."""
        module._on_joint_state(joint_state_zeros)

        target = JointState(position=[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        result = module.plan_to_joints({"manipulator": target})

        assert result.succeeded, result.message
        assert module._state == ManipulationState.COMPLETED
        assert module.has_planned_path() is True

        assert module._last_plan is not None
        assert len(module._last_plan.trajectory.points) > 1
        assert module._last_plan.trajectory.duration > 0
        assert module._last_plan.group_ids == ("manipulator",)

    def test_plan_to_explicit_joint_target(self, module, joint_state_zeros):
        """Test planning to an explicit planning-group joint target."""
        module._on_joint_state(joint_state_zeros)

        result = module.plan_to_joints({"manipulator": JointState(position=[0.05] * 7)})

        assert result.succeeded, result.message
        assert module._state == ManipulationState.COMPLETED
        assert module._last_plan is not None
        assert module._last_plan.group_ids == ("manipulator",)
        assert module.has_planned_path() is True
        assert module._last_plan.trajectory.points

    def test_add_and_remove_obstacle(self, module, joint_state_zeros):
        """Test adding and removing obstacles."""
        module._on_joint_state(joint_state_zeros)

        pose = Pose(
            position=Vector3(0.5, 0.0, 0.3),
            orientation=Quaternion(),  # default is identity (w=1)
        )
        obstacle_id = module.add_obstacle("test_box", pose, "box", [0.1, 0.1, 0.1])

        assert obstacle_id != ""
        assert obstacle_id is not None

        removed = module.remove_obstacle(obstacle_id)
        assert removed is True

    def test_model_info(self, module):
        """Test getting model information."""
        info = module.get_model_info()

        assert len(info["joint_names"]) == 7
        groups = info["planning_groups"]
        assert len(groups) == 1
        assert groups[0].id == "manipulator"
        assert groups[0].tip_frame == "link7"

        all_groups = module.list_planning_groups()
        assert [group.id for group in all_groups] == ["manipulator"]

    def test_ee_pose(self, module, joint_state_zeros):
        """Test getting end-effector pose."""
        module._on_joint_state(joint_state_zeros)

        pose = module.get_ee_pose()

        assert pose is not None
        assert pose == module.get_state().groups["manipulator"].end_effector_pose

    def test_trajectory_uses_canonical_names(self, module, joint_state_zeros):
        """Test that execution preserves canonical model joint names."""
        module._on_joint_state(joint_state_zeros)

        result = module.plan_to_joints({"manipulator": JointState(position=[0.05] * 7)})
        assert result.succeeded, result.message

        assert module._last_plan is not None
        assert module.execute().status is ExecutionStatus.COMPLETED
        trajectory = _executed(module._control_coordinator)

        assert trajectory.joint_names == module.config.model.joint_names


@pytest.mark.skipif(not _drake_available(), reason="Drake not installed")
class TestCoordinatorIntegration:
    """Test coordinator integration with mocked RPC client."""

    def test_execute_with_mock_coordinator(self, module, joint_state_zeros):
        """Test execute sends trajectory to coordinator."""
        module._on_joint_state(joint_state_zeros)

        result = module.plan_to_joints({"manipulator": JointState(position=[0.05] * 7)})
        assert result.succeeded, result.message

        result = module.execute()

        assert result.status is ExecutionStatus.COMPLETED
        assert module._state == ManipulationState.COMPLETED

        # Verify coordinator was called
        trajectory = _executed(module._control_coordinator)
        assert trajectory is not None

        assert len(trajectory.points) > 1
        assert trajectory.joint_names == module.config.model.joint_names

    def test_execute_rejected_by_coordinator(self, module, joint_state_zeros):
        """Test handling of coordinator rejection."""
        module._on_joint_state(joint_state_zeros)

        plan_result = module.plan_to_joints({"manipulator": JointState(position=[0.05] * 7)})
        assert plan_result.succeeded, plan_result.message

        module._control_coordinator.execute_result = TrajectoryExecutionResult(
            TrajectoryExecutionStatus.INVALID_TRAJECTORY
        )

        result = module.execute()

        assert result.status is ExecutionStatus.REJECTED
        assert module._state == ManipulationState.IDLE
        assert "rejected" in module._error_message.lower()

    def test_state_transitions_during_execution(self, module, joint_state_zeros):
        """Test state transitions during plan and execute."""
        assert module._state == ManipulationState.IDLE

        module._on_joint_state(joint_state_zeros)

        # Plan - should go through PLANNING -> COMPLETED
        result = module.plan_to_joints({"manipulator": JointState(position=[0.05] * 7)})
        assert result.succeeded, result.message
        assert module._state == ManipulationState.COMPLETED

        assert module.clear_planned_path().succeeded
        assert not module.has_planned_path()
        assert module.execute().status is ExecutionStatus.NO_PLAN

        # Plan again
        result = module.plan_to_joints({"manipulator": JointState(position=[0.05] * 7)})
        assert result.succeeded, result.message

        # Execute - should go to EXECUTING then COMPLETED
        assert module.execute(blocking=False).status is ExecutionStatus.ACCEPTED
        assert module._state == ManipulationState.EXECUTING
        assert module.wait_for_execution().status is ExecutionStatus.COMPLETED
        assert module._state == ManipulationState.COMPLETED


def _executed(coordinator):
    """The trajectory the joint trajectory task was asked to execute, or None."""
    for invocation in coordinator.task_invoke.call_args_list:
        if invocation.args[1] == "execute":
            return invocation.args[2]["trajectory"]
    return None
