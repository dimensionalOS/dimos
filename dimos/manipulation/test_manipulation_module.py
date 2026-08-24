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
from unittest.mock import MagicMock

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationState,
)
from dimos.manipulation.manipulation_spec import ExecutionStatus, PlanStatus
from dimos.manipulation.planning.planners.config import RRTConnectPlannerConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.xarm.config import XARM_MODEL_PATH, make_xarm7_model_config

pytestmark = pytest.mark.self_hosted


def _drake_available() -> bool:
    return importlib.util.find_spec("pydrake") is not None


def _xarm_urdf_available() -> bool:
    try:
        model_path = XARM_MODEL_PATH
        return model_path.exists()
    except Exception:
        return False


def _get_xarm7_config() -> RobotModelConfig:
    """Create XArm7 robot config for testing."""
    return make_xarm7_model_config(
        name="test_arm",
        add_gripper=False,
        joint_prefix="arm/",
    )


@pytest.fixture
def xarm7_config():
    return _get_xarm7_config()


@pytest.fixture
def joint_state_zeros():
    """Create a JointState message with zeros for XArm7."""
    return JointState(
        name=[
            "arm/joint1",
            "arm/joint2",
            "arm/joint3",
            "arm/joint4",
            "arm/joint5",
            "arm/joint6",
            "arm/joint7",
        ],
        position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        effort=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )


@pytest.fixture
def module(xarm7_config):
    """Create a started ManipulationModule with ports disabled."""
    coordinator = MagicMock(spec=ControlCoordinator)
    coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    coordinator.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.ALREADY_STOPPED
    )
    mod = ManipulationModule(
        robots=[xarm7_config],
        planning_timeout=10.0,
        world_backend="drake",
        planner=RRTConnectPlannerConfig(),
        visualization={"backend": "none"},
    )
    mod._control_coordinator = coordinator
    mod.coordinator_joint_state = None
    mod.objects = None
    mod.start()
    yield mod
    mod.stop()


@pytest.mark.skipif(not _drake_available(), reason="Drake not installed")
@pytest.mark.skipif(not _xarm_urdf_available(), reason="XArm URDF not available")
class TestManipulationModuleIntegration:
    """Integration tests for ManipulationModule with real Drake backend."""

    def test_module_initialization(self, module):
        """Test module initializes with real Drake world."""
        assert module._state == ManipulationState.IDLE
        assert module._world_monitor is not None
        assert module._planner is not None
        assert module._kinematics is not None
        assert "test_arm" in module._robots

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

        [group] = module.list_planning_groups()
        target = JointState(
            name=list(group.joint_names),
            position=[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
        )
        result = module.plan_to_joints({group.id: target})

        assert result.status is PlanStatus.SUCCEEDED
        assert module._state == ManipulationState.COMPLETED
        assert module.has_planned_path() is True

        assert module._last_plan is not None
        assert len(module._last_plan.trajectory.points) > 1
        assert module._last_plan.trajectory.duration > 0
        assert module._last_plan.group_ids == ("test_arm/manipulator",)

    def test_plan_to_explicit_joint_target(self, module, joint_state_zeros):
        """Test planning to an explicit planning-group joint target."""
        module._on_joint_state(joint_state_zeros)

        success = module.plan_to_joint_targets(
            {"test_arm/manipulator": JointState(position=[0.05] * 7)}
        )

        assert success is True
        assert module._state == ManipulationState.COMPLETED
        assert module._last_plan is not None
        assert module._last_plan.group_ids == ("test_arm/manipulator",)
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

    def test_group_info(self, module):
        """Test group-native discovery."""
        all_groups = module.list_planning_groups()
        assert [group.id for group in all_groups] == ["test_arm/manipulator"]
        assert all_groups[0].tip_frame == "link7"
        assert len(all_groups[0].joint_names) == 7

    def test_ee_pose(self, module, joint_state_zeros):
        """Test getting end-effector pose."""
        module._on_joint_state(joint_state_zeros)

        pose = module.get_ee_pose()

        assert pose is not None
        assert hasattr(pose, "x")
        assert hasattr(pose, "y")
        assert hasattr(pose, "z")

    def test_trajectory_name_translation(self, module, joint_state_zeros):
        """Test that trajectory joint names are translated for coordinator."""
        module._on_joint_state(joint_state_zeros)

        [group] = module.list_planning_groups()
        result = module.plan_to_joints(
            {group.id: JointState(name=list(group.joint_names), position=[0.05] * 7)}
        )
        assert result.succeeded

        assert module._last_plan is not None
        robot_config = module._robots["test_arm"][1]
        assert module.execute(blocking=False).status is ExecutionStatus.ACCEPTED
        trajectory = module._control_coordinator.execute_trajectory.call_args.args[0]

        assert trajectory.joint_names == list(robot_config.joint_name_mapping.keys())


@pytest.mark.skipif(not _drake_available(), reason="Drake not installed")
@pytest.mark.skipif(not _xarm_urdf_available(), reason="XArm URDF not available")
class TestCoordinatorIntegration:
    """Test coordinator integration with mocked RPC client."""

    def test_execute_with_mock_coordinator(self, module, joint_state_zeros):
        """Test execute sends trajectory to coordinator."""
        module._on_joint_state(joint_state_zeros)

        [group] = module.list_planning_groups()
        planned = module.plan_to_joints(
            {group.id: JointState(name=list(group.joint_names), position=[0.05] * 7)}
        )
        assert planned.succeeded

        result = module.execute(blocking=False)

        assert result.status is ExecutionStatus.ACCEPTED
        assert module._state == ManipulationState.EXECUTING

        # Verify coordinator was called
        module._control_coordinator.execute_trajectory.assert_called_once()
        trajectory = module._control_coordinator.execute_trajectory.call_args.args[0]

        assert len(trajectory.points) > 1
        # Joint names should be translated
        robot_config = module._robots["test_arm"][1]
        assert trajectory.joint_names == list(robot_config.joint_name_mapping.keys())

    def test_execute_rejected_by_coordinator(self, module, joint_state_zeros):
        """Test handling of coordinator rejection."""
        module._on_joint_state(joint_state_zeros)

        [group] = module.list_planning_groups()
        module.plan_to_joints(
            {group.id: JointState(name=list(group.joint_names), position=[0.05] * 7)}
        )

        module._control_coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(
            TrajectoryExecutionStatus.INVALID_TRAJECTORY
        )

        result = module.execute(blocking=False)

        assert result.status is ExecutionStatus.REJECTED
        assert module._state == ManipulationState.IDLE
        assert "rejected" in module._error_message.lower()

    def test_state_transitions_during_execution(self, module, joint_state_zeros):
        """Test state transitions during plan and execute."""
        assert module._state == ManipulationState.IDLE

        module._on_joint_state(joint_state_zeros)

        # Plan - should go through PLANNING -> COMPLETED
        [group] = module.list_planning_groups()
        target = JointState(name=list(group.joint_names), position=[0.05] * 7)
        module.plan_to_joints({group.id: target})
        assert module._state == ManipulationState.COMPLETED

        module.execute(blocking=False)
        assert module._state == ManipulationState.EXECUTING
