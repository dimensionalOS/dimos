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
from unittest.mock import MagicMock, patch

import pytest

from dimos.agents.skill_result import SkillResult
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationState,
    PlanningJob,
)
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.spec.config import RobotModelConfig
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
def robot_config_with_mapping():
    """Create a robot config with joint name mapping (dual-arm scenario)."""
    return RobotModelConfig(
        name="left_arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1", "joint2", "joint3"],
        end_effector_link="link_tcp",
        base_link="link_base",
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


def _make_module():
    """Create a ManipulationModule instance with mocked __init__."""
    with patch.object(ManipulationModule, "__init__", lambda self: None):
        module = ManipulationModule.__new__(ManipulationModule)
        module._state = ManipulationState.IDLE
        module._lock = threading.Lock()
        module._error_message = ""
        module._robots = {}
        module._planned_paths = {}
        module._planned_trajectories = {}
        module._planning_jobs = {}
        module._executing = {}
        module._last_op_success = {}
        module._next_request_id = 0
        module._planning_pool = None
        module._world_monitor = None
        module._planner = None
        module._kinematics = None
        module._coordinator_client = None
        return module


class TestStateMachine:
    """Test state transitions."""

    def test_cancel_only_during_execution(self):
        """Cancel only works for robots in the execute accept window."""
        module = _make_module()

        assert module.cancel() is False

        module._executing["test_arm"] = True
        assert module.cancel() is True
        assert module._executing["test_arm"] is False

    def test_reset_not_during_execution(self):
        """Reset works in any state except EXECUTING."""
        module = _make_module()

        module._state = ManipulationState.FAULT
        module._error_message = "Error"
        result = module.reset()
        assert result.is_success()
        assert module._state == ManipulationState.IDLE
        assert module._error_message == ""

        module._executing["test_arm"] = True
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

    def test_begin_planning_registers_per_robot_job(self, robot_config):
        """_begin_planning registers a per-robot async planning job."""
        module = _make_module()
        module._world_monitor = MagicMock()
        module._planner = MagicMock()
        module._planning_pool = MagicMock()
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}

        assert module._begin_planning() == ("test_arm", "robot_id", 0)
        assert module.get_state() == ManipulationState.PLANNING.name
        assert module._planning_jobs["test_arm"].request_id == 0

    def test_begin_planning_rejects_fault_and_duplicate_active_job(self, robot_config):
        """_begin_planning rejects faulted modules and duplicate jobs per robot."""
        module = _make_module()
        module._world_monitor = MagicMock()
        module._planner = MagicMock()
        module._planning_pool = MagicMock()
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}

        assert module._begin_planning() == ("test_arm", "robot_id", 0)
        assert module._begin_planning() is None

        module.reset()
        module._state = ManipulationState.FAULT
        assert module._begin_planning() is None

    def test_clear_failed_plan_for_retry_clears_only_completed_failed_plan(self):
        """Internal retry clear removes one terminal failed plan."""
        module = _make_module()
        module._planning_jobs["test_arm"] = PlanningJob(
            request_id=1,
            accepted_at=1.0,
            completed_at=2.0,
            success=False,
            error="no IK",
        )
        module._last_op_success["test_arm"] = False
        module._planned_paths["test_arm"] = MagicMock()
        module._planned_trajectories["test_arm"] = MagicMock()
        module._error_message = "no IK"

        assert module.get_state() == ManipulationState.FAULT.name
        assert module._clear_failed_plan_for_retry("test_arm") is True
        assert module.get_state() == ManipulationState.IDLE.name
        assert "test_arm" not in module._planning_jobs
        assert "test_arm" not in module._last_op_success
        assert module._error_message == ""

    def test_clear_failed_plan_for_retry_preserves_other_fault(self):
        """Internal retry clear does not mask another robot's failed plan."""
        module = _make_module()
        for name, success in {"left": False, "right": False}.items():
            module._planning_jobs[name] = PlanningJob(
                request_id=1,
                accepted_at=1.0,
                completed_at=2.0,
                success=success,
                error=f"{name} failed",
            )
            module._last_op_success[name] = success

        assert module._clear_failed_plan_for_retry("left") is True
        assert module.get_state() == ManipulationState.FAULT.name
        assert module._last_op_success == {"right": False}


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


class TestPickAndPlaceRetry:
    """Test pick() grasp-candidate retry behavior."""

    def test_pick_clears_failed_candidate_plan_before_retry(self, robot_config):
        """A failed approach plan for one grasp candidate does not block the next."""
        with patch.object(PickAndPlaceModule, "__init__", lambda self: None):
            module = PickAndPlaceModule.__new__(PickAndPlaceModule)
        module._robots = {"test_arm": ("robot_id", robot_config, MagicMock())}

        grasp_poses = [
            Pose(Vector3(0.2, 0.0, 0.2), Quaternion()),
            Pose(Vector3(0.25, 0.0, 0.2), Quaternion()),
        ]
        module._generate_grasps_for_pick = MagicMock(return_value=grasp_poses)
        module._lift_if_low = MagicMock(return_value=SkillResult.ok())
        module._compute_pre_grasp_pose = MagicMock(side_effect=lambda pose, _offset: pose)
        module._wait_plan = MagicMock(side_effect=["Error: Planning failed: no IK", None])
        module._set_gripper_position = MagicMock()
        module._preview_execute_wait = MagicMock(return_value=SkillResult.ok())

        events: list[str] = []

        def plan_to_pose(_pose: Pose, _robot_name: str) -> bool:
            events.append("plan")
            return True

        def clear_failed_plan(_robot_name: str) -> bool:
            events.append("clear")
            return True

        module.plan_to_pose = MagicMock(side_effect=plan_to_pose)
        module._clear_failed_plan_for_retry = MagicMock(side_effect=clear_failed_plan)

        with patch("dimos.manipulation.pick_and_place_module.time.sleep", return_value=None):
            result = module.pick("cup", robot_name="test_arm")

        assert result.is_success()
        assert "Pick complete" in result.message
        assert "'cup'" in result.message
        assert module._wait_plan.call_count == 2
        assert module._clear_failed_plan_for_retry.call_count == 1
        assert events[:3] == ["plan", "clear", "plan"]
        assert module.plan_to_pose.call_count == 4


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
        module._planned_trajectories = {}
        module._coordinator_client = MagicMock()

        assert module.execute() is False

    def test_execute_requires_task_name(self):
        """Execute fails without coordinator_task_name."""
        module = _make_module()
        config_no_task = RobotModelConfig(
            name="arm",
            model_path=Path("/path"),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j1"],
            end_effector_link="ee",
        )
        module._robots = {"arm": ("id", config_no_task, MagicMock())}
        module._planned_trajectories = {"arm": MagicMock()}

        assert module.execute() is False

    def test_execute_success(self, robot_config, simple_trajectory):
        """Successful execute calls coordinator via task_invoke."""
        module = _make_module()
        module._robots = {"test_arm": ("id", robot_config, MagicMock())}
        module._planned_trajectories = {"test_arm": simple_trajectory}

        mock_client = MagicMock()
        mock_client.task_invoke.return_value = True
        module._coordinator_client = mock_client

        assert module.execute() is True
        assert module.get_state() == ManipulationState.COMPLETED.name
        mock_client.task_invoke.assert_called_once_with(
            "traj_arm", "execute", {"trajectory": simple_trajectory}
        )

    def test_execute_rejected(self, robot_config, simple_trajectory):
        """Rejected execution sets FAULT state."""
        module = _make_module()
        module._robots = {"test_arm": ("id", robot_config, MagicMock())}
        module._planned_trajectories = {"test_arm": simple_trajectory}

        mock_client = MagicMock()
        mock_client.task_invoke.return_value = False
        module._coordinator_client = mock_client

        assert module.execute() is False
        assert module.get_state() == ManipulationState.FAULT.name

    def test_execute_task_invoke_exception_sets_fault(self, robot_config, simple_trajectory):
        """Coordinator exceptions are recorded as execution failures."""
        module = _make_module()
        module._robots = {"test_arm": ("id", robot_config, MagicMock())}
        module._planned_trajectories = {"test_arm": simple_trajectory}

        mock_client = MagicMock()
        mock_client.task_invoke.side_effect = RuntimeError("coordinator crashed")
        module._coordinator_client = mock_client

        assert module.execute() is False
        assert module._executing["test_arm"] is False
        assert module.get_state() == ManipulationState.FAULT.name
        assert "coordinator crashed" in module.get_error()

    def test_execute_translation_exception_sets_fault(self, robot_config, simple_trajectory):
        """Trajectory translation exceptions are recorded as execution failures."""
        module = _make_module()
        module._robots = {"test_arm": ("id", robot_config, MagicMock())}
        module._planned_trajectories = {"test_arm": simple_trajectory}
        module._coordinator_client = MagicMock()
        module._translate_trajectory_to_coordinator = MagicMock(
            side_effect=RuntimeError("bad trajectory")
        )

        assert module.execute() is False
        assert module._executing["test_arm"] is False
        assert module.get_state() == ManipulationState.FAULT.name
        assert "bad trajectory" in module.get_error()


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
            end_effector_link="ee",
            base_link="base",
            joint_name_mapping={"left/j1": "j1", "left/j2": "j2"},
            coordinator_task_name="traj_left",
        )
        right_config = RobotModelConfig(
            name="right",
            model_path=Path("/path/to/robot.urdf"),
            base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            joint_names=["j1", "j2"],
            end_effector_link="ee",
            base_link="base",
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
