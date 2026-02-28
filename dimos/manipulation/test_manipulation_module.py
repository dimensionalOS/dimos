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

from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationState,
)
from dimos.manipulation.planning.spec import RobotModelConfig
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Vector3
from dimos.msgs.sensor_msgs import JointState
from dimos.utils.data import get_data


def _drake_available() -> bool:
    return importlib.util.find_spec("pydrake") is not None


def _xarm_urdf_available() -> bool:
    try:
        desc_path = get_data("xarm_description")
        urdf_path = desc_path / "urdf/xarm_device.urdf.xacro"
        return urdf_path.exists()
    except Exception:
        return False


def _get_xarm7_config() -> RobotModelConfig:
    """Create XArm7 robot config for testing."""
    desc_path = get_data("xarm_description")
    return RobotModelConfig(
        name="test_arm",
        urdf_path=desc_path / "urdf/xarm_device.urdf.xacro",
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
        end_effector_link="link7",
        base_link="link_base",
        package_paths={"xarm_description": desc_path},
        xacro_args={"dof": "7", "limited": "true"},
        auto_convert_meshes=True,
        max_velocity=1.0,
        max_acceleration=2.0,
        joint_name_mapping={
            "arm_joint1": "joint1",
            "arm_joint2": "joint2",
            "arm_joint3": "joint3",
            "arm_joint4": "joint4",
            "arm_joint5": "joint5",
            "arm_joint6": "joint6",
            "arm_joint7": "joint7",
        },
        coordinator_task_name="traj_arm",
    )


@pytest.fixture
def xarm7_config():
    return _get_xarm7_config()


@pytest.fixture
def joint_state_zeros():
    """Create a JointState message with zeros for XArm7."""
    return JointState(
        name=[
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
            "arm_joint6",
            "arm_joint7",
        ],
        position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        effort=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )


@pytest.fixture
def module(xarm7_config):
    """Create a started ManipulationModule with ports disabled."""
    mod = ManipulationModule(
        robots=[xarm7_config],
        planning_timeout=10.0,
        enable_viz=False,
    )
    mod.joint_state = None
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

        target = JointState(position=[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        success = module.plan_to_joints(target)

        assert success is True
        assert module._state == ManipulationState.COMPLETED
        assert module.has_planned_path() is True

        assert "test_arm" in module._planned_trajectories
        traj = module._planned_trajectories["test_arm"]
        assert len(traj.points) > 1
        assert traj.duration > 0

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

    def test_robot_info(self, module):
        """Test getting robot information."""
        info = module.get_robot_info()

        assert info is not None
        assert info["name"] == "test_arm"
        assert len(info["joint_names"]) == 7
        assert info["end_effector_link"] == "link7"
        assert info["coordinator_task_name"] == "traj_arm"
        assert info["has_joint_name_mapping"] is True

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

        success = module.plan_to_joints(JointState(position=[0.05] * 7))
        assert success is True

        traj = module._planned_trajectories["test_arm"]
        robot_config = module._robots["test_arm"][1]

        translated = module._translate_trajectory_to_coordinator(traj, robot_config)

        for name in translated.joint_names:
            assert name.startswith("arm_")  # Should have arm_ prefix


@pytest.mark.skipif(not _drake_available(), reason="Drake not installed")
@pytest.mark.skipif(not _xarm_urdf_available(), reason="XArm URDF not available")
class TestCoordinatorIntegration:
    """Test coordinator integration with mocked RPC client."""

    def test_execute_with_mock_coordinator(self, module, joint_state_zeros):
        """Test execute sends trajectory to coordinator."""
        module._on_joint_state(joint_state_zeros)

        success = module.plan_to_joints(JointState(position=[0.05] * 7))
        assert success is True

        # Mock the coordinator client
        mock_client = MagicMock()
        mock_client.task_invoke.return_value = True
        module._coordinator_client = mock_client

        result = module.execute()

        assert result is True
        assert module._state == ManipulationState.COMPLETED

        # Verify coordinator was called
        mock_client.task_invoke.assert_called_once()
        call_args = mock_client.task_invoke.call_args
        task_name, method_name, kwargs = call_args[0]

        assert task_name == "traj_arm"
        assert method_name == "execute"
        trajectory = kwargs["trajectory"]
        assert len(trajectory.points) > 1
        # Joint names should be translated
        assert all(n.startswith("arm_") for n in trajectory.joint_names)

    def test_execute_rejected_by_coordinator(self, module, joint_state_zeros):
        """Test handling of coordinator rejection."""
        module._on_joint_state(joint_state_zeros)

        module.plan_to_joints(JointState(position=[0.05] * 7))

        # Mock coordinator to reject
        mock_client = MagicMock()
        mock_client.task_invoke.return_value = False
        module._coordinator_client = mock_client

        result = module.execute()

        assert result is False
        assert module._state == ManipulationState.FAULT
        assert "rejected" in module._error_message.lower()

    def test_state_transitions_during_execution(self, module, joint_state_zeros):
        """Test state transitions during plan and execute."""
        assert module._state == ManipulationState.IDLE

        module._on_joint_state(joint_state_zeros)

        # Plan - should go through PLANNING -> COMPLETED
        module.plan_to_joints(JointState(position=[0.05] * 7))
        assert module._state == ManipulationState.COMPLETED

        # Reset works from COMPLETED
        module.reset()
        assert module._state == ManipulationState.IDLE

        # Plan again
        module.plan_to_joints(JointState(position=[0.05] * 7))

        # Mock coordinator
        mock_client = MagicMock()
        mock_client.task_invoke.return_value = True
        module._coordinator_client = mock_client

        # Execute - should go to EXECUTING then COMPLETED
        module.execute()
        assert module._state == ManipulationState.COMPLETED
