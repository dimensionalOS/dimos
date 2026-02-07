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

"""End-to-end tests for the ControlCoordinator.

These tests start a real coordinator process and communicate via LCM/RPC.
Unlike unit tests, these verify the full system integration.
"""

import os
import time

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.rpc_client import RPCClient
from dimos.msgs.sensor_msgs import JointState
from dimos.msgs.trajectory_msgs import JointTrajectory, TrajectoryPoint, TrajectoryState


@pytest.mark.skipif(bool(os.getenv("CI")), reason="LCM doesn't work in CI.")
@pytest.mark.e2e
class TestControlCoordinatorE2E:
    """End-to-end tests for ControlCoordinator."""

    def test_coordinator_starts_and_responds_to_rpc(self, lcm_spy, start_blueprint) -> None:
        """Test that coordinator starts and responds to RPC queries."""
        # Save topics we care about (LCM topics include type suffix)
        joint_state_topic = "/coordinator/joint_state#sensor_msgs.JointState"
        lcm_spy.save_topic(joint_state_topic)
        lcm_spy.save_topic("/rpc/ControlCoordinator/list_joints/res")
        lcm_spy.save_topic("/rpc/ControlCoordinator/list_tasks/res")

        # Start the mock coordinator blueprint
        start_blueprint("coordinator-mock")

        # Wait for joint state to be published (proves tick loop is running)
        lcm_spy.wait_for_saved_topic(
            joint_state_topic,
            timeout=10.0,
        )

        # Create RPC client and query
        client = RPCClient(None, ControlCoordinator)
        try:
            # Test list_joints RPC
            joints = client.list_joints()
            assert joints is not None
            assert len(joints) == 7  # Mock arm has 7 DOF
            assert "arm_joint1" in joints

            # Test list_tasks RPC
            tasks = client.list_tasks()
            assert tasks is not None
            assert "traj_arm" in tasks

            # Test list_hardware RPC
            hardware = client.list_hardware()
            assert hardware is not None
            assert "arm" in hardware
        finally:
            client.stop_rpc_client()

    def test_coordinator_executes_trajectory(self, lcm_spy, start_blueprint) -> None:
        """Test that coordinator executes a trajectory via RPC."""
        # Save topics
        lcm_spy.save_topic("/coordinator/joint_state#sensor_msgs.JointState")

        # Start coordinator
        start_blueprint("coordinator-mock")

        # Wait for it to be ready
        lcm_spy.wait_for_saved_topic(
            "/coordinator/joint_state#sensor_msgs.JointState", timeout=10.0
        )

        # Create RPC client
        client = RPCClient(None, ControlCoordinator)
        try:
            # Get initial joint positions
            initial_positions = client.get_joint_positions()
            assert initial_positions is not None

            # Create a simple trajectory
            trajectory = JointTrajectory(
                joint_names=[f"arm_joint{i + 1}" for i in range(7)],
                points=[
                    TrajectoryPoint(
                        time_from_start=0.0,
                        positions=[0.0] * 7,
                        velocities=[0.0] * 7,
                    ),
                    TrajectoryPoint(
                        time_from_start=0.5,
                        positions=[0.1] * 7,
                        velocities=[0.0] * 7,
                    ),
                ],
            )

            # Execute trajectory via task_invoke
            result = client.task_invoke("traj_arm", "execute", {"trajectory": trajectory})
            assert result is True

            # Poll for completion
            timeout = 5.0
            start_time = time.time()
            completed = False

            while time.time() - start_time < timeout:
                state = client.task_invoke("traj_arm", "get_state")
                if state is not None and state == TrajectoryState.COMPLETED:
                    completed = True
                    break
                time.sleep(0.1)

            assert completed, "Trajectory did not complete within timeout"
        finally:
            client.stop_rpc_client()

    def test_coordinator_joint_state_published(self, lcm_spy, start_blueprint) -> None:
        """Test that joint state messages are published at expected rate."""
        joint_state_topic = "/coordinator/joint_state#sensor_msgs.JointState"
        lcm_spy.save_topic(joint_state_topic)

        # Start coordinator
        start_blueprint("coordinator-mock")

        # Wait for initial message
        lcm_spy.wait_for_saved_topic(joint_state_topic, timeout=10.0)

        # Collect messages for 1 second
        time.sleep(1.0)

        # Check we received messages (should be ~100 at 100Hz)
        with lcm_spy._messages_lock:
            message_count = len(lcm_spy.messages.get(joint_state_topic, []))

        # Allow some tolerance (at least 50 messages in 1 second)
        assert message_count >= 50, f"Expected ~100 messages, got {message_count}"

        # Decode a message to verify structure
        with lcm_spy._messages_lock:
            raw_msg = lcm_spy.messages[joint_state_topic][0]

        joint_state = JointState.lcm_decode(raw_msg)
        assert len(joint_state.name) == 7
        assert len(joint_state.position) == 7
        assert "arm_joint1" in joint_state.name

    def test_coordinator_cancel_trajectory(self, lcm_spy, start_blueprint) -> None:
        """Test that a running trajectory can be cancelled."""
        lcm_spy.save_topic("/coordinator/joint_state#sensor_msgs.JointState")

        # Start coordinator
        start_blueprint("coordinator-mock")
        lcm_spy.wait_for_saved_topic(
            "/coordinator/joint_state#sensor_msgs.JointState", timeout=10.0
        )

        client = RPCClient(None, ControlCoordinator)
        try:
            # Create a long trajectory (5 seconds)
            trajectory = JointTrajectory(
                joint_names=[f"arm_joint{i + 1}" for i in range(7)],
                points=[
                    TrajectoryPoint(
                        time_from_start=0.0,
                        positions=[0.0] * 7,
                        velocities=[0.0] * 7,
                    ),
                    TrajectoryPoint(
                        time_from_start=5.0,
                        positions=[1.0] * 7,
                        velocities=[0.0] * 7,
                    ),
                ],
            )

            # Start trajectory via task_invoke
            result = client.task_invoke("traj_arm", "execute", {"trajectory": trajectory})
            assert result is True

            # Wait a bit then cancel
            time.sleep(0.5)
            cancel_result = client.task_invoke("traj_arm", "cancel")
            assert cancel_result is True

            # Check status is ABORTED
            state = client.task_invoke("traj_arm", "get_state")
            assert state is not None
            assert state == TrajectoryState.ABORTED
        finally:
            client.stop_rpc_client()

    def test_dual_arm_coordinator(self, lcm_spy, start_blueprint) -> None:
        """Test dual-arm coordinator with independent trajectories."""
        lcm_spy.save_topic("/coordinator/joint_state#sensor_msgs.JointState")

        # Start dual-arm mock coordinator
        start_blueprint("coordinator-dual-mock")
        lcm_spy.wait_for_saved_topic(
            "/coordinator/joint_state#sensor_msgs.JointState", timeout=10.0
        )

        client = RPCClient(None, ControlCoordinator)
        try:
            # Verify both arms present
            joints = client.list_joints()
            assert "left_arm_joint1" in joints
            assert "right_arm_joint1" in joints

            tasks = client.list_tasks()
            assert "traj_left" in tasks
            assert "traj_right" in tasks

            # Create trajectories for both arms
            left_trajectory = JointTrajectory(
                joint_names=[f"left_arm_joint{i + 1}" for i in range(7)],
                points=[
                    TrajectoryPoint(time_from_start=0.0, positions=[0.0] * 7),
                    TrajectoryPoint(time_from_start=0.5, positions=[0.2] * 7),
                ],
            )

            right_trajectory = JointTrajectory(
                joint_names=[f"right_arm_joint{i + 1}" for i in range(6)],
                points=[
                    TrajectoryPoint(time_from_start=0.0, positions=[0.0] * 6),
                    TrajectoryPoint(time_from_start=0.5, positions=[0.3] * 6),
                ],
            )

            # Execute both via task_invoke
            assert (
                client.task_invoke("traj_left", "execute", {"trajectory": left_trajectory}) is True
            )
            assert (
                client.task_invoke("traj_right", "execute", {"trajectory": right_trajectory})
                is True
            )

            # Wait for completion
            time.sleep(1.0)

            # Both should complete
            left_state = client.task_invoke("traj_left", "get_state")
            right_state = client.task_invoke("traj_right", "get_state")

            assert left_state == TrajectoryState.COMPLETED
            assert right_state == TrajectoryState.COMPLETED
        finally:
            client.stop_rpc_client()
