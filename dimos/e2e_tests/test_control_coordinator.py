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

import time

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
    TrajectoryCancellationStatus,
    TrajectoryExecutionStatus,
)
from dimos.core.rpc_client import RPCClient
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState


class TestControlCoordinatorE2E:
    """End-to-end tests for ControlCoordinator."""

    def test_coordinator_starts_and_responds_to_rpc(self, lcm_spy, start_blueprint) -> None:
        """Test that coordinator starts and responds to RPC queries."""
        # Save topics we care about (LCM topics include type suffix)
        joint_state_topic = "/coordinator_joint_state#sensor_msgs.JointState"
        lcm_spy.save_topic(joint_state_topic)
        lcm_spy.save_topic("/rpc/ControlCoordinator/list_joints/res")
        lcm_spy.save_topic("/rpc/ControlCoordinator/list_tasks/res")

        # Start the mock coordinator blueprint
        start_blueprint("coordinator-mock")

        # Wait for joint state to be published (proves tick loop is running)
        lcm_spy.wait_for_saved_topic(joint_state_topic)

        # Create RPC client and query
        client = RPCClient(None, ControlCoordinator)
        try:
            # Test list_joints RPC
            joints = client.list_joints()
            assert joints is not None
            assert len(joints) == 7  # Mock arm has 7 DOF
            assert "arm/joint1" in joints

            # Test list_tasks RPC
            tasks = client.list_tasks()
            assert tasks is not None
            assert JOINT_TRAJECTORY_TASK_NAME in tasks

            # Test list_hardware RPC
            hardware = client.list_hardware()
            assert hardware is not None
            assert "arm" in hardware
        finally:
            client.stop_rpc_client()

    def test_coordinator_executes_trajectory(self, lcm_spy, start_blueprint, wait_until) -> None:
        """Test that coordinator executes a trajectory via RPC."""
        # Save topics
        lcm_spy.save_topic("/coordinator_joint_state#sensor_msgs.JointState")

        # Start coordinator
        start_blueprint("coordinator-mock")

        # Wait for it to be ready
        lcm_spy.wait_for_saved_topic("/coordinator_joint_state#sensor_msgs.JointState")

        # Create RPC client
        client = RPCClient(None, ControlCoordinator)
        try:
            # Get initial joint positions
            initial_positions = client.get_joint_positions()
            assert initial_positions is not None

            # Create a simple trajectory
            trajectory = JointTrajectory(
                joint_names=[f"arm/joint{i + 1}" for i in range(7)],
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

            # Execute through the coordinator's trajectory RPC (it injects
            # the authoritative current positions for start-state validation)
            result = client.execute_trajectory(trajectory)
            assert result.status is TrajectoryExecutionStatus.ACCEPTED, result

            # Poll for completion
            wait_until(
                lambda: client.task_invoke(JOINT_TRAJECTORY_TASK_NAME, "get_state")
                == TrajectoryState.COMPLETED,
                timeout=5.0,
                message="Trajectory did not complete within timeout",
            )
        finally:
            client.stop_rpc_client()

    def test_coordinator_joint_state_published(self, lcm_spy, start_blueprint) -> None:
        """Test that joint state messages are published at expected rate."""
        joint_state_topic = "/coordinator_joint_state#sensor_msgs.JointState"
        lcm_spy.save_topic(joint_state_topic)

        # Start coordinator
        start_blueprint("coordinator-mock")

        # Wait for initial message
        lcm_spy.wait_for_saved_topic(joint_state_topic)

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
        assert "arm/joint1" in joint_state.name

    def test_coordinator_cancel_trajectory(self, lcm_spy, start_blueprint) -> None:
        """Test that a running trajectory can be cancelled."""
        lcm_spy.save_topic("/coordinator_joint_state#sensor_msgs.JointState")

        # Start coordinator
        start_blueprint("coordinator-mock")
        lcm_spy.wait_for_saved_topic("/coordinator_joint_state#sensor_msgs.JointState")

        client = RPCClient(None, ControlCoordinator)
        try:
            # Create a long trajectory (5 seconds)
            trajectory = JointTrajectory(
                joint_names=[f"arm/joint{i + 1}" for i in range(7)],
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

            # Start trajectory
            result = client.execute_trajectory(trajectory)
            assert result.status is TrajectoryExecutionStatus.ACCEPTED, result

            # Wait a bit then cancel
            time.sleep(0.5)
            cancel_result = client.cancel_trajectory()
            assert cancel_result.status is TrajectoryCancellationStatus.CANCELLED, cancel_result

            # Check status is ABORTED
            state = client.task_invoke(JOINT_TRAJECTORY_TASK_NAME, "get_state")
            assert state is not None
            assert state == TrajectoryState.ABORTED
        finally:
            client.stop_rpc_client()

    def test_dual_arm_coordinator(self, lcm_spy, start_blueprint, wait_until) -> None:
        """Test dual-arm coordinator moving both arms with one combined trajectory."""
        lcm_spy.save_topic("/coordinator_joint_state#sensor_msgs.JointState")

        # Start dual-arm mock coordinator
        start_blueprint("coordinator-dual-mock")
        lcm_spy.wait_for_saved_topic("/coordinator_joint_state#sensor_msgs.JointState")

        client = RPCClient(None, ControlCoordinator)
        try:
            # Verify both arms present
            joints = client.list_joints()
            assert "left_arm/joint1" in joints
            assert "right_arm/joint1" in joints

            # The coordinator supports exactly one trajectory task, so the
            # dual-arm blueprint has a single task spanning both arms
            tasks = client.list_tasks()
            assert tasks == [JOINT_TRAJECTORY_TASK_NAME]

            # One trajectory moving the left arm (7 joints) and right arm
            # (6 joints) together
            trajectory = JointTrajectory(
                joint_names=[f"left_arm/joint{i + 1}" for i in range(7)]
                + [f"right_arm/joint{i + 1}" for i in range(6)],
                points=[
                    TrajectoryPoint(time_from_start=0.0, positions=[0.0] * 13),
                    TrajectoryPoint(time_from_start=0.5, positions=[0.2] * 7 + [0.3] * 6),
                ],
            )

            result = client.execute_trajectory(trajectory)
            assert result.status is TrajectoryExecutionStatus.ACCEPTED, result

            wait_until(
                lambda: client.task_invoke(JOINT_TRAJECTORY_TASK_NAME, "get_state")
                == TrajectoryState.COMPLETED,
                timeout=5.0,
                message="Trajectory did not complete within timeout",
            )
        finally:
            client.stop_rpc_client()
