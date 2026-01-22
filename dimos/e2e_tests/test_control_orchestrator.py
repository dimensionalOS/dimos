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

"""End-to-end tests for the ControlOrchestrator.

These tests start a real orchestrator process and communicate via LCM/RPC.
Unlike unit tests, these verify the full system integration.

Run with:
    pytest dimos/e2e_tests/test_control_orchestrator.py -v -s
"""

import os
import time

import pytest

from dimos.control.orchestrator import ControlOrchestrator
from dimos.core.rpc_client import RPCClient
from dimos.msgs.sensor_msgs import JointState
from dimos.msgs.trajectory_msgs import JointTrajectory, TrajectoryPoint, TrajectoryState


@pytest.mark.skipif(bool(os.getenv("CI")), reason="LCM doesn't work in CI.")
@pytest.mark.e2e
class TestControlOrchestratorE2E:
    """End-to-end tests for ControlOrchestrator."""

    def test_orchestrator_starts_and_responds_to_rpc(self, lcm_spy, start_blueprint) -> None:
        """Test that orchestrator starts and responds to RPC queries."""
        # Save topics we care about (LCM topics include type suffix)
        joint_state_topic = "/orchestrator/joint_state#sensor_msgs.JointState"
        lcm_spy.save_topic(joint_state_topic)
        lcm_spy.save_topic("/rpc/ControlOrchestrator/list_joints/res")
        lcm_spy.save_topic("/rpc/ControlOrchestrator/list_tasks/res")

        # Start the mock orchestrator blueprint
        start_blueprint("orchestrator-mock")

        # Wait for joint state to be published (proves tick loop is running)
        lcm_spy.wait_for_saved_topic(
            joint_state_topic,
            timeout=10.0,
        )

        # Create RPC client and query
        client = RPCClient(None, ControlOrchestrator)
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

    def test_orchestrator_executes_trajectory(self, lcm_spy, start_blueprint) -> None:
        """Test that orchestrator executes a trajectory via RPC."""
        # Save topics
        lcm_spy.save_topic("/orchestrator/joint_state#sensor_msgs.JointState")
        lcm_spy.save_topic("/rpc/ControlOrchestrator/execute_trajectory/res")
        lcm_spy.save_topic("/rpc/ControlOrchestrator/get_trajectory_status/res")

        # Start orchestrator
        start_blueprint("orchestrator-mock")

        # Wait for it to be ready
        lcm_spy.wait_for_saved_topic(
            "/orchestrator/joint_state#sensor_msgs.JointState", timeout=10.0
        )

        # Create RPC client
        client = RPCClient(None, ControlOrchestrator)
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

            # Execute trajectory
            result = client.execute_trajectory("traj_arm", trajectory)
            assert result is True

            # Poll for completion
            timeout = 5.0
            start_time = time.time()
            completed = False

            while time.time() - start_time < timeout:
                status = client.get_trajectory_status("traj_arm")
                if status is not None and status.state == TrajectoryState.COMPLETED.name:
                    completed = True
                    break
                time.sleep(0.1)

            assert completed, "Trajectory did not complete within timeout"
        finally:
            client.stop_rpc_client()

    def test_orchestrator_joint_state_published(self, lcm_spy, start_blueprint) -> None:
        """Test that joint state messages are published at expected rate."""
        joint_state_topic = "/orchestrator/joint_state#sensor_msgs.JointState"
        lcm_spy.save_topic(joint_state_topic)

        # Start orchestrator
        start_blueprint("orchestrator-mock")

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

    def test_orchestrator_cancel_trajectory(self, lcm_spy, start_blueprint) -> None:
        """Test that a running trajectory can be cancelled."""
        lcm_spy.save_topic("/orchestrator/joint_state#sensor_msgs.JointState")

        # Start orchestrator
        start_blueprint("orchestrator-mock")
        lcm_spy.wait_for_saved_topic(
            "/orchestrator/joint_state#sensor_msgs.JointState", timeout=10.0
        )

        client = RPCClient(None, ControlOrchestrator)
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

            # Start trajectory
            result = client.execute_trajectory("traj_arm", trajectory)
            assert result is True

            # Wait a bit then cancel
            time.sleep(0.5)
            cancel_result = client.cancel_trajectory("traj_arm")
            assert cancel_result is True

            # Check status is ABORTED
            status = client.get_trajectory_status("traj_arm")
            assert status is not None
            assert status.state == TrajectoryState.ABORTED.name
        finally:
            client.stop_rpc_client()

    def test_dual_arm_orchestrator(self, lcm_spy, start_blueprint) -> None:
        """Test dual-arm orchestrator with independent trajectories."""
        lcm_spy.save_topic("/orchestrator/joint_state#sensor_msgs.JointState")

        # Start dual-arm mock orchestrator
        start_blueprint("orchestrator-dual-mock")
        lcm_spy.wait_for_saved_topic(
            "/orchestrator/joint_state#sensor_msgs.JointState", timeout=10.0
        )

        client = RPCClient(None, ControlOrchestrator)
        try:
            # Verify both arms present
            joints = client.list_joints()
            assert "left_joint1" in joints
            assert "right_joint1" in joints

            tasks = client.list_tasks()
            assert "traj_left" in tasks
            assert "traj_right" in tasks

            # Create trajectories for both arms
            left_trajectory = JointTrajectory(
                joint_names=[f"left_joint{i + 1}" for i in range(7)],
                points=[
                    TrajectoryPoint(time_from_start=0.0, positions=[0.0] * 7),
                    TrajectoryPoint(time_from_start=0.5, positions=[0.2] * 7),
                ],
            )

            right_trajectory = JointTrajectory(
                joint_names=[f"right_joint{i + 1}" for i in range(6)],
                points=[
                    TrajectoryPoint(time_from_start=0.0, positions=[0.0] * 6),
                    TrajectoryPoint(time_from_start=0.5, positions=[0.3] * 6),
                ],
            )

            # Execute both
            assert client.execute_trajectory("traj_left", left_trajectory) is True
            assert client.execute_trajectory("traj_right", right_trajectory) is True

            # Wait for completion
            time.sleep(1.0)

            # Both should complete
            left_status = client.get_trajectory_status("traj_left")
            right_status = client.get_trajectory_status("traj_right")

            assert left_status is not None and left_status.state == TrajectoryState.COMPLETED.name
            assert right_status is not None and right_status.state == TrajectoryState.COMPLETED.name
        finally:
            client.stop_rpc_client()
