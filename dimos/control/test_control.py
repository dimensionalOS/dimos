# Copyright 2026 Dimensional Inc.
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

"""Tests for the Control Orchestrator module."""

from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock

import pytest

from dimos.control.hardware_interface import BackendHardwareInterface
from dimos.control.task import (
    ControlMode,
    JointCommandOutput,
    JointStateSnapshot,
    OrchestratorState,
    ResourceClaim,
)
from dimos.control.tasks.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryState,
)
from dimos.control.tick_loop import TickLoop
from dimos.hardware.manipulators.spec import ManipulatorBackend
from dimos.msgs.trajectory_msgs import JointTrajectory, TrajectoryPoint

# =============================================================================
# Fixtures
# =============================================================================


@pytest.fixture
def mock_backend():
    """Create a mock manipulator backend."""
    backend = MagicMock(spec=ManipulatorBackend)
    backend.get_dof.return_value = 6
    backend.read_joint_positions.return_value = [0.0] * 6
    backend.read_joint_velocities.return_value = [0.0] * 6
    backend.read_joint_efforts.return_value = [0.0] * 6
    backend.write_joint_positions.return_value = True
    backend.write_joint_velocities.return_value = True
    backend.set_control_mode.return_value = True
    return backend


@pytest.fixture
def hardware_interface(mock_backend):
    """Create a BackendHardwareInterface with mock backend."""
    return BackendHardwareInterface(
        backend=mock_backend,
        hardware_id="test_arm",
        joint_prefix="arm",
    )


@pytest.fixture
def trajectory_task():
    """Create a JointTrajectoryTask for testing."""
    config = JointTrajectoryTaskConfig(
        joint_names=["arm_joint1", "arm_joint2", "arm_joint3"],
        priority=10,
    )
    return JointTrajectoryTask(name="test_traj", config=config)


@pytest.fixture
def simple_trajectory():
    """Create a simple 2-point trajectory."""
    return JointTrajectory(
        joint_names=["arm_joint1", "arm_joint2", "arm_joint3"],
        points=[
            TrajectoryPoint(
                positions=[0.0, 0.0, 0.0],
                velocities=[0.0, 0.0, 0.0],
                time_from_start=0.0,
            ),
            TrajectoryPoint(
                positions=[1.0, 0.5, 0.25],
                velocities=[0.0, 0.0, 0.0],
                time_from_start=1.0,
            ),
        ],
    )


@pytest.fixture
def orchestrator_state():
    """Create a sample OrchestratorState."""
    joints = JointStateSnapshot(
        joint_positions={"arm_joint1": 0.0, "arm_joint2": 0.0, "arm_joint3": 0.0},
        joint_velocities={"arm_joint1": 0.0, "arm_joint2": 0.0, "arm_joint3": 0.0},
        joint_efforts={"arm_joint1": 0.0, "arm_joint2": 0.0, "arm_joint3": 0.0},
        timestamp=time.perf_counter(),
    )
    return OrchestratorState(joints=joints, t_now=time.perf_counter(), dt=0.01)


# =============================================================================
# Test JointCommandOutput
# =============================================================================


class TestJointCommandOutput:
    def test_position_output(self):
        output = JointCommandOutput(
            joint_names=["j1", "j2"],
            positions=[0.5, 1.0],
            mode=ControlMode.POSITION,
        )
        assert output.get_values() == [0.5, 1.0]
        assert output.mode == ControlMode.POSITION

    def test_velocity_output(self):
        output = JointCommandOutput(
            joint_names=["j1", "j2"],
            velocities=[0.1, 0.2],
            mode=ControlMode.VELOCITY,
        )
        assert output.get_values() == [0.1, 0.2]
        assert output.mode == ControlMode.VELOCITY

    def test_torque_output(self):
        output = JointCommandOutput(
            joint_names=["j1", "j2"],
            efforts=[5.0, 10.0],
            mode=ControlMode.TORQUE,
        )
        assert output.get_values() == [5.0, 10.0]
        assert output.mode == ControlMode.TORQUE

    def test_no_values_returns_none(self):
        output = JointCommandOutput(
            joint_names=["j1"],
            mode=ControlMode.POSITION,
        )
        assert output.get_values() is None


# =============================================================================
# Test JointStateSnapshot
# =============================================================================


class TestJointStateSnapshot:
    def test_get_position(self):
        snapshot = JointStateSnapshot(
            joint_positions={"j1": 0.5, "j2": 1.0},
            joint_velocities={"j1": 0.0, "j2": 0.1},
            joint_efforts={"j1": 1.0, "j2": 2.0},
            timestamp=100.0,
        )
        assert snapshot.get_position("j1") == 0.5
        assert snapshot.get_position("j2") == 1.0
        assert snapshot.get_position("nonexistent") is None


# =============================================================================
# Test BackendHardwareInterface
# =============================================================================


class TestBackendHardwareInterface:
    def test_joint_names_prefixed(self, hardware_interface):
        names = hardware_interface.joint_names
        assert names == [
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
            "arm_joint6",
        ]

    def test_read_state(self, hardware_interface):
        state = hardware_interface.read_state()
        assert "arm_joint1" in state
        assert len(state) == 6
        pos, vel, eff = state["arm_joint1"]
        assert pos == 0.0
        assert vel == 0.0
        assert eff == 0.0

    def test_write_command(self, hardware_interface, mock_backend):
        commands = {
            "arm_joint1": 0.5,
            "arm_joint2": 1.0,
        }
        hardware_interface.write_command(commands, ControlMode.POSITION)
        mock_backend.write_joint_positions.assert_called()


# =============================================================================
# Test JointTrajectoryTask
# =============================================================================


class TestJointTrajectoryTask:
    def test_initial_state(self, trajectory_task):
        assert trajectory_task.name == "test_traj"
        assert not trajectory_task.is_active()
        assert trajectory_task.get_state() == TrajectoryState.IDLE

    def test_claim(self, trajectory_task):
        claim = trajectory_task.claim()
        assert claim.priority == 10
        assert "arm_joint1" in claim.joints
        assert "arm_joint2" in claim.joints
        assert "arm_joint3" in claim.joints

    def test_execute_trajectory(self, trajectory_task, simple_trajectory):
        time.perf_counter()
        result = trajectory_task.execute(simple_trajectory)
        assert result is True
        assert trajectory_task.is_active()
        assert trajectory_task.get_state() == TrajectoryState.EXECUTING

    def test_compute_during_trajectory(
        self, trajectory_task, simple_trajectory, orchestrator_state
    ):
        t_start = time.perf_counter()
        trajectory_task.execute(simple_trajectory)

        # First compute sets start time (deferred start)
        state0 = OrchestratorState(
            joints=orchestrator_state.joints,
            t_now=t_start,
            dt=0.01,
        )
        trajectory_task.compute(state0)

        # Compute at 0.5s into trajectory
        state = OrchestratorState(
            joints=orchestrator_state.joints,
            t_now=t_start + 0.5,
            dt=0.01,
        )
        output = trajectory_task.compute(state)

        assert output is not None
        assert output.mode == ControlMode.SERVO_POSITION
        assert len(output.positions) == 3
        assert 0.4 < output.positions[0] < 0.6

    def test_trajectory_completes(self, trajectory_task, simple_trajectory, orchestrator_state):
        t_start = time.perf_counter()
        trajectory_task.execute(simple_trajectory)

        # First compute sets start time (deferred start)
        state0 = OrchestratorState(
            joints=orchestrator_state.joints,
            t_now=t_start,
            dt=0.01,
        )
        trajectory_task.compute(state0)

        # Compute past trajectory duration
        state = OrchestratorState(
            joints=orchestrator_state.joints,
            t_now=t_start + 1.5,
            dt=0.01,
        )
        output = trajectory_task.compute(state)

        # On completion, returns final position (not None) to hold at goal
        assert output is not None
        assert output.positions == [1.0, 0.5, 0.25]  # Final trajectory point
        assert not trajectory_task.is_active()
        assert trajectory_task.get_state() == TrajectoryState.COMPLETED

    def test_cancel_trajectory(self, trajectory_task, simple_trajectory):
        trajectory_task.execute(simple_trajectory)
        assert trajectory_task.is_active()

        trajectory_task.cancel()
        assert not trajectory_task.is_active()
        assert trajectory_task.get_state() == TrajectoryState.ABORTED

    def test_preemption(self, trajectory_task, simple_trajectory):
        trajectory_task.execute(simple_trajectory)

        trajectory_task.on_preempted("safety_task", frozenset({"arm_joint1"}))
        assert trajectory_task.get_state() == TrajectoryState.ABORTED
        assert not trajectory_task.is_active()

    def test_progress(self, trajectory_task, simple_trajectory, orchestrator_state):
        t_start = time.perf_counter()
        trajectory_task.execute(simple_trajectory)

        # First compute sets start time (deferred start)
        state0 = OrchestratorState(
            joints=orchestrator_state.joints,
            t_now=t_start,
            dt=0.01,
        )
        trajectory_task.compute(state0)

        assert trajectory_task.get_progress(t_start) == pytest.approx(0.0, abs=0.01)
        assert trajectory_task.get_progress(t_start + 0.5) == pytest.approx(0.5, abs=0.01)
        assert trajectory_task.get_progress(t_start + 1.0) == pytest.approx(1.0, abs=0.01)


# =============================================================================
# Test Arbitration Logic
# =============================================================================


class TestArbitration:
    def test_single_task_wins(self):
        outputs = [
            (
                MagicMock(name="task1"),
                ResourceClaim(joints=frozenset({"j1"}), priority=10),
                JointCommandOutput(joint_names=["j1"], positions=[0.5], mode=ControlMode.POSITION),
            ),
        ]

        winners = {}
        for task, claim, output in outputs:
            if output is None:
                continue
            values = output.get_values()
            if values is None:
                continue
            for i, joint in enumerate(output.joint_names):
                if joint not in winners:
                    winners[joint] = (claim.priority, values[i], output.mode, task.name)

        assert "j1" in winners
        assert winners["j1"][1] == 0.5

    def test_higher_priority_wins(self):
        task_low = MagicMock()
        task_low.name = "low_priority"
        task_high = MagicMock()
        task_high.name = "high_priority"

        outputs = [
            (
                task_low,
                ResourceClaim(joints=frozenset({"j1"}), priority=10),
                JointCommandOutput(joint_names=["j1"], positions=[0.5], mode=ControlMode.POSITION),
            ),
            (
                task_high,
                ResourceClaim(joints=frozenset({"j1"}), priority=100),
                JointCommandOutput(joint_names=["j1"], positions=[0.0], mode=ControlMode.POSITION),
            ),
        ]

        winners = {}
        for task, claim, output in outputs:
            if output is None:
                continue
            values = output.get_values()
            if values is None:
                continue
            for i, joint in enumerate(output.joint_names):
                if joint not in winners:
                    winners[joint] = (claim.priority, values[i], output.mode, task.name)
                elif claim.priority > winners[joint][0]:
                    winners[joint] = (claim.priority, values[i], output.mode, task.name)

        assert winners["j1"][3] == "high_priority"
        assert winners["j1"][1] == 0.0

    def test_non_overlapping_joints(self):
        task1 = MagicMock()
        task1.name = "task1"
        task2 = MagicMock()
        task2.name = "task2"

        outputs = [
            (
                task1,
                ResourceClaim(joints=frozenset({"j1", "j2"}), priority=10),
                JointCommandOutput(
                    joint_names=["j1", "j2"],
                    positions=[0.5, 0.6],
                    mode=ControlMode.POSITION,
                ),
            ),
            (
                task2,
                ResourceClaim(joints=frozenset({"j3", "j4"}), priority=10),
                JointCommandOutput(
                    joint_names=["j3", "j4"],
                    positions=[0.7, 0.8],
                    mode=ControlMode.POSITION,
                ),
            ),
        ]

        winners = {}
        for task, claim, output in outputs:
            if output is None:
                continue
            values = output.get_values()
            if values is None:
                continue
            for i, joint in enumerate(output.joint_names):
                if joint not in winners:
                    winners[joint] = (claim.priority, values[i], output.mode, task.name)

        assert winners["j1"][3] == "task1"
        assert winners["j2"][3] == "task1"
        assert winners["j3"][3] == "task2"
        assert winners["j4"][3] == "task2"


# =============================================================================
# Test TickLoop
# =============================================================================


class TestTickLoop:
    def test_tick_loop_starts_and_stops(self, mock_backend):
        hw = BackendHardwareInterface(mock_backend, "arm", "arm")
        hardware = {"arm": hw}
        tasks: dict = {}
        joint_to_hardware = {f"arm_joint{i + 1}": "arm" for i in range(6)}

        tick_loop = TickLoop(
            tick_rate=100.0,
            hardware=hardware,
            hardware_lock=threading.Lock(),
            tasks=tasks,
            task_lock=threading.Lock(),
            joint_to_hardware=joint_to_hardware,
        )

        tick_loop.start()
        time.sleep(0.05)
        assert tick_loop.tick_count > 0

        tick_loop.stop()
        final_count = tick_loop.tick_count
        time.sleep(0.02)
        assert tick_loop.tick_count == final_count

    def test_tick_loop_calls_compute(self, mock_backend):
        hw = BackendHardwareInterface(mock_backend, "arm", "arm")
        hardware = {"arm": hw}

        mock_task = MagicMock()
        mock_task.name = "test_task"
        mock_task.is_active.return_value = True
        mock_task.claim.return_value = ResourceClaim(
            joints=frozenset({"arm_joint1"}),
            priority=10,
        )
        mock_task.compute.return_value = JointCommandOutput(
            joint_names=["arm_joint1"],
            positions=[0.5],
            mode=ControlMode.POSITION,
        )

        tasks = {"test_task": mock_task}
        joint_to_hardware = {f"arm_joint{i + 1}": "arm" for i in range(6)}

        tick_loop = TickLoop(
            tick_rate=100.0,
            hardware=hardware,
            hardware_lock=threading.Lock(),
            tasks=tasks,
            task_lock=threading.Lock(),
            joint_to_hardware=joint_to_hardware,
        )

        tick_loop.start()
        time.sleep(0.05)
        tick_loop.stop()

        assert mock_task.compute.call_count > 0


# =============================================================================
# Integration Test
# =============================================================================


class TestIntegration:
    def test_full_trajectory_execution(self, mock_backend):
        hw = BackendHardwareInterface(mock_backend, "arm", "arm")
        hardware = {"arm": hw}

        config = JointTrajectoryTaskConfig(
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            priority=10,
        )
        traj_task = JointTrajectoryTask(name="traj_arm", config=config)
        tasks = {"traj_arm": traj_task}

        joint_to_hardware = {f"arm_joint{i + 1}": "arm" for i in range(6)}

        tick_loop = TickLoop(
            tick_rate=100.0,
            hardware=hardware,
            hardware_lock=threading.Lock(),
            tasks=tasks,
            task_lock=threading.Lock(),
            joint_to_hardware=joint_to_hardware,
        )

        trajectory = JointTrajectory(
            joint_names=[f"arm_joint{i + 1}" for i in range(6)],
            points=[
                TrajectoryPoint(
                    positions=[0.0] * 6,
                    velocities=[0.0] * 6,
                    time_from_start=0.0,
                ),
                TrajectoryPoint(
                    positions=[0.5] * 6,
                    velocities=[0.0] * 6,
                    time_from_start=0.5,
                ),
            ],
        )

        tick_loop.start()
        traj_task.execute(trajectory)

        time.sleep(0.6)
        tick_loop.stop()

        assert traj_task.get_state() == TrajectoryState.COMPLETED
        assert mock_backend.write_joint_positions.call_count > 0
