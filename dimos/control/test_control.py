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

"""Tests for the Control Coordinator module."""

from __future__ import annotations

from collections.abc import Callable, Iterator
import math
import threading
from threading import Event, Thread
import time
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.control._control_test_helpers import RecordingTask
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    JointStateSnapshot,
    ResourceClaim,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryCancellationStatus,
    TrajectoryExecutionStatus,
    joint_trajectory_task,
)
from dimos.control.tick_loop import TickLoop
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState


@pytest.fixture
def trajectory_task():
    """Create a JointTrajectoryTask for testing."""
    config = JointTrajectoryTaskConfig(
        joint_names=["arm/joint1", "arm/joint2", "arm/joint3"],
        priority=10,
        velocity_limits={
            "arm/joint1": 1000.0,
            "arm/joint2": 1000.0,
            "arm/joint3": 1000.0,
        },
    )
    return JointTrajectoryTask(config=config)


@pytest.fixture
def simple_trajectory():
    """Create a simple 2-point trajectory."""
    return JointTrajectory(
        joint_names=["arm/joint1", "arm/joint2", "arm/joint3"],
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


def trajectory_start_positions(trajectory: JointTrajectory) -> dict[str, float]:
    return dict(
        zip(
            trajectory.joint_names,
            trajectory.points[0].positions,
            strict=True,
        )
    )


@pytest.fixture
def coordinator_state():
    """Create a sample CoordinatorState."""
    joints = JointStateSnapshot(
        joint_positions={"arm/joint1": 0.0, "arm/joint2": 0.0, "arm/joint3": 0.0},
        joint_velocities={"arm/joint1": 0.0, "arm/joint2": 0.0, "arm/joint3": 0.0},
        joint_efforts={"arm/joint1": 0.0, "arm/joint2": 0.0, "arm/joint3": 0.0},
        timestamp=time.perf_counter(),
    )
    return CoordinatorState(joints=joints, t_now=time.perf_counter(), dt=0.01)


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


@pytest.fixture
def make_coordinator() -> Iterator[Callable[..., ControlCoordinator]]:
    """Factory for real coordinators, all stopped on teardown."""
    coordinators: list[ControlCoordinator] = []

    def make(
        cls: type[ControlCoordinator] = ControlCoordinator, **kwargs: Any
    ) -> ControlCoordinator:
        coordinator = cls(publish_joint_state=False, **kwargs)
        coordinators.append(coordinator)
        return coordinator

    try:
        yield make
    finally:
        for coordinator in coordinators:
            coordinator.stop()


class _EEFTwistCoordinator(ControlCoordinator):
    ee_twist_command: In[TwistStamped]


class TestControlCoordinatorLifecycle:
    def test_start_subscribes_ee_twist_only_for_eef_twist_tasks(self, make_coordinator, mocker):
        mocker.patch("dimos.core.module.Module.start")
        mocker.patch("dimos.control.coordinator.TickLoop")

        def start_coordinator(tasks):
            coordinator = make_coordinator(cls=_EEFTwistCoordinator, tasks=tasks)
            coordinator._create_task_from_config = lambda cfg: RecordingTask(cfg.name)
            subscribe = mocker.patch.object(coordinator.ee_twist_command, "subscribe")
            coordinator.start()
            return coordinator, subscribe

        _, eef_twist_subscribe = start_coordinator(
            [
                TaskConfig(
                    name="eef",
                    type="eef_twist",
                    joint_names=["arm/joint1"],
                    params={"model_path": "fake"},
                )
            ]
        )
        _, non_eef_twist_subscribe = start_coordinator(
            [TaskConfig(name="traj", type="trajectory", joint_names=["arm/joint1"])]
        )

        eef_twist_subscribe.assert_called_once()
        non_eef_twist_subscribe.assert_not_called()

    def test_stop_unsubscribes_ee_twist_subscription(self, make_coordinator, mocker):
        coordinator = make_coordinator()
        unsubscribe = mocker.Mock()
        coordinator._stream_unsubs = {"ee_twist_command": unsubscribe}

        coordinator.stop()

        unsubscribe.assert_called_once_with()
        assert coordinator._stream_unsubs == {}

    def test_stream_callback_only_dispatches_the_original_message(self, make_coordinator, mocker):
        coordinator = make_coordinator()
        dispatch = mocker.patch.object(coordinator, "_dispatch")
        msg = Twist(linear=[1.0, 2.0, 0.0], angular=[0.0, 0.0, 3.0])
        coordinator._make_stream_cb("twist_command")(msg)
        dispatch.assert_called_once_with("twist_command", msg)

    def test_reset_runtime_state_calls_task_hooks(self, make_coordinator):
        class ResettableTask(BaseControlTask):
            def __init__(self) -> None:
                self._name = "resettable"
                self.reset_reactivate_args: list[bool | None] = []

            def claim(self) -> ResourceClaim:
                return ResourceClaim(joints=frozenset())

            def is_active(self) -> bool:
                return True

            def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
                return None

            def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
                pass

            def on_twist_command(self, msg: Any, t_now: float) -> None:
                # The g1_groot_wbc card binds twist_command; add_task now
                # resolves handlers at registration, so the stub needs it.
                pass

            def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
                self.reset_reactivate_args.append(reactivate)
                return True

        coordinator = make_coordinator()
        task = ResettableTask()

        # reset_runtime_state is card-gated; g1_groot_wbc declares it.
        assert coordinator.add_task(task, task_type="g1_groot_wbc")

        assert coordinator.reset_runtime_state(reactivate=True) == {"resettable": True}
        assert task.reset_reactivate_args == [True]


class TestControlCoordinatorTrajectoryExecution:
    def test_trajectory_config_requires_canonical_name(self, make_coordinator):
        coordinator = make_coordinator()
        config = TaskConfig(
            name="other_name",
            type="trajectory",
            joint_names=["arm/joint1"],
        )

        with pytest.raises(ValueError, match="must be named 'joint_trajectory'"):
            coordinator._create_task_from_config(config)

    def test_joint_trajectory_task_factory(self):
        config = joint_trajectory_task(
            ("arm/joint1", "arm/joint2"),
            priority=7,
            start_position_tolerance=0.02,
        )

        assert config.name == JOINT_TRAJECTORY_TASK_NAME
        assert config.type == "trajectory"
        assert config.joint_names == ["arm/joint1", "arm/joint2"]
        assert config.priority == 7
        assert config.params == {"start_position_tolerance": 0.02}

    def test_removing_trajectory_task_allows_replacement(self, make_coordinator):
        coordinator = make_coordinator()
        first = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["arm/joint1"]))
        second = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["arm/joint2"]))
        coordinator.add_task(first, task_type="trajectory")

        assert coordinator.remove_task(JOINT_TRAJECTORY_TASK_NAME)
        assert coordinator.add_task(second, task_type="trajectory")
        assert coordinator.get_task(JOINT_TRAJECTORY_TASK_NAME) is second

    def test_execute_and_cancel_without_trajectory_task_are_semantic(self, make_coordinator):
        coordinator = make_coordinator()

        execute_result = coordinator.execute_trajectory(JointTrajectory())
        cancel_result = coordinator.cancel_trajectory()

        assert execute_result.status is TrajectoryExecutionStatus.NO_TRAJECTORY_TASK
        assert cancel_result.status is TrajectoryCancellationStatus.NO_TRAJECTORY_TASK

    def test_cancel_waits_for_submission_and_leaves_no_active_trajectory(
        self, make_coordinator, trajectory_task, simple_trajectory, mocker
    ):
        coordinator = make_coordinator()
        coordinator.add_task(trajectory_task, task_type="trajectory")
        reading, release, cancelling, cancelled = Event(), Event(), Event(), Event()
        executions, cancellations = [], []

        def delayed_positions():
            reading.set()
            assert release.wait(timeout=2)
            return trajectory_start_positions(simple_trajectory)

        def cancel():
            cancelling.set()
            cancellations.append(coordinator.cancel_trajectory())
            cancelled.set()

        mocker.patch.object(coordinator, "get_joint_positions", side_effect=delayed_positions)
        execution = Thread(
            target=lambda: executions.append(coordinator.execute_trajectory(simple_trajectory))
        )
        cancellation = Thread(target=cancel)
        execution.start()
        try:
            assert reading.wait(timeout=2)
            cancellation.start()
            assert cancelling.wait(timeout=2)
            assert not cancelled.wait(timeout=0.05)
        finally:
            release.set()
            execution.join(timeout=2)
            if cancellation.ident is not None:
                cancellation.join(timeout=2)
        assert not execution.is_alive() and not cancellation.is_alive()
        assert executions[0].status is TrajectoryExecutionStatus.ACCEPTED
        assert cancellations[0].status is TrajectoryCancellationStatus.CANCELLED
        assert not trajectory_task.is_active()


class TestJointTrajectoryTask:
    def test_config_requires_at_least_one_joint(self):
        with pytest.raises(ValueError):
            JointTrajectoryTaskConfig(joint_names=[])

    @pytest.mark.parametrize(
        "tolerance",
        [-0.01, math.nan, math.inf, -math.inf],
    )
    def test_config_requires_finite_non_negative_start_tolerance(self, tolerance):
        with pytest.raises(ValueError):
            JointTrajectoryTaskConfig(
                joint_names=["arm/joint1"],
                start_position_tolerance=tolerance,
            )

    def test_initial_state(self, trajectory_task):
        assert trajectory_task.name == JOINT_TRAJECTORY_TASK_NAME
        assert not trajectory_task.is_active()
        assert trajectory_task.get_state() == TrajectoryState.IDLE

    def test_claim(self, trajectory_task):
        claim = trajectory_task.claim()
        assert claim.priority == 10
        assert "arm/joint1" in claim.joints
        assert "arm/joint2" in claim.joints
        assert "arm/joint3" in claim.joints

    def test_execute_trajectory(self, trajectory_task, simple_trajectory):
        result = trajectory_task.execute(
            simple_trajectory, trajectory_start_positions(simple_trajectory)
        )
        assert result.status is TrajectoryExecutionStatus.ACCEPTED
        assert trajectory_task.is_active()
        assert trajectory_task.get_state() == TrajectoryState.EXECUTING

    def test_joint_command_handler_converts_claimed_positions(self, trajectory_task):
        accepted = trajectory_task.on_joint_command(
            JointState(
                name=["arm/joint2", "other/joint"],
                position=[0.25, 9.0],
            ),
            t_now=1.0,
        )

        assert accepted
        assert trajectory_task._trajectory is not None
        assert trajectory_task._trajectory.joint_names == ["arm/joint2"]
        assert trajectory_task._trajectory.points[-1].positions == [0.25]

    @pytest.mark.parametrize(
        "command",
        [
            JointState(name=["arm/joint1"], velocity=[0.5]),
            JointState(name=["arm/joint1", "arm/joint2"], position=[0.5]),
            JointState(name=["other/joint"], position=[0.5]),
        ],
    )
    def test_joint_command_handler_ignores_non_position_commands(self, trajectory_task, command):
        assert not trajectory_task.on_joint_command(command, t_now=1.0)
        assert trajectory_task._trajectory is None

    def test_status_snapshot_is_non_destructive(self, trajectory_task, simple_trajectory):
        trajectory_task.execute(simple_trajectory, trajectory_start_positions(simple_trajectory))
        trajectory_task.compute(CoordinatorState(joints=MagicMock(), t_now=10.0, dt=0.01))

        active = trajectory_task.get_status(10.25)
        assert active.state is TrajectoryState.EXECUTING
        assert active.progress == pytest.approx(0.25)

        trajectory_task.compute(CoordinatorState(joints=MagicMock(), t_now=11.5, dt=0.01))
        terminal = trajectory_task.get_status(11.5)
        assert terminal.state is TrajectoryState.COMPLETED
        assert terminal.progress == pytest.approx(1.0)
        assert trajectory_task.get_status(11.6).state is TrajectoryState.COMPLETED

    def test_execute_partial_subset_and_claims_full_configuration(self, trajectory_task):
        trajectory = JointTrajectory(
            joint_names=["arm/joint2", "arm/joint3"],
            points=[
                TrajectoryPoint(positions=[0.0, 0.0], velocities=[0.0, 0.0], time_from_start=0.0),
                TrajectoryPoint(positions=[0.5, 1.0], velocities=[0.0, 0.0], time_from_start=1.0),
            ],
        )

        assert (
            trajectory_task.execute(trajectory, trajectory_start_positions(trajectory)).status
            is TrajectoryExecutionStatus.ACCEPTED
        )
        assert trajectory_task.claim().joints == frozenset(
            {"arm/joint1", "arm/joint2", "arm/joint3"}
        )

    def test_execute_rejects_missing_start_position(self, trajectory_task, simple_trajectory):
        result = trajectory_task.execute(
            simple_trajectory,
            {"arm/joint1": 0.0, "arm/joint2": 0.0},
        )

        assert result.status is TrajectoryExecutionStatus.START_STATE_UNAVAILABLE
        assert "arm/joint3" in result.message
        assert trajectory_task.get_state() == TrajectoryState.IDLE

    def test_execute_rejects_start_position_outside_tolerance(
        self, trajectory_task, simple_trajectory
    ):
        current_positions = trajectory_start_positions(simple_trajectory)
        current_positions["arm/joint2"] = 0.051

        result = trajectory_task.execute(simple_trajectory, current_positions)

        assert result.status is TrajectoryExecutionStatus.START_STATE_MISMATCH
        assert "arm/joint2" in result.message
        assert trajectory_task.get_state() == TrajectoryState.IDLE

    def test_execute_accepts_start_position_at_tolerance(self, trajectory_task, simple_trajectory):
        current_positions = trajectory_start_positions(simple_trajectory)
        current_positions["arm/joint2"] = 0.05

        result = trajectory_task.execute(simple_trajectory, current_positions)

        assert result.status is TrajectoryExecutionStatus.ACCEPTED

    @pytest.mark.parametrize(
        "trajectory",
        [
            JointTrajectory(
                joint_names=[],
                points=[TrajectoryPoint(time_from_start=0.0, positions=[], velocities=[])],
            ),
            JointTrajectory(
                joint_names=["arm/joint1", "arm/joint1"],
                points=[
                    TrajectoryPoint(
                        time_from_start=0.0, positions=[0.0, 0.0], velocities=[0.0, 0.0]
                    )
                ],
            ),
            JointTrajectory(
                joint_names=["arm/missing"],
                points=[TrajectoryPoint(time_from_start=0.0, positions=[0.0], velocities=[0.0])],
            ),
            JointTrajectory(joint_names=["arm/joint1"], points=[]),
            JointTrajectory(
                joint_names=["arm/joint1"],
                points=[TrajectoryPoint(time_from_start=0.0, positions=[], velocities=[0.0])],
            ),
            JointTrajectory(
                joint_names=["arm/joint1"],
                points=[
                    TrajectoryPoint(time_from_start=0.0, positions=[float("nan")], velocities=[0.0])
                ],
            ),
            JointTrajectory(
                joint_names=["arm/joint1"],
                points=[TrajectoryPoint(time_from_start=0.1, positions=[0.0], velocities=[0.0])],
            ),
            JointTrajectory(
                joint_names=["arm/joint1"],
                points=[
                    TrajectoryPoint(time_from_start=0.0, positions=[0.0], velocities=[0.0]),
                    TrajectoryPoint(time_from_start=0.0, positions=[1.0], velocities=[0.0]),
                ],
            ),
        ],
    )
    def test_invalid_partial_inputs_reject_before_state_changes(self, trajectory_task, trajectory):
        assert trajectory_task.get_state() == TrajectoryState.IDLE
        assert (
            trajectory_task.execute(trajectory, {}).status
            is TrajectoryExecutionStatus.INVALID_TRAJECTORY
        )
        assert trajectory_task.get_state() == TrajectoryState.IDLE
        assert (
            trajectory_task.compute(CoordinatorState(joints=MagicMock(), t_now=0.0, dt=0.01))
            is None
        )

    def test_compute_emits_active_subset_and_retains_final_target(self, trajectory_task):
        trajectory = JointTrajectory(
            joint_names=["arm/joint2"],
            points=[
                TrajectoryPoint(positions=[0.0], velocities=[0.0], time_from_start=0.0),
                TrajectoryPoint(positions=[1.0], velocities=[0.0], time_from_start=1.0),
            ],
        )
        assert (
            trajectory_task.execute(trajectory, trajectory_start_positions(trajectory)).status
            is TrajectoryExecutionStatus.ACCEPTED
        )
        trajectory_task.compute(CoordinatorState(joints=MagicMock(), t_now=10.0, dt=0.01))
        output = trajectory_task.compute(CoordinatorState(joints=MagicMock(), t_now=10.5, dt=0.01))
        assert output is not None
        assert output.joint_names == ["arm/joint2"]
        assert output.positions == [pytest.approx(0.5)]

        final = trajectory_task.compute(CoordinatorState(joints=MagicMock(), t_now=11.5, dt=0.01))
        assert final is not None
        assert final.joint_names == ["arm/joint2"]
        assert trajectory_task.get_state() == TrajectoryState.COMPLETED
        assert (
            trajectory_task.compute(CoordinatorState(joints=MagicMock(), t_now=12.0, dt=0.01))
            is None
        )

    def test_replacement_reset_and_cancel_clear_active_subset(self, trajectory_task):
        first = JointTrajectory(
            joint_names=["arm/joint1"],
            points=[
                TrajectoryPoint(positions=[0.0], velocities=[0.0], time_from_start=0.0),
                TrajectoryPoint(positions=[1.0], velocities=[0.0], time_from_start=1.0),
            ],
        )
        second = JointTrajectory(
            joint_names=["arm/joint3"],
            points=[
                TrajectoryPoint(positions=[2.0], velocities=[0.0], time_from_start=0.0),
                TrajectoryPoint(positions=[3.0], velocities=[0.0], time_from_start=1.0),
            ],
        )
        assert (
            trajectory_task.execute(first, trajectory_start_positions(first)).status
            is TrajectoryExecutionStatus.ACCEPTED
        )
        assert (
            trajectory_task.execute(second, trajectory_start_positions(second)).status
            is TrajectoryExecutionStatus.ACCEPTED
        )
        trajectory_task.compute(CoordinatorState(joints=MagicMock(), t_now=1.0, dt=0.01))
        output = trajectory_task.compute(CoordinatorState(joints=MagicMock(), t_now=1.5, dt=0.01))
        assert output is not None
        assert output.joint_names == ["arm/joint1", "arm/joint3"]
        assert trajectory_task.cancel().status is TrajectoryCancellationStatus.CANCELLED
        assert (
            trajectory_task.compute(CoordinatorState(joints=MagicMock(), t_now=2.0, dt=0.01))
            is None
        )
        assert trajectory_task.reset() is True
        assert trajectory_task.claim().joints == frozenset(
            {"arm/joint1", "arm/joint2", "arm/joint3"}
        )

    def test_compute_during_trajectory(self, trajectory_task, simple_trajectory, coordinator_state):
        t_start = time.perf_counter()
        trajectory_task.execute(simple_trajectory, trajectory_start_positions(simple_trajectory))

        # First compute sets start time (deferred start)
        state0 = CoordinatorState(
            joints=coordinator_state.joints,
            t_now=t_start,
            dt=0.01,
        )
        trajectory_task.compute(state0)

        # Compute at 0.5s into trajectory
        state = CoordinatorState(
            joints=coordinator_state.joints,
            t_now=t_start + 0.5,
            dt=0.01,
        )
        output = trajectory_task.compute(state)

        assert output is not None
        assert output.mode == ControlMode.SERVO_POSITION
        assert len(output.positions) == 3
        assert 0.4 < output.positions[0] < 0.6

    def test_trajectory_completes(self, trajectory_task, simple_trajectory, coordinator_state):
        t_start = time.perf_counter()
        trajectory_task.execute(simple_trajectory, trajectory_start_positions(simple_trajectory))

        # First compute sets start time (deferred start)
        state0 = CoordinatorState(
            joints=coordinator_state.joints,
            t_now=t_start,
            dt=0.01,
        )
        trajectory_task.compute(state0)

        # Compute past trajectory duration
        state = CoordinatorState(
            joints=coordinator_state.joints,
            t_now=t_start + 1.5,
            dt=0.01,
        )
        output = trajectory_task.compute(state)

        # On completion, returns final position (not None) to hold at goal
        assert output is not None
        assert output.positions == [1.0, 0.5, 0.25]  # Final trajectory point
        assert not trajectory_task.is_active()
        assert trajectory_task.get_state() == TrajectoryState.COMPLETED

    def test_one_point_stream_target_is_velocity_bounded(self):
        task = JointTrajectoryTask(
            JointTrajectoryTaskConfig(
                joint_names=["arm/joint1"],
                velocity_limits={"arm/joint1": 0.5},
            )
        )
        target = JointTrajectory(
            joint_names=["arm/joint1"],
            points=[TrajectoryPoint(positions=[1.0])],
        )
        state = JointStateSnapshot(joint_positions={"arm/joint1": 0.0})

        assert task.execute(target, {}).status is TrajectoryExecutionStatus.ACCEPTED
        first = task.compute(CoordinatorState(joints=state, t_now=1.0, dt=0.1))
        second = task.compute(CoordinatorState(joints=state, t_now=1.1, dt=0.1))

        assert first is not None
        assert first.positions == [pytest.approx(0.05)]
        assert second is not None
        assert second.positions == [pytest.approx(0.1)]
        assert task.get_status(1.1).progress == 0.0

    @pytest.mark.parametrize("limit", [0.0, -1.0, float("inf"), float("nan")])
    def test_velocity_limits_must_be_finite_and_positive(self, limit):
        with pytest.raises(ValueError, match="finite and positive"):
            JointTrajectoryTask(
                JointTrajectoryTaskConfig(
                    joint_names=["arm/joint1"],
                    velocity_limits={"arm/joint1": limit},
                )
            )

    def test_velocity_limits_must_cover_every_joint(self):
        with pytest.raises(ValueError, match="every configured trajectory joint"):
            JointTrajectoryTask(
                JointTrajectoryTaskConfig(
                    joint_names=["arm/joint1", "arm/joint2"],
                    velocity_limits={"arm/joint1": 1.0},
                )
            )

    def test_replacement_anchors_at_bounded_command_and_preserves_other_joints(self):
        task = JointTrajectoryTask(
            JointTrajectoryTaskConfig(
                joint_names=["arm/joint1", "arm/joint2"],
                velocity_limits={"arm/joint1": 1.0, "arm/joint2": 1.0},
            )
        )
        state = JointStateSnapshot(joint_positions={"arm/joint1": 0.0, "arm/joint2": 0.0})
        first = JointTrajectory(
            joint_names=["arm/joint1"],
            points=[TrajectoryPoint(positions=[1.0])],
        )
        other = JointTrajectory(
            joint_names=["arm/joint2"],
            points=[TrajectoryPoint(positions=[-1.0])],
        )
        replacement = JointTrajectory(
            joint_names=["arm/joint1"],
            points=[TrajectoryPoint(positions=[-1.0])],
        )

        task.execute(first, {})
        task.execute(other, {})
        before = task.compute(CoordinatorState(joints=state, t_now=1.0, dt=0.1))
        assert before is not None
        assert before.positions == [pytest.approx(0.1), pytest.approx(-0.1)]

        assert task.execute(replacement, {}).status is TrajectoryExecutionStatus.ACCEPTED
        after = task.compute(CoordinatorState(joints=state, t_now=1.1, dt=0.1))
        assert after is not None
        assert after.positions == [pytest.approx(0.0), pytest.approx(-0.2)]

    def test_partial_replacement_status_includes_untouched_joint_run(self):
        task = JointTrajectoryTask(
            JointTrajectoryTaskConfig(
                joint_names=["arm/joint1", "arm/joint2"],
                velocity_limits={"arm/joint1": 1000.0, "arm/joint2": 1000.0},
            )
        )
        state = JointStateSnapshot(joint_positions={"arm/joint1": 0.0, "arm/joint2": 0.0})
        long_trajectory = JointTrajectory(
            joint_names=["arm/joint1", "arm/joint2"],
            points=[
                TrajectoryPoint(positions=[0.0, 0.0], velocities=[0.0, 0.0]),
                TrajectoryPoint(
                    positions=[10.0, 10.0],
                    velocities=[0.0, 0.0],
                    time_from_start=10.0,
                ),
            ],
        )
        replacement = JointTrajectory(
            joint_names=["arm/joint1"],
            points=[
                TrajectoryPoint(positions=[2.5], velocities=[0.0]),
                TrajectoryPoint(positions=[3.5], velocities=[0.0], time_from_start=1.0),
            ],
        )

        assert (
            task.execute(long_trajectory, state.joint_positions).status
            is TrajectoryExecutionStatus.ACCEPTED
        )
        task.compute(CoordinatorState(joints=state, t_now=10.0, dt=0.1))
        task.compute(CoordinatorState(joints=state, t_now=12.5, dt=0.1))
        assert task.execute(replacement, {}).status is TrajectoryExecutionStatus.ACCEPTED
        task.compute(CoordinatorState(joints=state, t_now=12.5, dt=0.1))
        task.compute(CoordinatorState(joints=state, t_now=13.5, dt=0.1))

        status = task.get_status(13.5)

        assert status.state is TrajectoryState.EXECUTING
        assert status.progress == pytest.approx(0.35)
        assert status.time_elapsed == pytest.approx(3.5)
        assert status.time_remaining == pytest.approx(6.5)

    def test_cancel_trajectory(self, trajectory_task, simple_trajectory):
        trajectory_task.execute(simple_trajectory, trajectory_start_positions(simple_trajectory))
        assert trajectory_task.is_active()

        result = trajectory_task.cancel()

        assert result.status is TrajectoryCancellationStatus.CANCELLED
        assert not trajectory_task.is_active()
        assert trajectory_task.get_state() == TrajectoryState.ABORTED

    def test_estop_cancels_and_refuses_trajectories_until_cleared(
        self, trajectory_task, simple_trajectory
    ):
        start = trajectory_start_positions(simple_trajectory)
        trajectory_task.execute(simple_trajectory, start)

        trajectory_task.set_estop(True)

        assert not trajectory_task.is_active()
        assert trajectory_task.get_state() == TrajectoryState.ABORTED
        refused = trajectory_task.execute(simple_trajectory, start)
        assert refused.status is TrajectoryExecutionStatus.ESTOPPED
        streamed = JointState(name=["arm/joint1"], position=[0.1])
        assert not trajectory_task.on_joint_command(streamed, t_now=0.0)

        trajectory_task.set_estop(False)

        # Not resumed; a new trajectory is taken again.
        assert not trajectory_task.is_active()
        accepted = trajectory_task.execute(simple_trajectory, start)
        assert accepted.status is TrajectoryExecutionStatus.ACCEPTED

    def test_cancel_when_stopped_reports_already_stopped(self, trajectory_task):
        result = trajectory_task.cancel()

        assert result.status is TrajectoryCancellationStatus.ALREADY_STOPPED

    def test_preemption(self, trajectory_task, simple_trajectory):
        trajectory_task.execute(simple_trajectory, trajectory_start_positions(simple_trajectory))

        trajectory_task.on_preempted("safety_task", frozenset({"arm/joint1"}))
        assert trajectory_task.get_state() == TrajectoryState.ABORTED
        assert not trajectory_task.is_active()

    def test_progress(self, trajectory_task, simple_trajectory, coordinator_state):
        t_start = time.perf_counter()
        trajectory_task.execute(simple_trajectory, trajectory_start_positions(simple_trajectory))

        # First compute sets start time (deferred start)
        state0 = CoordinatorState(
            joints=coordinator_state.joints,
            t_now=t_start,
            dt=0.01,
        )
        trajectory_task.compute(state0)

        assert trajectory_task.get_progress(t_start) == pytest.approx(0.0, abs=0.01)
        assert trajectory_task.get_progress(t_start + 0.5) == pytest.approx(0.5, abs=0.01)
        assert trajectory_task.get_progress(t_start + 1.0) == pytest.approx(1.0, abs=0.01)


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


class TestTickLoop:
    def test_tick_loop_runs_active_tasks_until_stopped(self, wait_until):
        task = MagicMock()
        task.name = "test_task"
        task.is_active.return_value = True
        task.claim.return_value = ResourceClaim(joints=frozenset({"arm/joint1"}), priority=10)
        task.compute.return_value = None
        tick_loop = TickLoop(
            tick_rate=100.0,
            hardware_lock=threading.Lock(),
            tasks={"test_task": task},
            task_lock=threading.Lock(),
            joint_to_hardware={},
        )

        tick_loop.start()
        wait_until(lambda: task.compute.call_count > 0, timeout=5.0, interval=0.01)
        tick_loop.stop()

        assert not tick_loop.is_running
        assert tick_loop._tick_thread is not None and not tick_loop._tick_thread.is_alive()
