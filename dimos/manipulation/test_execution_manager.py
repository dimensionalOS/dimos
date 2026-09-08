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

"""Tests for direct canonical trajectory execution."""

from concurrent.futures import ThreadPoolExecutor
import threading
from unittest.mock import MagicMock

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.execution_manager import PlanExecutionManager
from dimos.manipulation.manipulation_spec import ExecutionStatus
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus


def _plan(
    names: tuple[str, ...] = ("left/j1", "right/j1"),
    status: PlanningStatus = PlanningStatus.SUCCESS,
) -> GeneratedPlan:
    points = [
        TrajectoryPoint(positions=[0.0] * len(names), time_from_start=0.0),
        TrajectoryPoint(positions=[1.0] * len(names), time_from_start=1.0),
    ]
    return GeneratedPlan(
        group_ids=("both_arms",),
        trajectory=JointTrajectory(joint_names=list(names), points=points),
        path=[JointState(name=list(names), position=point.positions) for point in points],
        status=status,
    )


def _coordinator() -> MagicMock:
    coordinator = MagicMock(spec=ControlCoordinator)
    coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    coordinator.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.ALREADY_STOPPED
    )
    coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.IDLE)
    return coordinator


def _manager(coordinator: MagicMock | None = None) -> PlanExecutionManager:
    return PlanExecutionManager(
        joint_names=("left/j1", "left/j2", "right/j1"),
        coordinator=coordinator or _coordinator(),
        default_timeout=1.0,
    )


def test_manager_rejects_empty_or_duplicate_model_joint_names() -> None:
    with pytest.raises(ValueError, match="non-empty and unique"):
        PlanExecutionManager(joint_names=(), coordinator=_coordinator(), default_timeout=1.0)
    with pytest.raises(ValueError, match="non-empty and unique"):
        PlanExecutionManager(
            joint_names=("j1", "j1"), coordinator=_coordinator(), default_timeout=1.0
        )


def test_execute_forwards_same_canonical_trajectory_object_unchanged() -> None:
    coordinator = _coordinator()
    plan = _plan()
    result = _manager(coordinator).execute(plan, blocking=False)
    assert result.status is ExecutionStatus.ACCEPTED
    assert coordinator.execute_trajectory.call_args.args[0] is plan.trajectory


@pytest.mark.parametrize(
    ("plan", "message"),
    [
        (_plan(status=PlanningStatus.NO_SOLUTION), "status is not successful"),
        (_plan(("unknown",)), "unknown joints"),
    ],
)
def test_execute_rejects_invalid_plan_before_rpc(plan: GeneratedPlan, message: str) -> None:
    coordinator = _coordinator()
    result = _manager(coordinator).execute(plan, blocking=False)
    assert result.status is ExecutionStatus.REJECTED
    assert message in result.message
    coordinator.execute_trajectory.assert_not_called()


def test_execute_preserves_coordinator_rejection() -> None:
    coordinator = _coordinator()
    rejection = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.INVALID_TRAJECTORY, "specific rejection"
    )
    coordinator.execute_trajectory.return_value = rejection
    result = _manager(coordinator).execute(_plan(), blocking=False)
    assert result.status is ExecutionStatus.REJECTED
    assert result.coordinator_result is rejection


def test_execute_rpc_failure_is_uncertain() -> None:
    coordinator = _coordinator()
    coordinator.execute_trajectory.side_effect = TimeoutError("timed out")
    result = _manager(coordinator).execute(_plan(), blocking=False)
    assert result.status is ExecutionStatus.UNCERTAIN
    assert "timed out" in result.message


def test_cancel_forwards_to_coordinator() -> None:
    coordinator = _coordinator()
    result = _manager(coordinator).cancel()
    assert result.status is ExecutionStatus.NO_EXECUTION
    coordinator.cancel_trajectory.assert_called_once_with()


@pytest.mark.parametrize("operation", ["wait", "cancel"])
def test_status_query_is_serialized_with_other_operations(operation):
    coordinator = _coordinator()
    manager = _manager(coordinator)
    assert manager.execute(_plan(), blocking=False).status is ExecutionStatus.ACCEPTED
    entered = threading.Event()
    release = threading.Event()
    attempted = threading.Event()
    terminal = TrajectoryState.COMPLETED if operation == "wait" else TrajectoryState.ABORTED

    def status(*_args):
        if not entered.is_set():
            entered.set()
            assert release.wait(2.0)
            return TrajectoryStatus(state=TrajectoryState.EXECUTING)
        return TrajectoryStatus(state=terminal)

    coordinator.task_invoke.side_effect = status

    def second_operation():
        attempted.set()
        return manager.wait(0.0) if operation == "wait" else manager.cancel()

    with ThreadPoolExecutor(max_workers=2) as executor:
        first = executor.submit(manager.wait, 0.0)
        try:
            assert entered.wait(2.0)
            second = executor.submit(second_operation)
            assert attempted.wait(2.0)
            with pytest.raises(TimeoutError):
                second.result(0.05)
        finally:
            release.set()
        assert first.result(2.0).status is ExecutionStatus.TIMED_OUT
        assert second.result(2.0).status.name == terminal.name

    assert manager.status.name == terminal.name


def test_blocking_wait_keeps_cancelled_result_after_new_dispatch(mocker):
    coordinator = _coordinator()
    manager = _manager(coordinator)
    assert manager.execute(_plan(), blocking=False).status is ExecutionStatus.ACCEPTED
    coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.EXECUTING)
    between_polls = threading.Event()
    resume = threading.Event()
    # Gate the wait between polls, without replacing threading globally or sleeping.
    timer = mocker.Mock()

    def pause(_timeout):
        between_polls.set()
        assert resume.wait(2.0)

    timer.wait.side_effect = pause
    threading_proxy = mocker.patch(
        "dimos.manipulation.execution_manager.threading", wraps=threading
    )
    threading_proxy.Event.return_value = timer

    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(manager.wait, 1.0)
        try:
            assert between_polls.wait(2.0)
            coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.ABORTED)
            assert manager.cancel().status is ExecutionStatus.ABORTED
            assert manager.execute(_plan(), blocking=False).status is ExecutionStatus.ACCEPTED
            coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.EXECUTING)
        finally:
            resume.set()
        assert future.result(2.0).status is ExecutionStatus.ABORTED

    assert manager.status is ExecutionStatus.ACCEPTED


def test_zero_timeout_preserves_execution_for_later_completion():
    coordinator = _coordinator()
    manager = _manager(coordinator)
    assert manager.execute(_plan(), blocking=False).status is ExecutionStatus.ACCEPTED
    coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.EXECUTING)

    assert manager.wait(0.0).status is ExecutionStatus.TIMED_OUT
    assert manager.status is ExecutionStatus.EXECUTING

    coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.COMPLETED)
    assert manager.wait(0.0).status is ExecutionStatus.COMPLETED
