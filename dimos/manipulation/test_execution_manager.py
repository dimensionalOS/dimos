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

from unittest.mock import MagicMock

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
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
    """A coordinator whose tasks accept everything, answering per invoked method."""
    coordinator = MagicMock(spec=ControlCoordinator)
    coordinator.get_joint_positions.return_value = {}

    def invoke(task: str, method: str, args: dict | None = None):
        if method == "execute":
            return TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)
        if method == "cancel":
            return TrajectoryCancellationResult(TrajectoryCancellationStatus.ALREADY_STOPPED)
        return TrajectoryStatus(state=TrajectoryState.IDLE)

    coordinator.task_invoke.side_effect = invoke
    return coordinator


def _dispatched(coordinator: MagicMock, task: str) -> JointTrajectory:
    """The trajectory a task was asked to execute."""
    for call in coordinator.task_invoke.call_args_list:
        if call.args[0] == task and call.args[1] == "execute":
            return call.args[2]["trajectory"]
    raise AssertionError(f"{task} was never asked to execute")


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
    assert _dispatched(coordinator, JOINT_TRAJECTORY_TASK_NAME) is plan.trajectory


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
    coordinator.task_invoke.assert_not_called()


def test_execute_preserves_coordinator_rejection() -> None:
    coordinator = _coordinator()
    rejection = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.INVALID_TRAJECTORY, "specific rejection"
    )
    coordinator.task_invoke.side_effect = lambda *_a, **_k: rejection
    result = _manager(coordinator).execute(_plan(), blocking=False)
    assert result.status is ExecutionStatus.REJECTED
    assert result.coordinator_result is rejection


def test_execute_rpc_failure_is_uncertain() -> None:
    coordinator = _coordinator()
    coordinator.task_invoke.side_effect = TimeoutError("timed out")
    result = _manager(coordinator).execute(_plan(), blocking=False)
    assert result.status is ExecutionStatus.UNCERTAIN
    assert "timed out" in result.message


def test_cancel_without_a_run_reports_no_execution() -> None:
    result = _manager(_coordinator()).cancel()
    assert result.status is ExecutionStatus.NO_EXECUTION


BASE = ("base/x", "base/y", "base/yaw")


class _WholeBody:
    """A coordinator running a joint trajectory task and a base trajectory task."""

    def __init__(self) -> None:
        self.states = {
            JOINT_TRAJECTORY_TASK_NAME: TrajectoryState.EXECUTING,
            "base_traj": TrajectoryState.EXECUTING,
        }
        self.errors: dict[str, str] = {}
        self.execute_results: dict[str, TrajectoryExecutionResult] = {}
        self.dispatched: dict[str, JointTrajectory] = {}
        self.cancellations: dict[str, TrajectoryCancellationResult] = {}
        self.coordinator = _coordinator()
        self.coordinator.task_invoke.side_effect = self._task_invoke

    def _task_invoke(self, task, method, args=None):
        if method == "execute":
            self.dispatched[task] = args["trajectory"]
            return self.execute_results.get(
                task, TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)
            )
        if method == "cancel":
            self.states[task] = TrajectoryState.ABORTED
            return self.cancellations.get(
                task, TrajectoryCancellationResult(TrajectoryCancellationStatus.CANCELLED)
            )
        return TrajectoryStatus(state=self.states[task], error=self.errors.get(task, ""))

    def manager(self) -> PlanExecutionManager:
        return PlanExecutionManager(
            joint_names=("left/j1", *BASE),
            coordinator=self.coordinator,
            default_timeout=1.0,
            poll_interval=0.01,
            bindings={JOINT_TRAJECTORY_TASK_NAME: ["left/j1"], "base_traj": list(BASE)},
        )


def test_whole_body_plan_splits_into_joint_and_base_columns() -> None:
    robot = _WholeBody()
    manager = robot.manager()
    plan = _plan(("base/yaw", "left/j1", "base/x", "base/y"))

    assert manager.execute(plan, blocking=False).status is ExecutionStatus.ACCEPTED

    assert robot.dispatched[JOINT_TRAJECTORY_TASK_NAME].joint_names == ["left/j1"]
    assert robot.dispatched["base_traj"].joint_names == list(BASE)
    assert [p.time_from_start for p in robot.dispatched["base_traj"].points] == [0.0, 1.0]
    robot.states = dict.fromkeys(robot.states, TrajectoryState.COMPLETED)
    assert manager.wait().status is ExecutionStatus.COMPLETED


@pytest.mark.parametrize("failing", [JOINT_TRAJECTORY_TASK_NAME, "base_traj"])
def test_a_failing_leg_cancels_the_other_without_a_caller_polling(failing: str, wait_until) -> None:
    robot = _WholeBody()
    manager = robot.manager()
    manager.execute(_plan(("left/j1", *BASE)), blocking=False)

    robot.states[failing] = TrajectoryState.ABORTED
    robot.errors[failing] = "preempted by teleop"
    wait_until(lambda: manager.status is ExecutionStatus.ABORTED, timeout=2.0)

    assert manager.status is ExecutionStatus.ABORTED
    assert set(robot.states.values()) == {TrajectoryState.ABORTED}
    assert "preempted by teleop" in manager.wait().message


def test_a_refused_part_cancels_the_parts_already_dispatched() -> None:
    robot = _WholeBody()
    robot.execute_results["base_traj"] = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.INVALID_TRAJECTORY, "too fast"
    )

    result = robot.manager().execute(_plan(("left/j1", *BASE)), blocking=False)

    assert result.status is ExecutionStatus.REJECTED
    assert "too fast" in result.message
    assert robot.states[JOINT_TRAJECTORY_TASK_NAME] is TrajectoryState.ABORTED


def test_a_refused_part_is_uncertain_when_the_others_cannot_be_confirmed_stopped() -> None:
    robot = _WholeBody()
    robot.execute_results["base_traj"] = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.INVALID_TRAJECTORY, "too fast"
    )
    robot.cancellations[JOINT_TRAJECTORY_TASK_NAME] = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.UNCERTAIN
    )

    result = robot.manager().execute(_plan(("left/j1", *BASE)), blocking=False)

    assert result.status is ExecutionStatus.UNCERTAIN
    assert "could not cancel" in result.message


def test_a_poll_from_a_finished_run_does_not_disturb_the_next_one() -> None:
    """A task poll that outlives its run must not cancel or overwrite the next."""
    robot = _WholeBody()
    manager = robot.manager()
    manager.execute(_plan(("left/j1", *BASE)), blocking=False)
    manager.cancel()

    robot.states = dict.fromkeys(robot.states, TrajectoryState.EXECUTING)
    assert manager.execute(_plan(("left/j1", *BASE)), blocking=False).status is (
        ExecutionStatus.ACCEPTED
    )

    manager._poll(run_id=1)
    manager.close()

    assert manager.status is ExecutionStatus.ACCEPTED
    assert set(robot.states.values()) == {TrajectoryState.EXECUTING}
