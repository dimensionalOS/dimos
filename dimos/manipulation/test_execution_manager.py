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


BASE = ("base/x", "base/y", "base/yaw")


class _WholeBody:
    """A coordinator running a joint trajectory task and a base trajectory task."""

    def __init__(self) -> None:
        self.states = {
            JOINT_TRAJECTORY_TASK_NAME: TrajectoryState.EXECUTING,
            "base_traj": TrajectoryState.EXECUTING,
        }
        self.errors: dict[str, str] = {}
        self.base_execute = TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)
        self.base_trajectory: JointTrajectory | None = None
        self.coordinator = _coordinator()
        self.coordinator.task_invoke.side_effect = self._task_invoke
        self.coordinator.cancel_trajectory.side_effect = self._cancel_joints

    def _task_invoke(self, task, method, args):
        if method == "execute":
            self.base_trajectory = args["trajectory"]
            return self.base_execute
        if method == "cancel":
            self.states[task] = TrajectoryState.ABORTED
            return True
        return TrajectoryStatus(state=self.states[task], error=self.errors.get(task, ""))

    def _cancel_joints(self):
        self.states[JOINT_TRAJECTORY_TASK_NAME] = TrajectoryState.ABORTED
        return TrajectoryCancellationResult(TrajectoryCancellationStatus.CANCELLED)

    def manager(self) -> PlanExecutionManager:
        return PlanExecutionManager(
            joint_names=("left/j1", *BASE),
            coordinator=self.coordinator,
            default_timeout=1.0,
            poll_interval=0.01,
            base_task="base_traj",
            base_joint_names=BASE,
        )


def test_whole_body_plan_splits_into_joint_and_base_columns() -> None:
    robot = _WholeBody()
    manager = robot.manager()
    plan = _plan(("base/yaw", "left/j1", "base/x", "base/y"))

    assert manager.execute(plan, blocking=False).status is ExecutionStatus.ACCEPTED

    joints = robot.coordinator.execute_trajectory.call_args.args[0]
    assert joints.joint_names == ["left/j1"]
    assert robot.base_trajectory.joint_names == list(BASE)
    assert [p.time_from_start for p in robot.base_trajectory.points] == [0.0, 1.0]
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


def test_a_refused_base_trajectory_cancels_the_joints() -> None:
    robot = _WholeBody()
    robot.base_execute = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.INVALID_TRAJECTORY, "too fast"
    )

    result = robot.manager().execute(_plan(("left/j1", *BASE)), blocking=False)

    assert result.status is ExecutionStatus.REJECTED
    assert "too fast" in result.message
    robot.coordinator.cancel_trajectory.assert_called_once_with()
