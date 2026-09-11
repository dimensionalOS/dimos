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
