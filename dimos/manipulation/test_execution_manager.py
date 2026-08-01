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

from __future__ import annotations

from threading import Event, Thread
from unittest.mock import MagicMock

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.execution_manager import (
    ExecutionDispatchResult,
    ExecutionOutcome,
    ExecutionTarget,
    PlanExecutionManager,
)
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


def _target(
    robot_name: str = "arm",
    *,
    model_joint_names: tuple[str, ...] = ("j1", "j2"),
    coordinator_to_model: dict[str, str] | None = None,
) -> ExecutionTarget:
    return ExecutionTarget.from_coordinator_mapping(
        robot_name=robot_name,
        model_joint_names=model_joint_names,
        coordinator_to_model=coordinator_to_model or {},
    )


def _plan(
    joint_names: tuple[str, ...] = ("arm/j1", "arm/j2"),
    *,
    status: PlanningStatus = PlanningStatus.SUCCESS,
) -> GeneratedPlan:
    width = len(joint_names)
    trajectory = JointTrajectory(
        joint_names=list(joint_names),
        points=[
            TrajectoryPoint(
                positions=[0.0] * width,
                velocities=[0.0] * width,
                time_from_start=0.0,
            ),
            TrajectoryPoint(
                positions=[1.0] * width,
                velocities=[0.0] * width,
                time_from_start=1.0,
            ),
        ],
    )
    return GeneratedPlan(
        group_ids=("arm/manipulator",),
        trajectory=trajectory,
        path=[
            JointState(name=list(joint_names), position=[0.0] * width),
            JointState(name=list(joint_names), position=[1.0] * width),
        ],
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
    return coordinator


def _manager(
    *targets: ExecutionTarget, coordinator: MagicMock | None = None
) -> PlanExecutionManager:
    return PlanExecutionManager(
        targets=targets or (_target(),),
        coordinator=coordinator if coordinator is not None else _coordinator(),
    )


def test_execution_target_inverts_coordinator_mapping() -> None:
    target = _target(coordinator_to_model={"hardware/a": "j1", "hardware/b": "j2"})

    assert dict(target.model_to_coordinator) == {
        "j1": "hardware/a",
        "j2": "hardware/b",
    }


@pytest.mark.parametrize(
    ("model_joint_names", "mapping", "message"),
    [
        ((), {}, "invalid local model joints"),
        (("j1", "j1"), {}, "duplicate local model joints"),
        (("arm/j1",), {}, "invalid local model joints"),
        (("j1",), {"hardware/a": "missing"}, "unknown model joint"),
        (
            ("j1", "j2"),
            {"hardware/a": "j1", "hardware/b": "j1"},
            "Multiple coordinator joints",
        ),
    ],
)
def test_execution_target_rejects_invalid_configuration(
    model_joint_names: tuple[str, ...],
    mapping: dict[str, str],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        _target(model_joint_names=model_joint_names, coordinator_to_model=mapping)


def test_manager_requires_unique_robot_targets() -> None:
    with pytest.raises(ValueError, match="unique robot names"):
        _manager(_target(), _target())


def test_execute_maps_all_robots_into_one_trajectory() -> None:
    coordinator = _coordinator()
    manager = _manager(
        _target(
            "left",
            model_joint_names=("j1",),
            coordinator_to_model={"left_hw/j1": "j1"},
        ),
        _target(
            "right",
            model_joint_names=("j1",),
            coordinator_to_model={"right_hw/j1": "j1"},
        ),
        coordinator=coordinator,
    )
    plan = _plan(("left/j1", "right/j1"))

    result = manager.execute(plan)

    assert result.outcome is ExecutionOutcome.ACCEPTED
    coordinator.execute_trajectory.assert_called_once()
    trajectory = coordinator.execute_trajectory.call_args.args[0]
    assert trajectory.joint_names == ["left_hw/j1", "right_hw/j1"]
    assert trajectory.points == plan.trajectory.points
    assert trajectory.timestamp == plan.trajectory.timestamp


def test_execute_preserves_single_robot_subset() -> None:
    coordinator = _coordinator()
    manager = _manager(
        _target("left", model_joint_names=("j1", "j2")),
        _target("right", model_joint_names=("j1",)),
        coordinator=coordinator,
    )

    result = manager.execute(_plan(("left/j2",)))

    assert result.accepted
    trajectory = coordinator.execute_trajectory.call_args.args[0]
    assert trajectory.joint_names == ["j2"]
    assert trajectory.points[0].positions == [0.0]


@pytest.mark.parametrize(
    ("plan", "message"),
    [
        (_plan(status=PlanningStatus.NO_SOLUTION), "status is not successful"),
        (_plan(("not-global",)), "not globally named"),
        (_plan(("unknown/j1",)), "unknown execution robot"),
        (_plan(("arm/missing",)), "is not configured"),
    ],
)
def test_execute_rejects_unmappable_plan_before_rpc(
    plan: GeneratedPlan,
    message: str,
) -> None:
    coordinator = _coordinator()
    manager = _manager(coordinator=coordinator)

    result = manager.execute(plan)

    assert result.outcome is ExecutionOutcome.REJECTED
    assert message in result.message
    coordinator.execute_trajectory.assert_not_called()


def test_execute_rejects_cross_robot_mapping_collision() -> None:
    coordinator = _coordinator()
    manager = _manager(
        _target(
            "left",
            model_joint_names=("j1",),
            coordinator_to_model={"shared/j1": "j1"},
        ),
        _target(
            "right",
            model_joint_names=("j1",),
            coordinator_to_model={"shared/j1": "j1"},
        ),
        coordinator=coordinator,
    )

    result = manager.execute(_plan(("left/j1", "right/j1")))

    assert result.outcome is ExecutionOutcome.REJECTED
    assert "duplicate coordinator joints" in result.message
    coordinator.execute_trajectory.assert_not_called()


@pytest.mark.parametrize(
    "status",
    [
        TrajectoryExecutionStatus.NO_TRAJECTORY_TASK,
        TrajectoryExecutionStatus.INVALID_TRAJECTORY,
        TrajectoryExecutionStatus.START_STATE_UNAVAILABLE,
        TrajectoryExecutionStatus.START_STATE_MISMATCH,
    ],
)
def test_execute_preserves_coordinator_rejection(status: TrajectoryExecutionStatus) -> None:
    coordinator = _coordinator()
    coordinator_result = TrajectoryExecutionResult(status, "specific rejection")
    coordinator.execute_trajectory.return_value = coordinator_result
    manager = _manager(coordinator=coordinator)

    result = manager.execute(_plan())

    assert result.outcome is ExecutionOutcome.REJECTED
    assert result.message == "specific rejection"
    assert result.coordinator_result is coordinator_result


def test_execute_rpc_failure_is_uncertain() -> None:
    coordinator = _coordinator()
    coordinator.execute_trajectory.side_effect = TimeoutError("timed out")

    result = _manager(coordinator=coordinator).execute(_plan())

    assert result.outcome is ExecutionOutcome.UNCERTAIN
    assert "timed out" in result.message


@pytest.mark.parametrize(
    ("status", "safe", "cancelled"),
    [
        (
            TrajectoryCancellationStatus.CANCELLED,
            True,
            True,
        ),
        (
            TrajectoryCancellationStatus.ALREADY_STOPPED,
            True,
            False,
        ),
        (
            TrajectoryCancellationStatus.NO_TRAJECTORY_TASK,
            True,
            False,
        ),
    ],
)
def test_cancel_preserves_coordinator_semantics(
    status: TrajectoryCancellationStatus,
    safe: bool,
    cancelled: bool,
) -> None:
    coordinator = _coordinator()
    coordinator_result = TrajectoryCancellationResult(status, "cancel result")
    coordinator.cancel_trajectory.return_value = coordinator_result

    result = _manager(coordinator=coordinator).cancel()

    assert result is coordinator_result
    assert result.safe is safe
    assert result.cancelled is cancelled


def test_cancel_rpc_failure_is_uncertain() -> None:
    coordinator = _coordinator()
    coordinator.cancel_trajectory.side_effect = TimeoutError("timed out")

    result = _manager(coordinator=coordinator).cancel()

    assert result.status is TrajectoryCancellationStatus.UNCERTAIN
    assert not result.safe
    assert not result.cancelled
    assert "timed out" in result.message


def test_cancel_waits_for_in_flight_execute_then_cancels() -> None:
    coordinator = _coordinator()
    execute_started = Event()
    release_execute = Event()

    def execute_trajectory(_trajectory: JointTrajectory) -> TrajectoryExecutionResult:
        execute_started.set()
        if not release_execute.wait(timeout=1.0):
            raise TimeoutError("test did not release execute RPC")
        return TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)

    coordinator.execute_trajectory.side_effect = execute_trajectory
    manager = _manager(coordinator=coordinator)
    execute_results: list[ExecutionDispatchResult] = []
    cancel_results: list[TrajectoryCancellationResult] = []

    execute_thread = Thread(target=lambda: execute_results.append(manager.execute(_plan())))
    cancel_thread = Thread(target=lambda: cancel_results.append(manager.cancel()))
    execute_thread.start()
    execute_was_started = execute_started.wait(timeout=1.0)
    cancel_was_started = False
    cancel_called_before_release = False
    try:
        if execute_was_started:
            cancel_thread.start()
            cancel_was_started = True
            cancel_called_before_release = coordinator.cancel_trajectory.called
    finally:
        release_execute.set()
        execute_thread.join(timeout=1.0)
        if cancel_was_started:
            cancel_thread.join(timeout=1.0)

    assert execute_was_started
    assert not execute_thread.is_alive()
    assert cancel_was_started
    assert not cancel_thread.is_alive()
    assert not cancel_called_before_release
    assert len(execute_results) == 1
    assert execute_results[0].outcome is ExecutionOutcome.ACCEPTED
    assert len(cancel_results) == 1
    assert cancel_results[0].status is TrajectoryCancellationStatus.ALREADY_STOPPED
    coordinator.cancel_trajectory.assert_called_once_with()
