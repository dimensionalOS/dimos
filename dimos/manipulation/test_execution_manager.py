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

"""Behavior tests for planned manipulation execution."""

from collections.abc import Callable
from dataclasses import dataclass, field
import threading
from unittest.mock import MagicMock

import pytest

from dimos.manipulation.execution_manager import (
    CancellationResult,
    CoordinatorCancelOutcome,
    CoordinatorExecutionAdapter,
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


@dataclass
class FakeCoordinator:
    execute_outcomes: list[ExecutionOutcome] = field(default_factory=list)
    cancel_outcomes: dict[str, CoordinatorCancelOutcome] = field(default_factory=dict)
    execute_calls: list[tuple[str, JointTrajectory]] = field(default_factory=list)
    cancel_calls: list[str] = field(default_factory=list)
    on_execute: Callable[[str], None] | None = None
    on_cancel: Callable[[str], None] | None = None

    def execute(self, task_name: str, trajectory: JointTrajectory) -> ExecutionOutcome:
        self.execute_calls.append((task_name, trajectory))
        if self.on_execute is not None:
            self.on_execute(task_name)
        if self.execute_outcomes:
            return self.execute_outcomes.pop(0)
        return ExecutionOutcome.ACCEPTED

    def cancel(self, task_name: str) -> CoordinatorCancelOutcome:
        self.cancel_calls.append(task_name)
        if self.on_cancel is not None:
            self.on_cancel(task_name)
        return self.cancel_outcomes.get(task_name, CoordinatorCancelOutcome.CANCELLED)


def _target(
    robot_name: str,
    task_name: str,
    joints: tuple[str, ...],
    mapping: dict[str, str] | None = None,
) -> ExecutionTarget:
    return ExecutionTarget.from_coordinator_mapping(
        robot_name=robot_name,
        model_joint_names=joints,
        coordinator_task_name=task_name,
        coordinator_to_model=mapping or {},
    )


def _plan(
    *,
    group_ids: tuple[str, ...] = ("arm/manipulator",),
    names: list[str] | None = None,
    start: list[float] | None = None,
    goal: list[float] | None = None,
    status: PlanningStatus = PlanningStatus.SUCCESS,
) -> GeneratedPlan:
    joint_names = names or ["arm/j0"]
    start_positions = start or [0.0] * len(joint_names)
    goal_positions = goal or [1.0] * len(joint_names)
    return GeneratedPlan(
        group_ids=group_ids,
        status=status,
        path=[
            JointState(name=joint_names, position=start_positions),
            JointState(name=joint_names, position=goal_positions),
        ],
        trajectory=JointTrajectory(
            joint_names=joint_names,
            timestamp=123.0,
            points=[
                TrajectoryPoint(
                    time_from_start=0.0,
                    positions=start_positions,
                    velocities=[0.0] * len(joint_names),
                ),
                TrajectoryPoint(
                    time_from_start=2.0,
                    positions=goal_positions,
                    velocities=[0.5] * len(joint_names),
                ),
            ],
        ),
    )


def _manager(
    coordinator: FakeCoordinator,
    *,
    targets: tuple[ExecutionTarget, ...] | None = None,
    current: JointState | None = None,
) -> PlanExecutionManager:
    configured_targets = targets or (_target("arm", "traj_arm", ("j0",)),)
    current_state = current or JointState(name=["arm/j0"], position=[0.0])
    return PlanExecutionManager(
        targets=configured_targets,
        current_joint_state=lambda: current_state,
        coordinator=coordinator,
    )


@pytest.mark.parametrize(
    ("rpc_result", "expected"),
    [
        (True, ExecutionOutcome.ACCEPTED),
        (False, ExecutionOutcome.REJECTED),
        (None, ExecutionOutcome.UNCERTAIN),
    ],
)
def test_coordinator_adapter_translates_execute_result(
    rpc_result: bool | None, expected: ExecutionOutcome
) -> None:
    client = MagicMock()
    client.execute_task.return_value = rpc_result
    trajectory = JointTrajectory()

    result = CoordinatorExecutionAdapter(client).execute("traj_arm", trajectory)

    assert result is expected
    client.execute_task.assert_called_once_with("traj_arm", trajectory)


def test_coordinator_adapter_translates_execute_exception_to_uncertain() -> None:
    client = MagicMock()
    client.execute_task.side_effect = RuntimeError("rpc failed")

    result = CoordinatorExecutionAdapter(client).execute("traj_arm", JointTrajectory())

    assert result is ExecutionOutcome.UNCERTAIN


@pytest.mark.parametrize(
    ("rpc_result", "expected"),
    [
        (True, CoordinatorCancelOutcome.CANCELLED),
        (False, CoordinatorCancelOutcome.ALREADY_STOPPED),
        (None, CoordinatorCancelOutcome.UNCERTAIN),
    ],
)
def test_coordinator_adapter_translates_cancel_result(
    rpc_result: bool | None, expected: CoordinatorCancelOutcome
) -> None:
    client = MagicMock()
    client.cancel_task.return_value = rpc_result

    result = CoordinatorExecutionAdapter(client).cancel("traj_arm")

    assert result is expected
    client.cancel_task.assert_called_once_with("traj_arm")


def test_execution_target_inverts_mapping_and_keeps_identity_names() -> None:
    target = _target(
        "arm",
        "traj_arm",
        ("j0", "j1"),
        {"hardware/j0": "j0"},
    )

    assert dict(target.model_to_coordinator) == {
        "j0": "hardware/j0",
        "j1": "j1",
    }


def test_execution_target_rejects_ambiguous_reverse_mapping() -> None:
    with pytest.raises(ValueError, match="Multiple coordinator joints"):
        _target(
            "arm",
            "traj_arm",
            ("j0",),
            {"hardware/j0": "j0", "legacy/j0": "j0"},
        )


def test_execute_dispatches_complete_multi_robot_plan_with_shared_timing() -> None:
    coordinator = FakeCoordinator()
    targets = (
        _target("left", "traj_left", ("j0", "j1"), {"left_hw/j1": "j1"}),
        _target("right", "traj_right", ("k0",)),
    )
    current = JointState(
        name=["left/j1", "right/k0"],
        position=[0.0, 1.0],
    )
    manager = _manager(coordinator, targets=targets, current=current)
    plan = _plan(
        group_ids=("left/wrist", "right/elbow"),
        names=["left/j1", "right/k0"],
        start=[0.0, 1.0],
        goal=[0.5, 1.5],
    )

    result = manager.execute(plan)

    assert result.outcome is ExecutionOutcome.ACCEPTED
    assert result.accepted_tasks == ("traj_left", "traj_right")
    left = coordinator.execute_calls[0][1]
    right = coordinator.execute_calls[1][1]
    assert left.joint_names == ["left_hw/j1"]
    assert right.joint_names == ["k0"]
    assert [point.time_from_start for point in left.points] == [0.0, 2.0]
    assert [point.time_from_start for point in right.points] == [0.0, 2.0]
    assert left.timestamp == right.timestamp == 123.0
    assert [point.positions for point in left.points] == [[0.0], [0.5]]
    assert [point.positions for point in right.points] == [[1.0], [1.5]]


def test_execute_rejects_unsuccessful_plan_before_dispatch() -> None:
    coordinator = FakeCoordinator()
    manager = _manager(coordinator)

    result = manager.execute(_plan(status=PlanningStatus.NO_SOLUTION))

    assert result.outcome is ExecutionOutcome.REJECTED
    assert coordinator.execute_calls == []


@pytest.mark.parametrize(
    "current",
    [
        JointState(name=[], position=[]),
        JointState(name=["arm/j0", "arm/j0"], position=[0.0, 0.0]),
        JointState(name=["arm/j0"], position=[0.1]),
    ],
)
def test_execute_rejects_missing_duplicate_or_mismatched_current_state(
    current: JointState,
) -> None:
    coordinator = FakeCoordinator()
    manager = _manager(coordinator, current=current)

    result = manager.execute(_plan())

    assert result.outcome is ExecutionOutcome.REJECTED
    assert coordinator.execute_calls == []


def test_execute_rejects_plan_when_group_and_trajectory_robots_differ() -> None:
    coordinator = FakeCoordinator()
    manager = _manager(coordinator)

    result = manager.execute(_plan(group_ids=("other/manipulator",)))

    assert result.outcome is ExecutionOutcome.REJECTED
    assert coordinator.execute_calls == []


def test_execute_cancels_previous_plan_before_dispatching_replacement() -> None:
    coordinator = FakeCoordinator()
    manager = _manager(coordinator)

    first = manager.execute(_plan())
    second = manager.execute(_plan())

    assert first.accepted
    assert second.accepted
    assert coordinator.cancel_calls == ["traj_arm"]
    assert [task for task, _ in coordinator.execute_calls] == ["traj_arm", "traj_arm"]


def test_execute_blocks_replacement_when_previous_task_is_unresolved() -> None:
    coordinator = FakeCoordinator(cancel_outcomes={"traj_arm": CoordinatorCancelOutcome.UNCERTAIN})
    manager = _manager(coordinator)
    assert manager.execute(_plan()).accepted

    result = manager.execute(_plan())

    assert result.outcome is ExecutionOutcome.UNCERTAIN
    assert result.unresolved_tasks == ("traj_arm",)
    assert len(coordinator.execute_calls) == 1


def test_execute_rolls_back_accepted_task_when_later_task_rejects() -> None:
    coordinator = FakeCoordinator(
        execute_outcomes=[
            ExecutionOutcome.ACCEPTED,
            ExecutionOutcome.REJECTED,
        ]
    )
    targets = (
        _target("left", "traj_left", ("j0",)),
        _target("right", "traj_right", ("k0",)),
    )
    manager = _manager(
        coordinator,
        targets=targets,
        current=JointState(
            name=["left/j0", "right/k0"],
            position=[0.0, 0.0],
        ),
    )
    plan = _plan(
        group_ids=("left/arm", "right/arm"),
        names=["left/j0", "right/k0"],
    )

    result = manager.execute(plan)

    assert result.outcome is ExecutionOutcome.REJECTED
    assert coordinator.cancel_calls == ["traj_left"]


def test_execute_is_uncertain_when_dispatch_cannot_be_cancelled() -> None:
    coordinator = FakeCoordinator(
        execute_outcomes=[ExecutionOutcome.UNCERTAIN],
        cancel_outcomes={"traj_arm": CoordinatorCancelOutcome.UNCERTAIN},
    )
    manager = _manager(coordinator)

    result = manager.execute(_plan())

    assert result.outcome is ExecutionOutcome.UNCERTAIN
    assert result.unresolved_tasks == ("traj_arm",)


def test_cancel_is_idempotently_safe_without_tracked_tasks() -> None:
    result = _manager(FakeCoordinator()).cancel()

    assert result == CancellationResult(safe=True)


def test_cancel_waits_for_in_flight_dispatch_and_stops_accepted_task() -> None:
    dispatch_started = threading.Event()
    release_dispatch = threading.Event()
    coordinator = FakeCoordinator()

    def block_dispatch(_task_name: str) -> None:
        dispatch_started.set()
        assert release_dispatch.wait(timeout=1.0)

    coordinator.on_execute = block_dispatch
    manager = _manager(coordinator)
    execute_results: list[ExecutionDispatchResult] = []
    cancel_results: list[CancellationResult] = []
    executing = threading.Thread(target=lambda: execute_results.append(manager.execute(_plan())))
    executing.start()
    assert dispatch_started.wait(timeout=1.0)

    cancelling = threading.Thread(target=lambda: cancel_results.append(manager.cancel()))
    cancelling.start()
    release_dispatch.set()
    executing.join(timeout=1.0)
    cancelling.join(timeout=1.0)

    assert not executing.is_alive()
    assert not cancelling.is_alive()
    assert execute_results[0].outcome is ExecutionOutcome.REJECTED
    assert cancel_results[0].safe
    assert coordinator.cancel_calls == ["traj_arm"]


def test_cancel_stops_multi_robot_dispatch_before_the_next_robot() -> None:
    dispatch_started = threading.Event()
    release_dispatch = threading.Event()
    coordinator = FakeCoordinator()

    def block_first_dispatch(task_name: str) -> None:
        if task_name == "traj_left":
            dispatch_started.set()
            assert release_dispatch.wait(timeout=1.0)

    coordinator.on_execute = block_first_dispatch
    targets = (
        _target("left", "traj_left", ("j0",)),
        _target("right", "traj_right", ("k0",)),
    )
    manager = _manager(
        coordinator,
        targets=targets,
        current=JointState(name=["left/j0", "right/k0"], position=[0.0, 0.0]),
    )
    plan = _plan(
        group_ids=("left/arm", "right/arm"),
        names=["left/j0", "right/k0"],
    )
    execute_results: list[ExecutionDispatchResult] = []
    cancel_results: list[CancellationResult] = []
    executing = threading.Thread(target=lambda: execute_results.append(manager.execute(plan)))
    executing.start()
    assert dispatch_started.wait(timeout=1.0)

    cancelling = threading.Thread(target=lambda: cancel_results.append(manager.cancel()))
    cancelling.start()
    release_dispatch.set()
    executing.join(timeout=1.0)
    cancelling.join(timeout=1.0)

    assert not executing.is_alive()
    assert not cancelling.is_alive()
    assert execute_results[0].outcome is ExecutionOutcome.REJECTED
    assert cancel_results[0].safe
    assert [task for task, _ in coordinator.execute_calls] == ["traj_left"]
    assert coordinator.cancel_calls == ["traj_left"]


def test_concurrent_cancellations_serialize_and_remain_safe() -> None:
    cancel_started = threading.Event()
    release_cancel = threading.Event()
    coordinator = FakeCoordinator()
    manager = _manager(coordinator)
    assert manager.execute(_plan()).accepted

    def block_cancel(_task_name: str) -> None:
        cancel_started.set()
        assert release_cancel.wait(timeout=1.0)

    coordinator.on_cancel = block_cancel
    results: list[CancellationResult] = []
    first = threading.Thread(target=lambda: results.append(manager.cancel()))
    second = threading.Thread(target=lambda: results.append(manager.cancel()))
    first.start()
    assert cancel_started.wait(timeout=1.0)
    second.start()
    release_cancel.set()
    first.join(timeout=1.0)
    second.join(timeout=1.0)

    assert not first.is_alive()
    assert not second.is_alive()
    assert [result.safe for result in results] == [True, True]
    assert coordinator.cancel_calls == ["traj_arm"]


def test_concurrent_execute_fails_fast_without_queueing() -> None:
    dispatch_started = threading.Event()
    release_dispatch = threading.Event()
    coordinator = FakeCoordinator()

    def block_dispatch(_task_name: str) -> None:
        dispatch_started.set()
        assert release_dispatch.wait(timeout=1.0)

    coordinator.on_execute = block_dispatch
    manager = _manager(coordinator)
    first_results: list[ExecutionDispatchResult] = []
    first = threading.Thread(target=lambda: first_results.append(manager.execute(_plan())))
    first.start()
    assert dispatch_started.wait(timeout=1.0)

    second = manager.execute(_plan())
    release_dispatch.set()
    first.join(timeout=1.0)

    assert second.outcome is ExecutionOutcome.REJECTED
    assert first_results[0].accepted
    assert len(coordinator.execute_calls) == 1
