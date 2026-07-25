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

"""Transactional dispatch of generated manipulation plans."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum, auto
import math
import threading
from types import MappingProxyType
from typing import Any, Protocol

from dimos.manipulation.planning.groups.identifiers import parse_planning_group_id
from dimos.manipulation.planning.spec.models import GeneratedPlan, RobotName
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TaskName = str


class CoordinatorDispatchOutcome(Enum):
    """Result of asking one coordinator task to execute."""

    ACCEPTED = auto()
    REJECTED = auto()
    UNKNOWN = auto()


class CoordinatorCancelOutcome(Enum):
    """Result of asking one coordinator task to cancel."""

    CANCELLED = auto()
    ALREADY_STOPPED = auto()
    UNKNOWN = auto()


class ExecutionDispatchOutcome(Enum):
    """Aggregate result of a planned execution request."""

    ACCEPTED = auto()
    REJECTED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class ExecutionDispatchResult:
    """Structured result of validating and dispatching one generated plan."""

    outcome: ExecutionDispatchOutcome
    message: str = ""
    accepted_tasks: tuple[TaskName, ...] = ()
    unresolved_tasks: tuple[TaskName, ...] = ()

    @property
    def accepted(self) -> bool:
        """Return whether every task accepted the plan."""
        return self.outcome is ExecutionDispatchOutcome.ACCEPTED


@dataclass(frozen=True)
class CancellationResult:
    """Safety result of cancelling tracked coordinator tasks."""

    safe: bool
    had_tracked_tasks: bool = False
    cancelled_tasks: tuple[TaskName, ...] = ()
    already_stopped_tasks: tuple[TaskName, ...] = ()
    unresolved_tasks: tuple[TaskName, ...] = ()


@dataclass(frozen=True)
class ExecutionPolicy:
    """Configuration for planned execution validation."""

    plan_start_tolerance: float = 1e-6

    def __post_init__(self) -> None:
        if not math.isfinite(self.plan_start_tolerance) or self.plan_start_tolerance < 0:
            raise ValueError("plan_start_tolerance must be finite and non-negative")


@dataclass(frozen=True)
class ExecutionTarget:
    """Immutable coordinator dispatch metadata for one robot."""

    robot_name: RobotName
    model_joint_names: tuple[str, ...]
    coordinator_task_name: TaskName
    model_to_coordinator: Mapping[str, str] = field(repr=False)

    def __post_init__(self) -> None:
        model_names = tuple(self.model_joint_names)
        if not self.robot_name:
            raise ValueError("Execution target requires a robot name")
        if not self.coordinator_task_name:
            raise ValueError(f"Execution target '{self.robot_name}' requires a task name")
        if not model_names or any(not name or "/" in name for name in model_names):
            raise ValueError(f"Execution target '{self.robot_name}' has invalid local model joints")
        if len(set(model_names)) != len(model_names):
            raise ValueError(
                f"Execution target '{self.robot_name}' has duplicate local model joints"
            )

        mapping = dict(self.model_to_coordinator)
        if set(mapping) != set(model_names):
            raise ValueError(f"Execution target '{self.robot_name}' must resolve every model joint")
        resolved_names = list(mapping.values())
        if any(not name for name in resolved_names) or len(set(resolved_names)) != len(
            resolved_names
        ):
            raise ValueError(
                f"Execution target '{self.robot_name}' has ambiguous coordinator joints"
            )

        object.__setattr__(self, "model_joint_names", model_names)
        object.__setattr__(self, "model_to_coordinator", MappingProxyType(mapping))

    @classmethod
    def from_coordinator_mapping(
        cls,
        *,
        robot_name: RobotName,
        model_joint_names: Sequence[str],
        coordinator_task_name: TaskName,
        coordinator_to_model: Mapping[str, str],
    ) -> ExecutionTarget:
        """Validate and invert a coordinator-to-model joint mapping."""
        local_names = tuple(model_joint_names)
        known = set(local_names)
        reverse: dict[str, str] = {}
        for coordinator_name, model_name in coordinator_to_model.items():
            if model_name not in known:
                raise ValueError(
                    f"Coordinator joint '{coordinator_name}' maps to unknown model joint "
                    f"'{model_name}' for '{robot_name}'"
                )
            if model_name in reverse:
                raise ValueError(
                    f"Multiple coordinator joints map to model joint '{model_name}' "
                    f"for '{robot_name}'"
                )
            reverse[model_name] = coordinator_name

        return cls(
            robot_name=robot_name,
            model_joint_names=local_names,
            coordinator_task_name=coordinator_task_name,
            model_to_coordinator={
                model_name: reverse.get(model_name, model_name) for model_name in local_names
            },
        )


class CoordinatorExecutionPort(Protocol):
    """Coordinator commands required by planned execution."""

    def execute(
        self, task_name: TaskName, trajectory: JointTrajectory
    ) -> CoordinatorDispatchOutcome:
        """Ask a coordinator task to execute a trajectory."""
        ...

    def cancel(self, task_name: TaskName) -> CoordinatorCancelOutcome:
        """Ask a coordinator task to stop executing."""
        ...


class CoordinatorExecutionAdapter:
    """Translate generic coordinator RPC results into typed outcomes."""

    def __init__(self, rpc_client: Any) -> None:
        self._rpc_client = rpc_client

    def execute(
        self, task_name: TaskName, trajectory: JointTrajectory
    ) -> CoordinatorDispatchOutcome:
        """Dispatch a trajectory through the existing coordinator task RPC."""
        try:
            result = self._rpc_client.task_invoke(task_name, "execute", {"trajectory": trajectory})
        except Exception:
            logger.exception("Coordinator execute RPC failed", task_name=task_name)
            return CoordinatorDispatchOutcome.UNKNOWN
        if result is True:
            return CoordinatorDispatchOutcome.ACCEPTED
        if result is False:
            return CoordinatorDispatchOutcome.REJECTED
        return CoordinatorDispatchOutcome.UNKNOWN

    def cancel(self, task_name: TaskName) -> CoordinatorCancelOutcome:
        """Cancel a trajectory through the existing coordinator task RPC."""
        try:
            result = self._rpc_client.task_invoke(task_name, "cancel", {})
        except Exception:
            logger.exception("Coordinator cancel RPC failed", task_name=task_name)
            return CoordinatorCancelOutcome.UNKNOWN
        if result is True:
            return CoordinatorCancelOutcome.CANCELLED
        if result is False:
            return CoordinatorCancelOutcome.ALREADY_STOPPED
        return CoordinatorCancelOutcome.UNKNOWN


@dataclass(frozen=True)
class _PreparedDispatch:
    robot_name: RobotName
    task_name: TaskName
    trajectory: JointTrajectory


class PlanExecutionManager:
    """Validate, dispatch, replace, and cancel complete generated plans."""

    def __init__(
        self,
        *,
        targets: Iterable[ExecutionTarget],
        current_joint_state: Callable[[], JointState],
        coordinator: CoordinatorExecutionPort,
        policy: ExecutionPolicy | None = None,
    ) -> None:
        target_items = tuple(targets)
        target_names = [target.robot_name for target in target_items]
        task_names = [target.coordinator_task_name for target in target_items]
        if len(set(target_names)) != len(target_names):
            raise ValueError("Execution targets must have unique robot names")
        if len(set(task_names)) != len(task_names):
            raise ValueError("Execution targets must have unique coordinator task names")

        self._targets = {target.robot_name: target for target in target_items}
        self._current_joint_state = current_joint_state
        self._coordinator = coordinator
        self._policy = policy or ExecutionPolicy()

        self._state_lock = threading.Lock()
        self._dispatch_lock = threading.Lock()
        self._cancel_lock = threading.Lock()
        self._dispatch_in_progress = False
        self._cancellation_in_progress = False
        self._generation = 0
        self._possibly_active_tasks: set[TaskName] = set()

    def execute(self, plan: GeneratedPlan) -> ExecutionDispatchResult:
        """Validate and dispatch one complete generated plan."""
        prepared = self._prepare_plan(plan)
        if isinstance(prepared, str):
            return self._rejected(prepared)

        with self._state_lock:
            if self._cancellation_in_progress:
                return self._rejected("Cancellation is in progress")
            if self._dispatch_in_progress:
                return self._rejected("Another execution dispatch is in progress")
            self._dispatch_in_progress = True
            self._generation += 1
            token = self._generation

        try:
            with self._dispatch_lock:
                with self._state_lock:
                    if self._cancellation_in_progress or self._generation != token:
                        return self._rejected("Execution was superseded by cancellation")

                replacement = self._cancel_tracked_tasks()
                if not replacement.safe:
                    return ExecutionDispatchResult(
                        outcome=ExecutionDispatchOutcome.FAULT,
                        message="Cannot replace unresolved coordinator tasks",
                        unresolved_tasks=replacement.unresolved_tasks,
                    )

                freshness_error = self._freshness_error(plan)
                if freshness_error is not None:
                    return self._rejected(freshness_error)

                accepted: list[TaskName] = []
                for dispatch in prepared:
                    with self._state_lock:
                        if self._generation != token or self._cancellation_in_progress:
                            return self._rejected("Execution was superseded by cancellation")
                        self._possibly_active_tasks.add(dispatch.task_name)

                    outcome = self._coordinator.execute(dispatch.task_name, dispatch.trajectory)
                    if outcome is CoordinatorDispatchOutcome.ACCEPTED:
                        accepted.append(dispatch.task_name)
                    elif outcome is CoordinatorDispatchOutcome.REJECTED:
                        with self._state_lock:
                            self._possibly_active_tasks.discard(dispatch.task_name)
                        return self._rollback_dispatch(
                            accepted,
                            f"Coordinator task '{dispatch.task_name}' rejected trajectory",
                        )
                    else:
                        return self._rollback_dispatch(
                            accepted,
                            f"Coordinator task '{dispatch.task_name}' dispatch is unknown",
                        )

                    with self._state_lock:
                        if self._generation != token or self._cancellation_in_progress:
                            return self._rejected("Execution was superseded by cancellation")

                return ExecutionDispatchResult(
                    outcome=ExecutionDispatchOutcome.ACCEPTED,
                    message="Every coordinator task accepted the generated plan",
                    accepted_tasks=tuple(accepted),
                )
        finally:
            with self._state_lock:
                self._dispatch_in_progress = False

    def cancel(self) -> CancellationResult:
        """Cancel every task that might have accepted a generated plan."""
        with self._cancel_lock:
            with self._state_lock:
                self._cancellation_in_progress = True
                self._generation += 1
            try:
                with self._dispatch_lock:
                    return self._cancel_tracked_tasks()
            finally:
                with self._state_lock:
                    self._cancellation_in_progress = False

    def _prepare_plan(self, plan: GeneratedPlan) -> tuple[_PreparedDispatch, ...] | str:
        if not isinstance(plan, GeneratedPlan):
            return "Execution requires a generated plan"
        if not plan.is_success():
            return "Generated plan status is not successful"
        if not plan.path:
            return "Generated plan has no path"

        trajectory = plan.trajectory
        names = list(trajectory.joint_names)
        if not names or len(set(names)) != len(names):
            return "Generated trajectory has missing or duplicate joint names"
        if not trajectory.points:
            return "Generated trajectory has no points"

        width = len(names)
        previous_time: float | None = None
        for index, point in enumerate(trajectory.points):
            if len(point.positions) != width or len(point.velocities) != width:
                return f"Generated trajectory point {index} has invalid width"
            values = [point.time_from_start, *point.positions, *point.velocities]
            if not all(math.isfinite(float(value)) for value in values):
                return f"Generated trajectory point {index} has non-finite values"
            if index == 0 and point.time_from_start != 0.0:
                return "Generated trajectory must start at time zero"
            if previous_time is not None and point.time_from_start <= previous_time:
                return "Generated trajectory times must increase"
            previous_time = point.time_from_start
        if trajectory.duration <= 0:
            return "Generated trajectory duration must be positive"

        for index, state in enumerate(plan.path):
            if list(state.name) != names or len(state.position) != width:
                return f"Generated path state {index} does not match trajectory joints"
            if not all(math.isfinite(float(value)) for value in state.position):
                return f"Generated path state {index} has non-finite positions"

        affected: list[RobotName] = []
        try:
            for group_id in plan.group_ids:
                robot_name, _ = parse_planning_group_id(group_id)
                if robot_name not in affected:
                    affected.append(robot_name)
        except ValueError as exc:
            return str(exc)
        if not affected:
            return "Generated plan has no planning groups"

        indices_by_robot: dict[RobotName, list[tuple[int, str]]] = {}
        for index, global_name in enumerate(names):
            parts = global_name.split("/")
            if len(parts) != 2 or not parts[0] or not parts[1]:
                return f"Generated trajectory joint '{global_name}' is not globally named"
            robot_name, local_name = parts
            target = self._targets.get(robot_name)
            if target is None:
                return f"Generated plan references unknown execution robot '{robot_name}'"
            if local_name not in target.model_joint_names:
                return f"Generated trajectory joint '{global_name}' is not configured"
            indices_by_robot.setdefault(robot_name, []).append((index, local_name))

        if set(indices_by_robot) != set(affected):
            return "Generated plan groups and trajectory robots do not match"

        prepared: list[_PreparedDispatch] = []
        for robot_name in affected:
            target = self._targets.get(robot_name)
            indexed_names = indices_by_robot.get(robot_name)
            if target is None or indexed_names is None:
                return f"Generated plan has no trajectory for '{robot_name}'"
            indices = [index for index, _ in indexed_names]
            coordinator_names = [
                target.model_to_coordinator[local_name] for _, local_name in indexed_names
            ]
            points = [
                TrajectoryPoint(
                    time_from_start=point.time_from_start,
                    positions=[point.positions[index] for index in indices],
                    velocities=[point.velocities[index] for index in indices],
                )
                for point in trajectory.points
            ]
            prepared.append(
                _PreparedDispatch(
                    robot_name=robot_name,
                    task_name=target.coordinator_task_name,
                    trajectory=JointTrajectory(
                        joint_names=coordinator_names,
                        points=points,
                        timestamp=trajectory.timestamp,
                    ),
                )
            )
        return tuple(prepared)

    def _freshness_error(self, plan: GeneratedPlan) -> str | None:
        try:
            current = self._current_joint_state()
        except Exception as exc:
            return f"Current planned joints are unavailable: {exc}"
        if not isinstance(current, JointState) or len(current.name) != len(current.position):
            return "Current planned joints are malformed"

        values: dict[str, float] = {}
        ordered_subset: list[tuple[str, float]] = []
        planned_names = list(plan.trajectory.joint_names)
        planned = set(planned_names)
        for raw_name, raw_position in zip(current.name, current.position, strict=True):
            name = str(raw_name)
            if name in values:
                return "Current planned joints contain duplicate names"
            try:
                value = float(raw_position)
            except (TypeError, ValueError):
                return f"Current planned joint '{name}' is malformed"
            if not math.isfinite(value):
                return f"Current planned joint '{name}' is malformed"
            values[name] = value
            if name in planned:
                ordered_subset.append((name, value))

        if len(ordered_subset) != len(planned_names):
            return "Current planned joints are missing or stale"

        first = plan.trajectory.points[0]
        for (actual_name, actual), expected_name, expected in zip(
            ordered_subset, planned_names, first.positions, strict=True
        ):
            if actual_name != expected_name:
                return "Current planned joints are not in generated plan order"
            if abs(actual - float(expected)) > self._policy.plan_start_tolerance:
                return (
                    f"Current planned joint '{expected_name}' no longer matches "
                    "the generated plan start"
                )
        return None

    def _rollback_dispatch(
        self, accepted_tasks: Sequence[TaskName], message: str
    ) -> ExecutionDispatchResult:
        cancellation = self._cancel_tracked_tasks()
        if cancellation.safe:
            return ExecutionDispatchResult(
                outcome=ExecutionDispatchOutcome.REJECTED,
                message=message,
                accepted_tasks=tuple(accepted_tasks),
            )
        return ExecutionDispatchResult(
            outcome=ExecutionDispatchOutcome.FAULT,
            message=f"{message}; coordinator task safety is unknown",
            accepted_tasks=tuple(accepted_tasks),
            unresolved_tasks=cancellation.unresolved_tasks,
        )

    def _cancel_tracked_tasks(self) -> CancellationResult:
        with self._state_lock:
            tasks = tuple(sorted(self._possibly_active_tasks))
        cancelled: list[TaskName] = []
        already_stopped: list[TaskName] = []
        unresolved: list[TaskName] = []
        for task_name in tasks:
            outcome = self._coordinator.cancel(task_name)
            if outcome is CoordinatorCancelOutcome.CANCELLED:
                cancelled.append(task_name)
            elif outcome is CoordinatorCancelOutcome.ALREADY_STOPPED:
                already_stopped.append(task_name)
            else:
                unresolved.append(task_name)

        with self._state_lock:
            self._possibly_active_tasks = set(unresolved)
        return CancellationResult(
            safe=not unresolved,
            had_tracked_tasks=bool(tasks),
            cancelled_tasks=tuple(cancelled),
            already_stopped_tasks=tuple(already_stopped),
            unresolved_tasks=tuple(unresolved),
        )

    @staticmethod
    def _rejected(message: str) -> ExecutionDispatchResult:
        return ExecutionDispatchResult(
            outcome=ExecutionDispatchOutcome.REJECTED,
            message=message,
        )
