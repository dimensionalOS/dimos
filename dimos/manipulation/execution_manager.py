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
from enum import Enum, auto
import math
import threading
from types import MappingProxyType
from typing import Protocol

import attrs

from dimos.control.coordinator_client import ControlCoordinatorClient
from dimos.manipulation.planning.groups.identifiers import parse_planning_group_id
from dimos.manipulation.planning.spec.models import GeneratedPlan, RobotName
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TaskName = str
DEFAULT_PLAN_START_TOLERANCE = 1e-6


class ExecutionOutcome(Enum):
    """Safety-aware outcome of dispatching planned execution."""

    ACCEPTED = auto()
    REJECTED = auto()
    UNCERTAIN = auto()


class CoordinatorCancelOutcome(Enum):
    """Result of asking one coordinator task to cancel."""

    CANCELLED = auto()
    ALREADY_STOPPED = auto()
    UNCERTAIN = auto()


@attrs.frozen(slots=False)
class ExecutionDispatchResult:
    """Structured result of validating and dispatching one generated plan."""

    outcome: ExecutionOutcome
    message: str = ""
    accepted_tasks: tuple[TaskName, ...] = ()
    unresolved_tasks: tuple[TaskName, ...] = ()

    @property
    def accepted(self) -> bool:
        """Return whether every task accepted the plan."""
        return self.outcome is ExecutionOutcome.ACCEPTED


@attrs.frozen(slots=False)
class CancellationResult:
    """Safety result of cancelling tracked coordinator tasks."""

    safe: bool
    had_tracked_tasks: bool = False
    cancelled_tasks: tuple[TaskName, ...] = ()
    already_stopped_tasks: tuple[TaskName, ...] = ()
    unresolved_tasks: tuple[TaskName, ...] = ()


@attrs.frozen(slots=False)
class ExecutionPolicy:
    """Configuration for planned execution validation."""

    plan_start_tolerance: float = attrs.field(default=DEFAULT_PLAN_START_TOLERANCE)

    @plan_start_tolerance.validator
    def _validate_plan_start_tolerance(
        self,
        _attribute: attrs.Attribute[float],
        value: float,
    ) -> None:
        if not math.isfinite(value) or value < 0:
            raise ValueError("plan_start_tolerance must be finite and non-negative")


def _model_joint_names(value: Sequence[str]) -> tuple[str, ...]:
    return tuple(value)


def _immutable_joint_mapping(value: Mapping[str, str]) -> Mapping[str, str]:
    return MappingProxyType(dict(value))


@attrs.frozen(slots=False)
class ExecutionTarget:
    """Immutable coordinator dispatch metadata for one robot."""

    robot_name: RobotName = attrs.field()
    model_joint_names: tuple[str, ...] = attrs.field(converter=_model_joint_names)
    coordinator_task_name: TaskName = attrs.field()
    model_to_coordinator: Mapping[str, str] = attrs.field(
        converter=_immutable_joint_mapping,
        repr=False,
    )

    @robot_name.validator
    def _validate_robot_name(
        self,
        _attribute: attrs.Attribute[RobotName],
        value: RobotName,
    ) -> None:
        if not value:
            raise ValueError("Execution target requires a robot name")

    @coordinator_task_name.validator
    def _validate_coordinator_task_name(
        self,
        _attribute: attrs.Attribute[TaskName],
        value: TaskName,
    ) -> None:
        if not value:
            raise ValueError(f"Execution target '{self.robot_name}' requires a task name")

    @model_joint_names.validator
    def _validate_model_joint_names(
        self,
        _attribute: attrs.Attribute[tuple[str, ...]],
        value: tuple[str, ...],
    ) -> None:
        if not value or any(not name or "/" in name for name in value):
            raise ValueError(f"Execution target '{self.robot_name}' has invalid local model joints")
        if len(set(value)) != len(value):
            raise ValueError(
                f"Execution target '{self.robot_name}' has duplicate local model joints"
            )

    @model_to_coordinator.validator
    def _validate_model_to_coordinator(
        self,
        _attribute: attrs.Attribute[Mapping[str, str]],
        value: Mapping[str, str],
    ) -> None:
        if set(value) != set(self.model_joint_names):
            raise ValueError(f"Execution target '{self.robot_name}' must resolve every model joint")
        resolved_names = list(value.values())
        if any(not name for name in resolved_names) or len(set(resolved_names)) != len(
            resolved_names
        ):
            raise ValueError(
                f"Execution target '{self.robot_name}' has ambiguous coordinator joints"
            )

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

    def execute(self, task_name: TaskName, trajectory: JointTrajectory) -> ExecutionOutcome:
        """Ask a coordinator task to execute a trajectory."""
        ...

    def cancel(self, task_name: TaskName) -> CoordinatorCancelOutcome:
        """Ask a coordinator task to stop executing."""
        ...


class CoordinatorExecutionAdapter:
    """Translate generic coordinator RPC results into typed outcomes."""

    def __init__(self, client: ControlCoordinatorClient) -> None:
        self._client = client

    def execute(self, task_name: TaskName, trajectory: JointTrajectory) -> ExecutionOutcome:
        """Dispatch a trajectory through the existing coordinator task RPC."""
        try:
            result = self._client.execute_task(task_name, trajectory)
        except Exception:
            logger.exception("Coordinator execute RPC failed", task_name=task_name)
            return ExecutionOutcome.UNCERTAIN
        match result:
            case True:
                return ExecutionOutcome.ACCEPTED
            case False:
                return ExecutionOutcome.REJECTED
            case _:
                return ExecutionOutcome.UNCERTAIN

    def cancel(self, task_name: TaskName) -> CoordinatorCancelOutcome:
        """Cancel a trajectory through the existing coordinator task RPC."""
        try:
            result = self._client.cancel_task(task_name)
        except Exception:
            logger.exception("Coordinator cancel RPC failed", task_name=task_name)
            return CoordinatorCancelOutcome.UNCERTAIN
        match result:
            case True:
                return CoordinatorCancelOutcome.CANCELLED
            case False:
                return CoordinatorCancelOutcome.ALREADY_STOPPED
            case _:
                return CoordinatorCancelOutcome.UNCERTAIN


@attrs.frozen(slots=False)
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

        self._operation_condition = threading.Condition()
        self._operation_active = False
        self._cancel_waiters = 0
        self._possibly_active_tasks: set[TaskName] = set()

    def execute(self, plan: GeneratedPlan) -> ExecutionDispatchResult:
        """Validate and dispatch one complete generated plan."""
        prepared = self._prepare_plan(plan)
        if isinstance(prepared, str):
            return self._rejected(prepared)

        with self._operation_condition:
            if self._cancel_waiters:
                return self._rejected("Cancellation is in progress")
            if self._operation_active:
                return self._rejected("Another execution dispatch is in progress")
            self._operation_active = True

        try:
            replacement = self._cancel_tracked_tasks()
            if not replacement.safe:
                return ExecutionDispatchResult(
                    outcome=ExecutionOutcome.UNCERTAIN,
                    message="Cannot replace unresolved coordinator tasks",
                    unresolved_tasks=replacement.unresolved_tasks,
                )

            freshness_error = self._freshness_error(plan)
            if freshness_error is not None:
                return self._rejected(freshness_error)

            accepted: list[TaskName] = []
            for dispatch in prepared:
                with self._operation_condition:
                    if self._cancel_waiters:
                        return self._rejected("Execution was superseded by cancellation")
                    self._possibly_active_tasks.add(dispatch.task_name)

                outcome = self._coordinator.execute(dispatch.task_name, dispatch.trajectory)
                match outcome:
                    case ExecutionOutcome.ACCEPTED:
                        accepted.append(dispatch.task_name)
                    case ExecutionOutcome.REJECTED:
                        with self._operation_condition:
                            self._possibly_active_tasks.discard(dispatch.task_name)
                        return self._rollback_dispatch(
                            accepted,
                            f"Coordinator task '{dispatch.task_name}' rejected trajectory",
                        )
                    case ExecutionOutcome.UNCERTAIN:
                        return self._rollback_dispatch(
                            accepted,
                            f"Coordinator task '{dispatch.task_name}' dispatch is uncertain",
                        )

                with self._operation_condition:
                    if self._cancel_waiters:
                        return self._rejected("Execution was superseded by cancellation")

            return ExecutionDispatchResult(
                outcome=ExecutionOutcome.ACCEPTED,
                message="Every coordinator task accepted the generated plan",
                accepted_tasks=tuple(accepted),
            )
        finally:
            with self._operation_condition:
                self._operation_active = False
                self._operation_condition.notify_all()

    def cancel(self) -> CancellationResult:
        """Cancel every task that might have accepted a generated plan."""
        with self._operation_condition:
            self._cancel_waiters += 1
            try:
                while self._operation_active:
                    self._operation_condition.wait()
                self._operation_active = True
            finally:
                self._cancel_waiters -= 1
        try:
            return self._cancel_tracked_tasks()
        finally:
            with self._operation_condition:
                self._operation_active = False
                self._operation_condition.notify_all()

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
                outcome=ExecutionOutcome.REJECTED,
                message=message,
                accepted_tasks=tuple(accepted_tasks),
            )
        return ExecutionDispatchResult(
            outcome=ExecutionOutcome.UNCERTAIN,
            message=f"{message}; coordinator task safety is uncertain",
            accepted_tasks=tuple(accepted_tasks),
            unresolved_tasks=cancellation.unresolved_tasks,
        )

    def _cancel_tracked_tasks(self) -> CancellationResult:
        with self._operation_condition:
            tasks = tuple(sorted(self._possibly_active_tasks))
        cancelled: list[TaskName] = []
        already_stopped: list[TaskName] = []
        unresolved: list[TaskName] = []
        for task_name in tasks:
            outcome = self._coordinator.cancel(task_name)
            match outcome:
                case CoordinatorCancelOutcome.CANCELLED:
                    cancelled.append(task_name)
                case CoordinatorCancelOutcome.ALREADY_STOPPED:
                    already_stopped.append(task_name)
                case CoordinatorCancelOutcome.UNCERTAIN:
                    unresolved.append(task_name)

        with self._operation_condition:
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
            outcome=ExecutionOutcome.REJECTED,
            message=message,
        )
