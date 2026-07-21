# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0 (the "License").
"""Owner-thread manipulation execution runtime.

Coordinator calls are effects.  They never mutate runtime state; they only
post correlated events back to the owner.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field, replace
from enum import Enum
import queue
import threading
import time
from typing import Any, Generic, Protocol, TypeVar, cast
import uuid

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState


class LifecycleState(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    READY = "READY"
    DISPATCHING = "DISPATCHING"
    RUNNING = "RUNNING"
    CANCELLING = "CANCELLING"
    FAULT = "FAULT"


class TaskActivity(str, Enum):
    NOT_STARTED = "not_started"
    EXECUTE_UNRESOLVED = "execute_unresolved"
    ACTIVE = "active"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    INACTIVE = "inactive"
    REMOTE_FAULT = "remote_fault"
    UNKNOWN = "unknown"


class ActionMethod(str, Enum):
    EXECUTE = "execute"
    CANCEL = "cancel"
    STATUS = "get_state"
    RESET = "reset"
    GRIPPER_SET = "set_gripper_position"
    GRIPPER_GET = "get_gripper_position"


class Outcome(str, Enum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    INACTIVE = "inactive"
    REJECTED = "rejected"
    FAILED = "failed"
    UNKNOWN = "unknown"


class ShutdownState(str, Enum):
    OPEN = "open"
    CLOSING = "closing"
    CLOSED = "closed"


class _GatewayGateState(str, Enum):
    OPEN = "open"
    CANCEL_ONLY = "cancel_only"
    SEALED = "sealed"
    STOPPED = "stopped"


@dataclass(frozen=True)
class ExecutionTopology:
    """Canonical group -> robot -> coordinator-task routing."""

    routes: tuple[tuple[str, str, str], ...]
    robot_configs: tuple[RobotModelConfig, ...] = ()
    inverse_joint_mappings: tuple[tuple[str, tuple[tuple[str, str], ...]], ...] = ()

    @classmethod
    def from_robot_configs(
        cls,
        robot_configs: Iterable[RobotModelConfig],
        group_routes: Mapping[str, tuple[str, str]] | Iterable[tuple[str, str, str]] | None = None,
        group_resolver: Callable[[str], tuple[str, str]] | None = None,
    ) -> ExecutionTopology:
        """Build canonical routes while retaining the supplied config references."""
        configs = tuple(robot_configs)
        by_robot: dict[str, RobotModelConfig] = {}
        for config in configs:
            if config.name in by_robot:
                raise ValueError(f"duplicate robot config: {config.name}")
            by_robot[config.name] = config
        explicit: dict[str, tuple[str, str]] = {}
        if isinstance(group_routes, Mapping):
            for group, route in group_routes.items():
                if not isinstance(group, str) or not isinstance(route, tuple) or len(route) != 2:
                    raise ValueError("invalid group route mapping")
                explicit[group] = (str(route[0]), str(route[1]))
        elif group_routes is not None:
            for group, robot, task in group_routes:
                if group in explicit:
                    raise ValueError(f"duplicate group route: {group}")
                explicit[group] = (robot, task)
        routes: list[tuple[str, str, str]] = []
        inverse: list[tuple[str, tuple[tuple[str, str], ...]]] = []
        for config in configs:
            local_names = set(config.joint_names)
            local_to_coordinator: dict[str, str] = {}
            for coordinator_name, local_name in config.joint_name_mapping.items():
                if local_name not in local_names:
                    raise ValueError(
                        f"mapped local joint is not configured: {config.name}/{local_name}"
                    )
                previous = local_to_coordinator.get(local_name)
                if previous is not None and previous != coordinator_name:
                    raise ValueError(
                        f"duplicate local joint mapping for {config.name}/{local_name}"
                    )
                local_to_coordinator[local_name] = coordinator_name
            coordinator_names: set[str] = set()
            for local_name in config.joint_names:
                coordinator_name = local_to_coordinator.get(local_name, local_name)
                if coordinator_name in coordinator_names:
                    raise ValueError(
                        f"duplicate coordinator joint mapping for {config.name}/{coordinator_name}"
                    )
                coordinator_names.add(coordinator_name)
            inverse.append(
                (
                    config.name,
                    tuple(
                        (local, local_to_coordinator.get(local, local))
                        for local in config.joint_names
                    ),
                )
            )
            for definition in config.planning_groups:
                if not set(definition.joint_names).issubset(local_names):
                    raise ValueError(
                        f"planning group joints are not configured: {config.name}/{definition.name}"
                    )
                group_id = f"{config.name}/{definition.name}"
                if group_resolver is not None:
                    route_robot, route_task = group_resolver(group_id)
                else:
                    if not config.coordinator_task_name and group_id not in explicit:
                        continue
                    route_robot, route_task = explicit.get(
                        group_id, (config.name, config.coordinator_task_name or "")
                    )
                if route_robot != config.name or route_task != config.coordinator_task_name:
                    raise ValueError(f"route/config mismatch for {group_id}")
                routes.append((group_id, route_robot, route_task))
        if explicit and set(explicit) != {group for group, _, _ in routes}:
            raise ValueError("route set does not match configured planning groups")
        return cls(tuple(routes), configs, tuple(inverse))

    from_configs = from_robot_configs
    from_robot_model_configs = from_robot_configs

    def config_for_robot(self, robot_name: str) -> RobotModelConfig:
        for config in self.robot_configs:
            if config.name == robot_name:
                return config
        raise KeyError(f"unknown robot: {robot_name}")

    def coordinator_joint_name(self, robot_name: str, local_name: str) -> str:
        for name, mappings in self.inverse_joint_mappings:
            if name == robot_name:
                for local, coordinator in mappings:
                    if local == local_name:
                        return coordinator
                break
        return local_name

    def route_for_group(self, group_id: str) -> tuple[str, str]:
        routes = {group: (robot, task) for group, robot, task in self.routes}
        try:
            return routes[group_id]
        except KeyError as exc:
            raise ValueError(f"missing route for planning group {group_id}") from exc

    def affected_robots(self, group_ids: Iterable[str]) -> tuple[str, ...]:
        robots: list[str] = []
        for group_id in group_ids:
            robot_name, _task_name = self.route_for_group(group_id)
            if robot_name in robots:
                raise ValueError(f"multiple selected groups resolve to robot {robot_name}")
            robots.append(robot_name)
        return tuple(robots)

    @property
    def groups(self) -> tuple[str, ...]:
        return tuple(group for group, _, _ in self.routes)


@dataclass(frozen=True)
class TaskEntry:
    planning_group: str
    task_name: str
    request: Any
    robot_name: str | None = None


@dataclass(frozen=True)
class ExecutionPlan:
    generated_plan: Any
    entries: tuple[TaskEntry, ...]


@dataclass(frozen=True)
class PreparedPlan:
    generated_plan: GeneratedPlan
    entries: tuple[TaskEntry, ...]
    topology: ExecutionTopology | None = None
    _canonical: bool = field(default=False, repr=False, compare=False)


def prepare_generated_plan(
    generated_plan: GeneratedPlan, topology: ExecutionTopology
) -> PreparedPlan:
    """Purely bind a generated multi-robot plan to canonical coordinator tasks."""
    if not isinstance(generated_plan, GeneratedPlan):
        raise TypeError("expected GeneratedPlan")
    group_ids = tuple(generated_plan.group_ids)
    if not group_ids or len(set(group_ids)) != len(group_ids):
        raise ValueError("generated plan must contain unique planning groups")
    route_by_group = {group: (robot, task) for group, robot, task in topology.routes}
    if len(route_by_group) != len(topology.routes):
        raise ValueError("topology contains duplicate group routes")
    selected_routes: list[tuple[str, str, str, RobotModelConfig, tuple[str, ...]]] = []
    affected: set[str] = set()
    for group_id in group_ids:
        route = route_by_group.get(group_id)
        if route is None:
            robot_name = group_id.split("/", 1)[0]
            try:
                if not topology.config_for_robot(robot_name).coordinator_task_name:
                    raise ValueError(f"no coordinator task for selected robot {robot_name}")
            except KeyError:
                pass
            raise ValueError(f"missing route for planning group {group_id}")
        robot_name, task_name = route
        if not task_name:
            raise ValueError(f"missing coordinator task for {group_id}")
        if robot_name in affected:
            raise ValueError(f"multiple selected groups resolve to robot {robot_name}")
        config = topology.config_for_robot(robot_name)
        prefix, _, definition_name = group_id.partition("/")
        if prefix != robot_name or not definition_name:
            raise ValueError(f"invalid canonical planning group {group_id}")
        definitions = [
            definition
            for definition in config.planning_groups
            if definition.name == definition_name
        ]
        if len(definitions) != 1:
            raise ValueError(f"missing config group for {group_id}")
        expected_local = tuple(definitions[0].joint_names)
        if len(set(expected_local)) != len(expected_local):
            raise ValueError(f"duplicate intended joints for {group_id}")
        affected.add(robot_name)
        selected_routes.append((group_id, robot_name, task_name, config, expected_local))
    expected_by_robot = {robot: set(expected) for _, robot, _, _, expected in selected_routes}

    trajectory = generated_plan.trajectory
    names = tuple(trajectory.joint_names)
    if not names:
        raise ValueError("generated trajectory has no joint names")
    if len(set(names)) != len(names):
        raise ValueError("duplicate global trajectory joint assignment")
    columns: dict[str, dict[str, int]] = {robot: {} for robot in affected}
    known_robots = {config.name for config in topology.robot_configs}
    for index, global_name in enumerate(names):
        if global_name.count("/") != 1:
            raise ValueError(f"malformed global joint name: {global_name}")
        robot_name, local_name = global_name.split("/", 1)
        if not robot_name or not local_name or robot_name not in known_robots:
            raise ValueError(f"unknown global joint assignment: {global_name}")
        if robot_name in columns:
            if local_name not in expected_by_robot[robot_name]:
                raise ValueError(f"additional joint column for selected robot: {global_name}")
            if local_name in columns[robot_name]:
                raise ValueError(f"duplicate robot joint assignment: {global_name}")
            columns[robot_name][local_name] = index
    points = tuple(trajectory.points)
    for point in points:
        if len(point.positions) != len(names):
            raise ValueError("trajectory point dimension does not match joint names")
        if point.velocities and len(point.velocities) != len(names):
            raise ValueError("trajectory velocity dimension does not match joint names")

    entries: list[TaskEntry] = []
    for group_id, robot_name, task_name, _config, expected_local in selected_routes:
        robot_columns = columns[robot_name]
        missing = [local for local in expected_local if local not in robot_columns]
        if missing:
            raise ValueError(f"missing robot joints for {robot_name}: {missing}")
        intended = set(expected_local)
        local_order = tuple(local for local in robot_columns if local in intended)
        coordinator_names = tuple(
            topology.coordinator_joint_name(robot_name, local) for local in local_order
        )
        if len(set(coordinator_names)) != len(coordinator_names):
            raise ValueError(f"duplicate coordinator joint names for {robot_name}")
        local_trajectory = JointTrajectory(
            joint_names=list(coordinator_names),
            points=[
                TrajectoryPoint(
                    time_from_start=point.time_from_start,
                    positions=[point.positions[robot_columns[local]] for local in local_order],
                    velocities=(
                        [point.velocities[robot_columns[local]] for local in local_order]
                        if point.velocities
                        else None
                    ),
                )
                for point in points
            ],
        )
        entries.append(TaskEntry(group_id, task_name, {"trajectory": local_trajectory}, robot_name))
    return PreparedPlan(generated_plan, tuple(entries), topology, True)


prepare_plan = prepare_generated_plan
prepare_execution_plan = prepare_generated_plan
materialize_prepared_plan = prepare_generated_plan

PlanInput = ExecutionPlan | PreparedPlan


@dataclass(frozen=True)
class OperationHandle:
    plan_id: str
    operation_id: str
    attempt_id: str


ExecutionHandle = OperationHandle


@dataclass(frozen=True)
class ActionRecord:
    action_id: str
    method: ActionMethod
    started: float
    deadline: float
    deadline_reported: bool = False
    reset_id: str | None = None


@dataclass(frozen=True)
class TaskRecord:
    task_id: str
    task_name: str
    entry: TaskEntry
    activity: TaskActivity
    action: ActionRecord | None = None
    cancel_required: bool = False
    reset_required: bool = False


@dataclass(frozen=True)
class Operation:
    handle: OperationHandle
    plan: PlanInput
    tasks: tuple[TaskRecord, ...]
    next_index: int = 0
    cancel_requested: bool = False
    cleanup_required: bool = False
    uncertain: bool = False
    rejected: bool = False
    failed: bool = False
    diagnostic: str = ""


@dataclass(frozen=True)
class ExecutionResult:
    handle: OperationHandle
    outcome: Outcome
    diagnostic: str = ""


@dataclass(frozen=True)
class ResetHandle:
    reset_id: str


@dataclass(frozen=True)
class ResetResult:
    handle: ResetHandle
    success: bool
    diagnostic: str = ""


@dataclass(frozen=True)
class ShutdownResult:
    success: bool
    diagnostic: str = ""


@dataclass(frozen=True)
class RuntimeContext:
    state: LifecycleState = LifecycleState.IDLE
    ready_plan: PlanInput | None = None
    ready_plan_id: str | None = None
    planning_token: str | None = None
    active: Operation | None = None
    fault: str | None = None
    diagnostic: str | None = None
    shutdown: ShutdownState = ShutdownState.OPEN
    reset_handle: ResetHandle | None = None
    reset_result: ResetResult | None = None
    retired_plan_ids: frozenset[str] = frozenset()
    dispatch_results: tuple[tuple[OperationHandle, ExecutionResult], ...] = ()
    terminal_results: tuple[tuple[OperationHandle, ExecutionResult], ...] = ()
    reset_results: tuple[tuple[ResetHandle, ResetResult], ...] = ()
    reset_proven_inactive: frozenset[str] = frozenset()
    reset_completed_tasks: frozenset[str] = frozenset()
    reset_deadline: float | None = None
    shutdown_result: ShutdownResult | None = None
    shutdown_deadline: float | None = None
    revision: int = 0


@dataclass(frozen=True)
class RuntimeSnapshot:
    state: LifecycleState
    ready_plan: PlanInput | None
    ready_plan_id: str | None
    planning_token: str | None
    operation: Operation | None
    fault: str | None
    diagnostic: str | None
    shutdown: ShutdownState
    shutdown_result: ShutdownResult | None
    revision: int


T = TypeVar("T")


@dataclass(frozen=True)
class CommandResult(Generic[T]):
    accepted: bool
    value: T | None = None
    diagnostic: str = ""
    snapshot: RuntimeSnapshot | None = None


class CoordinatorGateway(Protocol):
    def execute(self, task_name: str, request: Any) -> Outcome: ...
    def cancel(self, task_name: str) -> Outcome: ...
    def status(self, task_name: str) -> Outcome: ...
    def reset(self, task_name: str) -> Outcome: ...
    def set_gripper_position(self, hardware_id: str, position: float) -> Outcome: ...
    def get_gripper_position(self, hardware_id: str) -> float | None: ...
    def stop(self) -> None: ...


class ControlCoordinatorGateway:
    """Adapter for ControlCoordinator.task_invoke(task, method, kwargs)."""

    def __init__(self, client: Any) -> None:
        self._client = client

    def _invoke(self, task: str, method: str, kwargs: dict[str, Any]) -> Any:
        return self._client.task_invoke(task, method, kwargs)

    def execute(self, task_name: str, request: Any) -> Outcome:
        try:
            value = self._invoke(
                task_name,
                "execute",
                request if isinstance(request, dict) else {"trajectory": request},
            )
            return (
                Outcome.ACCEPTED
                if value is True
                else Outcome.REJECTED
                if value is False
                else Outcome.UNKNOWN
            )
        except Exception:
            return Outcome.UNKNOWN

    def cancel(self, task_name: str) -> Outcome:
        try:
            value = self._invoke(task_name, "cancel", {})
            return (
                Outcome.CANCELLED
                if value is True
                else Outcome.INACTIVE
                if value is False
                else Outcome.UNKNOWN
            )
        except Exception:
            return Outcome.UNKNOWN

    def status(self, task_name: str) -> Outcome:
        try:
            value = self._invoke(task_name, "get_state", {})
            value = getattr(value, "state", value)
            state = TrajectoryState(value)
        except (TypeError, ValueError, KeyError):
            return Outcome.UNKNOWN
        except Exception:
            return Outcome.UNKNOWN
        return {
            TrajectoryState.IDLE: Outcome.INACTIVE,
            TrajectoryState.EXECUTING: Outcome.RUNNING,
            TrajectoryState.COMPLETED: Outcome.COMPLETED,
            TrajectoryState.ABORTED: Outcome.CANCELLED,
            TrajectoryState.FAULT: Outcome.FAILED,
        }[state]

    def reset(self, task_name: str) -> Outcome:
        try:
            value = self._invoke(task_name, "reset", {})
            return (
                Outcome.INACTIVE
                if value is True
                else Outcome.RUNNING
                if value is False
                else Outcome.UNKNOWN
            )
        except Exception:
            return Outcome.UNKNOWN

    def set_gripper_position(self, hardware_id: str, position: float) -> Outcome:
        try:
            value = self._client.set_gripper_position(hardware_id, position)
            return (
                Outcome.ACCEPTED
                if value is True
                else Outcome.REJECTED
                if value is False
                else Outcome.UNKNOWN
            )
        except Exception:
            return Outcome.UNKNOWN

    def get_gripper_position(self, hardware_id: str) -> float | None:
        try:
            value = self._client.get_gripper_position(hardware_id)
            return float(value) if isinstance(value, (int, float)) else None
        except Exception:
            return None

    def stop(self) -> None:
        stop = getattr(self._client, "stop_rpc_client", None)
        if callable(stop):
            stop()


@dataclass(frozen=True)
class _Event:
    kind: str
    operation_id: str | None = None
    attempt_id: str | None = None
    task_id: str | None = None
    action_id: str | None = None
    method: ActionMethod | None = None
    outcome: Outcome | None = None
    diagnostic: str = ""
    payload: Any = None
    reset_id: str | None = None


@dataclass
class _Call:
    function: Callable[[], Any]
    answer: queue.Queue[Any] = field(default_factory=lambda: queue.Queue(1))


class ExecutionRuntime:
    """All context commits and effect launches happen on one owner thread."""

    def __init__(
        self,
        gateway_factory: Callable[[], CoordinatorGateway],
        *,
        topology: ExecutionTopology | None = None,
        plan_validator: Callable[[PlanInput], bool] | None = None,
        action_timeout: float = 1.0,
        poll_interval: float = 0.1,
    ) -> None:
        self._gateway = gateway_factory()
        self._topology = topology
        self._validate_plan = plan_validator or self._valid_plan
        self._timeout = action_timeout
        self._poll_interval = poll_interval
        self._context = RuntimeContext()
        self._effects: list[tuple[Operation, TaskRecord, ActionMethod]] = []
        self._context_lock = threading.Lock()
        self._events: queue.Queue[_Call | None] = queue.Queue()
        self._closed = False
        self._stop = threading.Event()
        self._condition = threading.Condition(self._context_lock)
        self._owner = threading.Thread(
            target=self._loop, daemon=True, name="manipulation-execution-owner"
        )
        self._owner.start()
        self._poller = threading.Thread(
            target=self._poll_loop, daemon=True, name="manipulation-execution-poller"
        )
        self._poller.start()
        self._shutdown_lock = threading.Lock()
        self._auxiliary_lock = threading.Lock()
        self._auxiliary: dict[threading.Thread, threading.Event] = {}
        self._closing_requested = False
        self._gateway_condition = threading.Condition()
        self._gateway_gate = _GatewayGateState.OPEN
        self._gateway_close_started = False
        self._gateway_close_done = False
        self._gateway_close_error: str | None = None
        self._rpc_inflight = 0
        self._admitted_actions: set[str] = set()
        self._started_actions: set[str] = set()

    def _start_auxiliary(
        self,
        target: Callable[..., None],
        *args: Any,
        name: str,
    ) -> None:
        """Start a tracked effect thread with cooperative cancellation."""
        cancel = threading.Event()

        def run() -> None:
            try:
                target(cancel, *args)
            finally:
                with self._auxiliary_lock:
                    self._auxiliary.pop(threading.current_thread(), None)

        thread = threading.Thread(target=run, name=name, daemon=True)
        with self._auxiliary_lock:
            self._auxiliary[thread] = cancel
            thread.start()

    def _stop_auxiliary(self, deadline: float) -> bool:
        """Cancel and join tracked effects until the caller's deadline."""
        with self._auxiliary_lock:
            pending = tuple(self._auxiliary.items())
            for _thread, cancel in pending:
                cancel.set()
        for thread, _cancel in pending:
            thread.join(max(0.0, deadline - time.monotonic()))
        with self._auxiliary_lock:
            return not self._auxiliary

    def _valid_plan(self, plan: PlanInput) -> bool:
        if not isinstance(plan, (ExecutionPlan, PreparedPlan)) or not plan.entries:
            return False
        if isinstance(plan, ExecutionPlan) and isinstance(plan.generated_plan, GeneratedPlan):
            return False
        groups = tuple(getattr(plan.generated_plan, "group_ids", ()))
        represented = tuple(entry.planning_group for entry in plan.entries)
        if len(set(represented)) != len(represented) or len(
            set(entry.task_name for entry in plan.entries)
        ) != len(plan.entries):
            return False
        if isinstance(plan, PreparedPlan) and (
            not plan._canonical
            or not isinstance(plan.generated_plan, GeneratedPlan)
            or plan.topology is None
        ):
            return False
        if isinstance(plan, PreparedPlan) and groups != represented:
            return False
        if not isinstance(plan, PreparedPlan) and groups and set(groups) != set(represented):
            return False
        topology = plan.topology if isinstance(plan, PreparedPlan) else self._topology
        if topology is not None:
            routes = {group: (robot, task) for group, robot, task in topology.routes}
            selected_groups = groups or represented
            if len(routes) != len(topology.routes) or any(
                group not in routes for group in selected_groups
            ):
                return False
            selected = tuple(routes[group] for group in selected_groups)
            if tuple(entry.planning_group for entry in plan.entries) != selected_groups:
                return False
            if any(
                (
                    route[1] != entry.task_name
                    or (entry.robot_name is not None and route[0] != entry.robot_name)
                )
                for entry, route in zip(plan.entries, selected, strict=True)
            ):
                return False
        return True

    def snapshot(self) -> RuntimeSnapshot:
        with self._context_lock:
            c = self._context
            return RuntimeSnapshot(
                c.state,
                c.ready_plan,
                c.ready_plan_id,
                c.planning_token,
                c.active,
                c.fault,
                c.diagnostic,
                c.shutdown,
                c.shutdown_result,
                c.revision,
            )

    def _loop(self) -> None:
        while True:
            call = self._events.get()
            if call is None:
                return
            try:
                value = call.function()
                self._process_effects()
                call.answer.put(value)
            except BaseException as exc:
                call.answer.put(exc)

    def _submit(self, fn: Callable[[], T], *, allow_closing: bool = False) -> T:
        with self._context_lock:
            reject = self._closed or (
                not allow_closing
                and (self._closing_requested or self._context.shutdown != ShutdownState.OPEN)
            )
            if reject:
                diagnostic = "runtime is closed" if self._closed else "runtime is closing"
                return cast(
                    "T",
                    CommandResult(False, diagnostic=diagnostic, snapshot=self._snapshot_unlocked()),
                )
            call = _Call(fn)
            self._events.put(call)
        result = call.answer.get()
        if isinstance(result, BaseException):
            raise result
        return cast("T", result)

    def _snapshot_unlocked(self) -> RuntimeSnapshot:
        c = self._context
        return RuntimeSnapshot(
            c.state,
            c.ready_plan,
            c.ready_plan_id,
            c.planning_token,
            c.active,
            c.fault,
            c.diagnostic,
            c.shutdown,
            c.shutdown_result,
            c.revision,
        )

    def _commit(self, context: RuntimeContext) -> RuntimeContext:
        with self._condition:
            self._context = replace(context, revision=context.revision + 1)
            self._condition.notify_all()
        return self._context

    def _record_dispatch_result_if_absent(self, result: ExecutionResult) -> None:
        """Record the first dispatch outcome; later cleanup never replaces it."""
        if dict(self._context.dispatch_results).get(result.handle) is None:
            self._commit(
                replace(
                    self._context,
                    dispatch_results=(*self._context.dispatch_results, (result.handle, result))[
                        -16:
                    ],
                )
            )

    def _reset_pending(self) -> bool:
        handle = self._context.reset_handle
        return (
            self._context.state == LifecycleState.FAULT
            and handle is not None
            and handle not in dict(self._context.reset_results)
        )

    def _finish_reset(self, success: bool, diagnostic: str = "") -> None:
        handle = self._context.reset_handle
        if handle is None or handle in dict(self._context.reset_results):
            return
        result = ResetResult(handle, success, diagnostic)
        reset_results = (*self._context.reset_results, (handle, result))[-16:]
        self._commit(
            replace(
                self._context,
                state=LifecycleState.IDLE if success else LifecycleState.FAULT,
                active=None if success else self._context.active,
                fault=None if success else self._context.fault,
                diagnostic=None if success else diagnostic,
                reset_results=reset_results,
                # A failed attempt is retained in reset_results for its
                # caller, but must not remain the active reconciliation gate:
                # a later reset command must be able to start a fresh attempt.
                reset_handle=None,
                reset_proven_inactive=frozenset(),
                reset_completed_tasks=frozenset(),
                reset_deadline=None,
            )
        )

    def _finish_shutdown(self, success: bool, diagnostic: str = "") -> None:
        if self._context.shutdown_result is not None:
            return
        self._commit(replace(self._context, shutdown_result=ShutdownResult(success, diagnostic)))

    def _maybe_finish_shutdown(self) -> None:
        if (
            self._context.shutdown != ShutdownState.CLOSING
            or self._context.shutdown_result is not None
        ):
            return
        operation = self._context.active
        if operation is None:
            self._finish_shutdown(True)
            return
        if any(task.action is not None for task in operation.tasks):
            return
        if any(task.activity == TaskActivity.REMOTE_FAULT for task in operation.tasks):
            self._finish_shutdown(False, "shutdown safety unresolved: remote fault")
            return
        pending = next(
            (
                task
                for task in operation.tasks
                if task.activity
                in (
                    TaskActivity.NOT_STARTED,
                    TaskActivity.EXECUTE_UNRESOLVED,
                    TaskActivity.ACTIVE,
                    TaskActivity.UNKNOWN,
                )
            ),
            None,
        )
        if pending is not None:
            self._schedule_action(operation, pending, ActionMethod.CANCEL)
            return
        if all(
            task.activity in (TaskActivity.INACTIVE, TaskActivity.CANCELLED, TaskActivity.COMPLETED)
            for task in operation.tasks
        ):
            self._finish_shutdown(True)

    def _reconcile_reset(self, op: Operation | None, event: _Event | None = None) -> None:
        if not self._reset_pending():
            return
        op = op or self._context.active
        if op is None:
            self._finish_reset(False, "fault has no operation to reconcile")
            return
        if (
            self._context.reset_deadline is not None
            and time.monotonic() >= self._context.reset_deadline
        ):
            self._finish_reset(False, "fault reset deadline exceeded")
            return
        if any(task.action is not None for task in op.tasks):
            return

        proven = self._context.reset_proven_inactive
        if event is not None and event.method == ActionMethod.STATUS:
            if event.outcome in (Outcome.INACTIVE, Outcome.CANCELLED, Outcome.COMPLETED):
                proven = proven | {event.task_id} if event.task_id is not None else proven
            elif event.outcome in (Outcome.RUNNING, Outcome.ACCEPTED):
                task = next(task for task in op.tasks if task.task_id == event.task_id)
                self._schedule_action(self._context.active or op, task, ActionMethod.CANCEL)
                return
            else:
                self._finish_reset(False, "status reconciliation is unsafe")
                return
        elif event is not None and event.method == ActionMethod.RESET:
            if event.outcome != Outcome.INACTIVE:
                self._finish_reset(False, "remote reset failed")
                return
            if event.task_id is not None:
                self._commit(
                    replace(
                        self._context,
                        reset_completed_tasks=self._context.reset_completed_tasks | {event.task_id},
                    )
                )
        elif event is not None and event.outcome in (Outcome.UNKNOWN, Outcome.FAILED):
            self._finish_reset(False, "fault reconciliation is unsafe")
            return

        if proven != self._context.reset_proven_inactive:
            self._commit(replace(self._context, reset_proven_inactive=proven))

        current = self._context.active or op
        unproven = next(
            (
                task
                for task in current.tasks
                if task.task_id not in self._context.reset_proven_inactive
            ),
            None,
        )
        if unproven is not None:
            method = (
                ActionMethod.CANCEL
                if unproven.activity
                in (
                    TaskActivity.NOT_STARTED,
                    TaskActivity.EXECUTE_UNRESOLVED,
                    TaskActivity.ACTIVE,
                    TaskActivity.UNKNOWN,
                )
                else ActionMethod.STATUS
            )
            self._schedule_action(current, unproven, method)
            return

        pending_reset = next(
            (
                task
                for task in current.tasks
                if task.reset_required and task.task_id not in self._context.reset_completed_tasks
            ),
            None,
        )
        if pending_reset is not None:
            self._schedule_action(current, pending_reset, ActionMethod.RESET)
            return
        self._finish_reset(True)

    def _reduce(self, event: _Event) -> Any:
        """Pure transition function plus immutable context commit."""
        c = self._context
        if c.shutdown == ShutdownState.CLOSED:
            return False
        if c.shutdown == ShutdownState.CLOSING and event.kind not in (
            "action_deadline",
            "action_finished",
            "effect_rejected",
            "shutdown",
            "shutdown_deadline",
        ):
            return False
        if event.kind == "start_planning" and c.state in (
            LifecycleState.IDLE,
            LifecycleState.READY,
        ):
            token = str(uuid.uuid4())
            self._commit(
                replace(
                    c,
                    state=LifecycleState.PLANNING,
                    planning_token=token,
                    ready_plan=None,
                    ready_plan_id=None,
                    diagnostic=None,
                )
            )
            return token
        if event.kind == "cancel_planning" and c.state == LifecycleState.PLANNING:
            self._commit(replace(c, state=LifecycleState.IDLE, planning_token=None))
            return True
        if (
            event.kind == "plan_ready"
            and c.state == LifecycleState.PLANNING
            and event.payload is not None
        ):
            plan_id = str(uuid.uuid4())
            self._commit(
                replace(
                    c,
                    state=LifecycleState.READY,
                    ready_plan=event.payload,
                    ready_plan_id=plan_id,
                    planning_token=None,
                    diagnostic=None,
                )
            )
            return plan_id
        if event.kind == "planning_failed" and c.state == LifecycleState.PLANNING:
            self._commit(
                replace(
                    c,
                    state=LifecycleState.IDLE,
                    planning_token=None,
                    ready_plan=None,
                    ready_plan_id=None,
                    diagnostic=event.diagnostic or "planning failed",
                )
            )
            return True
        if event.kind == "clear_ready_plan" and c.state == LifecycleState.READY:
            self._commit(replace(c, state=LifecycleState.IDLE, ready_plan=None, ready_plan_id=None))
            return True
        if event.kind == "dispatch_start" and c.state in (
            LifecycleState.IDLE,
            LifecycleState.READY,
        ):
            handle, plan = event.payload
            self._commit(
                replace(
                    c,
                    state=LifecycleState.DISPATCHING,
                    ready_plan=None,
                    ready_plan_id=None,
                    active=Operation(
                        handle,
                        plan,
                        tuple(
                            TaskRecord(str(uuid.uuid4()), e.task_name, e, TaskActivity.NOT_STARTED)
                            for e in plan.entries
                        ),
                    ),
                    fault=None,
                    diagnostic=None,
                    reset_handle=None,
                    reset_proven_inactive=frozenset(),
                    reset_completed_tasks=frozenset(),
                    reset_deadline=None,
                    shutdown_result=None,
                    shutdown_deadline=None,
                )
            )
            return True
        if event.kind == "reset_deadline":
            reset_handle = self._context.reset_handle
            if (
                self._reset_pending()
                and reset_handle is not None
                and event.payload == reset_handle.reset_id
            ):
                self._finish_reset(False, "fault reset deadline exceeded")
                return True
            return False
        if event.kind == "shutdown_deadline":
            if (
                c.shutdown == ShutdownState.CLOSING
                and c.shutdown_result is None
                and event.payload == c.shutdown_deadline
            ):
                self._finish_shutdown(False, "shutdown safety deadline exceeded")
                return True
            return False
        op = c.active
        if event.kind == "effect_rejected":
            if (
                op is None
                or event.operation_id != op.handle.operation_id
                or event.attempt_id != op.handle.attempt_id
            ):
                return False
            task_index = next(
                (i for i, task in enumerate(op.tasks) if task.task_id == event.task_id), None
            )
            if task_index is None:
                return False
            task = op.tasks[task_index]
            if (
                task.action is None
                or task.action.action_id != event.action_id
                or task.action.method != event.method
            ):
                return False
            if event.reset_id is not None:
                current_reset_id = c.reset_handle.reset_id if c.reset_handle is not None else None
                if event.reset_id != current_reset_id:
                    tasks = list(op.tasks)
                    tasks[task_index] = replace(task, action=None)
                    self._commit(replace(c, active=replace(op, tasks=tuple(tasks))))
                    if self._reset_pending():
                        self._reconcile_reset(self._context.active)
                    return True
            activity = (
                TaskActivity.INACTIVE if event.method == ActionMethod.EXECUTE else task.activity
            )
            tasks = list(op.tasks)
            tasks[task_index] = replace(task, action=None, activity=activity)
            self._commit(replace(c, active=replace(op, tasks=tuple(tasks))))
            if c.shutdown == ShutdownState.CLOSING and c.shutdown_result is None:
                self._finish_shutdown(False, "shutdown dropped an unresolved coordinator effect")
            return True
        if event.kind in ("action_deadline", "action_finished"):
            if (
                op is None
                or event.operation_id != op.handle.operation_id
                or event.attempt_id != op.handle.attempt_id
                or event.task_id not in {t.task_id for t in op.tasks}
            ):
                return False
            task_index = next(i for i, task in enumerate(op.tasks) if task.task_id == event.task_id)
            task = op.tasks[task_index]
            if (
                task.action is None
                or task.action.action_id != event.action_id
                or task.action.method != event.method
            ):
                return False
            if event.reset_id is not None:
                current_reset_id = c.reset_handle.reset_id if c.reset_handle is not None else None
                if event.reset_id != current_reset_id:
                    if event.kind == "action_deadline":
                        return False
                    tasks = list(op.tasks)
                    tasks[task_index] = replace(task, action=None)
                    refreshed = replace(op, tasks=tuple(tasks))
                    self._commit(replace(c, active=refreshed))
                    self._reconcile_reset(refreshed)
                    return True
            event_detail = f"task={task.task_name} action={event.action_id} outcome={event.outcome.value if event.outcome is not None else 'unknown'}"
            if event.kind == "action_deadline":
                if task.action.deadline_reported:
                    return False
                if not self._action_was_started(task.action.action_id):
                    return self._reduce(replace(event, kind="effect_rejected"))
                action = replace(task.action, deadline_reported=True)
                tasks = list(op.tasks)
                tasks[task_index] = replace(
                    task, action=action, activity=TaskActivity.UNKNOWN, cancel_required=True
                )
                with self._gateway_condition:
                    self._started_actions.discard(task.action.action_id)
                fault_op = replace(op, tasks=tuple(tasks), uncertain=True)
                result = ExecutionResult(op.handle, Outcome.UNKNOWN, "coordinator action deadline")
                self._commit(
                    replace(
                        c,
                        state=LifecycleState.FAULT,
                        fault="coordinator action deadline",
                        diagnostic="coordinator action deadline",
                        active=fault_op,
                    )
                )
                if c.shutdown == ShutdownState.CLOSING:
                    self._finish_shutdown(False, "shutdown safety unresolved: action deadline")
                    return True
                if self._reset_pending():
                    self._finish_reset(False, "fault reconciliation action deadline")
                    return True
                self._record_dispatch_result_if_absent(result)
                self._schedule_actions(fault_op, fault_op.tasks, ActionMethod.CANCEL)
                return True
            tasks = list(op.tasks)
            with self._gateway_condition:
                self._started_actions.discard(task.action.action_id)
            if event.outcome in (Outcome.ACCEPTED, Outcome.RUNNING):
                activity = TaskActivity.ACTIVE
            elif event.outcome == Outcome.COMPLETED:
                activity = TaskActivity.COMPLETED
            elif event.outcome == Outcome.CANCELLED:
                activity = (
                    TaskActivity.CANCELLED
                    if event.method == ActionMethod.CANCEL or c.state == LifecycleState.CANCELLING
                    else TaskActivity.UNKNOWN
                )
            elif event.outcome == Outcome.INACTIVE:
                activity = TaskActivity.INACTIVE
            elif event.outcome == Outcome.FAILED:
                activity = TaskActivity.REMOTE_FAULT
            elif event.outcome == Outcome.REJECTED and event.method == ActionMethod.EXECUTE:
                activity = TaskActivity.INACTIVE
            else:
                activity = TaskActivity.UNKNOWN
            tasks[task_index] = replace(
                task,
                activity=activity,
                action=None,
                cancel_required=task.cancel_required
                or (c.state in (LifecycleState.CANCELLING, LifecycleState.FAULT)),
                reset_required=task.reset_required or event.outcome == Outcome.FAILED,
            )
            next_op = replace(op, tasks=tuple(tasks))
            self._commit(replace(c, active=next_op))
            if c.shutdown == ShutdownState.CLOSING:
                if self._context.shutdown_result is None:
                    self._maybe_finish_shutdown()
                return True
            if self._reset_pending():
                self._reconcile_reset(next_op, event)
                return True
            if c.state == LifecycleState.FAULT:
                if event.method != ActionMethod.CANCEL:
                    self._schedule_actions(next_op, next_op.tasks, ActionMethod.CANCEL)
                return True
            if event.method == ActionMethod.STATUS and event.outcome in (
                Outcome.UNKNOWN,
                Outcome.FAILED,
            ):
                fault_op = replace(next_op, uncertain=True)
                diagnostic = f"status is unsafe: {event_detail}"
                result = ExecutionResult(op.handle, Outcome.UNKNOWN, diagnostic)
                self._commit(
                    replace(
                        self._context,
                        state=LifecycleState.FAULT,
                        fault=diagnostic,
                        diagnostic=diagnostic,
                        active=fault_op,
                    )
                )
                self._record_dispatch_result_if_absent(result)
                self._schedule_actions(fault_op, fault_op.tasks, ActionMethod.CANCEL)
                return True
            if event.method == ActionMethod.CANCEL and event.outcome in (
                Outcome.UNKNOWN,
                Outcome.FAILED,
            ):
                fault_op = replace(next_op, uncertain=True)
                diagnostic = (
                    "cancellation is uncertain"
                    if event.outcome == Outcome.UNKNOWN
                    else "cancellation failed"
                )
                result = ExecutionResult(op.handle, Outcome.UNKNOWN, diagnostic)
                self._commit(
                    replace(
                        self._context,
                        state=LifecycleState.FAULT,
                        fault=diagnostic,
                        diagnostic=diagnostic,
                        active=fault_op,
                    )
                )
                self._record_dispatch_result_if_absent(result)
                if event.outcome == Outcome.FAILED:
                    self._schedule_actions(fault_op, fault_op.tasks, ActionMethod.CANCEL)
                return True
            if (
                c.state == LifecycleState.CANCELLING
                and event.method == ActionMethod.STATUS
                and event.outcome == Outcome.RUNNING
            ):
                self._schedule_actions(next_op, (tasks[task_index],), ActionMethod.CANCEL)
                return True
            if (
                event.method == ActionMethod.STATUS
                and event.outcome in (Outcome.INACTIVE, Outcome.CANCELLED)
                and c.state == LifecycleState.RUNNING
            ):
                diagnostic = f"status reported terminal failure: {event_detail}"
                failed_op = replace(
                    next_op,
                    cancel_requested=True,
                    cleanup_required=True,
                    rejected=False,
                    failed=True,
                    diagnostic=diagnostic,
                )
                self._commit(
                    replace(
                        self._context,
                        state=LifecycleState.CANCELLING,
                        active=failed_op,
                        diagnostic=diagnostic,
                    )
                )
                self._schedule_actions(failed_op, failed_op.tasks, ActionMethod.CANCEL)
                return True
            if event.method == ActionMethod.EXECUTE and event.outcome in (
                Outcome.REJECTED,
                Outcome.UNKNOWN,
                Outcome.FAILED,
            ):
                if event.outcome == Outcome.REJECTED:
                    skipped = tuple(
                        replace(item, activity=TaskActivity.INACTIVE)
                        if item.activity == TaskActivity.NOT_STARTED
                        else item
                        for item in next_op.tasks
                    )
                    next_op = replace(next_op, tasks=skipped)
                    self._commit(replace(self._context, active=next_op))
                if event.outcome != Outcome.REJECTED:
                    fault_op = replace(next_op, uncertain=True)
                    diagnostic = f"execute outcome is uncertain: {event_detail}"
                    result = ExecutionResult(op.handle, Outcome.UNKNOWN, diagnostic)
                    self._commit(
                        replace(
                            self._context,
                            state=LifecycleState.FAULT,
                            fault=diagnostic,
                            diagnostic=diagnostic,
                            active=fault_op,
                        )
                    )
                    self._record_dispatch_result_if_absent(result)
                    self._schedule_actions(fault_op, fault_op.tasks, ActionMethod.CANCEL)
                elif any(
                    t.activity in (TaskActivity.ACTIVE, TaskActivity.UNKNOWN) for t in next_op.tasks
                ):
                    diagnostic = f"execute rejected: {event_detail}"
                    rejected_op = replace(
                        next_op,
                        cancel_requested=True,
                        rejected=True,
                        cleanup_required=True,
                        diagnostic=diagnostic,
                    )
                    rejection_result = ExecutionResult(op.handle, Outcome.REJECTED, diagnostic)
                    self._commit(
                        replace(
                            self._context,
                            state=LifecycleState.CANCELLING,
                            active=rejected_op,
                            diagnostic=diagnostic,
                        )
                    )
                    self._record_dispatch_result_if_absent(rejection_result)
                    self._schedule_actions(rejected_op, rejected_op.tasks, ActionMethod.CANCEL)
                else:
                    diagnostic = f"execute rejected: {event_detail}"
                    result = ExecutionResult(next_op.handle, Outcome.REJECTED, diagnostic)
                    self._commit(
                        replace(
                            self._context,
                            state=LifecycleState.IDLE,
                            active=None,
                            diagnostic=diagnostic,
                            terminal_results=(
                                *self._context.terminal_results,
                                (next_op.handle, result),
                            )[-16:],
                        )
                    )
                    self._record_dispatch_result_if_absent(result)
                return True
            self._advance(next_op, event)
            return True
        if (
            (
                event.kind == "cancel"
                or (
                    event.kind == "cancel_if_current"
                    and op is not None
                    and event.payload == op.handle
                )
            )
            and op is not None
            and c.state in (LifecycleState.DISPATCHING, LifecycleState.RUNNING)
        ):
            cancel_tasks: tuple[TaskRecord, ...] = tuple(
                replace(task, activity=TaskActivity.INACTIVE, cancel_required=True)
                if task.activity == TaskActivity.NOT_STARTED
                else replace(task, cancel_required=True)
                if task.activity == TaskActivity.EXECUTE_UNRESOLVED
                else task
                for task in op.tasks
            )
            next_op = replace(op, tasks=cancel_tasks, cancel_requested=True)
            self._commit(replace(c, state=LifecycleState.CANCELLING, active=next_op))
            if c.state == LifecycleState.DISPATCHING and not dict(c.dispatch_results).get(
                op.handle
            ):
                result = ExecutionResult(
                    op.handle, Outcome.CANCELLED, "cancelled before dispatch completed"
                )
                self._record_dispatch_result_if_absent(result)
            self._schedule_actions(next_op, cancel_tasks, ActionMethod.CANCEL)
            return True
        if event.kind == "poll_tick" and op is not None and c.state == LifecycleState.RUNNING:
            candidate = next(
                (
                    item
                    for item in op.tasks
                    if item.activity == TaskActivity.ACTIVE and item.action is None
                ),
                None,
            )
            if candidate is not None:
                self._schedule_actions(op, (candidate,), ActionMethod.STATUS)
            return True
        if event.kind == "fault":
            diagnostic = event.diagnostic or "remote uncertainty"
            self._commit(
                replace(
                    c,
                    state=LifecycleState.FAULT,
                    fault=diagnostic,
                    diagnostic=diagnostic,
                    active=replace(op, uncertain=True) if op else None,
                )
            )
            return True
        if event.kind == "shutdown":
            if self._reset_pending():
                self._finish_reset(False, "shutdown interrupted fault reset")
                c = self._context
                op = c.active
            if op is not None:
                shutdown_tasks: tuple[TaskRecord, ...] = tuple(
                    replace(task, activity=TaskActivity.INACTIVE, cancel_required=True)
                    if task.activity == TaskActivity.NOT_STARTED
                    else replace(task, cancel_required=True)
                    for task in op.tasks
                )
                op = replace(op, tasks=shutdown_tasks, cancel_requested=True, cleanup_required=True)
            self._commit(
                replace(
                    c,
                    shutdown=ShutdownState.CLOSING,
                    state=LifecycleState.CANCELLING if op else LifecycleState.IDLE,
                    planning_token=None,
                    ready_plan=None,
                    ready_plan_id=None,
                    active=op,
                    shutdown_deadline=event.payload,
                )
            )
            if op is None:
                self._finish_shutdown(True)
            else:
                self._schedule_actions(op, op.tasks, ActionMethod.CANCEL)
                self._maybe_finish_shutdown()
            return True
        return False

    def _advance(self, op: Operation, event: _Event) -> None:
        # This is called only by the owner after a correlated result.
        cancelling = op.cancel_requested or self._context.state in (
            LifecycleState.CANCELLING,
            LifecycleState.FAULT,
        )
        if cancelling:
            self._schedule_actions(op, op.tasks, ActionMethod.CANCEL)
        if (
            not cancelling
            and event.method == ActionMethod.EXECUTE
            and event.outcome == Outcome.ACCEPTED
        ):
            if op.next_index + 1 < len(op.tasks):
                self._commit(
                    replace(self._context, active=replace(op, next_index=op.next_index + 1))
                )
                self._schedule_execute(self._context.active)
                return
            if all(task.activity == TaskActivity.ACTIVE for task in op.tasks):
                result = ExecutionResult(op.handle, Outcome.ACCEPTED)
                self._commit(replace(self._context, state=LifecycleState.RUNNING))
                self._record_dispatch_result_if_absent(result)
                return
        terminal = all(
            task.action is None
            and task.activity
            in (TaskActivity.COMPLETED, TaskActivity.CANCELLED, TaskActivity.INACTIVE)
            for task in op.tasks
        )
        completed = all(
            task.activity == TaskActivity.COMPLETED and task.action is None for task in op.tasks
        )
        if terminal and (op.cancel_requested or completed):
            if self._context.state == LifecycleState.FAULT:
                return
            outcome = (
                Outcome.REJECTED
                if op.rejected
                else Outcome.FAILED
                if op.failed
                else Outcome.CANCELLED
                if op.cancel_requested
                else Outcome.COMPLETED
            )
            result = ExecutionResult(op.handle, outcome, op.diagnostic)
            self._commit(
                replace(
                    self._context,
                    state=LifecycleState.IDLE,
                    active=None,
                    diagnostic=op.diagnostic or self._context.diagnostic,
                    terminal_results=(*self._context.terminal_results, (op.handle, result))[-16:],
                )
            )

    def _schedule_execute(self, op: Operation | None) -> None:
        if op is None or op.next_index >= len(op.tasks):
            return
        task = op.tasks[op.next_index]
        if task.action is not None:
            return
        self._schedule_action(op, task, ActionMethod.EXECUTE)

    def _schedule_action(self, op: Operation, task: TaskRecord, method: ActionMethod) -> None:
        self._schedule_actions(op, (task,), method)

    def _schedule_actions(self, op: Operation, requested: Any, method: ActionMethod) -> None:
        """Allocate and commit a whole cleanup batch against one operation."""
        if (
            self._gateway_gate in (_GatewayGateState.SEALED, _GatewayGateState.STOPPED)
            or self._context.shutdown == ShutdownState.CLOSED
            or (self._context.shutdown == ShutdownState.CLOSING and method != ActionMethod.CANCEL)
        ):
            return
        candidates = tuple(
            task
            for task in requested
            if task.action is None
            and (
                method in (ActionMethod.STATUS, ActionMethod.RESET)
                or task.activity
                not in (
                    TaskActivity.INACTIVE,
                    TaskActivity.COMPLETED,
                    TaskActivity.CANCELLED,
                    TaskActivity.REMOTE_FAULT,
                )
            )
        )
        if not candidates:
            return
        now = time.monotonic()
        reset_id = (
            self._context.reset_handle.reset_id
            if self._reset_pending() and self._context.reset_handle is not None
            else None
        )
        actions = {
            task.task_id: ActionRecord(
                str(uuid.uuid4()), method, now, now + self._timeout, reset_id=reset_id
            )
            for task in candidates
        }
        task_map = {task.task_id: task for task in op.tasks}
        for task in candidates:
            task_map[task.task_id] = replace(
                task,
                action=actions[task.task_id],
                activity=TaskActivity.EXECUTE_UNRESOLVED
                if method == ActionMethod.EXECUTE
                else task.activity,
            )
        updated = replace(op, tasks=tuple(task_map[task.task_id] for task in op.tasks))
        self._commit(replace(self._context, active=updated))
        self._effects.extend((updated, task_map[task.task_id], method) for task in candidates)

    def _process_effects(self) -> None:
        effects, self._effects = self._effects, []
        if self._gateway_gate in (_GatewayGateState.SEALED, _GatewayGateState.STOPPED):
            if (
                self._context.shutdown == ShutdownState.CLOSING
                and self._context.shutdown_result is None
                and effects
            ):
                self._finish_shutdown(False, "shutdown gateway closed before pending effects")
            return
        for op, task, method in effects:
            admitted = self._run_action(op, task, method)
            if (
                not admitted
                and self._context.shutdown == ShutdownState.CLOSING
                and self._context.shutdown_result is None
            ):
                self._finish_shutdown(False, "shutdown dropped an unresolved coordinator effect")

    def _invoke_gateway_action(
        self, action_id: str, task_name: str, method: ActionMethod, request: Any
    ) -> tuple[bool, Any]:
        """Lease a gateway call immediately before invoking it; no context lock crosses I/O."""
        with self._gateway_condition:
            allowed = self._gateway_gate == _GatewayGateState.OPEN or (
                self._gateway_gate == _GatewayGateState.CANCEL_ONLY
                and method == ActionMethod.CANCEL
            )
            if not allowed:
                return False, None
            self._rpc_inflight += 1
            self._admitted_actions.add(action_id)
            self._started_actions.add(action_id)
        try:
            outcome: Any = Outcome.UNKNOWN
            try:
                if method == ActionMethod.EXECUTE:
                    outcome = self._gateway.execute(task_name, request)
                elif method == ActionMethod.CANCEL:
                    outcome = self._gateway.cancel(task_name)
                elif method == ActionMethod.STATUS:
                    outcome = self._gateway.status(task_name)
                elif method == ActionMethod.RESET:
                    outcome = self._gateway.reset(task_name)
                elif method == ActionMethod.GRIPPER_SET:
                    outcome = self._gateway.set_gripper_position(task_name, float(request))
                else:
                    outcome = self._gateway.get_gripper_position(task_name)
            except Exception:
                outcome = Outcome.UNKNOWN
            return True, outcome
        finally:
            with self._gateway_condition:
                self._rpc_inflight -= 1
                self._admitted_actions.discard(action_id)
                self._gateway_condition.notify_all()

    def _action_was_admitted(self, action_id: str) -> bool:
        with self._gateway_condition:
            return action_id in self._admitted_actions

    def _action_was_started(self, action_id: str) -> bool:
        with self._gateway_condition:
            return action_id in self._started_actions

    def _run_action(self, op: Operation, task: TaskRecord, method: ActionMethod) -> bool:
        action = task.action
        if action is None:
            return False
        request = task.entry.request

        def call(_cancel: threading.Event) -> None:
            admitted, outcome = self._invoke_gateway_action(
                action.action_id, task.task_name, method, request
            )
            if admitted:
                self._events.put(
                    _Call(
                        lambda: self._reduce(
                            _Event(
                                "action_finished",
                                op.handle.operation_id,
                                op.handle.attempt_id,
                                task.task_id,
                                action.action_id,
                                method,
                                outcome,
                                reset_id=action.reset_id,
                            )
                        )
                    )
                )
            else:
                self._events.put(
                    _Call(
                        lambda: self._reduce(
                            _Event(
                                "effect_rejected",
                                op.handle.operation_id,
                                op.handle.attempt_id,
                                task.task_id,
                                action.action_id,
                                method,
                                reset_id=action.reset_id,
                            )
                        )
                    )
                )

        def deadline(cancel: threading.Event) -> None:
            if cancel.wait(max(0.0, action.deadline - time.monotonic())):
                return
            self._events.put(
                _Call(
                    lambda: self._reduce(
                        _Event(
                            "action_deadline",
                            op.handle.operation_id,
                            op.handle.attempt_id,
                            task.task_id,
                            action.action_id,
                            method,
                            reset_id=action.reset_id,
                        )
                    )
                )
            )

        self._start_auxiliary(call, name=f"action-{action.action_id}")
        self._start_auxiliary(deadline, name=f"deadline-{action.action_id}")
        return True

    def start_planning(self) -> str | None:
        return cast("str | None", self._submit(lambda: self._reduce(_Event("start_planning"))))

    def complete_planning(self, token: str, plan: PlanInput) -> CommandResult[str]:
        def fn() -> CommandResult[str]:
            if self._context.planning_token != token:
                return CommandResult(
                    False, diagnostic="planning token mismatch", snapshot=self.snapshot()
                )
            if not self._validate_plan(plan):
                self._reduce(_Event("planning_failed", diagnostic="invalid prepared plan"))
                return CommandResult(
                    False, diagnostic="invalid prepared plan", snapshot=self.snapshot()
                )
            plan_id = self._reduce(_Event("plan_ready", payload=plan))
            return CommandResult(True, plan_id, snapshot=self.snapshot())

        return self._submit(fn)

    def fail_planning(self, token: str, diagnostic: str) -> CommandResult[None]:
        return self._submit(
            lambda: CommandResult(
                self._context.planning_token == token,
                diagnostic=diagnostic
                if self._context.planning_token == token
                else "planning token mismatch",
                snapshot=self.snapshot(),
            )
            if self._context.planning_token != token
            else (
                self._reduce(_Event("planning_failed", diagnostic=diagnostic))
                and CommandResult(True, diagnostic=diagnostic, snapshot=self.snapshot())
            )
        )

    def cancel_planning(self) -> CommandResult[None]:
        return self._submit(
            lambda: CommandResult(
                bool(self._reduce(_Event("cancel_planning"))), snapshot=self.snapshot()
            )
        )

    def clear_ready_plan(self) -> CommandResult[None]:
        return self._submit(
            lambda: CommandResult(
                bool(self._reduce(_Event("clear_ready_plan"))), snapshot=self.snapshot()
            )
        )

    def execute_explicit(self, prepared_plan: PlanInput) -> CommandResult[OperationHandle]:
        def fn() -> CommandResult[OperationHandle]:
            if self._context.state in (
                LifecycleState.DISPATCHING,
                LifecycleState.RUNNING,
                LifecycleState.CANCELLING,
            ) and self._validate_plan(prepared_plan):
                handle = OperationHandle(str(uuid.uuid4()), str(uuid.uuid4()), str(uuid.uuid4()))
                result = ExecutionResult(handle, Outcome.REJECTED, "execution already active")
                self._commit(
                    replace(
                        self._context,
                        diagnostic="execution already active",
                        terminal_results=(*self._context.terminal_results, (handle, result))[-16:],
                    )
                )
                return CommandResult(
                    False,
                    handle,
                    diagnostic="execution already active",
                    snapshot=self.snapshot(),
                )
            if self._context.state not in (
                LifecycleState.IDLE,
                LifecycleState.READY,
            ) or not self._validate_plan(prepared_plan):
                self._commit(
                    replace(self._context, diagnostic="execution unavailable or invalid plan")
                )
                return CommandResult(
                    False,
                    diagnostic="execution unavailable or invalid plan",
                    snapshot=self.snapshot(),
                )
            handle = OperationHandle(str(uuid.uuid4()), str(uuid.uuid4()), str(uuid.uuid4()))
            if self._reduce(_Event("dispatch_start", payload=(handle, prepared_plan))):
                self._schedule_execute(self._context.active)
                return CommandResult(True, handle, snapshot=self.snapshot())
            return CommandResult(False, diagnostic="dispatch rejected", snapshot=self.snapshot())

        return self._submit(fn)

    def execute_ready(self) -> CommandResult[OperationHandle]:
        def fn() -> CommandResult[OperationHandle]:
            if self._context.ready_plan is None or self._context.ready_plan_id is None:
                return CommandResult(False, diagnostic="no ready plan", snapshot=self.snapshot())
            plan = self._context.ready_plan
            return self._execute_owner(plan, self._context.ready_plan_id)

        return self._submit(fn)

    def _execute_owner(
        self, plan: PlanInput, plan_id: str | None = None
    ) -> CommandResult[OperationHandle]:
        handle = OperationHandle(plan_id or str(uuid.uuid4()), str(uuid.uuid4()), str(uuid.uuid4()))
        if self._reduce(_Event("dispatch_start", payload=(handle, plan))):
            self._schedule_execute(self._context.active)
            return CommandResult(True, handle, snapshot=self.snapshot())
        return CommandResult(False, diagnostic="dispatch rejected", snapshot=self.snapshot())

    def cancel(self) -> CommandResult[None]:
        return self._submit(
            lambda: CommandResult(bool(self._reduce(_Event("cancel"))), snapshot=self.snapshot())
        )

    def cancel_if_current(self, handle: OperationHandle) -> CommandResult[None]:
        """Cancel only if ``handle`` is the current operation at the owner event."""

        def fn() -> CommandResult[None]:
            accepted = bool(self._reduce(_Event("cancel_if_current", payload=handle)))
            current = self._context.active
            diagnostic = (
                ""
                if accepted
                else "operation is no longer current"
                if current is None or current.handle != handle
                else f"cancellation rejected in state {self._context.state.name}"
            )
            return CommandResult(
                accepted,
                diagnostic=diagnostic,
                snapshot=self.snapshot(),
            )

        return self._submit(fn)

    def poll(self) -> CommandResult[None]:
        return self._submit(
            lambda: CommandResult(bool(self._reduce(_Event("poll_tick"))), snapshot=self.snapshot())
        )

    def set_gripper_position(self, hardware_id: str, position: float) -> CommandResult[Outcome]:
        def fn() -> CommandResult[Outcome]:
            admitted, outcome = self._invoke_gateway_action(
                str(uuid.uuid4()), hardware_id, ActionMethod.GRIPPER_SET, position
            )
            if not admitted:
                return CommandResult(
                    False, diagnostic="gateway is closing", snapshot=self.snapshot()
                )
            return CommandResult(True, outcome, snapshot=self.snapshot())

        return self._submit(fn)

    def get_gripper_position(self, hardware_id: str) -> CommandResult[float]:
        def fn() -> CommandResult[float]:
            admitted, position = self._invoke_gateway_action(
                str(uuid.uuid4()), hardware_id, ActionMethod.GRIPPER_GET, None
            )
            if not admitted:
                return CommandResult(
                    False, diagnostic="gateway is closing", snapshot=self.snapshot()
                )
            if not isinstance(position, (int, float)):
                return CommandResult(
                    False,
                    diagnostic="gateway returned no gripper position",
                    snapshot=self.snapshot(),
                )
            return CommandResult(True, float(position), snapshot=self.snapshot())

        return self._submit(fn)

    def wait_for_dispatch(
        self, handle: OperationHandle, timeout: float = 1.0
    ) -> CommandResult[ExecutionResult]:
        return self._wait(handle, "dispatch", timeout)

    def wait_for_terminal(
        self, handle: OperationHandle, timeout: float = 1.0
    ) -> CommandResult[ExecutionResult]:
        return self._wait(handle, "terminal", timeout)

    def _wait(
        self, handle: OperationHandle, result_kind: str, timeout: float
    ) -> CommandResult[ExecutionResult]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._condition:
                results = (
                    self._context.dispatch_results
                    if result_kind == "dispatch"
                    else self._context.terminal_results
                )
                found = dict(results).get(handle)
                if found is None:
                    self._condition.wait(min(0.01, deadline - time.monotonic()))
            if found is not None:
                return CommandResult(True, found, snapshot=self.snapshot())
        return CommandResult(False, diagnostic="timeout", snapshot=self.snapshot())

    def _poll_loop(self) -> None:
        while not self._stop.wait(self._poll_interval):
            self._events.put(_Call(lambda: self._reduce(_Event("poll_tick"))))

    def reset(self) -> CommandResult[ResetHandle]:
        def fn() -> CommandResult[ResetHandle]:
            if self._context.state != LifecycleState.FAULT:
                return CommandResult(
                    False, diagnostic="reset requires FAULT", snapshot=self.snapshot()
                )
            handle = self._context.reset_handle or ResetHandle(str(uuid.uuid4()))
            if self._context.reset_handle is not None:
                return CommandResult(True, self._context.reset_handle, snapshot=self.snapshot())
            deadline = time.monotonic() + max(self._timeout, 0.05)
            self._commit(
                replace(
                    self._context,
                    reset_handle=handle,
                    reset_deadline=deadline,
                    reset_proven_inactive=frozenset(),
                    reset_completed_tasks=frozenset(),
                )
            )
            self._reconcile_reset(self._context.active)
            self._start_auxiliary(
                self._reset_deadline, handle, deadline, name=f"reset-deadline-{handle.reset_id}"
            )
            return CommandResult(True, handle, snapshot=self.snapshot())

        return self._submit(fn)

    def _reset_deadline(
        self, cancel: threading.Event, handle: ResetHandle, deadline: float
    ) -> None:
        if cancel.wait(max(0.0, deadline - time.monotonic())):
            return
        self._events.put(
            _Call(
                lambda: self._reduce(
                    _Event(
                        "reset_deadline",
                        payload=handle.reset_id,
                        reset_id=handle.reset_id,
                    )
                )
            )
        )

    def wait_for_reset(
        self, handle: ResetHandle, timeout: float = 1.0
    ) -> CommandResult[ResetResult]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._condition:
                result = dict(self._context.reset_results).get(handle)
                if result is None:
                    self._condition.wait(min(0.01, deadline - time.monotonic()))
            if result is not None:
                return CommandResult(True, result, snapshot=self.snapshot())
        return CommandResult(False, diagnostic="timeout", snapshot=self.snapshot())

    def _seal_gateway(self) -> None:
        with self._gateway_condition:
            if self._gateway_gate in (_GatewayGateState.SEALED, _GatewayGateState.STOPPED):
                return
            self._gateway_gate = _GatewayGateState.SEALED
            self._gateway_condition.notify_all()
        self._start_auxiliary(self._close_gateway, name="gateway-close")

    def _close_gateway(self, _cancel: threading.Event) -> None:
        with self._gateway_condition:
            while self._rpc_inflight:
                self._gateway_condition.wait()
            if self._gateway_close_started:
                return
            self._gateway_close_started = True
        try:
            self._gateway.stop()
        except BaseException as exc:
            with self._gateway_condition:
                self._gateway_close_error = f"gateway close failed: {exc}"
                self._gateway_condition.notify_all()
            return
        else:
            with self._gateway_condition:
                self._gateway_gate = _GatewayGateState.STOPPED
                self._gateway_close_done = True
                self._gateway_condition.notify_all()

    def _wait_gateway_closed(self, deadline: float) -> bool:
        with self._gateway_condition:
            while (
                not self._gateway_close_done
                and self._gateway_close_error is None
                and time.monotonic() < deadline
            ):
                self._gateway_condition.wait(min(0.01, deadline - time.monotonic()))
            return self._gateway_close_done and self._gateway_close_error is None

    def shutdown(self, timeout: float = 1.0) -> CommandResult[ShutdownResult]:
        with self._shutdown_lock:
            if self._closed:
                result = self._context.shutdown_result or ShutdownResult(
                    False, "runtime is closed without a shutdown result"
                )
                return CommandResult(result.success, result, snapshot=self.snapshot())
            if not self._owner.is_alive() and self._context.shutdown == ShutdownState.CLOSING:
                result = self._context.shutdown_result or ShutdownResult(
                    False, "shutdown result unavailable"
                )
                if not result.success:
                    return CommandResult(False, result, snapshot=self.snapshot())
                self._commit(
                    replace(self._context, shutdown=ShutdownState.CLOSED, shutdown_result=result)
                )
                self._closed = True
                return CommandResult(result.success, result, snapshot=self.snapshot())
            deadline = time.monotonic() + max(0.0, timeout)
            with self._condition:
                with self._gateway_condition:
                    self._closing_requested = True
                    self._context = replace(
                        self._context,
                        shutdown=ShutdownState.CLOSING,
                        shutdown_deadline=deadline,
                        revision=self._context.revision + 1,
                    )
                    if self._gateway_gate == _GatewayGateState.OPEN:
                        self._gateway_gate = _GatewayGateState.CANCEL_ONLY
                    self._gateway_condition.notify_all()
                self._condition.notify_all()

            self._stop.set()
            call = _Call(lambda: self._reduce(_Event("shutdown", payload=deadline)))
            self._events.put(call)
            remaining = max(0.0, deadline - time.monotonic())
            try:
                call.answer.get(timeout=remaining)
            except queue.Empty:
                pass

            with self._condition:
                while self._context.shutdown_result is None and time.monotonic() < deadline:
                    self._condition.wait(min(0.01, deadline - time.monotonic()))

            if self._context.shutdown_result is None:
                deadline_call = _Call(
                    lambda: self._reduce(_Event("shutdown_deadline", payload=deadline))
                )
                self._events.put(deadline_call)
                remaining = max(0.0, deadline - time.monotonic())
                if remaining:
                    try:
                        deadline_call.answer.get(timeout=remaining)
                    except queue.Empty:
                        pass
            if self._context.shutdown_result is None:
                self._commit(
                    replace(
                        self._context,
                        shutdown_result=ShutdownResult(False, "shutdown safety deadline exceeded"),
                    )
                )

            self._seal_gateway()
            close_done = self._wait_gateway_closed(deadline)
            if not close_done and (
                self._context.shutdown_result is None or self._context.shutdown_result.success
            ):
                with self._gateway_condition:
                    close_diagnostic = (
                        self._gateway_close_error
                        or "gateway close unresolved before shutdown deadline"
                    )
                self._commit(
                    replace(self._context, shutdown_result=ShutdownResult(False, close_diagnostic))
                )
            auxiliary_drained = self._stop_auxiliary(deadline)
            if not auxiliary_drained and (
                self._context.shutdown_result is None or self._context.shutdown_result.success
            ):
                self._commit(
                    replace(
                        self._context,
                        shutdown_result=ShutdownResult(
                            False, "auxiliary runtime thread unresolved before shutdown deadline"
                        ),
                    )
                )
            self._events.put(None)
            remaining = max(0.0, deadline - time.monotonic())
            self._owner.join(remaining)
            owner_stopped = not self._owner.is_alive()
            result = self._context.shutdown_result or ShutdownResult(
                False, "shutdown result unavailable"
            )
            if not owner_stopped:
                result = ShutdownResult(False, "owner thread did not stop before shutdown deadline")
                self._commit(replace(self._context, shutdown_result=result))
                return CommandResult(False, result, snapshot=self.snapshot())
            if not result.success:
                return CommandResult(False, result, snapshot=self.snapshot())
            self._commit(
                replace(self._context, shutdown=ShutdownState.CLOSED, shutdown_result=result)
            )
            self._closed = True
            return CommandResult(result.success and owner_stopped, result, snapshot=self.snapshot())

    close = shutdown
