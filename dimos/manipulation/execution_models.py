# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0 (the "License").
"""Pure public records and protocol models for execution runtime state."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Generic, Protocol, TypeVar

from dimos.manipulation.execution_topology import ExecutionPlan, PreparedPlan, TaskEntry


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


PlanInput = ExecutionPlan | PreparedPlan


@dataclass(frozen=True)
class Operation:
    handle: OperationHandle
    plan: PlanInput
    tasks: tuple[TaskRecord, ...]
    next_index: int = 0
    cancel_requested: bool = False
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


@dataclass
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
    dispatch_results: tuple[tuple[OperationHandle, ExecutionResult], ...] = ()
    terminal_results: tuple[tuple[OperationHandle, ExecutionResult], ...] = ()
    reset_results: tuple[tuple[ResetHandle, ResetResult], ...] = ()
    reset_proven_inactive: frozenset[str] = frozenset()
    reset_completed_tasks: frozenset[str] = frozenset()
    reset_deadline: float | None = None
    shutdown_result: ShutdownResult | None = None
    shutdown_deadline: float | None = None
    physical_operation_deadline: float | None = None
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
