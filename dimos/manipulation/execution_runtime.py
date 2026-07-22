# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0 (the "License").
"""Small, fail-closed owner-thread execution runtime.

The owner is the only thread which changes :class:`RuntimeContext`.  Gateway
calls are submitted to a fixed executor and return as owner events; in
particular, an RPC can never hold the owner while it is in flight.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
import queue
import threading
import time
from typing import Any, TypeVar, cast
import uuid

from dimos.manipulation.execution_auxiliary import AuxiliaryCallBook, AuxiliaryTicket
from dimos.manipulation.execution_clock import InvalidMonotonicClock, ValidatedMonotonicClock
from dimos.manipulation.execution_effects import (
    AuxiliaryDone,
    EffectDone,
    ExecutionEffectRunner,
    StopDone,
)
from dimos.manipulation.execution_gateway import (
    ControlCoordinatorGateway as ControlCoordinatorGateway,
)
from dimos.manipulation.execution_models import (
    ActionMethod as ActionMethod,
    ActionRecord as ActionRecord,
    CommandResult as CommandResult,
    CoordinatorGateway as CoordinatorGateway,
    ExecutionHandle as ExecutionHandle,
    ExecutionResult as ExecutionResult,
    LifecycleState as LifecycleState,
    Operation as Operation,
    OperationHandle as OperationHandle,
    Outcome as Outcome,
    PlanInput as PlanInput,
    ResetHandle as ResetHandle,
    ResetResult as ResetResult,
    RuntimeContext as RuntimeContext,
    RuntimeSnapshot as RuntimeSnapshot,
    ShutdownResult as ShutdownResult,
    ShutdownState as ShutdownState,
    TaskActivity as TaskActivity,
    TaskRecord as TaskRecord,
)
from dimos.manipulation.execution_policy import (
    reconcile_cancel_completion,
    reconcile_execute_completion,
    reconcile_reset_completion,
    reconcile_status_completion,
)
from dimos.manipulation.execution_topology import (
    ExecutionPlan as ExecutionPlan,
    ExecutionTopology as ExecutionTopology,
    PreparedPlan as PreparedPlan,
    TaskEntry as TaskEntry,
    materialize_prepared_plan as materialize_prepared_plan,
    prepare_execution_plan as prepare_execution_plan,
    prepare_generated_plan as prepare_generated_plan,
    prepare_plan as prepare_plan,
)
from dimos.manipulation.planning.spec.models import GeneratedPlan

T = TypeVar("T")


class _Call:
    def __init__(self, fn: Callable[[], Any]) -> None:
        self.fn = fn
        self.answer: queue.Queue[Any] = queue.Queue(1)


@dataclass(frozen=True)
class _PendingAction:
    handle: OperationHandle
    task_name: str
    task: TaskRecord
    method: ActionMethod
    deadline: float


class ExecutionRuntime:
    """Serialized plan executor with a sticky, fail-closed fault state."""

    def __init__(
        self,
        gateway_factory: Callable[[], CoordinatorGateway],
        *,
        topology: ExecutionTopology | None = None,
        plan_validator: Callable[[PlanInput], bool] | None = None,
        action_timeout: float = 1.0,
        physical_operation_timeout: float = 60.0,
        poll_interval: float = 0.1,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._gateway = gateway_factory()
        self._topology = topology
        self._validator = plan_validator
        self._action_timeout = max(0.0, action_timeout)
        self._physical_timeout = max(0.0, physical_operation_timeout)
        self._poll_interval = max(0.001, poll_interval)
        self._clock = ValidatedMonotonicClock(monotonic_clock)
        self._context = RuntimeContext()
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._events: queue.Queue[_Call | EffectDone | StopDone | AuxiliaryDone | None] = (
            queue.Queue()
        )
        self._effects = ExecutionEffectRunner()
        self._closed = False
        self._gate = "open"
        self._pending: dict[str, _PendingAction] = {}
        self._fault_terminal_results: dict[OperationHandle, str] = {}
        self._next_poll: float | None = None
        self._shutdown_lock = threading.Lock()
        self._shutdown_initiated = False
        self._shutdown_stop_started = False
        self._shutdown_stop_finished = False
        self._shutdown_drain_started = False
        self._clock_invalid = False
        self._auxiliary = AuxiliaryCallBook()
        self._owner = threading.Thread(target=self._run, daemon=True, name="execution-owner")
        self._owner.start()

    def _snapshot(self) -> RuntimeSnapshot:
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

    def snapshot(self) -> RuntimeSnapshot:
        with self._lock:
            return self._snapshot()

    def _commit(self, **kwargs: Any) -> None:
        with self._lock:
            self._context = replace(self._context, **kwargs, revision=self._context.revision + 1)
            self._condition.notify_all()

    def _submit(self, fn: Callable[[], T], *, closing: bool = False) -> T:
        with self._lock:
            if self._closed or (
                self._shutdown_drain_started
                or (self._shutdown_initiated and not self._owner.is_alive())
                or (
                    not closing
                    and (self._context.shutdown != ShutdownState.OPEN or self._gate != "open")
                )
            ):
                return cast(
                    "T",
                    CommandResult(
                        False, diagnostic="runtime is closing", snapshot=self._snapshot()
                    ),
                )
            call = _Call(fn)
            self._events.put(call)
        result = call.answer.get()
        if isinstance(result, BaseException):
            raise result
        return cast("T", result)

    def _run(self) -> None:
        while True:
            timeout = self._wait_timeout()
            try:
                event = self._events.get(timeout=timeout)
            except queue.Empty:
                with self._lock:
                    self._timers()
                continue
            if event is None:
                return
            try:
                with self._lock:
                    if isinstance(event, _Call):
                        self._timers()
                        event.answer.put(event.fn())
                    elif isinstance(event, EffectDone):
                        self._done(event)
                    elif isinstance(event, AuxiliaryDone):
                        self._aux_done(event)
                    else:
                        self._stop_done(event)
                        self._timers()
            except BaseException as exc:
                if isinstance(event, _Call):
                    event.answer.put(exc)

    def _wait_timeout(self) -> float:
        if self._clock_invalid:
            return 3600.0
        deadlines = [
            action.deadline
            for action in self._pending.values()
            if action.task.action is None or not action.task.action.deadline_reported
        ]
        deadlines.extend(self._auxiliary.deadlines())
        if self._next_poll is not None:
            deadlines.append(self._next_poll)
        if self._context.reset_deadline is not None:
            deadlines.append(self._context.reset_deadline)
        if self._context.shutdown_deadline is not None:
            deadlines.append(self._context.shutdown_deadline)
        if self._context.physical_operation_deadline is not None:
            deadlines.append(self._context.physical_operation_deadline)
        if not deadlines:
            return 3600.0
        now = self._clock_now()
        if now is None:
            return 3600.0
        return max(0.0, min(deadlines) - now)

    def _clock_failure_owner(self, diagnostic: str) -> None:
        self._gate = "sealed"
        self._shutdown_initiated = True
        op = self._context.active
        if op is not None:
            potentially_active = {
                TaskActivity.EXECUTE_UNRESOLVED,
                TaskActivity.ACTIVE,
                TaskActivity.UNKNOWN,
                TaskActivity.REMOTE_FAULT,
            }
            op = replace(
                op,
                tasks=tuple(
                    replace(
                        task,
                        cancel_required=task.cancel_required or task.activity in potentially_active,
                        reset_required=(task.reset_required or task.activity in potentially_active),
                    )
                    for task in op.tasks
                ),
                uncertain=True,
                diagnostic=diagnostic,
            )
        self._commit(
            state=LifecycleState.FAULT
            if self._context.state == LifecycleState.FAULT
            else LifecycleState.CANCELLING,
            shutdown=ShutdownState.CLOSING,
            shutdown_result=self._context.shutdown_result or ShutdownResult(False, diagnostic),
            active=op,
            diagnostic=diagnostic,
        )
        if op is not None:
            self._schedule_cancels(op, emergency=True)
        self._shutdown_progress()

    def _clock_now(self) -> float | None:
        if self._clock_invalid:
            return None
        try:
            return self._clock.now()
        except InvalidMonotonicClock:
            self._clock_invalid = True
            if threading.current_thread() is self._owner:
                self._clock_failure_owner("invalid monotonic clock")
            else:
                with self._lock:
                    if not self._shutdown_drain_started and self._owner.is_alive():
                        self._events.put(
                            _Call(lambda: self._clock_failure_owner("invalid monotonic clock"))
                        )
            return None

    def _timers(self) -> None:
        now = self._clock_now()
        if now is None:
            return
        for action_id, item in tuple(self._pending.items()):
            if item.deadline <= now:
                handle, task_name, task, method = (
                    item.handle,
                    item.task_name,
                    item.task,
                    item.method,
                )
                op = self._context.active
                report = False
                if op is not None and op.handle == handle:
                    current = next((t for t in op.tasks if t.task_id == task.task_id), None)
                    if (
                        current is not None
                        and current.action is not None
                        and not current.action.deadline_reported
                    ):
                        report = True
                        action = replace(current.action, deadline_reported=True)
                        tasks = tuple(
                            replace(
                                t,
                                action=action,
                                activity=TaskActivity.UNKNOWN,
                                cancel_required=True,
                            )
                            if t.task_id == task.task_id
                            else t
                            for t in op.tasks
                        )
                        self._commit(active=replace(op, tasks=tasks, uncertain=True))
                        self._pending[action_id] = replace(
                            item,
                            task=replace(task, action=action),
                        )
                if report:
                    self._fault(
                        f"coordinator {method.value} deadline for {task_name}",
                        handle,
                        task,
                    )
        if self._auxiliary.expire(now, "coordinator auxiliary action deadline exceeded"):
            self._condition.notify_all()
        physical_deadline = self._context.physical_operation_deadline
        if (
            physical_deadline is not None
            and now >= physical_deadline
            and self._context.active is not None
            and self._context.state == LifecycleState.RUNNING
        ):
            self._commit(physical_operation_deadline=None)
            self._begin_cancellation("physical operation deadline exceeded")
        if self._next_poll is not None and now >= self._next_poll:
            self._next_poll = now + self._poll_interval
            self._poll_active()
        if self._context.reset_deadline is not None and now >= self._context.reset_deadline:
            if self._context.reset_handle and self._context.reset_handle not in dict(
                self._context.reset_results
            ):
                self._finish_reset(False, "reset deadline exceeded")
        if self._context.shutdown_deadline is not None and now >= self._context.shutdown_deadline:
            if self._context.shutdown_result is None:
                self._finish_shutdown(False, "shutdown safety deadline exceeded")

    def _valid_plan(self, plan: PlanInput) -> bool:
        if not isinstance(plan, (ExecutionPlan, PreparedPlan)) or not plan.entries:
            return False
        if isinstance(plan, ExecutionPlan) and isinstance(plan.generated_plan, GeneratedPlan):
            return False
        if isinstance(plan, PreparedPlan) and (not plan._canonical or plan.topology is None):
            return False
        groups = tuple(getattr(plan.generated_plan, "group_ids", ()))
        names = tuple(e.planning_group for e in plan.entries)
        if len(set(names)) != len(names) or len({e.task_name for e in plan.entries}) != len(names):
            return False
        if groups and groups != names:
            return False
        topology = plan.topology if isinstance(plan, PreparedPlan) else self._topology
        if topology is not None:
            routes = {g: (r, t) for g, r, t in topology.routes}
            for entry in plan.entries:
                route = routes.get(entry.planning_group)
                if (
                    route is None
                    or route[1] != entry.task_name
                    or (entry.robot_name and route[0] != entry.robot_name)
                ):
                    return False
        return True

    def _valid(self, plan: PlanInput) -> bool:
        try:
            return bool(self._validator(plan) if self._validator else self._valid_plan(plan))
        except Exception:
            return False

    def _start_action(
        self,
        op: Operation,
        task: TaskRecord,
        method: ActionMethod,
        *,
        emergency: bool = False,
    ) -> None:
        if task.action is not None or (self._gate == "sealed" and not emergency):
            return
        if self._gate != "open" and not (
            method == ActionMethod.CANCEL
            and task.cancel_required
            and task.activity != TaskActivity.NOT_STARTED
            and (emergency or self._gate == "cancel_only")
        ):
            return
        if method not in (ActionMethod.STATUS, ActionMethod.RESET) and task.activity in (
            TaskActivity.INACTIVE,
            TaskActivity.COMPLETED,
            TaskActivity.CANCELLED,
        ):
            return
        now = self._clock_now()
        if now is None and method != ActionMethod.CANCEL:
            return
        action = ActionRecord(
            str(uuid.uuid4()),
            method,
            now or 0.0,
            float("inf") if now is None else now + self._action_timeout,
        )
        tasks = tuple(
            replace(
                t,
                action=action,
                activity=TaskActivity.EXECUTE_UNRESOLVED
                if method == ActionMethod.EXECUTE
                else t.activity,
            )
            if t.task_id == task.task_id
            else t
            for t in op.tasks
        )
        op = replace(op, tasks=tasks)
        self._commit(active=op)
        self._pending[action.action_id] = _PendingAction(
            handle=op.handle,
            task_name=task.task_name,
            task=next(t for t in tasks if t.task_id == task.task_id),
            method=method,
            deadline=action.deadline,
        )
        fn: Callable[[], Any]
        if method == ActionMethod.EXECUTE:
            fn = lambda: self._gateway.execute(task.task_name, task.entry.request)  # noqa: E731
        elif method == ActionMethod.CANCEL:
            fn = lambda: self._gateway.cancel(task.task_name)  # noqa: E731
        elif method == ActionMethod.STATUS:
            fn = lambda: self._gateway.status(task.task_name)  # noqa: E731
        else:
            fn = lambda: self._gateway.reset(task.task_name)  # noqa: E731
        self._effects.submit_action(action.action_id, fn, self._events.put)

    @dataclass(frozen=True)
    class _RetiredAction:
        operation: Operation
        index: int
        task: TaskRecord
        method: ActionMethod

    def _retire_action(self, done: EffectDone) -> _RetiredAction | None:
        item = self._pending.get(done.action_id)
        active = self._context.active
        if item is None or active is None:
            return None
        if active.handle != item.handle:
            return None
        index = next(
            (i for i, t in enumerate(active.tasks) if t.task_id == item.task.task_id), None
        )
        if index is None:
            return None
        current_task = active.tasks[index]
        if current_task.action is None or current_task.action.action_id != done.action_id:
            # The registered operation/task identity is sufficient to consume this
            # physical completion, but it must not retire a newer action.
            self._pending.pop(done.action_id, None)
            return None
        self._pending.pop(done.action_id, None)
        task = replace(active.tasks[index], action=None)
        operation = replace(active, tasks=(*active.tasks[:index], task, *active.tasks[index + 1 :]))
        self._commit(active=operation)
        return self._RetiredAction(operation, index, task, item.method)

    def _done(self, done: EffectDone) -> None:
        retired = self._retire_action(done)
        if retired is None:
            return
        op, index, task, method = (
            retired.operation,
            retired.index,
            retired.task,
            retired.method,
        )
        outcome = done.outcome if isinstance(done.outcome, Outcome) else Outcome.UNKNOWN
        if self._context.reset_handle is not None and method in (
            ActionMethod.STATUS,
            ActionMethod.CANCEL,
            ActionMethod.RESET,
        ):
            self._done_reset(op, index, task, method, outcome)
            return
        if method == ActionMethod.EXECUTE:
            self._done_execute(op, index, task, outcome)
            return
        if method == ActionMethod.CANCEL:
            self._done_cancel(op, index, task, outcome)
            return
        if method == ActionMethod.STATUS:
            self._done_status(op, index, task, outcome)
            return

    def _aux_done(self, done: AuxiliaryDone) -> None:
        self._auxiliary.complete(done)
        self._condition.notify_all()
        self._shutdown_progress()

    def _reset_next(self, op: Operation) -> None:
        reset = self._context.reset_handle
        if reset is None or reset in dict(self._context.reset_results):
            return
        if self._gate == "sealed":
            self._finish_reset(False, "reset unavailable while runtime is closing")
            self._shutdown_progress()
            return
        proven = self._context.reset_proven_inactive
        completed = self._context.reset_completed_tasks
        for task in op.tasks:
            if task.action is not None or task.activity == TaskActivity.NOT_STARTED:
                continue
            if task.task_id in completed:
                continue
            if task.task_id in proven:
                if task.reset_required:
                    self._start_action(op, task, ActionMethod.RESET)
                    return
                self._commit(reset_completed_tasks=frozenset((*completed, task.task_id)))
                completed = self._context.reset_completed_tasks
                continue
            self._start_action(op, task, ActionMethod.STATUS)
            return
        if all(t.task_id in completed or t.activity == TaskActivity.NOT_STARTED for t in op.tasks):
            self._finish_reset(True)

    def _done_reset(
        self,
        op: Operation,
        index: int,
        task: TaskRecord,
        method: ActionMethod,
        outcome: Outcome,
    ) -> None:
        current = op.tasks[index]
        decision = reconcile_reset_completion(
            method=method,
            outcome=outcome,
            activity=current.activity,
            gate_sealed=self._gate == "sealed",
            clock_failed=self._clock_invalid,
        )
        current = replace(
            current,
            activity=decision.activity or current.activity,
            cancel_required=(
                current.cancel_required
                if decision.cancel_required is None
                else decision.cancel_required
            ),
            reset_required=(
                current.reset_required
                if decision.reset_required is None
                else decision.reset_required
            ),
        )
        self._commit(active=replace(op, tasks=(*op.tasks[:index], current, *op.tasks[index + 1 :])))
        if decision.prove_inactive:
            self._commit(
                reset_proven_inactive=frozenset(
                    (*self._context.reset_proven_inactive, current.task_id)
                )
            )
        if decision.complete_reset:
            self._commit(
                reset_completed_tasks=frozenset(
                    (*self._context.reset_completed_tasks, current.task_id)
                )
            )
        diagnostic = decision.diagnostic or (
            f"task={current.task_name} action={method.value} outcome={outcome.value}"
        )
        if decision.reset_success is not None:
            self._finish_reset(decision.reset_success, diagnostic)
        if decision.request_cancel:
            self._schedule_cancels(
                self._context.active or op,
                emergency=decision.emergency_cancel,
            )
        if decision.advance_reset:
            self._reset_next(self._context.active or op)
        self._shutdown_progress()

    def _record_dispatch(self, op: Operation, outcome: Outcome, diagnostic: str = "") -> None:
        if dict(self._context.dispatch_results).get(op.handle) is None:
            self._commit(
                dispatch_results=(
                    *self._context.dispatch_results,
                    (op.handle, ExecutionResult(op.handle, outcome, diagnostic)),
                )[-16:]
            )

    def _begin_cancellation(self, diagnostic: str = "") -> None:
        op = self._context.active
        if op is None:
            return
        tasks = tuple(
            replace(
                t,
                cancel_required=(
                    t.cancel_required
                    or t.activity
                    in (
                        TaskActivity.EXECUTE_UNRESOLVED,
                        TaskActivity.ACTIVE,
                        TaskActivity.UNKNOWN,
                        TaskActivity.REMOTE_FAULT,
                    )
                ),
            )
            for t in op.tasks
        )
        op = replace(
            op,
            tasks=tasks,
            cancel_requested=True,
            failed=op.failed or diagnostic == "physical operation deadline exceeded",
            diagnostic=diagnostic or op.diagnostic,
        )
        self._commit(state=LifecycleState.CANCELLING, active=op)
        self._schedule_cancels(op)

    def _done_execute(self, op: Operation, index: int, task: TaskRecord, outcome: Outcome) -> None:
        if self._context.shutdown == ShutdownState.CLOSING:
            op = replace(op, cancel_requested=True)
        decision = reconcile_execute_completion(outcome)
        if decision.fault:
            diagnostic = f"task={task.task_name} action=execute outcome={outcome.value}"
            self._record_dispatch(op, Outcome.UNKNOWN, diagnostic)
            self._fault(
                diagnostic,
                op.handle,
                replace(task, action=None),
            )
            return
        if decision.rejected:
            tasks = tuple(
                replace(
                    t,
                    action=None,
                    activity=TaskActivity.INACTIVE if i >= index else t.activity,
                    cancel_required=(t.cancel_required or i < index) if i < index else False,
                )
                for i, t in enumerate(op.tasks)
            )
            op = replace(op, tasks=tasks, rejected=True, cancel_requested=True)
            diagnostic = f"task={task.task_name} action=execute outcome={outcome.value}"
            op = replace(op, diagnostic=diagnostic)
            if self._context.state == LifecycleState.FAULT or op.uncertain:
                self._commit(active=op)
            else:
                self._commit(state=LifecycleState.CANCELLING, active=op)
            self._record_dispatch(op, Outcome.REJECTED, diagnostic)
            self._schedule_cancels(op)
            self._finish_if_terminal(op)
            return
        activity = TaskActivity.ACTIVE
        tasks = tuple(
            replace(
                t,
                action=None,
                activity=activity,
                cancel_required=(
                    t.cancel_required
                    or op.cancel_requested
                    or self._context.state == LifecycleState.FAULT
                ),
            )
            if t.task_id == task.task_id
            else t
            for t in op.tasks
        )
        op = replace(op, tasks=tasks)
        self._commit(active=op)
        if op.cancel_requested or self._context.state in (
            LifecycleState.CANCELLING,
            LifecycleState.FAULT,
        ):
            self._record_dispatch(op, Outcome.CANCELLED)
            self._schedule_cancels(op)
            self._shutdown_progress()
            return
        if index != op.next_index:
            self._fault("out-of-order execute completion", op.handle, task)
            return
        if index + 1 < len(op.tasks):
            op = replace(op, next_index=index + 1)
            self._commit(active=op)
            self._start_action(op, op.tasks[index + 1], ActionMethod.EXECUTE)
        else:
            self._record_dispatch(op, Outcome.ACCEPTED)
            now = self._clock_now()
            if now is None:
                return
            self._commit(
                state=LifecycleState.RUNNING,
                physical_operation_deadline=now + self._physical_timeout,
            )
            self._next_poll = now + self._poll_interval

    def _done_cancel(self, op: Operation, index: int, task: TaskRecord, outcome: Outcome) -> None:
        decision = reconcile_cancel_completion(outcome)
        if decision.fault and outcome in (Outcome.UNKNOWN, Outcome.FAILED):
            diagnostic = decision.diagnostic or "cancellation failed"
            self._fault(
                diagnostic,
                op.handle,
                replace(task, action=None),
                schedule_cancels=False,
                mark_peers=False,
            )
            current = self._context.active
            if current is not None and current.handle == op.handle:
                self._commit(
                    active=replace(
                        current,
                        tasks=tuple(
                            replace(
                                item,
                                action=None,
                                cancel_required=False,
                                reset_required=True,
                            )
                            if item.task_id == task.task_id
                            else item
                            for item in current.tasks
                        ),
                    )
                )
                current = self._context.active
                if current is not None:
                    if self._context.shutdown == ShutdownState.CLOSING:
                        self._finish_shutdown(False, diagnostic)
                    self._schedule_cancels(current)
                    self._shutdown_progress()
            return
        if decision.fault:
            diagnostic = f"malformed cancel outcome for {task.task_name}"
            self._fault(
                diagnostic,
                op.handle,
                replace(task, action=None),
                schedule_cancels=False,
                mark_peers=False,
            )
            current = self._context.active
            if current is not None and current.handle == op.handle:
                current = replace(
                    current,
                    tasks=tuple(
                        replace(item, action=None, cancel_required=False)
                        if item.task_id == task.task_id
                        else item
                        for item in current.tasks
                    ),
                )
                self._commit(active=current)
                if self._context.shutdown == ShutdownState.CLOSING:
                    self._finish_shutdown(False, diagnostic)
                self._schedule_cancels(current)
                self._shutdown_progress()
            return
        tasks = tuple(
            replace(t, action=None, activity=TaskActivity.CANCELLED, cancel_required=False)
            if t.task_id == task.task_id
            else t
            for t in op.tasks
        )
        op = replace(op, tasks=tasks)
        self._commit(active=op)
        self._schedule_cancels(op)
        self._finish_if_terminal(op)
        self._shutdown_progress()

    def _done_status(self, op: Operation, index: int, task: TaskRecord, outcome: Outcome) -> None:
        decision = reconcile_status_completion(outcome)
        if decision.fault:
            self._fault(
                f"status is unsafe: task={task.task_name} action=get_state outcome={outcome.value}",
                op.handle,
                replace(task, action=None),
            )
            return
        activity = cast("TaskActivity", decision.activity)
        tasks = tuple(
            replace(
                t,
                action=None,
                activity=activity,
                cancel_required=False
                if t.task_id == task.task_id
                and activity
                in (TaskActivity.INACTIVE, TaskActivity.CANCELLED, TaskActivity.COMPLETED)
                else t.cancel_required,
            )
            if t.task_id == task.task_id
            else t
            for t in op.tasks
        )
        op = replace(op, tasks=tasks)
        if activity in (TaskActivity.INACTIVE, TaskActivity.CANCELLED):
            op = replace(
                op,
                cancel_requested=True,
                failed=True,
                diagnostic=(
                    f"task={task.task_name} outcome={outcome.value}"
                    if activity in (TaskActivity.INACTIVE, TaskActivity.CANCELLED)
                    else op.diagnostic
                ),
                tasks=tuple(
                    replace(
                        t,
                        cancel_required=(
                            False
                            if t.task_id == task.task_id
                            else t.cancel_required
                            or t.activity
                            in (
                                TaskActivity.EXECUTE_UNRESOLVED,
                                TaskActivity.ACTIVE,
                                TaskActivity.UNKNOWN,
                                TaskActivity.REMOTE_FAULT,
                            )
                        ),
                    )
                    for t in op.tasks
                ),
            )
            self._commit(state=LifecycleState.CANCELLING, active=op)
            self._schedule_cancels(op)
            self._finish_if_terminal(op)
            return
        self._commit(active=op)
        self._finish_if_terminal(op)
        if self._context.state in (LifecycleState.CANCELLING, LifecycleState.FAULT):
            self._schedule_cancels(op)

    def _poll_active(self) -> None:
        op = self._context.active
        if op is not None and self._context.state == LifecycleState.RUNNING:
            task = next(
                (t for t in op.tasks if t.activity == TaskActivity.ACTIVE and t.action is None),
                None,
            )
            if task:
                self._start_action(op, task, ActionMethod.STATUS)

    def _schedule_cancels(self, op: Operation, *, emergency: bool = False) -> None:
        for task in op.tasks:
            current = self._context.active
            if current is None or current.handle != op.handle:
                return
            task = next(t for t in current.tasks if t.task_id == task.task_id)
            if task.action is None and task.cancel_required:
                self._start_action(
                    current, task, ActionMethod.CANCEL, emergency=emergency or self._clock_invalid
                )

    def _finish_if_terminal(self, op: Operation) -> None:
        if any(t.action is not None for t in op.tasks):
            return
        if self._context.state == LifecycleState.CANCELLING:
            if all(
                t.activity
                in (
                    TaskActivity.NOT_STARTED,
                    TaskActivity.CANCELLED,
                    TaskActivity.INACTIVE,
                    TaskActivity.COMPLETED,
                )
                and not t.cancel_required
                for t in op.tasks
            ):
                if self._context.shutdown == ShutdownState.CLOSING:
                    self._shutdown_progress()
                else:
                    self._terminal(
                        Outcome.REJECTED
                        if op.rejected and not op.failed
                        else Outcome.FAILED
                        if op.failed
                        else Outcome.CANCELLED
                    )
        elif all(t.activity == TaskActivity.COMPLETED for t in op.tasks):
            self._terminal(Outcome.COMPLETED)

    def _terminal(self, outcome: Outcome) -> None:
        op = self._context.active
        if op is None:
            return
        result = ExecutionResult(op.handle, outcome, op.diagnostic)
        self._commit(
            state=LifecycleState.IDLE,
            active=None,
            diagnostic=op.diagnostic,
            terminal_results=(*self._context.terminal_results, (op.handle, result))[-16:],
        )

    def _fault(
        self,
        diagnostic: str,
        handle: OperationHandle | None = None,
        task: TaskRecord | None = None,
        *,
        schedule_cancels: bool = True,
        mark_peers: bool = True,
    ) -> None:
        op = self._context.active
        if op is not None:
            tasks = tuple(
                replace(
                    t,
                    action=None if task is not None and task.action is None else t.action,
                    activity=TaskActivity.UNKNOWN,
                    cancel_required=(
                        t.cancel_required
                        or t.activity
                        in (
                            TaskActivity.EXECUTE_UNRESOLVED,
                            TaskActivity.ACTIVE,
                            TaskActivity.UNKNOWN,
                            TaskActivity.REMOTE_FAULT,
                        )
                    ),
                    reset_required=(
                        t.reset_required
                        or t.activity
                        in (
                            TaskActivity.EXECUTE_UNRESOLVED,
                            TaskActivity.ACTIVE,
                            TaskActivity.UNKNOWN,
                            TaskActivity.REMOTE_FAULT,
                        )
                    ),
                )
                if (
                    mark_peers
                    and t.activity
                    in (
                        TaskActivity.EXECUTE_UNRESOLVED,
                        TaskActivity.ACTIVE,
                        TaskActivity.UNKNOWN,
                        TaskActivity.REMOTE_FAULT,
                    )
                )
                or (task is not None and t.task_id == task.task_id)
                else t
                for t in op.tasks
            )
            op = replace(op, tasks=tasks, uncertain=True, diagnostic=diagnostic)
        self._commit(
            state=LifecycleState.FAULT,
            fault=self._context.fault or diagnostic,
            diagnostic=diagnostic,
            active=op,
        )
        fault_handle = handle or (op.handle if op is not None else None)
        if fault_handle is not None:
            self._fault_terminal_results[fault_handle] = diagnostic
            if len(self._fault_terminal_results) > 8:
                oldest = next(iter(self._fault_terminal_results))
                del self._fault_terminal_results[oldest]
        if handle and not dict(self._context.dispatch_results).get(handle):
            self._commit(
                dispatch_results=(
                    *self._context.dispatch_results,
                    (handle, ExecutionResult(handle, Outcome.UNKNOWN, diagnostic)),
                )[-16:]
            )
        if op and schedule_cancels:
            self._schedule_cancels(op)

    def _stop_done(self, done: StopDone) -> None:
        self._shutdown_stop_finished = True
        if self._context.shutdown_result is not None:
            self._shutdown_drain_owner()
            return
        if not done.success:
            self._finish_shutdown(
                False,
                f"gateway close failed: {done.diagnostic or 'stop failed'}",
            )
            self._shutdown_drain_owner()
            return
        deadline = self._context.shutdown_deadline
        now = self._clock_now()
        if now is None:
            return
        if deadline is None or now >= deadline:
            self._finish_shutdown(False, "shutdown safety deadline exceeded")
            self._shutdown_drain_owner()
            return
        self._commit(
            state=LifecycleState.IDLE,
            shutdown=ShutdownState.CLOSED,
            shutdown_result=ShutdownResult(True, done.diagnostic),
            shutdown_deadline=None,
        )
        self._shutdown_drain_owner()

    def _has_unsettled_shutdown_work(self) -> bool:
        active = self._context.active
        return bool(
            self._pending
            or self._auxiliary.has_unsettled()
            or (
                active is not None
                and any(t.action is not None or t.cancel_required for t in active.tasks)
            )
        )

    def _shutdown_progress(self) -> None:
        if self._context.shutdown != ShutdownState.CLOSING:
            return
        if self._has_unsettled_shutdown_work():
            return
        if self._shutdown_stop_started:
            return
        self._shutdown_stop_started = True
        self._gate = "sealed"
        self._effects.submit_stop(self._gateway.stop, self._events.put)

    def _shutdown_drain_owner(self) -> None:
        if self._shutdown_drain_started:
            return
        if self._has_unsettled_shutdown_work():
            return
        self._shutdown_drain_started = True
        self._effects.shutdown()
        self._events.put(None)

    def _finish_reset(self, success: bool, diagnostic: str = "") -> None:
        h = self._context.reset_handle
        if h is None or h in dict(self._context.reset_results):
            return
        result = ResetResult(h, success, diagnostic)
        self._commit(
            state=LifecycleState.IDLE if success else LifecycleState.FAULT,
            reset_results=(*self._context.reset_results, (h, result))[-16:],
            reset_handle=None if success else h,
            reset_deadline=None,
            fault=None if success else self._context.fault or diagnostic,
            active=None if success else self._context.active,
            physical_operation_deadline=None
            if success
            else self._context.physical_operation_deadline,
            ready_plan=None if success else self._context.ready_plan,
            ready_plan_id=None if success else self._context.ready_plan_id,
            diagnostic=diagnostic,
        )

    def _finish_shutdown(self, success: bool, diagnostic: str = "") -> None:
        if self._context.shutdown_result is None:
            self._commit(
                shutdown_result=ShutdownResult(success, diagnostic),
                shutdown_deadline=None if not success else self._context.shutdown_deadline,
            )

    def start_planning(self) -> str | None:
        def fn() -> str | None:
            if self._context.state not in (LifecycleState.IDLE, LifecycleState.READY):
                return None
            token = str(uuid.uuid4())
            self._commit(
                state=LifecycleState.PLANNING,
                planning_token=token,
                ready_plan=None,
                ready_plan_id=None,
                diagnostic=None,
            )
            return token

        return self._submit(fn)

    def complete_planning(self, token: str, plan: PlanInput) -> CommandResult[str]:
        def fn() -> CommandResult[str]:
            if token != self._context.planning_token:
                return CommandResult(
                    False, diagnostic="planning token mismatch", snapshot=self._snapshot()
                )
            if not self._valid(plan):
                self._commit(
                    state=LifecycleState.IDLE,
                    planning_token=None,
                    diagnostic="invalid prepared plan",
                )
                return CommandResult(
                    False, diagnostic="invalid prepared plan", snapshot=self._snapshot()
                )
            pid = str(uuid.uuid4())
            self._commit(
                state=LifecycleState.READY, ready_plan=plan, ready_plan_id=pid, planning_token=None
            )
            return CommandResult(True, pid, snapshot=self._snapshot())

        return self._submit(fn)

    def fail_planning(self, token: str, diagnostic: str) -> CommandResult[None]:
        def fn() -> CommandResult[None]:
            ok = token == self._context.planning_token
            if ok:
                self._commit(state=LifecycleState.IDLE, planning_token=None, diagnostic=diagnostic)
            return CommandResult(
                ok,
                diagnostic=diagnostic if ok else "planning token mismatch",
                snapshot=self._snapshot(),
            )

        return self._submit(fn)

    def cancel_planning(self) -> CommandResult[None]:
        return self._submit(lambda: self._planning_command(LifecycleState.PLANNING, "cancelled"))

    def _planning_command(self, state: LifecycleState, diagnostic: str) -> CommandResult[None]:
        ok = self._context.state == state
        if ok:
            self._commit(state=LifecycleState.IDLE, planning_token=None, diagnostic=diagnostic)
        return CommandResult(ok, snapshot=self._snapshot())

    def clear_ready_plan(self) -> CommandResult[None]:
        def fn() -> CommandResult[None]:
            if self._context.state != LifecycleState.READY:
                return CommandResult(False, snapshot=self._snapshot())
            self._commit(
                state=LifecycleState.IDLE,
                ready_plan=None,
                ready_plan_id=None,
                diagnostic="",
            )
            return CommandResult(True, snapshot=self._snapshot())

        return self._submit(fn)

    def _execute(
        self, plan: PlanInput, plan_id: str | None = None
    ) -> CommandResult[OperationHandle]:
        if self._context.state not in (
            LifecycleState.IDLE,
            LifecycleState.READY,
        ) or not self._valid(plan):
            return CommandResult(
                False, diagnostic="execution unavailable or invalid plan", snapshot=self._snapshot()
            )
        h = OperationHandle(plan_id or str(uuid.uuid4()), str(uuid.uuid4()), str(uuid.uuid4()))
        op = Operation(
            h,
            plan,
            tuple(
                TaskRecord(str(uuid.uuid4()), e.task_name, e, TaskActivity.NOT_STARTED)
                for e in plan.entries
            ),
        )
        self._commit(
            state=LifecycleState.DISPATCHING,
            ready_plan=None,
            ready_plan_id=None,
            active=op,
            diagnostic=None,
        )
        self._start_action(op, op.tasks[0], ActionMethod.EXECUTE)
        return CommandResult(True, h, snapshot=self._snapshot())

    def execute_explicit(self, prepared_plan: PlanInput) -> CommandResult[OperationHandle]:
        return self._submit(lambda: self._execute(prepared_plan))

    def execute_ready(self) -> CommandResult[OperationHandle]:
        return self._submit(
            lambda: self._execute(self._context.ready_plan, self._context.ready_plan_id)
            if self._context.ready_plan is not None
            else CommandResult(False, diagnostic="no ready plan", snapshot=self._snapshot())
        )

    def cancel(self) -> CommandResult[None]:
        return self._submit(self._cancel_owner, closing=True)

    def _cancel_owner(self) -> CommandResult[None]:
        op = self._context.active
        if op is None or self._context.state not in (
            LifecycleState.DISPATCHING,
            LifecycleState.RUNNING,
        ):
            return CommandResult(False, snapshot=self._snapshot())
        self._begin_cancellation()
        return CommandResult(True, snapshot=self._snapshot())

    def cancel_if_current(self, handle: OperationHandle) -> CommandResult[None]:
        return self._submit(
            lambda: self._cancel_owner()
            if self._context.active and self._context.active.handle == handle
            else CommandResult(
                False, diagnostic="operation is no longer current", snapshot=self._snapshot()
            ),
            closing=True,
        )

    def poll(self) -> CommandResult[None]:
        def fn() -> CommandResult[None]:
            self._poll_active()
            return CommandResult(
                self._context.state == LifecycleState.RUNNING,
                snapshot=self._snapshot(),
            )

        return self._submit(fn)

    def set_gripper_position(self, hardware_id: str, position: float) -> CommandResult[Outcome]:
        result = self._submit(lambda: self._gripper(True, hardware_id, position))
        if isinstance(result, CommandResult):
            return cast("CommandResult[Outcome]", result)
        return cast("CommandResult[Outcome]", self._wait_aux(result))

    def get_gripper_position(self, hardware_id: str) -> CommandResult[float]:
        result = self._submit(lambda: self._gripper(False, hardware_id, None))
        if isinstance(result, CommandResult):
            return cast("CommandResult[float]", result)
        return cast("CommandResult[float]", self._wait_aux(result))

    def _gripper(
        self, setter: bool, hardware: str, position: float | None
    ) -> AuxiliaryTicket | CommandResult[Any]:
        if self._gate != "open":
            return CommandResult(False, diagnostic="gateway is closing", snapshot=self._snapshot())
        if setter and position is None:
            return CommandResult(
                False, diagnostic="position is required", snapshot=self._snapshot()
            )
        now = self._clock_now()
        if now is None:
            return CommandResult(
                False, diagnostic="invalid monotonic clock", snapshot=self._snapshot()
            )
        action_id = str(uuid.uuid4())
        ticket = self._auxiliary.register(action_id, now + self._action_timeout, setter)
        if setter:
            assert position is not None
            fn: Callable[[], Any] = lambda: self._gateway.set_gripper_position(hardware, position)  # noqa: E731
        else:
            fn = lambda: self._gateway.get_gripper_position(hardware)  # noqa: E731
        self._effects.submit_auxiliary(action_id, fn, self._events.put)
        return ticket

    def _wait_aux(self, ticket: AuxiliaryTicket) -> CommandResult[Any]:
        start = self._clock_now()
        if start is None:
            return CommandResult(
                False, diagnostic="invalid monotonic clock", snapshot=self._snapshot()
            )
        end = start + self._action_timeout
        with self._condition:
            while True:
                result = self._auxiliary.take_result(ticket.action_id)
                if result is not None:
                    accepted, value, diagnostic = result
                    if ticket.setter:
                        return CommandResult(
                            accepted and isinstance(value, Outcome),
                            value if isinstance(value, Outcome) else None,
                            diagnostic=diagnostic,
                            snapshot=self._snapshot(),
                        )
                    valid = isinstance(value, (int, float)) and not isinstance(value, bool)
                    return CommandResult(
                        accepted and valid,
                        float(value) if valid else None,
                        diagnostic=diagnostic,
                        snapshot=self._snapshot(),
                    )
                now = self._clock_now()
                if now is None:
                    return CommandResult(
                        False, diagnostic="invalid monotonic clock", snapshot=self._snapshot()
                    )
                if now >= end:
                    return CommandResult(
                        False,
                        diagnostic="coordinator auxiliary action timeout",
                        snapshot=self._snapshot(),
                    )
                self._condition.wait(max(0.0, end - now))

    def _wait(self, handle: Any, kind: str, timeout: float) -> CommandResult[Any]:
        end = time.monotonic() + max(0.0, timeout)
        with self._condition:
            while time.monotonic() < end:
                values = (
                    self._context.dispatch_results
                    if kind == "dispatch"
                    else self._context.terminal_results
                    if kind == "terminal"
                    else self._context.reset_results
                )
                found = dict(values).get(handle)
                if kind == "terminal":
                    fault_diagnostic = self._fault_terminal_results.get(handle)
                    if fault_diagnostic is not None:
                        return CommandResult(
                            False,
                            diagnostic=fault_diagnostic,
                            snapshot=self._snapshot(),
                        )
                if found is not None:
                    return CommandResult(True, found, snapshot=self._snapshot())
                if kind == "terminal":
                    active = self._context.active
                    if (
                        active is not None
                        and active.handle == handle
                        and self._context.state == LifecycleState.FAULT
                    ):
                        return CommandResult(
                            False,
                            diagnostic=self._context.diagnostic or "runtime fault",
                            snapshot=self._snapshot(),
                        )
                remaining = end - time.monotonic()
                self._condition.wait(min(0.1, max(0.0, remaining)))
        return CommandResult(False, diagnostic="timeout", snapshot=self._snapshot())

    def wait_for_dispatch(
        self, handle: OperationHandle, timeout: float = 1.0
    ) -> CommandResult[ExecutionResult]:
        return self._wait(handle, "dispatch", timeout)

    def wait_for_terminal(
        self, handle: OperationHandle, timeout: float = 1.0
    ) -> CommandResult[ExecutionResult]:
        return self._wait(handle, "terminal", timeout)

    def reset(self) -> CommandResult[ResetHandle]:
        def fn() -> CommandResult[ResetHandle]:
            if self._gate != "open":
                return CommandResult(
                    False,
                    diagnostic="reset unavailable while runtime is closing",
                    snapshot=self._snapshot(),
                )
            existing = self._context.reset_handle
            if existing is not None:
                prior = dict(self._context.reset_results).get(existing)
                if prior is not None:
                    return CommandResult(
                        False,
                        existing,
                        diagnostic=prior.diagnostic,
                        snapshot=self._snapshot(),
                    )
                return CommandResult(True, existing, snapshot=self._snapshot())
            if self._context.state != LifecycleState.FAULT:
                return CommandResult(
                    False, diagnostic="reset requires FAULT", snapshot=self._snapshot()
                )
            if self._pending:
                return CommandResult(
                    False,
                    diagnostic="reset blocked by unresolved coordinator RPC",
                    snapshot=self._snapshot(),
                )
            now = self._clock_now()
            if now is None:
                return CommandResult(
                    False, diagnostic="invalid monotonic clock", snapshot=self._snapshot()
                )
            h = ResetHandle(str(uuid.uuid4()))
            self._commit(
                reset_handle=h,
                reset_deadline=now + self._action_timeout,
                reset_proven_inactive=frozenset(),
                reset_completed_tasks=frozenset(),
            )
            op = self._context.active
            if op:
                self._reset_next(op)
            else:
                self._finish_reset(True)
            return CommandResult(True, h, snapshot=self._snapshot())

        return self._submit(fn, closing=True)

    def wait_for_reset(
        self, handle: ResetHandle, timeout: float = 1.0
    ) -> CommandResult[ResetResult]:
        return self._wait(handle, "reset", timeout)

    def shutdown(self, timeout: float = 1.0) -> CommandResult[ShutdownResult]:
        with self._shutdown_lock:
            with self._lock:
                if self._context.shutdown_result is not None:
                    return CommandResult(
                        self._context.shutdown_result.success,
                        self._context.shutdown_result,
                        snapshot=self._snapshot(),
                    )
                if self._shutdown_initiated:
                    return CommandResult(
                        False,
                        diagnostic="shutdown already initiated",
                        snapshot=self._snapshot(),
                    )
                self._shutdown_initiated = True
                self._gate = "cancel_only"
        return self._shutdown_wait(timeout)

    def _shutdown_wait(self, timeout: float) -> CommandResult[ShutdownResult]:
        start = self._clock_now()
        if start is None:
            end = time.monotonic() + 1.0
            with self._condition:
                while self._context.shutdown_result is None and time.monotonic() < end:
                    self._condition.wait(0.01)
                result = self._context.shutdown_result
            if result is None:
                return CommandResult(
                    False, diagnostic="invalid monotonic clock", snapshot=self.snapshot()
                )
            return CommandResult(False, result, snapshot=self.snapshot())
        deadline = start + max(0.0, timeout)
        self._submit(lambda: self._shutdown_owner(deadline), closing=True)
        while True:
            with self._condition:
                if self._context.shutdown_result is not None:
                    break
            now = self._clock_now()
            if now is None or now >= deadline:
                break
            with self._condition:
                self._condition.wait(min(0.1, max(0.0, deadline - now)))
        if self._context.shutdown_result is None:
            self._submit(
                lambda: self._finish_shutdown(False, "shutdown safety deadline exceeded"),
                closing=True,
            )
        result = self._context.shutdown_result
        if result is None:
            return CommandResult(
                False,
                diagnostic="shutdown safety deadline exceeded",
                snapshot=self.snapshot(),
            )
        if not result.success:
            return CommandResult(False, result, snapshot=self.snapshot())
        self._owner.join()
        if self._owner.is_alive():
            return CommandResult(
                False,
                diagnostic="owner thread did not stop",
                snapshot=self.snapshot(),
            )
        self._closed = True
        return CommandResult(True, result, snapshot=self.snapshot())

    close = shutdown

    def _shutdown_owner(self, deadline: float) -> CommandResult[None]:
        self._commit(
            shutdown=ShutdownState.CLOSING,
            shutdown_deadline=deadline,
            state=LifecycleState.CANCELLING if self._context.active else LifecycleState.IDLE,
        )
        if self._context.active:
            self._begin_cancellation("shutdown cancellation requested")
        self._shutdown_progress()
        return CommandResult(True, snapshot=self._snapshot())
