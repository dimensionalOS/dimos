"""Focused owner/reconciliation tests."""

from dataclasses import dataclass, replace
from pathlib import Path
import threading
import time
from typing import Any

import pytest

from dimos.manipulation.execution_runtime import (
    ActionMethod,
    ControlCoordinatorGateway,
    ExecutionPlan,
    ExecutionRuntime,
    ExecutionTopology,
    LifecycleState,
    Outcome,
    PreparedPlan,
    ResetHandle,
    ResetResult,
    ShutdownState,
    TaskActivity,
    TaskEntry,
    _Call,
    _Event,
    _GatewayGateState,
    prepare_generated_plan,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState


@dataclass
class FakeRPC:
    values: dict[tuple[str, str], Any]
    calls: list[tuple[str, str, dict[str, Any]]]
    entered: threading.Event | None = None
    release: threading.Event | None = None

    def task_invoke(self, task: str, method: str, kwargs: dict[str, Any]) -> Any:
        self.calls.append((task, method, kwargs))
        if method == "execute" and self.entered is not None:
            self.entered.set()
            assert self.release is not None and self.release.wait(2)
        return self.values.get((task, method))

    def stop_rpc_client(self) -> None:
        self.calls.append(("", "stop", {}))

    def set_gripper_position(self, hardware_id: str, position: float) -> Any:
        self.calls.append((hardware_id, "set_gripper_position", {"position": position}))
        return self.values.get((hardware_id, "set_gripper_position"), True)

    def get_gripper_position(self, hardware_id: str) -> Any:
        self.calls.append((hardware_id, "get_gripper_position", {}))
        return self.values.get((hardware_id, "get_gripper_position"), 0.5)


class BlockingGateway:
    """Deterministic per-task barriers for effect/correlation assertions."""

    def __init__(
        self,
        executes: dict[str, Outcome],
        cancels: dict[str, Outcome] | None = None,
        *,
        block_execute: bool = False,
    ) -> None:
        self.executes = executes
        self.cancels = cancels or {task: Outcome.CANCELLED for task in executes}
        self.block_execute = block_execute
        self.calls: list[tuple[str, ActionMethod]] = []
        self.entered = {task: threading.Event() for task in executes}
        self.execute_entered = {task: threading.Event() for task in executes}
        self.cancel_entered = {task: threading.Event() for task in executes}
        self.release = {task: threading.Event() for task in executes}
        self._lock = threading.Lock()
        self.stop_calls = 0
        self.calls_at_stop = 0

    def _call(self, task: str, method: ActionMethod, outcome: Outcome) -> Outcome:
        with self._lock:
            self.calls.append((task, method))
        if method == ActionMethod.CANCEL:
            self.entered[task].set()
            self.cancel_entered[task].set()
            assert self.release[task].wait(2)
        return outcome

    def execute(self, task_name: str, request: Any) -> Outcome:
        self.execute_entered[task_name].set()
        if self.block_execute:
            assert self.release[task_name].wait(2)
        return self._call(task_name, ActionMethod.EXECUTE, self.executes[task_name])

    def cancel(self, task_name: str) -> Outcome:
        return self._call(task_name, ActionMethod.CANCEL, self.cancels[task_name])

    def status(self, task_name: str) -> Outcome:
        return Outcome.UNKNOWN

    def reset(self, task_name: str) -> Outcome:
        return self._call(task_name, ActionMethod.RESET, Outcome.INACTIVE)

    def set_gripper_position(self, hardware_id: str, position: float) -> Outcome:
        return self._call(hardware_id, ActionMethod.GRIPPER_SET, Outcome.ACCEPTED)

    def get_gripper_position(self, hardware_id: str) -> float:
        with self._lock:
            self.calls.append((hardware_id, ActionMethod.GRIPPER_GET))
        return 0.25

    def stop(self) -> None:
        with self._lock:
            self.stop_calls += 1
            self.calls_at_stop = len(self.calls)
        for event in self.release.values():
            event.set()

    def count(self, task: str, method: ActionMethod) -> int:
        with self._lock:
            return self.calls.count((task, method))


class BlockingAuxGateway(BlockingGateway):
    def __init__(self) -> None:
        super().__init__({"gripper": Outcome.ACCEPTED})
        self.aux_entered = threading.Event()
        self.aux_release = threading.Event()

    def set_gripper_position(self, hardware_id: str, position: float) -> Outcome:
        with self._lock:
            self.calls.append((hardware_id, ActionMethod.GRIPPER_SET))
        self.aux_entered.set()
        assert self.aux_release.wait(2)
        return Outcome.ACCEPTED


def plan(*names: str) -> ExecutionPlan:
    return ExecutionPlan(
        "generated", tuple(TaskEntry(name, name, {"trajectory": name}) for name in names)
    )


def wait_until(fn: Any, timeout: float = 1.0) -> None:
    end = time.monotonic() + timeout
    while time.monotonic() < end and not fn():
        time.sleep(0.01)
    assert fn()


def task_is(runtime: ExecutionRuntime, index: int, activity: TaskActivity) -> bool:
    operation = runtime.snapshot().operation
    return operation is not None and operation.tasks[index].activity == activity


def test_gateway_normalizes_task_invoke_and_real_states() -> None:
    rpc = FakeRPC(
        {
            ("a", "execute"): True,
            ("a", "cancel"): False,
            ("a", "get_state"): TrajectoryState.EXECUTING,
        },
        [],
    )
    gateway = ControlCoordinatorGateway(rpc)
    assert gateway.execute("a", {}) == Outcome.ACCEPTED
    assert gateway.cancel("a") == Outcome.INACTIVE
    assert gateway.status("a") == Outcome.RUNNING


def test_aggregate_dispatch_pre_registers_and_waits_for_each_acceptance() -> None:
    rpc = FakeRPC({("a", "execute"): True, ("b", "execute"): True}, [])
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        result = runtime.execute_explicit(plan("a", "b"))
        assert result.accepted

        def dispatch_processed() -> bool:
            snapshot = runtime.snapshot()
            return (
                len([call for call in rpc.calls if call[1] == "execute"]) == 2
                and snapshot.state == LifecycleState.RUNNING
                and snapshot.operation is not None
                and all(task.activity == TaskActivity.ACTIVE for task in snapshot.operation.tasks)
            )

        wait_until(dispatch_processed)
        assert runtime.snapshot().state == LifecycleState.RUNNING
        operation = runtime.snapshot().operation
        assert operation is not None
        assert len(operation.tasks) == 2
    finally:
        runtime.shutdown()


def test_concurrent_execute_admission_is_serial_and_snapshot_coherent() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    barrier = threading.Barrier(3)
    results: list[Any] = []

    def submit() -> None:
        barrier.wait()
        results.append(runtime.execute_explicit(plan("a")))

    threads = [threading.Thread(target=submit) for _ in range(2)]
    try:
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(1)
        assert len(results) == 2
        assert sum(result.accepted for result in results) == 1
        rejected = next(result for result in results if not result.accepted)
        assert rejected.value is not None
        retained = dict(runtime._context.terminal_results)[rejected.value]
        assert retained.outcome == Outcome.REJECTED
        assert runtime.snapshot().operation is not None
        assert runtime.snapshot().state in (LifecycleState.DISPATCHING, LifecycleState.RUNNING)
    finally:
        runtime.cancel()
        runtime.shutdown()


def test_ready_replacement_failure_and_consumed_identity() -> None:
    runtime = ExecutionRuntime(lambda: BlockingGateway({"a": Outcome.ACCEPTED}), poll_interval=10)
    try:
        first_token = runtime.start_planning()
        assert first_token is not None
        assert runtime.complete_planning(first_token, plan("a")).accepted
        replacement_token = runtime.start_planning()
        assert replacement_token is not None
        assert runtime.snapshot().ready_plan is None
        assert not runtime.complete_planning(
            replacement_token, ExecutionPlan("generated", ())
        ).accepted
        assert runtime.snapshot().state == LifecycleState.IDLE
        assert runtime.snapshot().ready_plan is None

        token = runtime.start_planning()
        assert token is not None
        assert runtime.complete_planning(token, plan("a")).accepted
        executed = runtime.execute_ready()
        assert executed.accepted and executed.value is not None
        assert runtime.snapshot().ready_plan is None
        consumed = runtime.execute_ready()
        assert not consumed.accepted
        assert consumed.value is None
    finally:
        runtime.cancel()
        runtime.shutdown()


def test_active_attempt_admission_rejects_plan_and_execute_but_accepts_cancel() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        planning = runtime.start_planning()
        assert planning is not None
        assert not runtime.execute_explicit(plan("a")).accepted
        assert runtime.cancel_planning().accepted

        started = runtime.execute_explicit(plan("a"))
        assert started.accepted
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        assert not runtime.start_planning()
        rejected = runtime.execute_explicit(plan("a"))
        assert not rejected.accepted and rejected.value is not None
        assert dict(runtime._context.terminal_results)[rejected.value].outcome == Outcome.REJECTED
        assert runtime.cancel().accepted
        assert not runtime.reset().accepted
    finally:
        gateway.release["a"].set()
        runtime.shutdown()


def test_rejection_compensates_prior_acceptance_and_unknown_is_fault() -> None:
    rpc = FakeRPC({("a", "execute"): True, ("b", "execute"): False, ("a", "cancel"): True}, [])
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        runtime.execute_explicit(plan("a", "b"))
        wait_until(lambda: any(call[1] == "cancel" for call in rpc.calls))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.IDLE)
        assert dict(runtime._context.terminal_results).popitem()[1].outcome == Outcome.REJECTED
    finally:
        runtime.shutdown()


def test_cancel_during_execute_handles_late_acceptance() -> None:
    entered, release = threading.Event(), threading.Event()
    rpc = FakeRPC({("a", "execute"): True, ("a", "cancel"): True}, [], entered, release)
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        assert entered.wait(1)
        assert runtime.cancel().accepted
        release.set()
        wait_until(lambda: any(call[1] == "cancel" for call in rpc.calls))
    finally:
        release.set()
        runtime.shutdown()


def test_planning_tokens_and_explicit_execution_are_atomic() -> None:
    rpc = FakeRPC({}, [])
    runtime = ExecutionRuntime(
        lambda: ControlCoordinatorGateway(rpc),
        topology=ExecutionTopology((("a", "robot", "a"),)),
        poll_interval=10,
    )
    try:
        token = runtime.start_planning()
        assert token is not None
        assert not runtime.complete_planning("stale", plan("a")).accepted
        planned = runtime.complete_planning(token, plan("a"))
        assert planned.accepted and planned.value is not None
        assert runtime.snapshot().state == LifecycleState.READY
        handle = runtime.execute_ready().value
        assert handle is not None
        assert handle.plan_id == planned.value
        assert runtime.snapshot().ready_plan is None
    finally:
        runtime.shutdown()


def test_wait_for_reset_snapshot_is_constructed_outside_context_lock() -> None:
    runtime = ExecutionRuntime(lambda: BlockingGateway({"a": Outcome.ACCEPTED}), poll_interval=10)
    reset_handle = ResetHandle("reset-attempt")
    try:
        reset_result = ResetResult(reset_handle, True)
        runtime._commit(replace(runtime._context, reset_results=((reset_handle, reset_result),)))
        result = runtime.wait_for_reset(reset_handle, timeout=1)
        assert result.accepted and result.value == reset_result
    finally:
        runtime.shutdown()


def test_deadline_preserves_unresolved_action_and_stop_rpc_is_deferred() -> None:
    entered, release = threading.Event(), threading.Event()
    rpc = FakeRPC({("a", "execute"): True}, [], entered, release)
    runtime = ExecutionRuntime(
        lambda: ControlCoordinatorGateway(rpc), action_timeout=0.01, poll_interval=10
    )
    runtime.execute_explicit(plan("a"))
    assert entered.wait(1)
    wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
    assert runtime.snapshot().operation is not None
    release.set()
    runtime.shutdown()
    assert any(call[1] == "stop" for call in rpc.calls)


def test_running_inactive_reconciles_peers_before_failed_terminal() -> None:
    rpc = FakeRPC(
        {
            ("a", "execute"): True,
            ("b", "execute"): True,
            ("a", "get_state"): TrajectoryState.IDLE,
            ("b", "get_state"): TrajectoryState.EXECUTING,
            ("b", "cancel"): False,
        },
        [],
    )
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        runtime.execute_explicit(plan("a", "b"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        runtime.poll()
        wait_until(lambda: runtime.snapshot().state == LifecycleState.IDLE)
        result = dict(runtime._context.terminal_results).popitem()[1]
        assert result.outcome == Outcome.FAILED
        assert any(task == "b" and method == "cancel" for task, method, _ in rpc.calls)
    finally:
        runtime.shutdown()


def test_status_unknown_faults_and_cleans_active_peers_without_completion() -> None:
    rpc = FakeRPC(
        {
            ("a", "execute"): True,
            ("b", "execute"): True,
            ("a", "get_state"): None,
            ("b", "cancel"): True,
        },
        [],
    )
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        runtime.execute_explicit(plan("a", "b"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        runtime.poll()
        wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
        assert not runtime._context.terminal_results
        wait_until(lambda: any(task == "b" and method == "cancel" for task, method, _ in rpc.calls))
    finally:
        runtime.shutdown()


def test_dispatch_result_is_stored_once_for_acceptance_and_rejection() -> None:
    rpc = FakeRPC({("a", "execute"): True, ("b", "execute"): True}, [])
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b")).value
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        records = [item for item in runtime._context.dispatch_results if item[0] == handle]
        assert len(records) == 1 and records[0][1].outcome == Outcome.ACCEPTED
    finally:
        runtime.shutdown()


def test_blocked_first_execute_preregisters_batch_and_delays_second() -> None:
    entered, release = threading.Event(), threading.Event()
    rpc = FakeRPC({("a", "execute"): True, ("b", "execute"): True}, [], entered, release)
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b")).value
        assert entered.wait(1)
        operation = runtime.snapshot().operation
        assert operation is not None and len(operation.tasks) == 2
        assert all(task.action is not None for task in operation.tasks[:1])
        assert not any(task == "b" and method == "execute" for task, method, _ in rpc.calls)
        assert runtime.snapshot().state == LifecycleState.DISPATCHING
        release.set()
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        assert len([item for item in runtime._context.dispatch_results if item[0] == handle]) == 1
    finally:
        release.set()
        runtime.shutdown()


def test_three_task_rejection_skips_unstarted_task_and_rejects_terminally() -> None:
    rpc = FakeRPC({("a", "execute"): True, ("b", "execute"): False, ("a", "cancel"): True}, [])
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b", "c")).value
        assert handle is not None
        wait_until(lambda: runtime.snapshot().state == LifecycleState.IDLE)
        assert not any(task == "c" and method == "execute" for task, method, _ in rpc.calls)
        assert len([item for item in runtime._context.dispatch_results if item[0] == handle]) == 1
        assert dict(runtime._context.terminal_results)[handle].outcome == Outcome.REJECTED
    finally:
        runtime.shutdown()


def test_cancel_unresolved_dispatch_skips_remaining_tasks() -> None:
    entered, release = threading.Event(), threading.Event()
    rpc = FakeRPC({("a", "execute"): True, ("a", "cancel"): True}, [], entered, release)
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b", "c")).value
        assert handle is not None
        assert entered.wait(1)
        runtime.cancel()
        release.set()
        wait_until(lambda: runtime.snapshot().state == LifecycleState.IDLE)
        assert not any(method == "execute" and task in ("b", "c") for task, method, _ in rpc.calls)
        assert len([item for item in runtime._context.dispatch_results if item[0] == handle]) == 1
        assert dict(runtime._context.terminal_results)[handle].outcome == Outcome.CANCELLED
    finally:
        release.set()
        runtime.shutdown()


def test_deadline_late_accept_and_reject_keep_fault_and_one_dispatch_result() -> None:
    for late, expected_calls in ((True, 1), (False, 0)):
        entered, release = threading.Event(), threading.Event()
        rpc = FakeRPC(
            {("a", "execute"): True if late else False, ("a", "cancel"): True}, [], entered, release
        )
        runtime = ExecutionRuntime(
            lambda rpc=rpc: ControlCoordinatorGateway(rpc), action_timeout=0.01, poll_interval=10
        )
        try:
            handle = runtime.execute_explicit(plan("a")).value
            assert entered.wait(1)
            wait_until(lambda runtime=runtime: runtime.snapshot().state == LifecycleState.FAULT)
            release.set()
            if late:
                wait_until(lambda rpc=rpc: any(method == "cancel" for _, method, _ in rpc.calls))
            time.sleep(0.03)
            assert runtime.snapshot().state == LifecycleState.FAULT
            assert (
                len([item for item in runtime._context.dispatch_results if item[0] == handle]) == 1
            )
            assert len([call for call in rpc.calls if call[1] == "cancel"]) == expected_calls
        finally:
            release.set()
            runtime.shutdown()


def test_remote_fault_marks_task_and_cancels_active_peer_without_success() -> None:
    rpc = FakeRPC(
        {
            ("a", "execute"): True,
            ("b", "execute"): True,
            ("a", "get_state"): TrajectoryState.FAULT,
            ("b", "cancel"): True,
        },
        [],
    )
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        runtime.execute_explicit(plan("a", "b"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        runtime.poll()
        wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
        operation = runtime.snapshot().operation
        assert operation is not None
        assert operation.tasks[0].activity.name == "REMOTE_FAULT"
        wait_until(lambda: any(task == "b" and method == "cancel" for task, method, _ in rpc.calls))
        assert not runtime._context.terminal_results
    finally:
        runtime.shutdown()

    rpc = FakeRPC({("a", "execute"): False}, [])
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a")).value
        wait_until(lambda: runtime.snapshot().state == LifecycleState.IDLE)
        records = [item for item in runtime._context.dispatch_results if item[0] == handle]
        assert len(records) == 1 and records[0][1].outcome == Outcome.REJECTED
    finally:
        runtime.shutdown()


def test_cleanup_allocates_distinct_cancel_actions_for_all_active_tasks() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED, "b": Outcome.ACCEPTED, "c": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b", "c")).value
        assert handle is not None
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        assert runtime.cancel().accepted
        for task in ("a", "b", "c"):
            assert gateway.entered[task].wait(1)
        operation = runtime.snapshot().operation
        assert operation is not None
        actions = {task.task_name: task.action for task in operation.tasks}
        assert all(actions[name] is not None for name in ("a", "b", "c"))
        assert len({actions[name].action_id for name in ("a", "b", "c")}) == 3  # type: ignore[union-attr]
        for task in ("a", "b", "c"):
            gateway.release[task].set()
            wait_until(lambda task=task: gateway.count(task, ActionMethod.CANCEL) == 1)
        wait_until(lambda: runtime.snapshot().state == LifecycleState.IDLE)
        assert dict(runtime._context.terminal_results)[handle].outcome == Outcome.CANCELLED
        assert all(gateway.count(task, ActionMethod.CANCEL) == 1 for task in ("a", "b", "c"))
    finally:
        runtime.shutdown()


def test_execute_unknown_faults_and_cleans_only_dispatched_execution() -> None:
    gateway = BlockingGateway({"a": Outcome.UNKNOWN, "b": Outcome.ACCEPTED, "c": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b", "c")).value
        assert handle is not None
        wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
        operation = runtime.snapshot().operation
        assert operation is not None and operation.uncertain
        assert gateway.entered["a"].wait(1)
        assert not any(
            task in ("b", "c") and method == ActionMethod.EXECUTE for task, method in gateway.calls
        )
        for event in gateway.release.values():
            event.set()
        wait_until(
            lambda: all(gateway.count(task, ActionMethod.CANCEL) == 1 for task in ("a", "b", "c"))
        )
        records = [item for item in runtime._context.dispatch_results if item[0] == handle]
        assert len(records) == 1 and records[0][1].outcome == Outcome.UNKNOWN
    finally:
        runtime.shutdown()


def test_cancelled_unresolved_dispatch_retains_cancelled_result_after_late_rejection() -> None:
    gateway = BlockingGateway({"a": Outcome.REJECTED}, {"a": Outcome.CANCELLED}, block_execute=True)
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b")).value
        assert handle is not None and gateway.execute_entered["a"].wait(1)
        assert runtime.cancel().accepted
        gateway.release["a"].set()
        wait_until(lambda: runtime.snapshot().state == LifecycleState.IDLE)
        records = [item for item in runtime._context.dispatch_results if item[0] == handle]
        assert len(records) == 1 and records[0][1].outcome == Outcome.CANCELLED
        assert gateway.count("a", ActionMethod.CANCEL) == 0
        assert not any(
            task == "b" and method == ActionMethod.EXECUTE for task, method in gateway.calls
        )
    finally:
        runtime.shutdown()


def test_deadline_duplicate_and_late_result_keep_one_original_dispatch_record() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED}, block_execute=True)
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=0.01, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a")).value
        assert handle is not None
        assert gateway.execute_entered["a"].wait(1)
        wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
        operation = runtime.snapshot().operation
        assert operation is not None and operation.tasks[0].action is not None
        task = operation.tasks[0]
        action = task.action
        assert action is not None
        runtime._submit(
            lambda: runtime._reduce(
                _Event(
                    "action_deadline",
                    handle.operation_id,
                    handle.attempt_id,
                    task.task_id,
                    action.action_id,
                    action.method,
                )
            )
        )
        gateway.release["a"].set()
        assert gateway.cancel_entered["a"].wait(1)
        records = [item for item in runtime._context.dispatch_results if item[0] == handle]
        assert len(records) == 1 and records[0][1].outcome == Outcome.UNKNOWN
        gateway.release["a"].set()
    finally:
        gateway.release["a"].set()
        runtime.shutdown()


def test_partial_rejection_after_three_accepts_retains_rejected_dispatch_once() -> None:
    gateway = BlockingGateway(
        {"a": Outcome.ACCEPTED, "b": Outcome.ACCEPTED, "c": Outcome.ACCEPTED, "d": Outcome.REJECTED}
    )
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b", "c", "d")).value
        assert handle is not None
        wait_until(lambda: gateway.count("d", ActionMethod.EXECUTE) == 1)

        def rejected_dispatch_processed() -> bool:
            records = [item for item in runtime._context.dispatch_results if item[0] == handle]
            return len(records) == 1 and records[0][1].outcome == Outcome.REJECTED

        wait_until(rejected_dispatch_processed)
        records = [item for item in runtime._context.dispatch_results if item[0] == handle]
        assert len(records) == 1 and records[0][1].outcome == Outcome.REJECTED
        for event in gateway.release.values():
            event.set()
    finally:
        for event in gateway.release.values():
            event.set()
        runtime.shutdown()


def test_late_response_updates_knowledge_without_accepting_stale_action() -> None:
    for outcome, expected_activity in (
        (Outcome.ACCEPTED, TaskActivity.CANCELLED),
        (Outcome.REJECTED, TaskActivity.INACTIVE),
    ):
        gateway = BlockingGateway({"a": outcome}, block_execute=True)
        runtime = ExecutionRuntime(
            lambda gateway=gateway: gateway, action_timeout=0.01, poll_interval=10
        )
        try:
            handle = runtime.execute_explicit(plan("a")).value
            assert handle is not None and gateway.execute_entered["a"].wait(1)
            wait_until(lambda runtime=runtime: runtime.snapshot().state == LifecycleState.FAULT)
            gateway.release["a"].set()
            if outcome == Outcome.ACCEPTED:
                wait_until(lambda gateway=gateway: gateway.count("a", ActionMethod.CANCEL) == 1)
                gateway.release["a"].set()
            wait_until(
                lambda runtime=runtime, expected_activity=expected_activity: task_is(
                    runtime, 0, expected_activity
                )
            )
            operation = runtime.snapshot().operation
            assert operation is not None
            assert operation.tasks[0].action is None
            task_id = operation.tasks[0].task_id
            operation_id, attempt_id = handle.operation_id, handle.attempt_id
            stale = runtime._submit(
                lambda runtime=runtime,
                operation_id=operation_id,
                attempt_id=attempt_id,
                task_id=task_id: runtime._reduce(
                    _Event(
                        "action_finished",
                        operation_id,
                        attempt_id,
                        task_id,
                        "wrong",
                        ActionMethod.CANCEL,
                        Outcome.CANCELLED,
                    )
                )
            )
            assert stale is False
        finally:
            gateway.release["a"].set()
            runtime.shutdown()


def test_remote_fault_requires_reset_and_cancels_peer_without_success() -> None:
    gateway = BlockingGateway(
        {"a": Outcome.ACCEPTED, "b": Outcome.ACCEPTED},
        {"a": Outcome.CANCELLED, "b": Outcome.CANCELLED},
    )
    gateway.status = lambda task: Outcome.FAILED if task == "a" else Outcome.RUNNING  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b")).value
        assert handle is not None
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        runtime.poll()
        wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
        operation = runtime.snapshot().operation
        assert operation is not None and operation.tasks[0].activity == TaskActivity.REMOTE_FAULT
        assert operation.tasks[0].reset_required
        assert gateway.cancel_entered["b"].wait(1)
        gateway.release["b"].set()
        wait_until(lambda: task_is(runtime, 1, TaskActivity.CANCELLED))
        operation = runtime.snapshot().operation
        assert operation is not None and all(task.action is None for task in operation.tasks)
        assert runtime.snapshot().state == LifecycleState.FAULT
        assert not runtime._context.terminal_results
    finally:
        for event in gateway.release.values():
            event.set()
        runtime.shutdown()


def test_waiters_use_only_the_requested_result_map() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED, "b": Outcome.REJECTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b")).value
        assert handle is not None
        assert gateway.cancel_entered["a"].wait(1)
        dispatch = runtime.wait_for_dispatch(handle, timeout=0.1)
        assert (
            dispatch.accepted
            and dispatch.value is not None
            and dispatch.value.outcome == Outcome.REJECTED
        )
        terminal = runtime.wait_for_terminal(handle, timeout=0.01)
        assert not terminal.accepted and terminal.diagnostic == "timeout"
        gateway.release["a"].set()
        terminal = runtime.wait_for_terminal(handle, timeout=1)
        assert (
            terminal.accepted
            and terminal.value is not None
            and terminal.value.outcome == Outcome.REJECTED
        )
    finally:
        gateway.release["a"].set()
        runtime.shutdown()


def test_execute_failure_latches_remote_fault_and_cleans_peer() -> None:
    gateway = BlockingGateway({"a": Outcome.FAILED, "b": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b")).value
        assert handle is not None
        wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
        operation = runtime.snapshot().operation
        assert operation is not None
        assert operation.tasks[0].activity == TaskActivity.REMOTE_FAULT
        assert operation.tasks[0].reset_required
        assert gateway.cancel_entered["b"].wait(1)
        assert not any(
            task == "b" and method == ActionMethod.EXECUTE for task, method in gateway.calls
        )
        gateway.release["b"].set()
        assert not runtime._context.terminal_results
    finally:
        gateway.release["b"].set()
        runtime.shutdown()


def test_cancel_failure_latches_remote_fault_and_cleans_peer() -> None:
    gateway = BlockingGateway(
        {"a": Outcome.ACCEPTED, "b": Outcome.ACCEPTED},
        {"a": Outcome.FAILED, "b": Outcome.CANCELLED},
    )
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b")).value
        assert handle is not None
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        assert runtime.cancel().accepted
        assert gateway.cancel_entered["a"].wait(1)
        assert gateway.cancel_entered["b"].wait(1)
        gateway.release["a"].set()
        wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
        operation = runtime.snapshot().operation
        assert operation is not None
        assert operation.tasks[0].activity == TaskActivity.REMOTE_FAULT
        assert operation.tasks[0].reset_required
        gateway.release["b"].set()
        wait_until(lambda: task_is(runtime, 1, TaskActivity.CANCELLED))
        operation = runtime.snapshot().operation
        assert operation is not None and all(task.action is None for task in operation.tasks)
        assert not runtime._context.terminal_results
    finally:
        for event in gateway.release.values():
            event.set()
        runtime.shutdown()


def test_result_retention_is_bounded_and_handles_remain_correlated() -> None:
    gateway = BlockingGateway({"a": Outcome.REJECTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    handles = []
    try:
        for _ in range(20):
            result = runtime.execute_explicit(plan("a"))
            assert result.accepted and result.value is not None
            handle = result.value
            handles.append(handle)
            dispatch = runtime.wait_for_dispatch(handle, timeout=1)
            terminal = runtime.wait_for_terminal(handle, timeout=1)
            assert dispatch.accepted and dispatch.value is not None
            assert terminal.accepted and terminal.value is not None
            assert dispatch.value.handle == handle
            assert terminal.value.handle == handle
            assert dispatch.value.outcome == Outcome.REJECTED
            assert terminal.value.outcome == Outcome.REJECTED
        assert len(runtime._context.dispatch_results) == 16
        assert len(runtime._context.terminal_results) == 16
        assert all(handle in dict(runtime._context.dispatch_results) for handle in handles[-16:])
        assert all(handle in dict(runtime._context.terminal_results) for handle in handles[-16:])
        assert handles[0] not in dict(runtime._context.dispatch_results)
        assert handles[0] not in dict(runtime._context.terminal_results)
    finally:
        runtime.shutdown()


def test_unknown_fault_reset_fails_and_leaves_fault_latched() -> None:
    gateway = BlockingGateway({"a": Outcome.UNKNOWN})
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=0.1, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
        assert gateway.cancel_entered["a"].wait(1)
        gateway.release["a"].set()
        handle_result = runtime.reset()
        assert handle_result.accepted and handle_result.value is not None
        reset = runtime.wait_for_reset(handle_result.value, timeout=1)
        assert reset.accepted and reset.value is not None and not reset.value.success
        assert runtime.snapshot().state == LifecycleState.FAULT
        time.sleep(0.05)
        assert runtime.snapshot().state == LifecycleState.FAULT
    finally:
        gateway.release["a"].set()
        runtime.shutdown()


def test_remote_fault_reset_proves_inactive_and_resets_remote_task() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED, "b": Outcome.ACCEPTED})
    status_calls: dict[str, int] = {"a": 0, "b": 0}

    def status(task: str) -> Outcome:
        status_calls[task] += 1
        return Outcome.FAILED if task == "a" and status_calls[task] == 1 else Outcome.INACTIVE

    gateway.status = status  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=0.2, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a", "b"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        runtime.poll()
        wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
        gateway.release["b"].set()
        wait_until(lambda: gateway.count("b", ActionMethod.CANCEL) == 1)
        reset_handle = runtime.reset().value
        assert reset_handle is not None
        reset = runtime.wait_for_reset(reset_handle, timeout=1)
        assert reset.accepted and reset.value is not None and reset.value.success
        assert runtime.snapshot().state == LifecycleState.IDLE
        assert gateway.count("a", ActionMethod.RESET) == 1
        assert runtime._context.reset_handle is None
        assert not runtime._context.reset_proven_inactive
        assert not runtime._context.reset_completed_tasks
        assert dict(runtime._context.reset_results)[reset_handle] == reset.value
    finally:
        for event in gateway.release.values():
            event.set()
        runtime.shutdown()


def test_remote_reset_failure_keeps_fault_latched() -> None:
    for reset_outcome in (Outcome.RUNNING, Outcome.UNKNOWN):
        gateway = BlockingGateway({"a": Outcome.ACCEPTED})
        gateway.status = lambda task: Outcome.INACTIVE  # type: ignore[method-assign]
        gateway.reset = lambda task, outcome=reset_outcome: outcome  # type: ignore[method-assign]
        runtime = ExecutionRuntime(
            lambda gateway=gateway: gateway, action_timeout=0.2, poll_interval=10
        )
        try:
            runtime.execute_explicit(plan("a"))
            wait_until(lambda runtime=runtime: runtime.snapshot().state == LifecycleState.RUNNING)
            runtime._submit(
                lambda runtime=runtime: runtime._reduce(_Event("fault", diagnostic="remote fault"))
            )
            reset_handle = runtime.reset().value
            assert reset_handle is not None
            reset = runtime.wait_for_reset(reset_handle, timeout=1)
            assert reset.accepted and reset.value is not None and not reset.value.success
            assert runtime.snapshot().state == LifecycleState.FAULT
        finally:
            runtime.shutdown()


def test_pending_late_execute_action_makes_reset_fail_permanently() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED}, block_execute=True)
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=0.01, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        assert gateway.execute_entered["a"].wait(1)
        wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
        reset_handle = runtime.reset().value
        assert reset_handle is not None
        reset = runtime.wait_for_reset(reset_handle, timeout=1)
        assert reset.accepted and reset.value is not None and not reset.value.success
        gateway.release["a"].set()
        time.sleep(0.05)
        assert runtime.snapshot().state == LifecycleState.FAULT
    finally:
        gateway.release["a"].set()
        runtime.shutdown()


def test_failed_reset_can_start_a_new_reconciliation_attempt() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    status_calls = 0

    def status(_task: str) -> Outcome:
        nonlocal status_calls
        status_calls += 1
        return Outcome.UNKNOWN if status_calls == 1 else Outcome.INACTIVE

    gateway.status = status  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=0.2, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        runtime._submit(lambda: runtime._reduce(_Event("fault", diagnostic="remote fault")))
        gateway.release["a"].set()

        first = runtime.reset()
        assert first.accepted and first.value is not None
        first_result = runtime.wait_for_reset(first.value, timeout=1)
        assert first_result.accepted and first_result.value is not None
        assert not first_result.value.success
        assert runtime.snapshot().state == LifecycleState.FAULT

        second = runtime.reset()
        assert second.accepted and second.value is not None
        assert second.value != first.value
        second_result = runtime.wait_for_reset(second.value, timeout=1)
        assert second_result.accepted and second_result.value is not None
        assert second_result.value.success
        assert runtime.snapshot().state == LifecycleState.IDLE
    finally:
        gateway.release["a"].set()
        runtime.shutdown()


def test_stale_reset_completion_cannot_clear_fault_for_retry() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    first_reset_entered = threading.Event()
    release_first_reset = threading.Event()
    second_reset_entered = threading.Event()
    release_second_reset = threading.Event()
    reset_calls = 0

    def status(_task: str) -> Outcome:
        return Outcome.INACTIVE

    def reset_remote(_task: str) -> Outcome:
        nonlocal reset_calls
        reset_calls += 1
        if reset_calls == 1:
            first_reset_entered.set()
            assert release_first_reset.wait(2)
        elif reset_calls == 2:
            second_reset_entered.set()
            assert release_second_reset.wait(2)
        return Outcome.INACTIVE

    gateway.status = status  # type: ignore[method-assign]
    gateway.reset = reset_remote  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=0.05, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        operation = runtime.snapshot().operation
        assert operation is not None
        task = operation.tasks[0]
        faulted = replace(
            operation,
            tasks=(replace(task, activity=TaskActivity.INACTIVE, reset_required=True),),
        )
        runtime._submit(
            lambda: runtime._commit(
                replace(
                    runtime._context,
                    state=LifecycleState.FAULT,
                    fault="remote fault",
                    diagnostic="remote fault",
                    active=faulted,
                )
            )
        )

        first = runtime.reset()
        assert first.accepted and first.value is not None
        assert first_reset_entered.wait(1)
        first_result = runtime.wait_for_reset(first.value, timeout=1)
        assert first_result.accepted and first_result.value is not None
        assert not first_result.value.success

        second = runtime.reset()
        assert second.accepted and second.value is not None and second.value != first.value
        release_first_reset.set()
        wait_until(lambda: reset_calls == 2)
        assert second_reset_entered.wait(1)
        assert runtime.snapshot().state == LifecycleState.FAULT
        assert runtime._context.reset_handle == second.value

        release_second_reset.set()
        second_result = runtime.wait_for_reset(second.value, timeout=1)
        assert second_result.accepted and second_result.value is not None
        assert second_result.value.success
        assert runtime.snapshot().state == LifecycleState.IDLE
    finally:
        release_first_reset.set()
        release_second_reset.set()
        runtime.shutdown()


def test_stale_reset_cancel_cannot_affect_retry_reconciliation() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    first_cancel_entered = threading.Event()
    release_first_cancel = threading.Event()
    second_cancel_entered = threading.Event()
    release_second_cancel = threading.Event()
    cancel_calls = 0

    def cancel(_task: str) -> Outcome:
        nonlocal cancel_calls
        cancel_calls += 1
        if cancel_calls == 1:
            first_cancel_entered.set()
            assert release_first_cancel.wait(2)
            return Outcome.UNKNOWN
        second_cancel_entered.set()
        assert release_second_cancel.wait(2)
        return Outcome.CANCELLED

    gateway.cancel = cancel  # type: ignore[method-assign]
    gateway.status = lambda _task: Outcome.INACTIVE  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=0.05, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        runtime._submit(lambda: runtime._reduce(_Event("fault", diagnostic="remote fault")))

        first = runtime.reset()
        assert first.accepted and first.value is not None
        assert first_cancel_entered.wait(1)
        first_result = runtime.wait_for_reset(first.value, timeout=1)
        assert first_result.accepted and first_result.value is not None
        assert not first_result.value.success

        second = runtime.reset()
        assert second.accepted and second.value is not None and second.value != first.value
        release_first_cancel.set()
        wait_until(lambda: cancel_calls == 2)
        assert second_cancel_entered.wait(1)
        assert runtime.snapshot().state == LifecycleState.FAULT
        assert runtime._context.reset_handle == second.value

        release_second_cancel.set()
        second_result = runtime.wait_for_reset(second.value, timeout=1)
        assert second_result.accepted and second_result.value is not None
        assert second_result.value.success
        assert runtime.snapshot().state == LifecycleState.IDLE
    finally:
        release_first_cancel.set()
        release_second_cancel.set()
        runtime.shutdown()


def test_reset_completion_after_failure_is_retired_before_retry_starts() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    entered = threading.Event()
    release = threading.Event()
    reset_calls = 0

    def reset_remote(_task: str) -> Outcome:
        nonlocal reset_calls
        reset_calls += 1
        if reset_calls == 1:
            entered.set()
            assert release.wait(2)
        return Outcome.INACTIVE

    gateway.reset = reset_remote  # type: ignore[method-assign]
    gateway.status = lambda _task: Outcome.INACTIVE  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=0.05, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        operation = runtime.snapshot().operation
        assert operation is not None
        task = operation.tasks[0]
        faulted = replace(
            operation,
            tasks=(replace(task, activity=TaskActivity.INACTIVE, reset_required=True),),
        )
        runtime._submit(
            lambda: runtime._commit(
                replace(runtime._context, state=LifecycleState.FAULT, active=faulted)
            )
        )
        first = runtime.reset()
        assert first.accepted and first.value is not None and entered.wait(1)
        first_result = runtime.wait_for_reset(first.value, timeout=1)
        assert first_result.accepted and first_result.value is not None
        assert not first_result.value.success
        release.set()
        wait_until(
            lambda: runtime._context.active is not None
            and runtime._context.active.tasks[0].action is None
        )
        assert runtime.snapshot().state == LifecycleState.FAULT

        second = runtime.reset()
        assert second.accepted and second.value is not None and second.value != first.value
        wait_until(lambda: reset_calls == 2)
        second_result = runtime.wait_for_reset(second.value, timeout=1)
        assert second_result.accepted and second_result.value is not None
        assert second_result.value.success
    finally:
        release.set()
        runtime.shutdown()


def test_stale_reset_deadline_does_not_retire_inflight_action() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    entered = threading.Event()
    release = threading.Event()
    second_entered = threading.Event()
    cancel_calls = 0

    def cancel(_task: str) -> Outcome:
        nonlocal cancel_calls
        cancel_calls += 1
        if cancel_calls == 2:
            second_entered.set()
        entered.set()
        assert release.wait(2)
        return Outcome.CANCELLED

    gateway.cancel = cancel  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=0.05, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        operation = runtime.snapshot().operation
        assert operation is not None
        task = operation.tasks[0]
        faulted = replace(operation, tasks=(replace(task, activity=TaskActivity.ACTIVE),))
        runtime._submit(
            lambda: runtime._commit(
                replace(runtime._context, state=LifecycleState.FAULT, active=faulted)
            )
        )
        first = runtime.reset()
        assert first.accepted and first.value is not None and entered.wait(1)
        old_action = runtime._context.active.tasks[0].action
        assert old_action is not None and old_action.reset_id == first.value.reset_id
        first_result = runtime.wait_for_reset(first.value, timeout=1)
        assert first_result.accepted and first_result.value is not None
        assert not first_result.value.success

        second = runtime.reset()
        assert second.accepted and second.value is not None
        runtime._submit(
            lambda: runtime._reduce(
                _Event(
                    "action_deadline",
                    runtime._context.active.handle.operation_id,
                    runtime._context.active.handle.attempt_id,
                    task.task_id,
                    old_action.action_id,
                    ActionMethod.CANCEL,
                    reset_id=first.value.reset_id,
                )
            )
        )
        assert runtime._context.active.tasks[0].action is old_action
        assert runtime._context.reset_handle == second.value
        assert cancel_calls == 1
        assert not second_entered.is_set()
        release.set()
    finally:
        release.set()
        runtime.shutdown()


def test_repeated_reset_calls_share_one_serialized_result() -> None:
    gateway = BlockingGateway({"a": Outcome.UNKNOWN})
    entered, release = threading.Event(), threading.Event()

    def status(task: str) -> Outcome:
        entered.set()
        assert release.wait(2)
        return Outcome.UNKNOWN

    gateway.status = status  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=0.2, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.FAULT)
        gateway.release["a"].set()
        first = runtime.reset()
        assert first.accepted and first.value is not None and entered.wait(1)
        second = runtime.reset()
        assert second.accepted and second.value == first.value
        release.set()
        result = runtime.wait_for_reset(first.value, timeout=1)
        assert result.accepted and result.value is not None and not result.value.success
        assert len(runtime._context.reset_results) == 1
    finally:
        release.set()
        gateway.release["a"].set()
        runtime.shutdown()


def test_idle_shutdown_is_idempotent_and_rejects_new_work() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    first = runtime.shutdown(timeout=1)
    assert first.accepted and first.value is not None and first.value.success
    assert runtime.snapshot().shutdown == ShutdownState.CLOSED
    assert not getattr(runtime.start_planning(), "accepted", False)
    assert not runtime.execute_explicit(plan("a")).accepted
    second = runtime.shutdown(timeout=1)
    assert second.accepted and second.value == first.value
    assert gateway.stop_calls == 1


def test_gateway_stop_failure_never_reports_closed_success() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})

    def fail_stop() -> None:
        gateway.stop_calls += 1
        raise RuntimeError("close exploded")

    gateway.stop = fail_stop  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    result = runtime.shutdown(timeout=1)
    assert not result.accepted
    assert result.value is not None and not result.value.success
    assert "gateway close failed: close exploded" in result.value.diagnostic
    assert result.snapshot is not None
    assert result.snapshot.shutdown == ShutdownState.CLOSING
    assert result.snapshot.shutdown_result == result.value
    assert gateway.stop_calls == 1
    assert runtime.shutdown(timeout=1).accepted is False
    assert gateway.stop_calls == 1


def test_shutdown_interrupts_early_action_deadline_thread() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=10.0, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        gateway.release["a"].set()
        assert runtime.shutdown(timeout=1).accepted
        assert runtime._auxiliary == {}
    finally:
        gateway.release["a"].set()
        if runtime.snapshot().shutdown != ShutdownState.CLOSED:
            runtime.shutdown(timeout=1)


def test_shutdown_interrupts_reset_deadline_thread() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=10.0, poll_interval=10)
    handle = ResetHandle("reset-deadline-test")
    try:
        runtime._start_auxiliary(
            runtime._reset_deadline,
            handle,
            time.monotonic() + 10.0,
            name="reset-deadline-test",
        )
        wait_until(lambda: len(runtime._auxiliary) == 1)
        result = runtime.shutdown(timeout=1)
        assert result.accepted
        assert runtime._auxiliary == {}
    finally:
        if runtime.snapshot().shutdown != ShutdownState.CLOSED:
            runtime.shutdown(timeout=1)


def test_cancel_if_current_cannot_cancel_replacement_operation() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    gateway.status = lambda task_name: Outcome.COMPLETED  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        original = runtime.execute_explicit(plan("a"))
        assert original.accepted and original.value is not None
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        runtime.poll()
        assert runtime.wait_for_terminal(original.value, timeout=1).accepted

        replacement = runtime.execute_explicit(plan("a"))
        assert replacement.accepted and replacement.value is not None
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)

        stale_cancel = runtime.cancel_if_current(original.value)
        assert not stale_cancel.accepted
        assert stale_cancel.diagnostic == "operation is no longer current"
        active = runtime.snapshot().operation
        assert active is not None
        assert active.handle == replacement.value
        assert gateway.count("a", ActionMethod.CANCEL) == 0
    finally:
        gateway.release["a"].set()
        if runtime.snapshot().shutdown != ShutdownState.CLOSED:
            runtime.shutdown(timeout=1)


def test_shutdown_during_pending_cancel_reconciles_and_closes_once() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        assert runtime.cancel().accepted
        assert gateway.cancel_entered["a"].wait(1)
        result_holder: list[Any] = []
        thread = threading.Thread(target=lambda: result_holder.append(runtime.shutdown(timeout=1)))
        thread.start()
        wait_until(lambda: runtime.snapshot().shutdown == ShutdownState.CLOSING)
        assert not runtime.execute_explicit(plan("a")).accepted
        assert not getattr(runtime.start_planning(), "accepted", False)
        gateway.release["a"].set()
        thread.join(2)
        assert (
            len(result_holder) == 1 and result_holder[0].accepted and result_holder[0].value.success
        )
        assert gateway.stop_calls == 1
    finally:
        gateway.release["a"].set()
        if runtime.snapshot().shutdown != ShutdownState.CLOSED:
            runtime.shutdown()


def test_shutdown_late_execute_acceptance_is_compensated_before_close() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED}, block_execute=True)
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        assert gateway.execute_entered["a"].wait(1)
        result_holder: list[Any] = []
        thread = threading.Thread(target=lambda: result_holder.append(runtime.shutdown(timeout=1)))
        thread.start()
        wait_until(lambda: runtime.snapshot().shutdown == ShutdownState.CLOSING)
        gateway.release["a"].set()
        thread.join(2)
        assert (
            len(result_holder) == 1 and result_holder[0].accepted and result_holder[0].value.success
        )
        assert gateway.count("a", ActionMethod.CANCEL) == 1
        assert runtime.snapshot().shutdown == ShutdownState.CLOSED
    finally:
        gateway.release["a"].set()
        if runtime.snapshot().shutdown != ShutdownState.CLOSED:
            runtime.shutdown()


def test_zero_timeout_does_not_publish_closed_while_owner_is_alive() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    entered, release = threading.Event(), threading.Event()
    runtime._events.put(_Call(lambda: (entered.set(), release.wait(1))))
    try:
        assert entered.wait(1)
        result = runtime.shutdown(timeout=0)
        assert not result.accepted and result.value is not None and not result.value.success
        assert runtime.snapshot().shutdown == ShutdownState.CLOSING
        assert runtime._owner.is_alive()
        assert not any(
            method in (ActionMethod.EXECUTE, ActionMethod.CANCEL) for _, method in gateway.calls
        )
    finally:
        release.set()
        gateway.release["a"].set()
        if runtime.snapshot().shutdown != ShutdownState.CLOSED:
            runtime.shutdown(timeout=1)


def test_zero_timeout_active_shutdown_never_starts_rpc_after_gateway_stop() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        result = runtime.shutdown(timeout=0)
        assert not result.accepted and result.value is not None and not result.value.success
        time.sleep(0.05)
        assert gateway.stop_calls == 1
        assert len(gateway.calls) == gateway.calls_at_stop
        assert gateway.count("a", ActionMethod.CANCEL) == 0
    finally:
        if runtime.snapshot().shutdown != ShutdownState.CLOSED:
            runtime.shutdown(timeout=1)


def test_gateway_lease_precedes_closure_and_closer_waits_for_inflight_call() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED}, block_execute=True)
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    result_holder: list[tuple[bool, Outcome | None]] = []
    try:
        thread = threading.Thread(
            target=lambda: result_holder.append(
                runtime._invoke_gateway_action("action", "a", ActionMethod.EXECUTE, {"x": 1})
            )
        )
        thread.start()
        assert gateway.execute_entered["a"].wait(1)
        runtime._seal_gateway()
        assert gateway.stop_calls == 0
        gateway.release["a"].set()
        thread.join(1)
        wait_until(lambda: gateway.stop_calls == 1)
        assert result_holder == [(True, Outcome.ACCEPTED)]
        assert gateway.calls_at_stop == len(gateway.calls)
    finally:
        gateway.release["a"].set()
        runtime.shutdown(timeout=1)


def test_sealed_gateway_rejects_prepared_action_without_inflight_or_rpc() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        runtime._seal_gateway()
        admitted, outcome = runtime._invoke_gateway_action(
            "action", "a", ActionMethod.EXECUTE, {"x": 1}
        )
        assert not admitted and outcome is None
        assert gateway.calls == []
        assert runtime._rpc_inflight == 0
    finally:
        runtime.shutdown(timeout=1)


def test_shutdown_transition_publishes_closing_and_cancel_only_atomically() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    runtime._gateway_condition.acquire()
    result_holder: list[Any] = []
    thread = threading.Thread(target=lambda: result_holder.append(runtime.shutdown(timeout=1)))
    thread.start()
    try:
        time.sleep(0.02)
        assert thread.is_alive()
        assert runtime._context.shutdown == ShutdownState.OPEN
        assert runtime._gateway_gate == _GatewayGateState.OPEN
    finally:
        runtime._gateway_condition.release()
        thread.join(2)
        assert len(result_holder) == 1 and result_holder[0].accepted
        assert runtime.snapshot().shutdown == ShutdownState.CLOSED


def test_shutdown_timeout_reports_unresolved_action_and_never_schedules_late_effects() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED}, block_execute=True)
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        assert gateway.execute_entered["a"].wait(1)
        result = runtime.shutdown(timeout=0.05)
        assert not result.accepted and result.value is not None and not result.value.success
        assert runtime.snapshot().shutdown == ShutdownState.CLOSING
        cancel_count = gateway.count("a", ActionMethod.CANCEL)
        gateway.release["a"].set()
        time.sleep(0.05)
        assert gateway.count("a", ActionMethod.CANCEL) == cancel_count
        assert gateway.stop_calls == 1
    finally:
        gateway.release["a"].set()
        if runtime.snapshot().shutdown != ShutdownState.CLOSED:
            runtime.shutdown()


def _prep_robot(
    name: str,
    joints: tuple[str, ...],
    group: str,
    task: str | None,
    mapping: dict[str, str] | None = None,
) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path(f"/{name}.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion([0.0, 0.0, 0.0, 1.0])),
        joint_names=list(joints),
        planning_groups=[PlanningGroupDefinition(name=group, joint_names=joints, base_link="base")],
        coordinator_task_name=task,
        joint_name_mapping=mapping or {},
    )


def _generated(names: list[str], groups: tuple[str, ...], width: int) -> GeneratedPlan:
    return GeneratedPlan(
        group_ids=groups,
        trajectory=JointTrajectory(
            joint_names=names,
            points=[
                TrajectoryPoint(
                    time_from_start=0.0, positions=[0.0] * width, velocities=[0.1] * width
                ),
                TrajectoryPoint(
                    time_from_start=1.5, positions=[1.0] * width, velocities=[0.2] * width
                ),
            ],
        ),
    )


def test_prepare_generated_plan_uses_ordered_groups_and_ignores_extraneous_robot_columns() -> None:
    left = _prep_robot("left", ("l0", "l1"), "arm", "left_task", {"left_l0": "l0"})
    right = _prep_robot("right", ("r0",), "arm", "right_task")
    extra = _prep_robot("extra", ("x0",), "arm", "extra_task")
    topology = ExecutionTopology.from_robot_configs((left, right, extra))
    generated = _generated(
        ["extra/x0", "right/r0", "left/l1", "left/l0"], ("left/arm", "right/arm"), 4
    )

    prepared = prepare_generated_plan(generated, topology)
    assert prepared.generated_plan is generated
    assert [entry.robot_name for entry in prepared.entries] == ["left", "right"]
    assert [entry.task_name for entry in prepared.entries] == ["left_task", "right_task"]
    assert prepared.entries[0].request["trajectory"].joint_names == ["l1", "left_l0"]
    assert prepared.entries[0].request["trajectory"].points[1].time_from_start == 1.5
    assert prepared.entries[1].request["trajectory"].joint_names == ["r0"]


def test_prepare_rejects_missing_routes_tasks_malformed_and_missing_or_duplicate_joints() -> None:
    left = _prep_robot("left", ("l0", "l1"), "arm", "left_task")
    right = _prep_robot("right", ("r0",), "arm", "right_task")
    topology = ExecutionTopology.from_robot_configs((left, right))
    with pytest.raises(ValueError, match="route set"):
        ExecutionTopology.from_robot_configs((left, right), {"left/arm": ("left", "left_task")})
    with pytest.raises(ValueError, match="malformed"):
        prepare_generated_plan(_generated(["left/l0", "bad"], ("left/arm",), 2), topology)
    with pytest.raises(ValueError, match="missing robot joints"):
        prepare_generated_plan(_generated(["left/l0"], ("left/arm",), 1), topology)
    with pytest.raises(ValueError, match="duplicate global"):
        prepare_generated_plan(_generated(["left/l0", "left/l0"], ("left/arm",), 2), topology)


def test_prepared_plan_is_real_type_and_partial_generated_entries_are_not_executable() -> None:
    robot = _prep_robot("arm", ("j0",), "manipulator", "arm_task", {"coord_j0": "j0"})
    topology = ExecutionTopology.from_robot_configs((robot,))
    generated = _generated(["arm/j0"], ("arm/manipulator",), 1)
    prepared = prepare_generated_plan(generated, topology)
    assert isinstance(prepared, PreparedPlan)
    assert prepared is not generated
    partial = PreparedPlan(generated, (), topology)
    runtime = ExecutionRuntime(lambda: BlockingGateway({"arm": Outcome.ACCEPTED}), poll_interval=10)
    try:
        assert not runtime.execute_explicit(partial).accepted
    finally:
        runtime.shutdown()


def test_real_generated_plan_cannot_use_legacy_synthetic_execution_plan() -> None:
    robot = _prep_robot("arm", ("j0",), "manipulator", "arm_task")
    topology = ExecutionTopology.from_robot_configs((robot,))
    generated = _generated(["arm/j0"], ("arm/manipulator",), 1)
    prepared = prepare_generated_plan(generated, topology)
    runtime = ExecutionRuntime(lambda: BlockingGateway({"arm": Outcome.ACCEPTED}), poll_interval=10)
    try:
        legacy = ExecutionPlan(generated, prepared.entries)
        assert not runtime.execute_explicit(legacy).accepted
        assert runtime.execute_explicit(prepared).accepted
    finally:
        runtime.shutdown()


def test_prepared_validation_selects_only_generated_groups_from_larger_topology() -> None:
    left = _prep_robot("left", ("l0",), "arm", "left_task")
    right = _prep_robot("right", ("r0",), "arm", "right_task")
    extra = _prep_robot("extra", ("x0",), "arm", "extra_task")
    topology = ExecutionTopology.from_robot_configs((left, right, extra))
    prepared = prepare_generated_plan(
        _generated(["left/l0", "right/r0"], ("right/arm", "left/arm"), 2), topology
    )
    runtime = ExecutionRuntime(
        lambda: BlockingGateway({"left": Outcome.ACCEPTED, "right": Outcome.ACCEPTED}),
        poll_interval=10,
    )
    try:
        result = runtime.execute_explicit(prepared)
        assert result.accepted
        assert [entry.planning_group for entry in prepared.entries] == ["right/arm", "left/arm"]
    finally:
        runtime.shutdown()


def test_prepare_rejects_additional_selected_robot_joint_column() -> None:
    robot = _prep_robot("arm", ("j0", "j1"), "manipulator", "arm_task")
    topology = ExecutionTopology.from_robot_configs((robot,))
    with pytest.raises(ValueError, match="additional joint column"):
        prepare_generated_plan(
            _generated(["arm/j0", "arm/j1", "arm/extra"], ("arm/manipulator",), 3), topology
        )


def test_topology_rejects_duplicate_routes_bad_mapping_targets_and_unknown_group_joints() -> None:
    robot = _prep_robot("arm", ("j0",), "manipulator", "arm_task")
    with pytest.raises(ValueError, match="duplicate group route"):
        ExecutionTopology.from_robot_configs(
            (robot,),
            [("arm/manipulator", "arm", "arm_task"), ("arm/manipulator", "arm", "arm_task")],
        )
    with pytest.raises(ValueError, match="mapped local joint"):
        ExecutionTopology.from_robot_configs(
            (_prep_robot("arm", ("j0",), "manipulator", "arm_task", {"coord": "missing"}),)
        )
    bad_group = _prep_robot("arm", ("j0",), "manipulator", "arm_task")
    bad_group.planning_groups[0] = PlanningGroupDefinition(
        name="manipulator", joint_names=("missing",), base_link="base"
    )
    with pytest.raises(ValueError, match="planning group joints"):
        ExecutionTopology.from_robot_configs((bad_group,))


def test_planning_only_config_has_no_route_and_generated_execution_rejects_it() -> None:
    planning_only = _prep_robot("planner", ("j0",), "manipulator", None)
    topology = ExecutionTopology.from_robot_configs((planning_only,))
    assert topology.routes == ()
    with pytest.raises(ValueError, match="no coordinator task"):
        prepare_generated_plan(_generated(["planner/j0"], ("planner/manipulator",), 1), topology)


def test_clear_ready_plan_is_a_dedicated_ready_to_idle_transition() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        token = runtime.start_planning()
        assert token is not None
        assert runtime.complete_planning(token, plan("a")).accepted
        assert runtime.snapshot().state == LifecycleState.READY
        cleared = runtime.clear_ready_plan()
        assert cleared.accepted
        snapshot = runtime.snapshot()
        assert snapshot.state == LifecycleState.IDLE
        assert snapshot.ready_plan is None and snapshot.ready_plan_id is None
        assert not runtime.clear_ready_plan().accepted
    finally:
        runtime.shutdown()


def test_runtime_diagnostic_persists_idle_planning_and_execution_errors_then_clears_on_planning() -> (
    None
):
    runtime = ExecutionRuntime(lambda: BlockingGateway({"a": Outcome.ACCEPTED}), poll_interval=10)
    try:
        token = runtime.start_planning()
        assert token is not None
        invalid = runtime.complete_planning(token, ExecutionPlan("generated", ()))
        assert not invalid.accepted
        assert runtime.snapshot().state == LifecycleState.IDLE
        assert runtime.snapshot().diagnostic == "invalid prepared plan"
        assert runtime.start_planning() is not None
        assert runtime.snapshot().diagnostic is None
    finally:
        runtime.shutdown()


def test_rejected_execute_retains_contextual_diagnostic_in_snapshot_and_result() -> None:
    gateway = BlockingGateway({"a": Outcome.REJECTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a")).value
        assert handle is not None
        result = runtime.wait_for_terminal(handle, timeout=1)
        assert result.accepted and result.value is not None
        assert result.value.outcome == Outcome.REJECTED
        assert "task=a" in result.value.diagnostic
        assert "action=" in result.value.diagnostic
        assert "outcome=rejected" in result.value.diagnostic
        snapshot = runtime.snapshot()
        assert snapshot.state == LifecycleState.IDLE
        assert snapshot.diagnostic == result.value.diagnostic
    finally:
        runtime.shutdown()


def test_status_terminal_failure_retains_contextual_diagnostic_in_terminal_result() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED, "b": Outcome.ACCEPTED})
    gateway.status = lambda task: Outcome.INACTIVE if task == "a" else Outcome.RUNNING  # type: ignore[method-assign]
    gateway.release["a"].set()
    gateway.release["b"].set()
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b")).value
        assert handle is not None
        wait_until(lambda: runtime.snapshot().state == LifecycleState.RUNNING)
        runtime.poll()
        result = runtime.wait_for_terminal(handle, timeout=1)
        assert result.accepted and result.value is not None
        assert result.value.outcome == Outcome.FAILED
        assert "task=a" in result.value.diagnostic
        assert "outcome=inactive" in result.value.diagnostic
        assert runtime.snapshot().state == LifecycleState.IDLE
        assert runtime.snapshot().diagnostic == result.value.diagnostic
    finally:
        for event in gateway.release.values():
            event.set()
        runtime.shutdown()


def test_auxiliary_gripper_calls_use_private_gateway_and_normalize_results() -> None:
    gateway = BlockingGateway({"gripper": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        set_result = runtime.set_gripper_position("gripper", 0.75)
        get_result = runtime.get_gripper_position("gripper")
        assert set_result.accepted and set_result.value == Outcome.ACCEPTED
        assert get_result.accepted and get_result.value == 0.25
        assert gateway.count("gripper", ActionMethod.GRIPPER_SET) == 1
        assert gateway.count("gripper", ActionMethod.GRIPPER_GET) == 1
        assert not hasattr(runtime, "gateway")
    finally:
        runtime.shutdown()


def test_control_gateway_uses_top_level_gripper_rpc_methods() -> None:
    rpc = FakeRPC(
        {("gripper", "set_gripper_position"): True, ("gripper", "get_gripper_position"): 0.6}, []
    )
    gateway = ControlCoordinatorGateway(rpc)
    assert gateway.set_gripper_position("gripper", 0.4) == Outcome.ACCEPTED
    assert gateway.get_gripper_position("gripper") == 0.6
    assert rpc.calls == [
        ("gripper", "set_gripper_position", {"position": 0.4}),
        ("gripper", "get_gripper_position", {}),
    ]


def test_auxiliary_calls_reject_after_closing_and_inflight_auxiliary_drains_before_close() -> None:
    gateway = BlockingAuxGateway()
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    results: list[Any] = []
    caller = threading.Thread(
        target=lambda: results.append(runtime.set_gripper_position("gripper", 0.5))
    )
    caller.start()
    assert gateway.aux_entered.wait(1)
    shutdown_results: list[Any] = []
    closer = threading.Thread(target=lambda: shutdown_results.append(runtime.shutdown(timeout=1)))
    closer.start()
    wait_until(lambda: runtime.snapshot().shutdown == ShutdownState.CLOSING)
    gateway.aux_release.set()
    caller.join(2)
    closer.join(2)
    assert results and results[0].accepted
    assert shutdown_results and shutdown_results[0].accepted
    assert gateway.calls_at_stop == len(gateway.calls)
    assert gateway.stop_calls == 1
    assert not runtime.set_gripper_position("gripper", 0.1).accepted
    assert not runtime.get_gripper_position("gripper").accepted


def test_gateway_factory_is_the_internal_runtime_injection_seam() -> None:
    gateway = BlockingGateway({"gripper": Outcome.ACCEPTED})
    factories: list[bool] = []

    def factory() -> BlockingGateway:
        factories.append(True)
        return gateway

    runtime = ExecutionRuntime(factory, poll_interval=10)
    try:
        assert factories == [True]
        assert runtime.get_gripper_position("gripper").accepted
        assert not hasattr(runtime, "client")
    finally:
        runtime.shutdown()
