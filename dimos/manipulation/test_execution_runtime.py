"""Black-box owner, reconciliation, deadline, and shutdown coverage."""

from dataclasses import dataclass, replace
from pathlib import Path
import threading
from typing import Any

import pytest

from dimos.manipulation.execution_effects import EffectDone
from dimos.manipulation.execution_models import ActionRecord, Operation, TaskRecord
from dimos.manipulation.execution_runtime import (
    ActionMethod,
    ControlCoordinatorGateway,
    ExecutionPlan,
    ExecutionRuntime,
    ExecutionTopology,
    LifecycleState,
    OperationHandle,
    Outcome,
    PreparedPlan,
    ShutdownState,
    TaskActivity,
    TaskEntry,
    _PendingAction,
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
        self.status_entered = threading.Event()
        self.release = {task: threading.Event() for task in executes}
        self._lock = threading.Lock()
        self.stop_calls = 0
        self.calls_at_stop = 0
        self.stop_entered = threading.Event()

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
        with self._lock:
            self.calls.append((task_name, ActionMethod.STATUS))
        self.status_entered.set()
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
        self.stop_entered.set()
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
        assert result.accepted and result.value is not None
        dispatch = runtime.wait_for_dispatch(result.value, timeout=1)
        assert dispatch.accepted and dispatch.value is not None
        assert len([call for call in rpc.calls if call[1] == "execute"]) == 2
        operation = runtime.snapshot().operation
        assert operation is not None
        assert len(operation.tasks) == 2
        assert runtime.snapshot().state == LifecycleState.RUNNING
        assert all(task.activity == TaskActivity.ACTIVE for task in operation.tasks)
    finally:
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


def test_cancel_during_execute_handles_late_acceptance() -> None:
    entered, release = (threading.Event(), threading.Event())
    rpc = FakeRPC({("a", "execute"): True, ("a", "cancel"): True}, [], entered, release)
    runtime = ExecutionRuntime(lambda: ControlCoordinatorGateway(rpc), poll_interval=10)
    try:
        started = runtime.execute_explicit(plan("a"))
        assert started.accepted and started.value is not None
        assert entered.wait(1)
        assert runtime.cancel().accepted
        release.set()
        terminal = runtime.wait_for_terminal(started.value, timeout=1)
        assert terminal.accepted and terminal.value is not None
        assert any(call[1] == "cancel" for call in rpc.calls)
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


def test_deadline_preserves_unresolved_action_and_stop_rpc_is_deferred() -> None:
    clock = AdjustableClock()
    entered, release = (threading.Event(), threading.Event())
    rpc = FakeRPC({("a", "execute"): True}, [], entered, release)
    runtime = ExecutionRuntime(
        lambda: ControlCoordinatorGateway(rpc),
        monotonic_clock=clock,
        action_timeout=1,
        poll_interval=100,
    )
    runtime.execute_explicit(plan("a"))
    assert entered.wait(1)
    clock.set(2)
    assert not runtime.poll().accepted
    assert runtime.snapshot().state == LifecycleState.FAULT
    assert runtime.snapshot().operation is not None
    release.set()
    runtime.shutdown()
    threading.Event().wait(0.1)
    assert ("", "stop", {}) in rpc.calls


def test_waiters_use_only_the_requested_result_map() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED, "b": Outcome.REJECTED})
    clock = AdjustableClock()
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, action_timeout=100, poll_interval=10
    )
    try:
        handle = runtime.execute_explicit(plan("a", "b")).value
        assert handle is not None
        assert gateway.cancel_entered["a"].wait(1)
        dispatch = runtime.wait_for_dispatch(handle, timeout=0.1)
        assert (
            dispatch.accepted
            and dispatch.value is not None
            and (dispatch.value.outcome == Outcome.REJECTED)
        )
        terminal = assert_public_wait_times_out(
            lambda: runtime.wait_for_terminal(handle, timeout=1)
        )
        assert not terminal.accepted and terminal.diagnostic == "timeout"
        gateway.release["a"].set()
        terminal = runtime.wait_for_terminal(handle, timeout=1)
        assert (
            terminal.accepted
            and terminal.value is not None
            and (terminal.value.outcome == Outcome.REJECTED)
        )
    finally:
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
    assert result.value is not None and (not result.value.success)
    assert "gateway close failed: close exploded" in result.value.diagnostic
    assert result.snapshot is not None
    assert result.snapshot.shutdown == ShutdownState.CLOSING
    assert result.snapshot.shutdown_result == result.value
    assert gateway.stop_calls == 1
    assert runtime.shutdown(timeout=1).accepted is False
    assert gateway.stop_calls == 1


def test_invalid_later_clock_sample_does_not_fault_or_kill_owner() -> None:
    samples = iter((0.0, 0.0, 0.0, float("nan")))
    invalid_seen = threading.Event()
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})

    def clock() -> float:
        value = next(samples, 0.0)
        if value != value:
            invalid_seen.set()
        return value

    runtime = ExecutionRuntime(lambda: gateway, monotonic_clock=clock)
    try:
        started = runtime.execute_explicit(plan("a"))
        assert started.accepted and started.value is not None
        assert runtime.snapshot().state != LifecycleState.FAULT
        assert runtime.snapshot().state == LifecycleState.CANCELLING
        assert runtime.snapshot().shutdown == ShutdownState.CLOSING
        polled = runtime.poll()
        assert not polled.accepted
        assert invalid_seen.wait(1)
        assert gateway.cancel_entered["a"].wait(1)
        assert gateway.count("a", ActionMethod.CANCEL) == 1
        gateway.release["a"].set()
        assert gateway.stop_entered.wait(1)
        assert polled.snapshot is not None
        assert polled.snapshot.state != LifecycleState.FAULT
    finally:
        gateway.release["a"].set()
        runtime.shutdown()


def test_failed_shutdown_drain_rejects_public_commands_after_owner_stops() -> None:
    gateway = BlockingGateway({})

    def fail_stop() -> None:
        gateway.stop_calls += 1
        raise RuntimeError("close exploded")

    gateway.stop = fail_stop  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    result = runtime.shutdown(timeout=1)
    assert not result.accepted
    assert result.value is not None and not result.value.success
    assert runtime.snapshot().shutdown == ShutdownState.CLOSING
    runtime._owner.join(1)
    assert not runtime._owner.is_alive()

    assert not runtime.cancel().accepted
    assert not runtime.cancel_if_current(OperationHandle("plan", "operation", "attempt")).accepted
    reset = runtime.reset()
    assert not reset.accepted
    assert reset.diagnostic == "runtime is closing"
    assert runtime.shutdown(timeout=1).value == result.value


def test_invalid_shutdown_clock_handoff_does_not_deadlock_caller() -> None:
    samples = iter((0.0, float("nan")))
    gateway = BlockingGateway({})
    runtime = ExecutionRuntime(lambda: gateway, monotonic_clock=lambda: next(samples))
    result = runtime.shutdown(timeout=1)
    assert not result.accepted
    assert result.value is not None
    assert result.value.diagnostic == "invalid monotonic clock"
    assert runtime.snapshot().shutdown == ShutdownState.CLOSING


def test_shutdown_interrupts_early_action_deadline_thread() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, action_timeout=10.0, poll_interval=10)
    try:
        started = runtime.execute_explicit(plan("a"))
        assert started.accepted and started.value is not None
        assert runtime.wait_for_dispatch(started.value, timeout=1).accepted
        gateway.release["a"].set()
        assert runtime.shutdown(timeout=1).accepted
    finally:
        gateway.release["a"].set()
        if runtime.snapshot().shutdown != ShutdownState.CLOSED:
            runtime.shutdown(timeout=1)


def test_cancel_if_current_cannot_cancel_replacement_operation() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    gateway.status = lambda task_name: Outcome.COMPLETED  # type: ignore[method-assign]
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        original = runtime.execute_explicit(plan("a"))
        assert original.accepted and original.value is not None
        assert runtime.wait_for_dispatch(original.value, timeout=1).accepted
        runtime.poll()
        assert runtime.wait_for_terminal(original.value, timeout=1).accepted
        replacement = runtime.execute_explicit(plan("a"))
        assert replacement.accepted and replacement.value is not None
        assert runtime.wait_for_dispatch(replacement.value, timeout=1).accepted
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


def test_shutdown_late_execute_acceptance_is_compensated_before_close() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED}, block_execute=True)
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        runtime.execute_explicit(plan("a"))
        assert gateway.execute_entered["a"].wait(1)
        result_holder: list[Any] = []
        thread = threading.Thread(target=lambda: result_holder.append(runtime.shutdown(timeout=1)))
        thread.start()
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
    assert id(prepared) != id(generated)
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
    gateway.status = lambda task_name: Outcome.INACTIVE if task_name == "a" else Outcome.RUNNING  # type: ignore[assignment,method-assign]
    gateway.release["a"].set()
    gateway.release["b"].set()
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=10)
    try:
        handle = runtime.execute_explicit(plan("a", "b")).value
        assert handle is not None
        assert runtime.wait_for_dispatch(handle, timeout=1).accepted
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
    assert closer.is_alive()
    gateway.aux_release.set()
    caller.join(2)
    closer.join(2)
    assert results and results[0].accepted
    assert shutdown_results and shutdown_results[0].accepted
    assert gateway.calls_at_stop == len(gateway.calls)
    assert gateway.stop_calls == 1
    assert not runtime.set_gripper_position("gripper", 0.1).accepted
    assert not runtime.get_gripper_position("gripper").accepted


def test_auxiliary_deadline_waits_for_late_callback_before_shutdown_stop() -> None:
    clock = AdjustableClock()
    gateway = BlockingAuxGateway()
    runtime = ExecutionRuntime(
        lambda: gateway,
        monotonic_clock=clock,
        action_timeout=1,
        poll_interval=100,
    )
    gripper_results: list[Any] = []
    shutdown_results: list[Any] = []
    gripper: threading.Thread | None = None
    closer: threading.Thread | None = None
    shutdown_started = False
    try:
        gripper = threading.Thread(
            target=lambda: gripper_results.append(runtime.set_gripper_position("gripper", 0.5))
        )
        gripper.start()
        assert gateway.aux_entered.wait(1)
        clock.set(2)
        assert not runtime.poll().accepted
        gripper.join(1)
        assert gripper_results and not gripper_results[0].accepted

        closer = threading.Thread(target=lambda: shutdown_results.append(runtime.shutdown(1)))
        shutdown_started = True
        closer.start()
        clock.set(3)
        closer.join(1)
        assert shutdown_results and not shutdown_results[0].accepted
        assert not gateway.stop_entered.is_set()

        gateway.aux_release.set()
        assert gateway.stop_entered.wait(1)
        assert runtime.snapshot().shutdown == ShutdownState.CLOSING
        shutdown_result = runtime.snapshot().shutdown_result
        assert shutdown_result is not None and not shutdown_result.success
    finally:
        gateway.aux_release.set()
        if gripper is not None:
            gripper.join(2)
        if closer is not None:
            closer.join(2)
        if not shutdown_started:
            runtime.shutdown(timeout=1)


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


class AdjustableClock:
    def __init__(self) -> None:
        self.value = 0.0
        self.lock = threading.Lock()
        self._barrier_thread: int | None = None
        self._barrier_pending = False
        self.sample_entered = threading.Event()
        self.sample_release = threading.Event()

    def __call__(self) -> float:
        with self.lock:
            value = self.value
            barrier = self._barrier_pending and self._barrier_thread == threading.get_ident()
            self._barrier_pending = False if barrier else self._barrier_pending
        if barrier:
            self.sample_entered.set()
            assert self.sample_release.wait(1)
        return value

    def arm_current_thread_sample_barrier(self) -> None:
        with self.lock:
            self._barrier_thread = threading.get_ident()
            self._barrier_pending = True
            self.sample_entered.clear()
            self.sample_release.clear()

    def set(self, value: float) -> None:
        with self.lock:
            self.value = value


def assert_public_wait_times_out(wait_call: Any) -> Any:
    result: list[Any] = []
    ready = threading.Event()

    def observe() -> None:
        ready.set()
        result.append(wait_call())

    waiter = threading.Thread(target=observe)
    waiter.start()
    assert ready.wait(1)
    waiter.join(2)
    assert not waiter.is_alive() and len(result) == 1
    return result[0]


class ScriptedGateway:
    def __init__(self, statuses: dict[str, Outcome] | None = None) -> None:
        self.statuses = statuses or {}
        self.calls: list[tuple[str, ActionMethod]] = []
        self.lock = threading.Lock()
        self.execute_entered = threading.Event()
        self.execute_release = threading.Event()
        self.block_execute = False
        self.cancel_entered = threading.Event()
        self.cancel_returned = threading.Event()
        self.cancel_entered_by_task: dict[str, threading.Event] = {}
        self.cancel_returned_by_task: dict[str, threading.Event] = {}
        self.cancel_release = threading.Event()
        self.block_cancel = False
        self.status_entered = threading.Event()
        self.status_release = threading.Event()
        self.block_status = False
        self.cancel_outcome = Outcome.CANCELLED
        self.reset_outcome = Outcome.INACTIVE
        self.reset_entered = threading.Event()
        self.reset_release = threading.Event()
        self.block_reset = False
        self.stop_calls = 0

    def _record(self, task: str, method: ActionMethod) -> None:
        with self.lock:
            self.calls.append((task, method))

    def execute(self, task_name: str, request: Any) -> Outcome:
        self._record(task_name, ActionMethod.EXECUTE)
        self.execute_entered.set()
        if self.block_execute:
            assert self.execute_release.wait(1)
        return Outcome.ACCEPTED

    def cancel(self, task_name: str) -> Outcome:
        self._record(task_name, ActionMethod.CANCEL)
        self.cancel_entered.set()
        self.cancel_entered_by_task.setdefault(task_name, threading.Event()).set()
        if self.block_cancel:
            assert self.cancel_release.wait(1)
        self.cancel_returned.set()
        self.cancel_returned_by_task.setdefault(task_name, threading.Event()).set()
        return self.cancel_outcome

    def status(self, task_name: str) -> Outcome:
        self._record(task_name, ActionMethod.STATUS)
        self.status_entered.set()
        if self.block_status:
            assert self.status_release.wait(1)
        return self.statuses.get(task_name, Outcome.RUNNING)

    def reset(self, task_name: str) -> Outcome:
        self._record(task_name, ActionMethod.RESET)
        self.reset_entered.set()
        if self.block_reset:
            assert self.reset_release.wait(1)
        return self.reset_outcome

    def set_gripper_position(self, hardware_id: str, position: float) -> Outcome:
        return Outcome.ACCEPTED

    def get_gripper_position(self, hardware_id: str) -> float:
        return 0.0

    def stop(self) -> None:
        self.stop_calls += 1
        self.execute_release.set()
        self.cancel_release.set()
        self.status_release.set()

    def count(self, task: str, method: ActionMethod) -> int:
        with self.lock:
            return self.calls.count((task, method))


class SequencedStatusGateway(ScriptedGateway):
    def __init__(self, statuses: list[Outcome]) -> None:
        super().__init__({"a": Outcome.ACCEPTED})
        self.status_sequence = statuses

    def status(self, task_name: str) -> Outcome:
        self._record(task_name, ActionMethod.STATUS)
        self.status_entered.set()
        return self.status_sequence.pop(0)


def scripted_plan(*names: str) -> ExecutionPlan:
    return ExecutionPlan(
        "generated", tuple(TaskEntry(name, name, {"trajectory": name}) for name in names)
    )


def start_scripted(runtime: ExecutionRuntime, gateway: ScriptedGateway, task: str = "a") -> Any:
    started = runtime.execute_explicit(scripted_plan(task))
    assert started.accepted and started.value is not None
    assert gateway.execute_entered.wait(1)
    dispatched = runtime.wait_for_dispatch(started.value, timeout=1)
    assert dispatched.accepted and dispatched.value is not None
    assert runtime.snapshot().state == LifecycleState.RUNNING
    return started.value


def close_scripted(runtime: ExecutionRuntime, gateway: ScriptedGateway) -> None:
    gateway.stop()
    if runtime.snapshot().shutdown != ShutdownState.CLOSED:
        runtime.shutdown(timeout=1)


@pytest.mark.parametrize("active_status", [Outcome.RUNNING, Outcome.ACCEPTED])
def test_reset_reconciles_active_status_then_inactive_and_succeeds(
    active_status: Outcome,
) -> None:
    clock = AdjustableClock()
    gateway = SequencedStatusGateway([active_status, Outcome.INACTIVE])
    gateway.cancel_outcome = Outcome.UNKNOWN
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, physical_operation_timeout=60, poll_interval=100
    )
    try:
        start_scripted(runtime, gateway)
        clock.set(60)
        runtime.poll()
        assert gateway.cancel_returned.wait(1)
        runtime.poll()
        assert runtime.snapshot().state == LifecycleState.FAULT

        gateway.cancel_outcome = Outcome.CANCELLED
        gateway.status_entered.clear()
        gateway.cancel_returned.clear()
        gateway.reset_entered.clear()
        reset = runtime.reset()
        assert reset.accepted and reset.value is not None
        assert gateway.status_entered.wait(1)
        assert gateway.count("a", ActionMethod.STATUS) == 1
        gateway.status_entered.clear()
        assert gateway.cancel_returned.wait(1)
        assert gateway.count("a", ActionMethod.CANCEL) == 2
        assert gateway.status_entered.wait(1)
        assert gateway.count("a", ActionMethod.STATUS) == 2
        assert gateway.reset_entered.wait(1)
        assert gateway.count("a", ActionMethod.RESET) == 1
        result = runtime.wait_for_reset(reset.value, timeout=1)
        assert result.accepted and result.value is not None and result.value.success
        assert result.snapshot is not None and result.snapshot.state == LifecycleState.IDLE
        assert [method for _, method in gateway.calls] == [
            ActionMethod.EXECUTE,
            ActionMethod.CANCEL,
            ActionMethod.STATUS,
            ActionMethod.CANCEL,
            ActionMethod.STATUS,
            ActionMethod.RESET,
        ]
    finally:
        gateway.cancel_release.set()
        gateway.reset_release.set()
        close_scripted(runtime, gateway)


def test_queued_poll_uses_injected_clock_and_cancels_due_operation_once() -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway()
    runtime = ExecutionRuntime(
        lambda: gateway,
        monotonic_clock=clock,
        physical_operation_timeout=60,
        action_timeout=1000,
        poll_interval=100,
    )
    try:
        start_scripted(runtime, gateway)
        clock.set(60)
        poll_result: list[Any] = []
        queued = threading.Thread(target=lambda: poll_result.append(runtime.poll()))
        queued.start()
        queued.join(1)
        assert not queued.is_alive()
        assert len(poll_result) == 1
        assert poll_result[0].snapshot is not None
        assert gateway.count("a", ActionMethod.STATUS) == 0
        assert gateway.count("a", ActionMethod.CANCEL) == 1
    finally:
        runtime.shutdown(timeout=0)


def test_stale_a_deadline_at_sixty_cannot_cancel_b_started_at_thirty() -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway({"a": Outcome.COMPLETED, "b": Outcome.RUNNING})
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, physical_operation_timeout=60, poll_interval=100
    )
    try:
        a = start_scripted(runtime, gateway)
        runtime.poll()
        assert runtime.wait_for_terminal(a, timeout=1).accepted
        clock.set(30)
        b = start_scripted(runtime, gateway, "b")
        gateway.status_entered.clear()
        clock.set(61)
        runtime.poll()
        assert gateway.status_entered.wait(1)
        snapshot = runtime.snapshot()
        assert snapshot.operation is not None
        assert snapshot.operation.handle == b
        assert gateway.count("b", ActionMethod.STATUS) == 1
        assert gateway.count("b", ActionMethod.CANCEL) == 0
    finally:
        close_scripted(runtime, gateway)


def test_cancel_clearance_survives_old_deadline_with_replacement_running() -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway()
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, physical_operation_timeout=60, poll_interval=100
    )
    try:
        a = start_scripted(runtime, gateway)
        assert runtime.cancel().accepted
        assert gateway.cancel_entered.wait(1)
        assert gateway.cancel_returned.wait(1)
        runtime.poll()
        assert runtime.wait_for_terminal(a, timeout=1).accepted
        clock.set(30)
        start_scripted(runtime, gateway, "b")
        clock.set(61)
        runtime.poll()
        assert gateway.count("a", ActionMethod.CANCEL) == 1
        assert gateway.count("b", ActionMethod.CANCEL) == 0
    finally:
        close_scripted(runtime, gateway)


def test_reset_rejects_while_fault_cleanup_rpc_is_unresolved() -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway({"a": Outcome.FAILED})
    gateway.block_cancel = True
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, physical_operation_timeout=60, poll_interval=100
    )
    try:
        start_scripted(runtime, gateway)
        runtime.poll()
        assert gateway.cancel_entered.wait(1)
        snapshot = runtime.snapshot()
        assert snapshot.operation is not None
        wait = threading.Event()
        for _ in range(100):
            if runtime.snapshot().state == LifecycleState.FAULT:
                break
            wait.wait(0.01)
        assert runtime.snapshot().state == LifecycleState.FAULT
        reset = runtime.reset()
        assert not reset.accepted
        assert reset.value is None
        assert reset.diagnostic == "reset blocked by unresolved coordinator RPC"
        assert runtime.snapshot().fault is not None
    finally:
        gateway.cancel_release.set()
        runtime.shutdown(timeout=0)


def test_shutdown_clearance_blocks_cancel_and_returns_truthful_deadline_result() -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway()
    gateway.block_cancel = True
    runtime = ExecutionRuntime(
        lambda: gateway,
        monotonic_clock=clock,
        physical_operation_timeout=60,
        action_timeout=100,
        poll_interval=100,
    )
    result: list[Any] = []
    try:
        start_scripted(runtime, gateway)
        thread = threading.Thread(target=lambda: result.append(runtime.shutdown(timeout=100)))
        thread.start()
        assert gateway.cancel_entered.wait(1)
        assert runtime.snapshot().shutdown == ShutdownState.CLOSING
        clock.set(61)
        gateway.cancel_release.set()
        thread.join(1)
        assert len(result) == 1 and result[0].value is not None
        assert result[0].value.success
        assert gateway.count("a", ActionMethod.CANCEL) == 1
    finally:
        gateway.cancel_release.set()
        close_scripted(runtime, gateway)


def test_reset_completes_before_terminal_waiter_observes_retained_diagnostic() -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway()
    gateway.cancel_outcome = Outcome.UNKNOWN
    gateway.statuses["a"] = Outcome.INACTIVE
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, physical_operation_timeout=60, poll_interval=100
    )
    try:
        handle = start_scripted(runtime, gateway)
        clock.set(60)
        runtime.poll()
        assert gateway.cancel_returned.wait(1)
        runtime.poll()
        assert runtime.snapshot().state == LifecycleState.FAULT
        gateway.cancel_outcome = Outcome.CANCELLED
        reset = runtime.reset()
        assert reset.accepted and reset.value is not None
        reset_result = runtime.wait_for_reset(reset.value, timeout=1)
        assert reset_result.accepted and reset_result.value is not None
        assert reset_result.value.success
        waited = runtime.wait_for_terminal(handle, timeout=float("inf"))
        assert not waited.accepted
        assert waited.diagnostic == "cancellation is uncertain"
    finally:
        close_scripted(runtime, gateway)


def test_safe_physical_expiry_returns_failed_terminal_result_and_stable_diagnostic() -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway()
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, physical_operation_timeout=60, poll_interval=100
    )
    try:
        handle = start_scripted(runtime, gateway)
        clock.set(60)
        runtime.poll()
        assert gateway.cancel_returned.wait(1)
        result = runtime.wait_for_terminal(handle, timeout=1)
        assert result.accepted and result.value is not None
        assert result.value.outcome == Outcome.FAILED
        assert result.value.diagnostic == "physical operation deadline exceeded"
        assert runtime.wait_for_terminal(handle, timeout=1).value == result.value
    finally:
        close_scripted(runtime, gateway)


def test_final_sequential_execute_acceptance_arms_physical_deadline() -> None:
    clock = AdjustableClock()
    gateway = BlockingGateway({"a": Outcome.ACCEPTED, "b": Outcome.ACCEPTED}, block_execute=True)
    runtime = ExecutionRuntime(
        lambda: gateway,
        monotonic_clock=clock,
        physical_operation_timeout=60,
        action_timeout=1000,
        poll_interval=100,
    )
    try:
        started = runtime.execute_explicit(plan("a", "b"))
        assert started.accepted and started.value is not None
        assert gateway.execute_entered["a"].wait(1)
        clock.set(59)
        runtime.poll()
        assert gateway.count("a", ActionMethod.CANCEL) == 0
        gateway.release["a"].set()
        assert gateway.execute_entered["b"].wait(1)
        clock.set(60)
        runtime.poll()
        assert gateway.count("a", ActionMethod.CANCEL) == 0
        gateway.release["b"].set()
        assert runtime.wait_for_dispatch(started.value, timeout=1).accepted
        clock.set(120)
        runtime.poll()
        assert gateway.cancel_entered["a"].wait(1)
        assert gateway.cancel_entered["b"].wait(1)
    finally:
        for release in gateway.release.values():
            release.set()
        runtime.shutdown()


def test_blocked_sequential_dispatch_does_not_expire_before_final_acceptance() -> None:
    clock = AdjustableClock()
    gateway = BlockingGateway({"a": Outcome.ACCEPTED, "b": Outcome.ACCEPTED}, block_execute=True)
    runtime = ExecutionRuntime(
        lambda: gateway,
        monotonic_clock=clock,
        physical_operation_timeout=60,
        action_timeout=1000,
        poll_interval=100,
    )
    try:
        started = runtime.execute_explicit(plan("a", "b"))
        assert started.accepted and gateway.execute_entered["a"].wait(1)
        clock.set(600)
        runtime.poll()
        assert gateway.count("a", ActionMethod.CANCEL) == 0
        gateway.release["a"].set()
        assert gateway.execute_entered["b"].wait(1)
        assert gateway.count("a", ActionMethod.CANCEL) == 0
        gateway.release["b"].set()
        assert runtime.wait_for_dispatch(started.value, timeout=1).accepted  # type: ignore[arg-type]
        assert gateway.count("a", ActionMethod.CANCEL) == 0
        assert gateway.count("b", ActionMethod.CANCEL) == 0
    finally:
        for release in gateway.release.values():
            release.set()
        runtime.shutdown()


def test_finite_terminal_wait_is_observer_only_before_physical_expiry() -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway()
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, physical_operation_timeout=60, poll_interval=100
    )
    try:
        handle = start_scripted(runtime, gateway)
        clock.set(59)
        observed: list[Any] = []
        ready = threading.Event()

        def observe() -> None:
            ready.set()
            observed.append(runtime.wait_for_terminal(handle, timeout=1))

        waiter = threading.Thread(target=observe)
        waiter.start()
        assert ready.wait(1)
        clock.set(60)
        waiter.join(2)
        assert not waiter.is_alive() and len(observed) == 1
        waited = observed[0]
        assert not waited.accepted and waited.diagnostic == "timeout"
        assert gateway.count("a", ActionMethod.CANCEL) == 0
        clock.set(60)
        runtime.poll()
        assert gateway.cancel_returned.wait(1)
    finally:
        close_scripted(runtime, gateway)


def test_infinite_terminal_wait_returns_after_public_cancellation() -> None:
    gateway = ScriptedGateway()
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    observed: list[Any] = []
    ready = threading.Event()
    try:
        handle = start_scripted(runtime, gateway)

        def observe() -> None:
            ready.set()
            observed.append(runtime.wait_for_terminal(handle, timeout=float("inf")))

        waiter = threading.Thread(target=observe)
        waiter.start()
        assert ready.wait(1)
        assert runtime.cancel().accepted
        waiter.join(1)
        assert len(observed) == 1
        assert observed[0].accepted
        assert observed[0].value is not None
        assert observed[0].value.outcome == Outcome.CANCELLED
    finally:
        close_scripted(runtime, gateway)


def test_authoritative_completion_before_physical_expiry_wins() -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway({"a": Outcome.COMPLETED})
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, physical_operation_timeout=60, poll_interval=100
    )
    try:
        handle = start_scripted(runtime, gateway)
        clock.set(59)
        runtime.poll()
        result = runtime.wait_for_terminal(handle, timeout=1)
        assert result.accepted and result.value is not None
        assert result.value.outcome == Outcome.COMPLETED
        clock.set(60)
        runtime.poll()
        assert gateway.count("a", ActionMethod.CANCEL) == 0
    finally:
        close_scripted(runtime, gateway)


def test_multitask_physical_expiry_cancels_each_task_and_reconciles_terminally() -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway()
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, physical_operation_timeout=60, poll_interval=100
    )
    try:
        started = runtime.execute_explicit(scripted_plan("a", "b"))
        assert started.accepted and started.value is not None
        gateway.cancel_entered_by_task["a"] = threading.Event()
        gateway.cancel_entered_by_task["b"] = threading.Event()
        gateway.cancel_returned_by_task["a"] = threading.Event()
        gateway.cancel_returned_by_task["b"] = threading.Event()
        assert gateway.execute_entered.wait(1)
        assert runtime.wait_for_dispatch(started.value, timeout=1).accepted
        clock.set(60)
        runtime.poll()
        assert gateway.cancel_returned_by_task["a"].wait(1)
        assert gateway.cancel_returned_by_task["b"].wait(1)
        runtime.poll()
        assert gateway.count("a", ActionMethod.CANCEL) == 1
        assert gateway.count("b", ActionMethod.CANCEL) == 1
        terminal = runtime.wait_for_terminal(started.value, timeout=1)
        assert terminal.accepted and terminal.value is not None
        assert terminal.value.outcome == Outcome.FAILED
    finally:
        close_scripted(runtime, gateway)


@pytest.mark.parametrize("cancel_outcome", [Outcome.UNKNOWN, Outcome.FAILED])
def test_physical_expiry_uncertain_cancellation_is_correlated_to_fault(
    cancel_outcome: Outcome,
) -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway()
    gateway.cancel_outcome = cancel_outcome
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, physical_operation_timeout=60, poll_interval=100
    )
    try:
        handle = start_scripted(runtime, gateway)
        clock.set(60)
        runtime.poll()
        assert gateway.cancel_returned.wait(1)
        observed = runtime.wait_for_terminal(handle, timeout=1)
        assert not observed.accepted
        assert observed.diagnostic in {"cancellation is uncertain", "cancellation failed"}
        assert observed.snapshot is not None
        assert observed.snapshot.state == LifecycleState.FAULT
    finally:
        close_scripted(runtime, gateway)


def test_shutdown_unknown_cancels_each_task_once_and_drains() -> None:
    gateway = BlockingGateway(
        {"a": Outcome.ACCEPTED, "b": Outcome.ACCEPTED},
        {"a": Outcome.UNKNOWN, "b": Outcome.UNKNOWN},
    )
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    result: list[Any] = []
    try:
        started = runtime.execute_explicit(plan("a", "b"))
        assert started.accepted and started.value is not None
        assert runtime.wait_for_dispatch(started.value, timeout=1).accepted
        closer = threading.Thread(target=lambda: result.append(runtime.shutdown(timeout=1)))
        closer.start()
        assert gateway.cancel_entered["a"].wait(1)
        assert gateway.cancel_entered["b"].wait(1)
        gateway.release["a"].set()
        gateway.release["b"].set()
        closer.join(2)
        assert result and not result[0].accepted
        assert gateway.count("a", ActionMethod.CANCEL) == 1
        assert gateway.count("b", ActionMethod.CANCEL) == 1
        assert runtime.snapshot().shutdown == ShutdownState.CLOSING
        assert runtime.snapshot().shutdown_result is not None
        assert not runtime.snapshot().shutdown_result.success
    finally:
        for release in gateway.release.values():
            release.set()


def test_shutdown_latches_cancel_only_before_physical_expiry_and_admits_one_cancel() -> None:
    clock = AdjustableClock()
    gateway = ScriptedGateway()
    gateway.block_cancel = True
    runtime = ExecutionRuntime(
        lambda: gateway,
        monotonic_clock=clock,
        physical_operation_timeout=60,
        action_timeout=100,
        poll_interval=100,
    )
    result: list[Any] = []
    try:
        start_scripted(runtime, gateway)
        thread = threading.Thread(target=lambda: result.append(runtime.shutdown(timeout=100)))
        thread.start()
        assert gateway.cancel_entered.wait(1)
        assert runtime.snapshot().shutdown == ShutdownState.CLOSING
        assert not runtime.poll().accepted
        clock.set(61)
        gateway.cancel_release.set()
        thread.join(1)
        assert result and result[0].value is not None and result[0].value.success
        assert gateway.count("a", ActionMethod.CANCEL) == 1
    finally:
        gateway.cancel_release.set()
        close_scripted(runtime, gateway)


def test_unresolved_shutdown_deadline_has_no_late_effects() -> None:
    clock = AdjustableClock()
    gateway = BlockingGateway({"a": Outcome.ACCEPTED}, block_execute=True)
    runtime = ExecutionRuntime(lambda: gateway, monotonic_clock=clock, poll_interval=100)
    shutdown_started = False
    try:
        started = runtime.execute_explicit(plan("a"))
        assert started.accepted and gateway.execute_entered["a"].wait(1)
        shutdown_started = True
        result = runtime.shutdown(timeout=0)
        assert not result.accepted
        assert result.value is not None and not result.value.success
        assert runtime.snapshot().shutdown == ShutdownState.CLOSING
        cancel_count = gateway.count("a", ActionMethod.CANCEL)
        gateway.release["a"].set()
        assert gateway.stop_entered.wait(1)
        assert gateway.count("a", ActionMethod.CANCEL) == cancel_count + 1
    finally:
        gateway.release["a"].set()
        if not shutdown_started:
            runtime.shutdown(timeout=0)


@pytest.mark.parametrize(
    ("execute_outcome", "expected_cancel"),
    [(Outcome.ACCEPTED, 1), (Outcome.REJECTED, 0)],
)
def test_late_action_deadline_completion_is_correlated(
    execute_outcome: Outcome, expected_cancel: int
) -> None:
    clock = AdjustableClock()
    gateway = BlockingGateway({"a": execute_outcome}, block_execute=True)
    runtime = ExecutionRuntime(
        lambda: gateway, monotonic_clock=clock, action_timeout=1, poll_interval=100
    )
    try:
        started = runtime.execute_explicit(plan("a"))
        assert started.accepted and started.value is not None
        assert gateway.execute_entered["a"].wait(1)
        clock.set(2)
        runtime.poll()
        gateway.release["a"].set()
        dispatch = runtime.wait_for_dispatch(started.value, timeout=1)
        assert dispatch.accepted and dispatch.value is not None
        assert dispatch.value.outcome == Outcome.UNKNOWN
        if expected_cancel:
            assert gateway.cancel_entered["a"].wait(1)
            gateway.release["a"].set()
        assert gateway.count("a", ActionMethod.CANCEL) == expected_cancel
    finally:
        gateway.release["a"].set()
        runtime.shutdown()


def test_unknown_partial_dispatch_compensates_only_admitted_effects() -> None:
    gateway = BlockingGateway(
        {"a": Outcome.ACCEPTED, "b": Outcome.UNKNOWN, "c": Outcome.ACCEPTED},
        {"a": Outcome.CANCELLED, "b": Outcome.CANCELLED},
    )
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    try:
        started = runtime.execute_explicit(plan("a", "b", "c"))
        assert started.accepted and started.value is not None
        assert gateway.cancel_entered["a"].wait(1)
        assert gateway.cancel_entered["b"].wait(1)
        assert not any(
            task == "c" and method == ActionMethod.EXECUTE for task, method in gateway.calls
        )
        for release in gateway.release.values():
            release.set()
        dispatch = runtime.wait_for_dispatch(started.value, timeout=1)
        assert dispatch.accepted and dispatch.value is not None
        assert dispatch.value.outcome == Outcome.UNKNOWN
        assert gateway.count("a", ActionMethod.CANCEL) == 1
        assert gateway.count("b", ActionMethod.CANCEL) == 1
    finally:
        for release in gateway.release.values():
            release.set()
        runtime.shutdown()


def _install_pending_action(
    runtime: ExecutionRuntime,
    *,
    active_handle: OperationHandle,
    pending_handle: OperationHandle,
    task_id: str = "task",
    active_task_id: str = "task",
    current_action_id: str = "new-action",
    pending_action_id: str = "old-action",
) -> tuple[ActionRecord, ActionRecord]:
    old_action = ActionRecord(pending_action_id, ActionMethod.EXECUTE, 1.0, 1e9)
    new_action = ActionRecord(current_action_id, ActionMethod.STATUS, 2.0, 1e9 + 1)
    task = TaskRecord(
        active_task_id,
        "task",
        plan("task").entries[0],
        TaskActivity.ACTIVE,
        new_action,
    )
    operation = Operation(active_handle, plan("task"), (task,))
    runtime._commit(state=LifecycleState.RUNNING, active=operation)
    runtime._pending[pending_action_id] = _PendingAction(
        pending_handle,
        "task",
        replace(task, task_id=task_id, action=old_action),
        ActionMethod.EXECUTE,
        1e9,
    )
    runtime._pending[current_action_id] = _PendingAction(
        active_handle,
        "task",
        task,
        ActionMethod.STATUS,
        1e9 + 1,
    )
    return old_action, new_action


def test_owner_completion_retires_old_pending_but_preserves_new_action() -> None:
    runtime = ExecutionRuntime(
        lambda: BlockingGateway({"task": Outcome.RUNNING}), poll_interval=100
    )
    try:
        handle = OperationHandle("plan", "operation", "attempt")
        _, newer = _install_pending_action(runtime, active_handle=handle, pending_handle=handle)
        before = runtime.snapshot()
        runtime._events.put(EffectDone("old-action", Outcome.ACCEPTED))
        runtime._submit(lambda: None)
        after = runtime.snapshot()
        assert "old-action" not in runtime._pending
        assert "new-action" in runtime._pending
        assert after == before
        assert after.operation is not None
        assert after.operation.tasks[0].action == newer
    finally:
        runtime._pending.clear()
        runtime._commit(active=None, state=LifecycleState.IDLE)
        runtime.shutdown()


@pytest.mark.parametrize("kind", ["unknown", "handle", "task"])
def test_owner_uncorrelatable_completion_preserves_pending_and_snapshot(kind: str) -> None:
    runtime = ExecutionRuntime(
        lambda: BlockingGateway({"task": Outcome.RUNNING}), poll_interval=100
    )
    try:
        handle = OperationHandle("plan", "operation", "attempt")
        pending_handle = (
            OperationHandle("other", "operation", "attempt") if kind == "handle" else handle
        )
        _install_pending_action(
            runtime,
            active_handle=handle,
            pending_handle=pending_handle,
            task_id="missing" if kind == "task" else "task",
        )
        before = runtime.snapshot()
        action_id = "unknown-action" if kind == "unknown" else "old-action"
        runtime._events.put(EffectDone(action_id, Outcome.ACCEPTED))
        runtime._submit(lambda: None)
        assert runtime.snapshot() == before
        assert set(runtime._pending) == {"old-action", "new-action"}
    finally:
        runtime._pending.clear()
        runtime._commit(active=None, state=LifecycleState.IDLE)
        runtime.shutdown()


def test_concurrent_explicit_admission_accepts_one_and_correlates_rejection() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    start = threading.Barrier(3)
    results: list[Any] = []

    def submit() -> None:
        start.wait()
        results.append(runtime.execute_explicit(plan("a")))

    threads = [threading.Thread(target=submit) for _ in range(2)]
    try:
        for thread in threads:
            thread.start()
        start.wait()
        for thread in threads:
            thread.join(1)
        assert len(results) == 2
        assert sum(result.accepted for result in results) == 1
        accepted = next(result for result in results if result.accepted)
        assert accepted.value is not None
        rejected = next(result for result in results if not result.accepted)
        assert rejected.value is None
        snapshot = runtime.snapshot()
        assert snapshot.operation is not None
        assert snapshot.operation.handle == accepted.value
        gateway.release["a"].set()
        assert gateway.count("a", ActionMethod.EXECUTE) == 1
    finally:
        gateway.release["a"].set()
        runtime.cancel()
        runtime.shutdown()


def test_concurrent_ready_execution_consumes_ready_plan_once() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    start = threading.Barrier(3)
    results: list[Any] = []
    token = runtime.start_planning()
    assert token is not None
    completed = runtime.complete_planning(token, plan("a"))
    assert completed.accepted and completed.value is not None
    plan_id = completed.value

    def execute_ready() -> None:
        start.wait()
        results.append(runtime.execute_ready())

    threads = [threading.Thread(target=execute_ready) for _ in range(2)]
    try:
        for thread in threads:
            thread.start()
        start.wait()
        for thread in threads:
            thread.join(1)
        assert all(not thread.is_alive() for thread in threads)
        assert len(results) == 2
        assert sum(result.accepted for result in results) == 1
        assert runtime.snapshot().ready_plan is None
        accepted = next(result for result in results if result.accepted)
        rejected = next(result for result in results if not result.accepted)
        assert accepted.value is not None
        assert accepted.value.plan_id == plan_id
        assert not rejected.accepted
        assert rejected.value is None
        assert rejected.diagnostic == "no ready plan"
        assert gateway.count("a", ActionMethod.EXECUTE) == 1
    finally:
        gateway.release["a"].set()
        runtime.cancel()
        runtime.shutdown()


def test_active_operation_rejects_planning_and_explicit_execution_but_accepts_cancel() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    try:
        started = runtime.execute_explicit(plan("a"))
        assert started.accepted and started.value is not None
        assert gateway.execute_entered["a"].wait(1)
        assert runtime.wait_for_dispatch(started.value, timeout=1).accepted
        original = runtime.snapshot()
        assert original.state == LifecycleState.RUNNING
        assert not runtime.start_planning()
        rejected = runtime.execute_explicit(plan("a"))
        assert not rejected.accepted
        assert rejected.value is None
        assert runtime.snapshot().operation == original.operation
        assert gateway.count("a", ActionMethod.EXECUTE) == 1
        assert runtime.snapshot().operation == original.operation
        assert runtime.cancel().accepted
        gateway.release["a"].set()
    finally:
        gateway.release["a"].set()
        runtime.shutdown()


def test_sequential_rejection_skips_remaining_task_and_compensates_prior_acceptance() -> None:
    gateway = BlockingGateway(
        {"a": Outcome.ACCEPTED, "b": Outcome.REJECTED, "c": Outcome.ACCEPTED},
        {"a": Outcome.CANCELLED},
    )
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    try:
        started = runtime.execute_explicit(plan("a", "b", "c"))
        assert started.accepted and started.value is not None
        assert gateway.cancel_entered["a"].wait(1)
        assert gateway.calls[:3] == [
            ("a", ActionMethod.EXECUTE),
            ("b", ActionMethod.EXECUTE),
            ("a", ActionMethod.CANCEL),
        ]
        assert not any(
            task == "c" and method == ActionMethod.EXECUTE for task, method in gateway.calls
        )
        gateway.release["a"].set()
        dispatch = runtime.wait_for_dispatch(started.value, timeout=1)
        terminal = runtime.wait_for_terminal(started.value, timeout=1)
        assert dispatch.accepted and dispatch.value is not None
        assert terminal.accepted and terminal.value is not None
        assert dispatch.value.outcome == Outcome.REJECTED
        assert terminal.value.outcome == Outcome.REJECTED
        assert gateway.count("a", ActionMethod.EXECUTE) == 1
        assert gateway.count("a", ActionMethod.CANCEL) == 1
        assert gateway.count("b", ActionMethod.EXECUTE) == 1
        assert gateway.count("c", ActionMethod.EXECUTE) == 0
        assert gateway.count("c", ActionMethod.CANCEL) == 0
    finally:
        for release in gateway.release.values():
            release.set()
        runtime.shutdown()


def test_cancel_during_unresolved_dispatch_retains_cancelled_result_after_late_acceptance() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED, "b": Outcome.ACCEPTED}, block_execute=True)
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    try:
        started = runtime.execute_explicit(plan("a", "b"))
        assert started.accepted and started.value is not None
        assert gateway.execute_entered["a"].wait(1)
        assert runtime.cancel().accepted
        gateway.release["a"].set()
        dispatch = runtime.wait_for_dispatch(started.value, timeout=1)
        terminal = runtime.wait_for_terminal(started.value, timeout=1)
        assert dispatch.accepted and dispatch.value is not None
        assert terminal.accepted and terminal.value is not None
        assert dispatch.value.outcome == Outcome.CANCELLED
        assert terminal.value.outcome == Outcome.CANCELLED
        assert gateway.count("a", ActionMethod.EXECUTE) == 1
        assert gateway.calls[:2] == [
            ("a", ActionMethod.EXECUTE),
            ("a", ActionMethod.CANCEL),
        ]
        assert gateway.count("a", ActionMethod.CANCEL) == 1
        assert gateway.count("b", ActionMethod.EXECUTE) == 0
        assert gateway.count("b", ActionMethod.CANCEL) == 0
    finally:
        for release in gateway.release.values():
            release.set()
        runtime.shutdown()


def test_unknown_execute_enters_fault_and_compensates_without_terminal_success() -> None:
    gateway = BlockingGateway(
        {"a": Outcome.UNKNOWN, "b": Outcome.REJECTED},
        {"a": Outcome.UNKNOWN, "b": Outcome.FAILED},
    )
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    try:
        started = runtime.execute_explicit(plan("a", "b"))
        assert started.accepted and started.value is not None
        assert gateway.cancel_entered["a"].wait(1)
        snapshot = runtime.snapshot()
        assert snapshot.state == LifecycleState.FAULT
        assert snapshot.operation is not None and snapshot.operation.uncertain
        assert gateway.count("a", ActionMethod.EXECUTE) == 1
        assert gateway.count("b", ActionMethod.EXECUTE) == 0
        assert gateway.count("a", ActionMethod.CANCEL) == 1
        assert gateway.count("b", ActionMethod.CANCEL) == 0
        gateway.release["a"].set()
        dispatch = runtime.wait_for_dispatch(started.value, timeout=1)
        assert dispatch.accepted and dispatch.value is not None
        assert dispatch.value.outcome == Outcome.UNKNOWN
        assert dispatch.value.diagnostic
        assert "task=a" in dispatch.value.diagnostic
        assert "outcome=unknown" in dispatch.value.diagnostic
        terminal = runtime.wait_for_terminal(started.value, timeout=1)
        assert not terminal.accepted
        assert terminal.diagnostic == dispatch.value.diagnostic
    finally:
        gateway.release["a"].set()
        runtime.shutdown()


def test_unknown_status_enters_fault_without_successful_terminal_result() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED, "b": Outcome.ACCEPTED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    try:
        started = runtime.execute_explicit(plan("a", "b"))
        assert started.accepted and started.value is not None
        handle = started.value
        assert gateway.execute_entered["a"].wait(1)
        assert runtime.wait_for_dispatch(started.value, timeout=1).accepted
        runtime.poll()
        assert gateway.status_entered.wait(1)
        assert gateway.cancel_entered["a"].wait(1)
        assert gateway.cancel_entered["b"].wait(1)
        gateway.release["a"].set()
        gateway.release["b"].set()
        terminal = runtime.wait_for_terminal(handle, timeout=1)
        assert not terminal.accepted
        snapshot = runtime.snapshot()
        assert snapshot.operation is not None
        assert terminal.diagnostic == runtime.snapshot().diagnostic
        assert terminal.diagnostic
        assert "task=a" in terminal.diagnostic
        assert "outcome=unknown" in terminal.diagnostic
        assert "status is unsafe" in terminal.diagnostic
        assert runtime.snapshot().state == LifecycleState.FAULT
        assert gateway.count("a", ActionMethod.STATUS) == 1
        assert gateway.count("a", ActionMethod.CANCEL) == 1
        assert gateway.count("b", ActionMethod.CANCEL) == 1
    finally:
        for release in gateway.release.values():
            release.set()
        runtime.shutdown()


def test_failed_cancel_latches_fault_and_preserves_reset_required_operation() -> None:
    gateway = BlockingGateway({"a": Outcome.ACCEPTED}, {"a": Outcome.FAILED})
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    try:
        started = runtime.execute_explicit(plan("a"))
        assert started.accepted and started.value is not None
        assert gateway.execute_entered["a"].wait(1)
        assert runtime.wait_for_dispatch(started.value, timeout=1).accepted
        assert runtime.cancel().accepted
        assert gateway.cancel_entered["a"].wait(1)
        gateway.release["a"].set()
        terminal = runtime.wait_for_terminal(started.value, timeout=1)
        assert not terminal.accepted
        assert terminal.diagnostic == "cancellation failed"
        assert runtime.snapshot().state == LifecycleState.FAULT
        snapshot = runtime.snapshot()
        assert snapshot.operation is not None
        assert snapshot.operation.tasks[0].reset_required
        assert snapshot.diagnostic == "cancellation failed"
        assert gateway.count("a", ActionMethod.CANCEL) == 1
    finally:
        gateway.release["a"].set()
        runtime.shutdown()


def test_reset_calls_share_one_public_handle_until_first_result_is_ready() -> None:
    gateway = ScriptedGateway({"a": Outcome.FAILED})
    gateway.block_reset = True
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    try:
        start_scripted(runtime, gateway)
        runtime.poll()
        snapshot = runtime.snapshot()
        assert snapshot.operation is not None
        assert not runtime.wait_for_terminal(snapshot.operation.handle, timeout=1).accepted
        gateway.statuses["a"] = Outcome.INACTIVE
        assert gateway.cancel_returned.wait(1)
        runtime.poll()
        assert runtime.snapshot().state == LifecycleState.FAULT
        first = runtime.reset()
        assert first.accepted and first.value is not None
        assert gateway.reset_entered.wait(1)
        second = runtime.reset()
        assert second.accepted and second.value == first.value
        gateway.reset_release.set()
        result = runtime.wait_for_reset(first.value, timeout=1)
        assert result.accepted and result.value is not None and result.value.success
        assert runtime.snapshot().state == LifecycleState.IDLE
        assert gateway.count("a", ActionMethod.RESET) == 1
    finally:
        gateway.reset_release.set()
        close_scripted(runtime, gateway)


def test_unsafe_reset_remains_faulted_without_an_in_process_retry() -> None:
    gateway = ScriptedGateway()
    gateway.block_cancel = True
    status_calls = 0

    def status(task_name: str) -> Outcome:
        nonlocal status_calls
        status_calls += 1
        return Outcome.FAILED if status_calls == 1 else Outcome.UNKNOWN

    gateway.status = status  # type: ignore[assignment,method-assign]
    runtime = ExecutionRuntime(lambda: gateway, poll_interval=100)
    try:
        start_scripted(runtime, gateway)
        runtime.poll()
        snapshot = runtime.snapshot()
        assert snapshot.operation is not None
        assert not runtime.wait_for_terminal(snapshot.operation.handle, timeout=1).accepted
        assert gateway.cancel_entered.wait(1)
        reset = runtime.reset()
        assert not reset.accepted and reset.value is None
        assert reset.diagnostic == "reset blocked by unresolved coordinator RPC"
        assert runtime.snapshot().state == LifecycleState.FAULT
        assert runtime.snapshot().diagnostic
        assert gateway.count("a", ActionMethod.RESET) == 0
    finally:
        gateway.cancel_release.set()
        close_scripted(runtime, gateway)


def test_public_results_retain_current_and_recent_handles_and_age_out_old_handles() -> None:
    gateway = BlockingGateway({"a": Outcome.REJECTED})
    clock = AdjustableClock()
    runtime = ExecutionRuntime(lambda: gateway, monotonic_clock=clock, poll_interval=100)
    handles: list[Any] = []
    try:
        for _ in range(20):
            started = runtime.execute_explicit(plan("a"))
            assert started.accepted and started.value is not None
            handles.append(started.value)
            dispatch = runtime.wait_for_dispatch(started.value, timeout=1)
            terminal = runtime.wait_for_terminal(started.value, timeout=1)
            assert dispatch.accepted and dispatch.value is not None
            assert terminal.accepted and terminal.value is not None
            assert dispatch.value.handle == started.value
            assert terminal.value.handle == started.value
        for handle in handles[-2:]:
            dispatch = runtime.wait_for_dispatch(handle, timeout=1)
            terminal = runtime.wait_for_terminal(handle, timeout=1)
            assert dispatch.accepted and terminal.accepted
            assert dispatch.value is not None and dispatch.value.outcome == Outcome.REJECTED
            assert terminal.value is not None and terminal.value.outcome == Outcome.REJECTED
            assert dispatch.value.handle == handle
            assert terminal.value.handle == handle
        aged_out = handles[0]
        assert (
            assert_public_wait_times_out(
                lambda: runtime.wait_for_dispatch(aged_out, timeout=1)
            ).diagnostic
            == "timeout"
        )
        assert (
            assert_public_wait_times_out(
                lambda: runtime.wait_for_terminal(aged_out, timeout=1)
            ).diagnostic
            == "timeout"
        )
        wrong = replace(handles[-1], attempt_id="missing-attempt")
        unknown = replace(handles[-1], operation_id="missing-operation")
        assert (
            assert_public_wait_times_out(
                lambda: runtime.wait_for_dispatch(wrong, timeout=1)
            ).diagnostic
            == "timeout"
        )
        assert (
            assert_public_wait_times_out(
                lambda: runtime.wait_for_terminal(wrong, timeout=1)
            ).diagnostic
            == "timeout"
        )
        assert (
            assert_public_wait_times_out(
                lambda: runtime.wait_for_dispatch(unknown, timeout=1)
            ).diagnostic
            == "timeout"
        )
        assert (
            assert_public_wait_times_out(
                lambda: runtime.wait_for_terminal(unknown, timeout=1)
            ).diagnostic
            == "timeout"
        )
    finally:
        runtime.shutdown()
