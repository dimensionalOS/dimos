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

"""Serialized dispatch of generated manipulation plans."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
import threading
import time

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.manipulation_spec import ExecutionResult, ExecutionStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_TERMINAL = frozenset({ExecutionStatus.COMPLETED, ExecutionStatus.ABORTED, ExecutionStatus.FAULT})
_FINISHED = _TERMINAL | {ExecutionStatus.UNCERTAIN}


class _PlanRejectedError(Exception):
    """Expected rejection while mapping a generated plan."""


class PlanExecutionManager:
    """Own mapping, dispatch, and polling for one trajectory execution.

    ``bindings`` says which task drives which joints. A plan is split into one
    part per owning task, and the parts run as one: all on the coordinator's
    tick clock, and when any part fails the others are cancelled.
    """

    def __init__(
        self,
        *,
        joint_names: Sequence[str],
        coordinator: ControlCoordinator,
        default_timeout: float,
        poll_interval: float = 0.1,
        bindings: Mapping[str, Sequence[str]] | None = None,
    ) -> None:
        self._joint_names = frozenset(joint_names)
        if not self._joint_names or len(self._joint_names) != len(joint_names):
            raise ValueError("Execution joint names must be non-empty and unique")
        self._bindings = _resolve_bindings(bindings, joint_names)
        self._coordinator = coordinator
        self._operation_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._poll_lock = threading.Lock()
        self._default_timeout = default_timeout
        self._poll_interval = poll_interval
        self._active = False
        self._latest_result: ExecutionResult | None = None
        self._legs: tuple[str, ...] = tuple(task for task, _ in self._bindings)
        self._cancelled_legs: set[str] = set()
        self._run_id = 0
        self._run_done = threading.Event()
        self._watchdog: threading.Thread | None = None

    @property
    def status(self) -> ExecutionStatus:
        """Return the latest known execution status."""
        with self._state_lock:
            if self._latest_result is None:
                return ExecutionStatus.IDLE
            return self._latest_result.status

    def execute(
        self,
        plan: GeneratedPlan,
        *,
        blocking: bool = True,
        timeout: float | None = None,
    ) -> ExecutionResult:
        """Dispatch a plan and optionally poll until it reaches a terminal state."""
        with self._operation_lock:
            with self._state_lock:
                if self._active:
                    return ExecutionResult(ExecutionStatus.REJECTED, "Another trajectory is active")
            try:
                parts = self._prepare_trajectory(plan)
            except _PlanRejectedError as exc:
                return ExecutionResult(ExecutionStatus.REJECTED, str(exc))
            with self._state_lock:
                self._cancelled_legs = set()

            try:
                current_positions = self._coordinator.get_joint_positions()
            except Exception as exc:
                logger.exception("Coordinator get_joint_positions RPC failed")
                execution_result = ExecutionResult(
                    ExecutionStatus.UNCERTAIN, f"Coordinator get_joint_positions failed: {exc}"
                )
                self._store(execution_result, active=False)
                return execution_result

            result: TrajectoryExecutionResult | None = None
            legs: list[str] = []
            for task, part in parts:
                outcome = self._start_part(task, part, current_positions)
                if not isinstance(outcome, TrajectoryExecutionResult):
                    failed = [leg for leg in legs if not self._cancel_leg(leg)]
                    if failed:
                        # Parts already accepted are still running without the rest.
                        outcome = ExecutionResult(
                            ExecutionStatus.UNCERTAIN,
                            f"{outcome.message}; could not cancel {', '.join(failed)}",
                        )
                    self._store(outcome, active=False)
                    return outcome
                result = result or outcome
                legs.append(task)

            accepted = ExecutionResult(
                ExecutionStatus.ACCEPTED,
                result.message if result is not None else "",
                coordinator_result=result,
            )
            # Under the poll lock so an in-flight poll of the previous run
            # cannot straddle the swap and act on this one.
            with self._poll_lock:
                with self._state_lock:
                    self._run_id += 1
                    run_id = self._run_id
                    self._legs = tuple(legs)
                    self._run_done = threading.Event()
                    run_done = self._run_done
                self._store(accepted, active=True)
                if len(legs) > 1:
                    self._watchdog = threading.Thread(
                        target=self._watch,
                        args=(run_done, run_id),
                        name="PlanExecutionWatchdog",
                        daemon=True,
                    )
                    self._watchdog.start()

        if not blocking:
            return accepted
        return self.wait(timeout)

    def wait(self, timeout: float | None = None) -> ExecutionResult:
        """Poll JTT status until terminal, preserving the active execution on timeout."""
        wait_timeout = self._default_timeout if timeout is None else timeout
        if not math.isfinite(wait_timeout) or wait_timeout < 0.0:
            return ExecutionResult(ExecutionStatus.REJECTED, "timeout must be finite and >= 0")
        with self._state_lock:
            latest = self._latest_result
            active = self._active
        if latest is None:
            return ExecutionResult(ExecutionStatus.NO_EXECUTION, "No execution exists")
        if not active:
            return latest

        deadline = time.monotonic() + wait_timeout
        while True:
            result = self._poll()
            if result.status in _FINISHED:
                return result
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return ExecutionResult(
                    ExecutionStatus.TIMED_OUT,
                    f"Execution did not finish within {wait_timeout:g}s",
                    trajectory_status=result.trajectory_status,
                )
            threading.Event().wait(min(self._poll_interval, remaining))

    def cancel(self, timeout: float = 1.0) -> ExecutionResult:
        """Cancel the active trajectory and return its authoritative terminal state."""
        if len(self._legs) > 1:
            with self._operation_lock:
                failed = [leg for leg in self._legs if not self._cancel_leg(leg)]
            if failed:
                result = ExecutionResult(
                    ExecutionStatus.UNCERTAIN, f"Could not cancel {', '.join(failed)}"
                )
                self._store(result, active=False)
                return result
            return self.wait(timeout)
        task = self._legs[0]
        with self._operation_lock:
            try:
                outcome = self._coordinator.task_invoke(task, "cancel", {})
            except Exception as exc:
                logger.exception(f"Cancelling {task} failed")
                result = ExecutionResult(
                    ExecutionStatus.UNCERTAIN,
                    f"{task} cancel RPC failed: {exc}",
                )
                self._store(result, active=False)
                return result
        cancellation = (
            outcome
            if isinstance(outcome, TrajectoryCancellationResult)
            else TrajectoryCancellationResult(TrajectoryCancellationStatus.CANCELLED)
        )
        if cancellation.status is TrajectoryCancellationStatus.UNCERTAIN:
            result = ExecutionResult(
                ExecutionStatus.UNCERTAIN,
                cancellation.message or "Coordinator cancellation outcome is uncertain",
            )
            self._store(result, active=False)
            return result

        with self._state_lock:
            latest = self._latest_result
        status = self._get_status(task)
        if isinstance(status, ExecutionResult):
            return status
        mapped = self._result_from_status(status)
        if mapped.status in {
            ExecutionStatus.COMPLETED,
            ExecutionStatus.ABORTED,
            ExecutionStatus.FAULT,
        }:
            self._store(mapped, active=False)
            return mapped
        if cancellation.status is TrajectoryCancellationStatus.CANCELLED:
            return self.wait(timeout)
        if latest is not None and latest.status in {
            ExecutionStatus.COMPLETED,
            ExecutionStatus.ABORTED,
            ExecutionStatus.FAULT,
        }:
            return latest
        if status.state is TrajectoryState.IDLE:
            result = ExecutionResult(ExecutionStatus.NO_EXECUTION, cancellation.message)
            self._store(result, active=False)
            return result
        result = ExecutionResult(
            ExecutionStatus.UNCERTAIN,
            "Coordinator reported no active trajectory while JTT is still executing",
            trajectory_status=status,
        )
        self._store(result, active=False)
        return result

    def _poll(self, run_id: int | None = None) -> ExecutionResult:
        """Read every leg of the run once and store the combined result.

        ``run_id`` names the run a watchdog polls for. A poll whose run has since
        finished or been replaced stores nothing and cancels nothing.
        """
        # One poller at a time, or a stale read could overwrite a finished run.
        with self._poll_lock:
            with self._state_lock:
                if run_id is not None and run_id != self._run_id:
                    return self._latest_result or ExecutionResult(ExecutionStatus.IDLE)
                if not self._active and self._latest_result is not None:
                    return self._latest_result
                run_id = self._run_id
                legs = self._legs
            statuses: dict[str, TrajectoryStatus] = {}
            for leg in legs:
                status = self._get_status(leg, run_id=run_id)
                if isinstance(status, ExecutionResult):
                    for other in legs:
                        if other != leg:
                            self._cancel_leg(other)
                    return status
                statuses[leg] = status
            with self._state_lock:
                if run_id != self._run_id or not self._active:
                    return self._latest_result or ExecutionResult(ExecutionStatus.IDLE)
            result = self._combine(statuses)
            self._store(result, active=result.status not in _FINISHED, run_id=run_id)
            return result

    def _combine(self, statuses: dict[str, TrajectoryStatus]) -> ExecutionResult:
        if len(statuses) == 1:
            return self._result_from_status(next(iter(statuses.values())))
        primary = next(
            (statuses[task] for task, _ in self._bindings if task in statuses),
            next(iter(statuses.values())),
        )
        for leg, status in statuses.items():
            if status.state not in {TrajectoryState.ABORTED, TrajectoryState.FAULT}:
                continue
            failed = [
                other
                for other, other_status in statuses.items()
                if other != leg
                and other_status.state is TrajectoryState.EXECUTING
                and not self._cancel_leg(other)
            ]
            if failed:
                return ExecutionResult(
                    ExecutionStatus.UNCERTAIN,
                    f"{leg}: {status.error}; could not cancel {', '.join(failed)}",
                    trajectory_status=primary,
                )
            mapped = self._result_from_status(status).status
            return ExecutionResult(mapped, f"{leg}: {status.error}", trajectory_status=primary)
        if all(status.state is TrajectoryState.COMPLETED for status in statuses.values()):
            return ExecutionResult(ExecutionStatus.COMPLETED, trajectory_status=primary)
        return ExecutionResult(ExecutionStatus.EXECUTING, trajectory_status=primary)

    def _watch(self, run_done: threading.Event, run_id: int) -> None:
        # Non-blocking runs are otherwise only polled when someone reads status.
        while not run_done.wait(self._poll_interval):
            self._poll(run_id)

    def _start_part(
        self, task: str, trajectory: JointTrajectory, current_positions: Mapping[str, float]
    ) -> TrajectoryExecutionResult | ExecutionResult:
        """Dispatch one part of a plan to the task that owns its joints."""
        try:
            result = self._coordinator.task_invoke(
                task, "execute", {"trajectory": trajectory, "current_positions": current_positions}
            )
        except Exception as exc:
            logger.exception(f"{task} execute RPC failed")
            self._cancel_leg(task)
            return ExecutionResult(ExecutionStatus.UNCERTAIN, f"{task} execute RPC failed: {exc}")
        if not isinstance(result, TrajectoryExecutionResult):
            return ExecutionResult(
                ExecutionStatus.REJECTED, f"Task '{task}' is not on the coordinator"
            )
        if result.status is not TrajectoryExecutionStatus.ACCEPTED:
            return ExecutionResult(
                ExecutionStatus.REJECTED,
                result.message or f"{task} rejected trajectory: {result.status.name}",
                coordinator_result=result,
            )
        return result

    def _cancel_leg(self, leg: str) -> bool:
        """Cancel one leg once; False when the outcome is unknown."""
        with self._state_lock:
            if leg in self._cancelled_legs:
                return True
            self._cancelled_legs.add(leg)
        try:
            outcome = self._coordinator.task_invoke(leg, "cancel", {})
            if isinstance(outcome, TrajectoryCancellationResult):
                return outcome.status is not TrajectoryCancellationStatus.UNCERTAIN
            # A task that reports no cancellation result had nothing left to stop.
            return True
        except Exception:
            logger.exception(f"Cancelling {leg} failed")
            return False

    def close(self) -> None:
        """Stop watching the running plan; cancel it first to stop the robot."""
        with self._state_lock:
            self._run_done.set()
            watchdog, self._watchdog = self._watchdog, None
        if watchdog is not None:
            watchdog.join(DEFAULT_THREAD_JOIN_TIMEOUT)

    def _get_status(
        self, task: str = JOINT_TRAJECTORY_TASK_NAME, *, run_id: int | None = None
    ) -> TrajectoryStatus | ExecutionResult:
        try:
            status = self._coordinator.task_invoke(task, "get_status", {"t_now": None})
        except Exception as exc:
            logger.exception(f"{task} get_status RPC failed")
            result = ExecutionResult(
                ExecutionStatus.UNCERTAIN,
                f"{task} get_status RPC failed: {exc}",
            )
            self._store(result, active=False, run_id=run_id)
            return result
        if not isinstance(status, TrajectoryStatus):
            result = ExecutionResult(
                ExecutionStatus.UNCERTAIN,
                f"{task} get_status returned {type(status).__name__}, expected TrajectoryStatus",
            )
            self._store(result, active=False, run_id=run_id)
            return result
        return status

    @staticmethod
    def _result_from_status(status: TrajectoryStatus) -> ExecutionResult:
        mapped = {
            TrajectoryState.IDLE: ExecutionStatus.IDLE,
            TrajectoryState.EXECUTING: ExecutionStatus.EXECUTING,
            TrajectoryState.COMPLETED: ExecutionStatus.COMPLETED,
            TrajectoryState.ABORTED: ExecutionStatus.ABORTED,
            TrajectoryState.FAULT: ExecutionStatus.FAULT,
        }[status.state]
        return ExecutionResult(mapped, status.error, trajectory_status=status)

    def _store(self, result: ExecutionResult, *, active: bool, run_id: int | None = None) -> None:
        with self._state_lock:
            if run_id is not None and run_id != self._run_id:
                return
            self._latest_result = result
            self._active = active
            if not active:
                self._run_done.set()

    def _prepare_trajectory(self, plan: GeneratedPlan) -> list[tuple[str, JointTrajectory]]:
        """Split a plan into one part per task that owns some of its joints."""
        if not isinstance(plan, GeneratedPlan):
            raise _PlanRejectedError("Execution requires a generated plan")
        if not plan.is_success():
            raise _PlanRejectedError("Generated plan status is not successful")

        names = plan.trajectory.joint_names
        unknown = [name for name in names if name not in self._joint_names]
        if unknown:
            raise _PlanRejectedError(f"Generated trajectory has unknown joints: {unknown}")
        if len(set(names)) != len(names):
            raise _PlanRejectedError("Generated trajectory has duplicate joints")

        owners = {joint: task for task, joints in self._bindings for joint in joints}
        unowned = sorted({name for name in names if name not in owners})
        if unowned:
            raise _PlanRejectedError(f"No trajectory task is bound to {unowned}")

        # Columns follow the binding's joint order: a task that reads them
        # positionally (x, y, yaw) must not depend on how the planner ordered them.
        grouped = [
            (task, [names.index(joint) for joint in joints if joint in names])
            for task, joints in self._bindings
        ]
        parts = [(task, columns) for task, columns in grouped if columns]
        if len(parts) == 1 and parts[0][1] == list(range(len(names))):
            # One task drives every column, already in order: forward the plan as-is.
            return [(parts[0][0], plan.trajectory)]
        return [(task, _columns(plan.trajectory, columns)) for task, columns in parts]


def _resolve_bindings(
    bindings: Mapping[str, Sequence[str]] | None, joint_names: Sequence[str]
) -> tuple[tuple[str, tuple[str, ...]], ...]:
    """Which task drives which joints. Without bindings the joint trajectory task drives all."""
    if not bindings:
        return ((JOINT_TRAJECTORY_TASK_NAME, tuple(joint_names)),)
    owners: dict[str, str] = {}
    resolved: list[tuple[str, tuple[str, ...]]] = []
    for task, joints in bindings.items():
        if not joints:
            raise ValueError(f"Trajectory task '{task}' is bound to no joints")
        for joint in joints:
            if joint in owners:
                raise ValueError(f"Joint '{joint}' is bound to both '{owners[joint]}' and '{task}'")
            owners[joint] = task
        resolved.append((task, tuple(joints)))
    return tuple(resolved)


def _columns(trajectory: JointTrajectory, columns: list[int]) -> JointTrajectory:
    return JointTrajectory(
        joint_names=[trajectory.joint_names[index] for index in columns],
        points=[
            TrajectoryPoint(
                time_from_start=point.time_from_start,
                positions=[point.positions[index] for index in columns],
                velocities=[point.velocities[index] for index in columns],
            )
            for point in trajectory.points
        ],
        timestamp=trajectory.timestamp,
    )
