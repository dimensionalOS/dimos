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

from collections.abc import Sequence
import math
import threading
import time

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
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

    With a base task, a plan's planar-base columns go to that task and the rest
    to the joint trajectory task, both on the coordinator's tick clock. The two
    run as one: when either fails, the other is cancelled.
    """

    def __init__(
        self,
        *,
        joint_names: Sequence[str],
        coordinator: ControlCoordinator,
        default_timeout: float,
        poll_interval: float = 0.1,
        base_task: str | None = None,
        base_joint_names: Sequence[str] = (),
    ) -> None:
        self._joint_names = frozenset(joint_names)
        if not self._joint_names or len(self._joint_names) != len(joint_names):
            raise ValueError("Execution joint names must be non-empty and unique")
        self._coordinator = coordinator
        self._operation_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._poll_lock = threading.Lock()
        self._default_timeout = default_timeout
        self._poll_interval = poll_interval
        self._base_task = base_task
        self._base_joint_names = list(base_joint_names)
        self._active = False
        self._latest_result: ExecutionResult | None = None
        self._legs: tuple[str, ...] = (JOINT_TRAJECTORY_TASK_NAME,)
        self._cancelled_legs: set[str] = set()
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
                trajectory, base_trajectory = self._prepare_trajectory(plan)
            except _PlanRejectedError as exc:
                return ExecutionResult(ExecutionStatus.REJECTED, str(exc))
            with self._state_lock:
                self._cancelled_legs = set()

            result: TrajectoryExecutionResult | None = None
            if trajectory is not None:
                try:
                    result = self._coordinator.execute_trajectory(trajectory)
                except Exception as exc:
                    logger.exception("Coordinator execute RPC failed")
                    execution_result = ExecutionResult(
                        ExecutionStatus.UNCERTAIN,
                        f"Coordinator execute RPC failed: {exc}",
                    )
                    self._store(execution_result, active=False)
                    return execution_result

                if result.status is not TrajectoryExecutionStatus.ACCEPTED:
                    execution_result = ExecutionResult(
                        ExecutionStatus.REJECTED,
                        result.message or f"Coordinator rejected trajectory: {result.status.name}",
                        coordinator_result=result,
                    )
                    self._store(execution_result, active=False)
                    return execution_result

            legs = [JOINT_TRAJECTORY_TASK_NAME] if trajectory is not None else []
            if base_trajectory is not None and self._base_task is not None:
                base_result = self._start_base(self._base_task, base_trajectory)
                if not isinstance(base_result, TrajectoryExecutionResult):
                    if trajectory is not None:
                        self._cancel_leg(JOINT_TRAJECTORY_TASK_NAME)
                    self._store(base_result, active=False)
                    return base_result
                result = result or base_result
                legs.append(self._base_task)

            accepted = ExecutionResult(
                ExecutionStatus.ACCEPTED,
                result.message if result is not None else "",
                coordinator_result=result,
            )
            with self._state_lock:
                self._legs = tuple(legs)
                self._run_done = threading.Event()
                run_done = self._run_done
            self._store(accepted, active=True)
            if base_trajectory is not None:
                self._watchdog = threading.Thread(
                    target=self._watch, args=(run_done,), name="PlanExecutionWatchdog", daemon=True
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
        if self._base_task is not None and self._base_task in self._legs:
            with self._operation_lock:
                failed = [leg for leg in self._legs if not self._cancel_leg(leg)]
            if failed:
                result = ExecutionResult(
                    ExecutionStatus.UNCERTAIN, f"Could not cancel {', '.join(failed)}"
                )
                self._store(result, active=False)
                return result
            return self.wait(timeout)
        with self._operation_lock:
            try:
                cancellation = self._coordinator.cancel_trajectory()
            except Exception as exc:
                logger.exception("Coordinator cancel RPC failed")
                result = ExecutionResult(
                    ExecutionStatus.UNCERTAIN,
                    f"Coordinator cancel RPC failed: {exc}",
                )
                self._store(result, active=False)
                return result
        if cancellation.status is TrajectoryCancellationStatus.UNCERTAIN:
            result = ExecutionResult(
                ExecutionStatus.UNCERTAIN,
                cancellation.message or "Coordinator cancellation outcome is uncertain",
            )
            self._store(result, active=False)
            return result

        with self._state_lock:
            latest = self._latest_result
        status = self._get_status()
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

    def _poll(self) -> ExecutionResult:
        """Read every leg of the run once and store the combined result."""
        # One poller at a time, or a stale read could overwrite a finished run.
        with self._poll_lock:
            with self._state_lock:
                if not self._active and self._latest_result is not None:
                    return self._latest_result
            statuses: dict[str, TrajectoryStatus] = {}
            for leg in self._legs:
                status = self._get_status(leg)
                if isinstance(status, ExecutionResult):
                    for other in self._legs:
                        if other != leg:
                            self._cancel_leg(other)
                    return status
                statuses[leg] = status
            result = self._combine(statuses)
            self._store(result, active=result.status not in _FINISHED)
            return result

    def _combine(self, statuses: dict[str, TrajectoryStatus]) -> ExecutionResult:
        if len(statuses) == 1:
            return self._result_from_status(next(iter(statuses.values())))
        primary = statuses.get(JOINT_TRAJECTORY_TASK_NAME) or next(iter(statuses.values()))
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

    def _watch(self, run_done: threading.Event) -> None:
        # Non-blocking runs are otherwise only polled when someone reads status.
        while not run_done.wait(self._poll_interval):
            self._poll()

    def _start_base(
        self, task: str, trajectory: JointTrajectory
    ) -> TrajectoryExecutionResult | ExecutionResult:
        try:
            result = self._coordinator.task_invoke(task, "execute", {"trajectory": trajectory})
        except Exception as exc:
            logger.exception("Base execute RPC failed")
            self._cancel_leg(task)
            return ExecutionResult(ExecutionStatus.UNCERTAIN, f"Base execute RPC failed: {exc}")
        if not isinstance(result, TrajectoryExecutionResult):
            return ExecutionResult(
                ExecutionStatus.REJECTED, f"Base task '{task}' is not on the coordinator"
            )
        if result.status is not TrajectoryExecutionStatus.ACCEPTED:
            return ExecutionResult(
                ExecutionStatus.REJECTED,
                result.message or f"Base task rejected trajectory: {result.status.name}",
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
            if leg == JOINT_TRAJECTORY_TASK_NAME:
                cancellation = self._coordinator.cancel_trajectory()
                return cancellation.status is not TrajectoryCancellationStatus.UNCERTAIN
            self._coordinator.task_invoke(leg, "cancel", {})
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
        self, task: str = JOINT_TRAJECTORY_TASK_NAME
    ) -> TrajectoryStatus | ExecutionResult:
        args: dict[str, None] = {"t_now": None} if task == JOINT_TRAJECTORY_TASK_NAME else {}
        try:
            status = self._coordinator.task_invoke(task, "get_status", args)
        except Exception as exc:
            logger.exception(f"{task} get_status RPC failed")
            result = ExecutionResult(
                ExecutionStatus.UNCERTAIN,
                f"{task} get_status RPC failed: {exc}",
            )
            self._store(result, active=False)
            return result
        if not isinstance(status, TrajectoryStatus):
            result = ExecutionResult(
                ExecutionStatus.UNCERTAIN,
                f"{task} get_status returned {type(status).__name__}, expected TrajectoryStatus",
            )
            self._store(result, active=False)
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

    def _store(self, result: ExecutionResult, *, active: bool) -> None:
        with self._state_lock:
            self._latest_result = result
            self._active = active
            if not active:
                self._run_done.set()

    def _prepare_trajectory(
        self, plan: GeneratedPlan
    ) -> tuple[JointTrajectory | None, JointTrajectory | None]:
        """Split a plan into its joint-trajectory part and its planar-base part."""
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

        base_columns = [names.index(name) for name in self._base_joint_names if name in names]
        if self._base_task is None or not base_columns:
            return plan.trajectory, None
        if len(base_columns) != len(self._base_joint_names):
            raise _PlanRejectedError("Generated trajectory moves only part of the planar base")
        joint_columns = [index for index in range(len(names)) if index not in base_columns]
        joint_part = _columns(plan.trajectory, joint_columns) if joint_columns else None
        return joint_part, _columns(plan.trajectory, base_columns)


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
