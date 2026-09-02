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

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
    TrajectoryCancellationStatus,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.manipulation_spec import ExecutionResult, ExecutionStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class _PlanRejectedError(Exception):
    """Expected rejection while mapping a generated plan."""


class PlanExecutionManager:
    """Own mapping, dispatch, and polling for one trajectory execution."""

    def __init__(
        self,
        *,
        joint_names: Sequence[str],
        coordinator: ControlCoordinator,
        default_timeout: float,
        poll_interval: float = 0.1,
    ) -> None:
        self._joint_names = frozenset(joint_names)
        if not self._joint_names or len(self._joint_names) != len(joint_names):
            raise ValueError("Execution joint names must be non-empty and unique")
        self._coordinator = coordinator
        self._operation_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._default_timeout = default_timeout
        self._poll_interval = poll_interval
        self._active = False
        self._latest_result: ExecutionResult | None = None

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
                trajectory = self._prepare_trajectory(plan)
            except _PlanRejectedError as exc:
                return ExecutionResult(ExecutionStatus.REJECTED, str(exc))

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

            accepted = ExecutionResult(
                ExecutionStatus.ACCEPTED,
                result.message,
                coordinator_result=result,
            )
            self._store(accepted, active=True)

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
            self._store(mapped, active=True)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return ExecutionResult(
                    ExecutionStatus.TIMED_OUT,
                    f"Execution did not finish within {wait_timeout:g}s",
                    trajectory_status=status,
                )
            threading.Event().wait(min(self._poll_interval, remaining))

    def cancel(self, timeout: float = 1.0) -> ExecutionResult:
        """Cancel the active trajectory and return its authoritative terminal state."""
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

    def _get_status(self) -> TrajectoryStatus | ExecutionResult:
        try:
            status = self._coordinator.task_invoke(
                JOINT_TRAJECTORY_TASK_NAME,
                "get_status",
                {"t_now": None},
            )
        except Exception as exc:
            logger.exception("JTT get_status RPC failed")
            result = ExecutionResult(
                ExecutionStatus.UNCERTAIN,
                f"JTT get_status RPC failed: {exc}",
            )
            self._store(result, active=False)
            return result
        if not isinstance(status, TrajectoryStatus):
            result = ExecutionResult(
                ExecutionStatus.UNCERTAIN,
                f"JTT get_status returned {type(status).__name__}, expected TrajectoryStatus",
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

    def _prepare_trajectory(self, plan: GeneratedPlan) -> JointTrajectory:
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
        return plan.trajectory
