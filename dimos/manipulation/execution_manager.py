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
from enum import Enum, auto
import threading

import attrs

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ExecutionOutcome(Enum):
    """Safety-aware outcome of dispatching planned execution."""

    ACCEPTED = auto()
    REJECTED = auto()
    UNCERTAIN = auto()


@attrs.frozen(slots=False)
class ExecutionDispatchResult:
    """Structured result of mapping and dispatching one generated plan."""

    outcome: ExecutionOutcome
    message: str = ""
    coordinator_result: TrajectoryExecutionResult | None = None

    @property
    def accepted(self) -> bool:
        """Return whether the coordinator accepted the trajectory."""
        return self.outcome is ExecutionOutcome.ACCEPTED


class _PlanRejectedError(Exception):
    """Expected rejection while mapping a generated plan."""


class PlanExecutionManager:
    """Map, dispatch, replace, and cancel complete generated plans."""

    def __init__(
        self,
        *,
        joint_names: Sequence[str],
        coordinator: ControlCoordinator,
    ) -> None:
        self._joint_names = frozenset(joint_names)
        if not self._joint_names or len(self._joint_names) != len(joint_names):
            raise ValueError("Execution joint names must be non-empty and unique")
        self._coordinator = coordinator
        self._operation_lock = threading.Lock()

    def execute(self, plan: GeneratedPlan) -> ExecutionDispatchResult:
        """Map and dispatch one complete generated plan."""
        with self._operation_lock:
            try:
                trajectory = self._prepare_trajectory(plan)
            except _PlanRejectedError as exc:
                return ExecutionDispatchResult(
                    outcome=ExecutionOutcome.REJECTED,
                    message=str(exc),
                )

            try:
                result = self._coordinator.execute_trajectory(trajectory)
            except Exception as exc:
                logger.exception("Coordinator execute RPC failed")
                return ExecutionDispatchResult(
                    outcome=ExecutionOutcome.UNCERTAIN,
                    message=f"Coordinator execute RPC failed: {exc}",
                )

            if result.status is TrajectoryExecutionStatus.ACCEPTED:
                return ExecutionDispatchResult(
                    outcome=ExecutionOutcome.ACCEPTED,
                    message=result.message,
                    coordinator_result=result,
                )
            return ExecutionDispatchResult(
                outcome=ExecutionOutcome.REJECTED,
                message=result.message or f"Coordinator rejected trajectory: {result.status.name}",
                coordinator_result=result,
            )

    def cancel(self) -> TrajectoryCancellationResult:
        """Cancel coordinator trajectory execution."""
        with self._operation_lock:
            try:
                return self._coordinator.cancel_trajectory()
            except Exception as exc:
                logger.exception("Coordinator cancel RPC failed")
                return TrajectoryCancellationResult(
                    status=TrajectoryCancellationStatus.UNCERTAIN,
                    message=f"Coordinator cancel RPC failed: {exc}",
                )

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
