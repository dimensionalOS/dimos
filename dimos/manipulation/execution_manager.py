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

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import field
import math
import threading
import time
from types import MappingProxyType
from typing import Annotated

from pydantic import AfterValidator, BeforeValidator, ConfigDict, Field, model_validator
from pydantic.dataclasses import dataclass as pydantic_dataclass
from typing_extensions import Self

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
    TrajectoryCancellationStatus,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.manipulation_spec import ExecutionResult, ExecutionStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan, RobotName
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _to_model_joint_names(value: Sequence[str]) -> tuple[str, ...]:
    return tuple(value)


def _to_immutable_joint_mapping(value: Mapping[str, str]) -> Mapping[str, str]:
    return MappingProxyType(dict(value))


@pydantic_dataclass(
    frozen=True,
    config=ConfigDict(extra="forbid", validate_default=True),
)
class ExecutionTarget:
    """Immutable coordinator joint mapping for one robot."""

    robot_name: Annotated[RobotName, Field(min_length=1)]
    model_joint_names: Annotated[
        tuple[str, ...],
        BeforeValidator(_to_model_joint_names),
    ]
    model_to_coordinator: Annotated[
        Mapping[str, str],
        AfterValidator(_to_immutable_joint_mapping),
    ] = field(repr=False)

    @model_validator(mode="after")
    def _validate_target(self) -> Self:
        if not self.model_joint_names or any(
            not name or "/" in name for name in self.model_joint_names
        ):
            raise ValueError(f"Execution target '{self.robot_name}' has invalid local model joints")
        if len(set(self.model_joint_names)) != len(self.model_joint_names):
            raise ValueError(
                f"Execution target '{self.robot_name}' has duplicate local model joints"
            )
        if set(self.model_to_coordinator) != set(self.model_joint_names):
            raise ValueError(f"Execution target '{self.robot_name}' must resolve every model joint")
        resolved_names = list(self.model_to_coordinator.values())
        if any(not name for name in resolved_names) or len(set(resolved_names)) != len(
            resolved_names
        ):
            raise ValueError(
                f"Execution target '{self.robot_name}' has ambiguous coordinator joints"
            )
        return self

    @classmethod
    def from_coordinator_mapping(
        cls,
        *,
        robot_name: RobotName,
        model_joint_names: Sequence[str],
        # TODO: unify coordinator joint name with planner
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
            model_to_coordinator={
                model_name: reverse.get(model_name, model_name) for model_name in local_names
            },
        )


class _PlanRejectedError(Exception):
    """Expected rejection while mapping a generated plan."""


class PlanExecutionManager:
    """Own mapping, dispatch, and polling for one trajectory execution."""

    def __init__(
        self,
        *,
        targets: Iterable[ExecutionTarget],
        coordinator: ControlCoordinator,
        default_timeout: float,
        poll_interval: float = 0.1,
    ) -> None:
        target_items = tuple(targets)
        target_names = [target.robot_name for target in target_items]
        if len(set(target_names)) != len(target_names):
            raise ValueError("Execution targets must have unique robot names")

        self._targets = {target.robot_name: target for target in target_items}
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

        coordinator_names: list[str] = []
        for global_name in plan.trajectory.joint_names:
            parts = global_name.split("/")
            if len(parts) != 2 or not parts[0] or not parts[1]:
                raise _PlanRejectedError(
                    f"Generated trajectory joint '{global_name}' is not globally named"
                )
            robot_name, local_name = parts
            target = self._targets.get(robot_name)
            if target is None:
                raise _PlanRejectedError(
                    f"Generated plan references unknown execution robot '{robot_name}'"
                )
            coordinator_name = target.model_to_coordinator.get(local_name)
            if coordinator_name is None:
                raise _PlanRejectedError(
                    f"Generated trajectory joint '{global_name}' is not configured"
                )
            coordinator_names.append(coordinator_name)

        if len(set(coordinator_names)) != len(coordinator_names):
            raise _PlanRejectedError(
                "Generated trajectory resolves to duplicate coordinator joints"
            )

        return JointTrajectory(
            joint_names=coordinator_names,
            points=plan.trajectory.points,
            timestamp=plan.trajectory.timestamp,
        )
