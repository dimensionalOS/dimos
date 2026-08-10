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
from enum import Enum, auto
import threading
from types import MappingProxyType

import attrs

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.planning.spec.models import GeneratedPlan, RobotName
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_NON_EMPTY_STRING = attrs.validators.and_(
    attrs.validators.instance_of(str),
    attrs.validators.min_len(1),
)


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


def _to_model_joint_names(value: Sequence[str]) -> tuple[str, ...]:
    return tuple(value)


def _to_immutable_joint_mapping(value: Mapping[str, str]) -> Mapping[str, str]:
    return MappingProxyType(dict(value))


@attrs.frozen(slots=False)
class ExecutionTarget:
    """Immutable coordinator joint mapping for one robot."""

    robot_name: RobotName = attrs.field(validator=_NON_EMPTY_STRING)
    model_joint_names: tuple[str, ...] = attrs.field(
        converter=_to_model_joint_names,
        validator=attrs.validators.deep_iterable(
            member_validator=attrs.validators.instance_of(str),
        ),
    )
    model_to_coordinator: Mapping[str, str] = attrs.field(
        converter=_to_immutable_joint_mapping,
        validator=attrs.validators.deep_mapping(
            key_validator=attrs.validators.instance_of(str),
            value_validator=attrs.validators.instance_of(str),
        ),
        repr=False,
    )

    @model_joint_names.validator
    def _validate_model_joint_names(
        self,
        _attribute: attrs.Attribute[tuple[str, ...]],
        value: tuple[str, ...],
    ) -> None:
        if not value or any(not name or "/" in name for name in value):
            raise ValueError(f"Execution target '{self.robot_name}' has invalid local model joints")
        if len(set(value)) != len(value):
            raise ValueError(
                f"Execution target '{self.robot_name}' has duplicate local model joints"
            )

    @model_to_coordinator.validator
    def _validate_model_to_coordinator(
        self,
        _attribute: attrs.Attribute[Mapping[str, str]],
        value: Mapping[str, str],
    ) -> None:
        if set(value) != set(self.model_joint_names):
            raise ValueError(f"Execution target '{self.robot_name}' must resolve every model joint")
        resolved_names = list(value.values())
        if any(not name for name in resolved_names) or len(set(resolved_names)) != len(
            resolved_names
        ):
            raise ValueError(
                f"Execution target '{self.robot_name}' has ambiguous coordinator joints"
            )

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
    """Map, dispatch, replace, and cancel complete generated plans."""

    def __init__(
        self,
        *,
        targets: Iterable[ExecutionTarget],
        coordinator: ControlCoordinator,
    ) -> None:
        target_items = tuple(targets)
        target_names = [target.robot_name for target in target_items]
        if len(set(target_names)) != len(target_names):
            raise ValueError("Execution targets must have unique robot names")

        self._targets = {target.robot_name: target for target in target_items}
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
