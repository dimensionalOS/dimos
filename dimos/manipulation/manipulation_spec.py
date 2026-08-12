# Copyright 2026 Dimensional Inc.
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

"""Small, group-native manipulation RPC contract."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Protocol

from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionResult
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningGroupID
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryStatus
from dimos.spec.utils import Spec

MAX_AGENT_REPR_LENGTH = 500


def _bounded_repr(text: str) -> str:
    if len(text) <= MAX_AGENT_REPR_LENGTH:
        return text
    return f"{text[: MAX_AGENT_REPR_LENGTH - 3]}..."


class OperationStatus(Enum):
    """Current manipulation-module operation."""

    IDLE = auto()
    PLANNING = auto()
    EXECUTING = auto()
    COMPLETED = auto()
    FAULT = auto()


class PlanStatus(Enum):
    """Outcome of a planning request."""

    SUCCEEDED = auto()
    FAILED = auto()
    INVALID_TARGET = auto()
    AMBIGUOUS_GROUP = auto()
    NO_MOTION = auto()


class ExecutionStatus(Enum):
    """Outcome or current state of trajectory execution."""

    IDLE = auto()
    ACCEPTED = auto()
    REJECTED = auto()
    EXECUTING = auto()
    COMPLETED = auto()
    ABORTED = auto()
    FAULT = auto()
    TIMED_OUT = auto()
    NO_PLAN = auto()
    NO_EXECUTION = auto()
    UNCERTAIN = auto()


class CommandStatus(Enum):
    """Outcome of a non-planning manipulation command."""

    SUCCEEDED = auto()
    REJECTED = auto()
    FAILED = auto()


@dataclass(frozen=True, repr=False)
class PlanningGroupInfo:
    """Public planning-group capability metadata."""

    id: PlanningGroupID
    joint_names: tuple[str, ...]
    base_frame: str
    tip_frame: str | None
    has_gripper: bool

    def __repr__(self) -> str:
        return _bounded_repr(
            f"PlanningGroupInfo({self.id!r}, joints={self.joint_names!r}, "
            f"base={self.base_frame!r}, tip={self.tip_frame!r}, "
            f"gripper={self.has_gripper})"
        )


@dataclass(frozen=True, repr=False)
class PlanningGroupState:
    """Latest state and read-only joint presets for one planning group."""

    joints: JointState | None
    end_effector_pose: PoseStamped | None
    gripper_position: float | None
    joint_presets: Mapping[str, JointState] = field(default_factory=dict)

    def __repr__(self) -> str:
        return _bounded_repr(
            f"PlanningGroupState(joints={self.joints!r}, "
            f"end_effector_pose={self.end_effector_pose!r}, "
            f"gripper_position={self.gripper_position!r}, "
            f"joint_presets={dict(self.joint_presets)!r})"
        )


@dataclass(frozen=True, repr=False)
class ManipulationSnapshot:
    """One bounded snapshot of every public manipulation group."""

    timestamp: float
    operation_status: OperationStatus
    error: str | None
    has_pending_plan: bool
    execution_status: ExecutionStatus
    groups: Mapping[PlanningGroupID, PlanningGroupState]

    def __repr__(self) -> str:
        return _bounded_repr(
            f"ManipulationSnapshot(timestamp={self.timestamp!r}, "
            f"operation_status={self.operation_status.name}, error={self.error!r}, "
            f"has_pending_plan={self.has_pending_plan}, "
            f"execution_status={self.execution_status.name}, groups={dict(self.groups)!r})"
        )


@dataclass(frozen=True, repr=False)
class PlanResult:
    """Typed planning outcome with an inspectable plan snapshot."""

    status: PlanStatus
    message: str = ""
    plan: GeneratedPlan | None = None

    @property
    def succeeded(self) -> bool:
        return self.status is PlanStatus.SUCCEEDED

    def __repr__(self) -> str:
        if self.plan is None:
            return _bounded_repr(f"PlanResult({self.status.name}, message={self.message!r})")
        return _bounded_repr(
            f"PlanResult({self.status.name}, groups={self.plan.group_ids!r}, "
            f"waypoints={len(self.plan.path)}, duration={self.plan.trajectory.duration:.3g}s, "
            f"planning_time={self.plan.planning_time:.3g}s, message={self.message!r})"
        )


@dataclass(frozen=True, repr=False)
class ExecutionResult:
    """Typed dispatch or physical-execution outcome."""

    status: ExecutionStatus
    message: str = ""
    coordinator_result: TrajectoryExecutionResult | None = None
    trajectory_status: TrajectoryStatus | None = None

    @property
    def succeeded(self) -> bool:
        return self.status in {
            ExecutionStatus.ACCEPTED,
            ExecutionStatus.COMPLETED,
        }

    def __repr__(self) -> str:
        return _bounded_repr(
            f"ExecutionResult({self.status.name}, message={self.message!r}, "
            f"coordinator_result={self.coordinator_result!r}, "
            f"trajectory_status={self.trajectory_status!r})"
        )


@dataclass(frozen=True, repr=False)
class CommandResult:
    """Typed result for gripper and reset commands."""

    status: CommandStatus
    message: str = ""

    @property
    def succeeded(self) -> bool:
        return self.status is CommandStatus.SUCCEEDED

    def __repr__(self) -> str:
        return _bounded_repr(f"CommandResult({self.status.name}, message={self.message!r})")


@dataclass(frozen=True, repr=False)
class MoveResult:
    """Planning and execution outcomes for an immediate linear move."""

    plan: PlanResult
    execution: ExecutionResult | None
    delta: tuple[float, float, float]
    check_collision: bool

    @property
    def succeeded(self) -> bool:
        return self.plan.succeeded and self.execution is not None and self.execution.succeeded

    def __repr__(self) -> str:
        execution = self.execution.status.name if self.execution is not None else None
        return _bounded_repr(
            f"MoveResult(plan={self.plan.status.name}, execution={execution}, "
            f"delta={self.delta!r}, collision_check={self.check_collision})"
        )


def assert_agent_readable_result_type(result_type: type[Any]) -> None:
    """Reject a public result type without an intentional representation."""

    if result_type.__repr__ is object.__repr__:
        raise TypeError(f"{result_type.__name__} must define __repr__")
    dataclass_params = getattr(result_type, "__dataclass_params__", None)
    if dataclass_params is not None and dataclass_params.repr:
        raise TypeError(f"{result_type.__name__} must not use the generated dataclass repr")


def assert_agent_readable_result(result: Any) -> None:
    """Reject public results whose REPL representation is missing or unbounded."""

    result_type = type(result)
    assert_agent_readable_result_type(result_type)
    text = repr(result)
    if len(text) > MAX_AGENT_REPR_LENGTH:
        raise ValueError(f"{result_type.__name__} repr exceeds {MAX_AGENT_REPR_LENGTH} characters")


AGENT_READABLE_TYPES = (
    PlanningGroupInfo,
    PlanningGroupState,
    ManipulationSnapshot,
    PlanResult,
    ExecutionResult,
    CommandResult,
    MoveResult,
)

for _result_type in AGENT_READABLE_TYPES:
    assert_agent_readable_result_type(_result_type)


class ManipulationSpec(Spec, Protocol):
    """Primitive cross-module manipulation interface."""

    def list_planning_groups(self) -> tuple[PlanningGroupInfo, ...]: ...

    def get_state(self) -> ManipulationSnapshot: ...

    def plan_to_joints(
        self,
        targets: Mapping[PlanningGroupID, JointState],
        speed_scale: float | None = None,
    ) -> PlanResult: ...

    def plan_to_poses(
        self,
        targets: Mapping[PlanningGroupID, PoseStamped],
        speed_scale: float | None = None,
    ) -> PlanResult: ...

    def execute(self, blocking: bool = True, timeout: float | None = None) -> ExecutionResult: ...

    def wait_for_execution(self, timeout: float | None = None) -> ExecutionResult: ...

    def move_linear(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        planning_group: PlanningGroupID | None = None,
        check_collision: bool = False,
        speed_scale: float | None = None,
        blocking: bool = True,
        timeout: float | None = None,
    ) -> MoveResult: ...

    def set_gripper_position(
        self,
        position: float,
        planning_group: PlanningGroupID | None = None,
    ) -> CommandResult: ...

    def cancel(self) -> ExecutionResult: ...

    def reset(self) -> CommandResult: ...
