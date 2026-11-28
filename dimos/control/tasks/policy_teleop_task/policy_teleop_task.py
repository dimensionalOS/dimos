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

"""Blend operator and policy joint targets on the coordinator's clock."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import importlib
import math
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, TypeAlias, cast

from pydantic import Field

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    ControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig

logger = setup_logger()


@dataclass(frozen=True)
class TakeoverContext:
    """Measured joints Q and latest operator U / policy P targets for one tick.

    ``joint_names`` contains only joints where both sides are present. Positions
    use the source tasks' joint units; ``t_now`` and ``dt`` are coordinator seconds.
    """

    joint_names: tuple[str, ...]
    measured: Mapping[str, float]
    operator: Mapping[str, float]
    policy: Mapping[str, float]
    t_now: float
    dt: float


TakeoverFn: TypeAlias = Callable[[TakeoverContext], float | Mapping[str, float]]


class PolicyTeleopTaskParams(BaseConfig):
    """Source task names and arbitration settings.

    Attributes:
        alpha: Policy share when both sides are present, in [0, 1].
        takeover_fn: Callable or import path ``package.module:function``.
        switch_joints: Joints selected at weight 0.5 instead of interpolated.
        weight_rate: Maximum weight change per second.
        source_timeout_s: Seconds a joint's last source output counts as present.
    """

    operator_tasks: list[str] = Field(min_length=1)
    policy_tasks: list[str] = Field(min_length=1)
    alpha: float = Field(default=0.5, ge=0, le=1, allow_inf_nan=False)
    takeover_fn: str | TakeoverFn | None = None
    switch_joints: list[str] = Field(default_factory=list)
    weight_rate: float = Field(default=2.0, gt=0, allow_inf_nan=False)
    source_timeout_s: float = Field(default=0.2, gt=0, allow_inf_nan=False)


def _resolve_takeover(value: str | TakeoverFn | None) -> TakeoverFn | None:
    if value is None:
        return None
    if isinstance(value, str):
        module, separator, attribute = value.partition(":")
        if not separator or not module or not attribute:
            raise ValueError("takeover_fn must be package.module:function")
        value = getattr(importlib.import_module(module), attribute)
    if not callable(value):
        raise ValueError("takeover_fn must resolve to a callable")
    return cast("TakeoverFn", value)


class PolicyTeleopTask(BaseControlTask):
    """Own the source tasks' joints and blend their per-tick position targets.

    Sources remain registered for their input routing, RPCs and lifecycle, but
    only this task computes them; they never arbitrate against one another.
    """

    def __init__(
        self,
        name: str,
        joint_names: Sequence[str],
        params: PolicyTeleopTaskParams,
        tasks: Mapping[str, ControlTask],
        priority: int = 10,
    ) -> None:
        names = tuple(joint_names)
        if not names or any(not joint for joint in names) or len(set(names)) != len(names):
            raise ValueError("policy_teleop joint_names must be non-empty and unique")
        sources = [*params.operator_tasks, *params.policy_tasks]
        if len(set(sources)) != len(sources) or name in sources:
            raise ValueError("policy_teleop sources must be distinct and cannot include itself")
        missing = set(sources) - tasks.keys()
        if missing:
            raise ValueError(f"policy_teleop sources must be listed first: {sorted(missing)}")
        for task in tasks.values():
            if isinstance(task, PolicyTeleopTask) and (
                task.name in sources or task.source_task_names.intersection(sources)
            ):
                raise ValueError("policy_teleop sources cannot be nested or shared")
        source_joints = frozenset(
            joint for source in sources for joint in tasks[source].claim().joints
        )
        if not source_joints.issubset(names):
            raise ValueError("policy_teleop joint_names must include every source joint")
        if not set(params.switch_joints).issubset(names):
            raise ValueError("switch_joints must be included in policy_teleop joint_names")

        self._name = name
        self._joint_names = names
        self._claim = ResourceClaim(frozenset(names), priority, ControlMode.SERVO_POSITION)
        self._params = params
        self._operator_tasks = tuple(tasks[source] for source in params.operator_tasks)
        self._policy_tasks = tuple(tasks[source] for source in params.policy_tasks)
        self._source_task_names = frozenset(sources)
        self._takeover_fn = _resolve_takeover(params.takeover_fn)
        self._takeover_failure_logged = False
        self._switch_joints = frozenset(params.switch_joints)
        self._operator: dict[str, float] = {}
        self._policy: dict[str, float] = {}
        self._operator_times: dict[str, float] = {}
        self._policy_times: dict[str, float] = {}
        self._weights = dict.fromkeys(names, 1.0)
        self._operator_present: set[str] = set()
        self._policy_present: set[str] = set()

    @property
    def source_task_names(self) -> frozenset[str]:
        """Registered tasks computed exclusively by this task."""
        return self._source_task_names

    def claim(self) -> ResourceClaim:
        return self._claim

    def is_active(self) -> bool:
        return True

    def _run_sources(
        self,
        sources: tuple[ControlTask, ...],
        values: dict[str, float],
        timestamps: dict[str, float],
        state: CoordinatorState,
    ) -> set[str]:
        for task in sources:
            output = task.compute(state) if task.is_active() else None
            if output is None:
                continue
            commands = output if isinstance(output, list) else [output]
            for command in commands:
                if command.mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
                    raise ValueError(f"policy_teleop source {task.name!r} must command positions")
                if command.positions is None:
                    continue
                for joint, position in zip(command.joint_names, command.positions, strict=True):
                    if joint not in self._weights or not math.isfinite(position):
                        raise ValueError(
                            f"policy_teleop source {task.name!r} returned an invalid joint target"
                        )
                    values[joint] = position
                    timestamps[joint] = state.t_now
        return {
            joint
            for joint, timestamp in timestamps.items()
            if state.t_now - timestamp <= self._params.source_timeout_s
        }

    def _target_weights(self, joints: tuple[str, ...], state: CoordinatorState) -> dict[str, float]:
        fallback = dict.fromkeys(joints, self._params.alpha)
        if not joints or self._takeover_fn is None:
            return fallback
        context = TakeoverContext(
            joint_names=joints,
            measured=MappingProxyType(state.joints.joint_positions.copy()),
            operator=MappingProxyType(self._operator.copy()),
            policy=MappingProxyType(self._policy.copy()),
            t_now=state.t_now,
            dt=state.dt,
        )
        try:
            result = self._takeover_fn(context)
            weights = (
                {joint: float(value) for joint, value in result.items()}
                if isinstance(result, Mapping)
                else dict.fromkeys(joints, float(result))
            )
            if not all(math.isfinite(value) for value in weights.values()):
                raise ValueError("takeover_fn returned a non-finite weight")
            return {
                joint: max(0.0, min(1.0, weights.get(joint, self._params.alpha)))
                for joint in joints
            }
        except Exception:
            if not self._takeover_failure_logged:
                logger.exception("Policy teleop takeover_fn failed; using alpha", task=self._name)
                self._takeover_failure_logged = True
            return fallback

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        self._operator_present = self._run_sources(
            self._operator_tasks, self._operator, self._operator_times, state
        )
        self._policy_present = self._run_sources(
            self._policy_tasks, self._policy, self._policy_times, state
        )
        both_present = self._operator_present & self._policy_present
        both = tuple(joint for joint in self._joint_names if joint in both_present)
        targets = self._target_weights(both, state)
        max_change = self._params.weight_rate * max(0.0, state.dt)
        commands: dict[str, float] = {}
        for joint in self._joint_names:
            operator_present = joint in self._operator_present
            policy_present = joint in self._policy_present
            if not operator_present and not policy_present:
                continue
            target = (
                targets[joint] if operator_present and policy_present else float(policy_present)
            )
            weight = self._weights[joint]
            if joint not in self._operator or joint not in self._policy:
                weight = target
            elif abs(target - weight) <= max_change:
                weight = target
            else:
                weight += math.copysign(max_change, target - weight)
            self._weights[joint] = weight
            operator = self._operator.get(joint, 0.0)
            policy = self._policy.get(joint, 0.0)
            commands[joint] = (
                (policy if weight >= 0.5 else operator)
                if joint in self._switch_joints
                else (1.0 - weight) * operator + weight * policy
            )
        if not commands:
            return None
        return JointCommandOutput(
            joint_names=list(commands),
            positions=list(commands.values()),
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        for task in (*self._operator_tasks, *self._policy_tasks):
            task.on_preempted(by_task, joints)

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        self._operator.clear()
        self._policy.clear()
        self._operator_times.clear()
        self._policy_times.clear()
        self._weights = dict.fromkeys(self._joint_names, 1.0)
        self._operator_present.clear()
        self._policy_present.clear()
        return True

    def set_estop(self, estopped: bool) -> None:
        if estopped:
            self.reset_runtime_state()

    def get_status(self) -> dict[str, Any]:
        """Return policy weights and source presence from the latest tick."""
        return {
            "weights": self._weights.copy(),
            "operator_present": sorted(self._operator_present),
            "policy_present": sorted(self._policy_present),
        }


def create_task(
    cfg: TaskConfig,
    tasks: Mapping[str, ControlTask],
) -> PolicyTeleopTask:
    return PolicyTeleopTask(
        cfg.name,
        cfg.joint_names,
        PolicyTeleopTaskParams.model_validate(cfg.params),
        tasks,
        cfg.priority,
    )
