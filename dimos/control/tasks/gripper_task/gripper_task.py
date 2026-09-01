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

"""Gripper intent policy for an ordinary configured set of joints."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import TYPE_CHECKING, Any

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.std_msgs.Float32 import Float32

logger = setup_logger()

# 0.0 = fully closed, 1.0 = fully open on every scalar API.
_CLOSED = 0.0
_OPEN = 1.0


@dataclass
class GripperControlTaskConfig:
    """Configuration for the gripper task.

    Attributes:
        joint_names: Ordinary joints this task owns, in command order.
        priority: Priority for arbitration.
        hold_duration: Seconds to keep emitting a target; 0.0 holds forever.
        requires_activation: Reject commands until explicitly activated, and
            deactivate after any control preemption.
    """

    joint_names: list[str]
    priority: int = 10
    hold_duration: float = 0.0
    requires_activation: bool = False


class GripperControlTask(BaseControlTask):
    """Normalized-command owner of a device's gripper joints."""

    def __init__(
        self,
        name: str,
        config: GripperControlTaskConfig,
        limits: list[tuple[float, float]],
    ) -> None:
        if not config.joint_names:
            raise ValueError(f"GripperControlTask '{name}' requires at least one joint")
        if len(limits) != len(config.joint_names):
            raise ValueError(
                f"GripperControlTask '{name}': got {len(limits)} limit pairs for "
                f"{len(config.joint_names)} joints"
            )
        for joint_name, (lo, hi) in zip(config.joint_names, limits, strict=True):
            if not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi:
                raise ValueError(
                    f"GripperControlTask '{name}': joint {joint_name!r} requires "
                    f"finite ordered limits, got ({lo}, {hi})"
                )

        self._name = name
        self._config = config
        self._joint_names = tuple(config.joint_names)
        self._limits = list(limits)

        self._lock = threading.Lock()
        self._target: list[float] | None = None
        self._target_set_at: float = 0.0
        # Commands without the coordinator clock are stamped on the next tick.
        self._stamp_pending = False
        self._estopped = False
        self._measured: dict[str, float] = {}
        self._warned_normalized_clamps: set[str] = set()
        self._activated = not config.requires_activation

    def claim(self) -> ResourceClaim:
        """Declare resource requirements."""
        return ResourceClaim(
            joints=frozenset(self._joint_names),
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        """Read state and emit commands only while activated."""
        return self._activated

    def activate(self) -> bool:
        """Allow new gripper commands."""
        with self._lock:
            self._activated = True
        return True

    def deactivate(self) -> bool:
        """Reject new commands and release the gripper joint."""
        with self._lock:
            self._activated = False
            self._target = None
            self._stamp_pending = False
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Record the measured snapshot, then emit the target while holding."""
        measured = {
            name: pos
            for name in self._joint_names
            if (pos := state.joints.get_position(name)) is not None
        }

        with self._lock:
            if measured:
                self._measured = measured
            if self._estopped or self._target is None:
                return None
            if self._stamp_pending:
                self._target_set_at = state.t_now
                self._stamp_pending = False
            hold = self._config.hold_duration
            if hold > 0.0 and (state.t_now - self._target_set_at) > hold:
                # Hold expired; ConnectedHardware keeps re-sending the last value.
                self._target = None
                return None
            target = list(self._target)

        return JointCommandOutput(
            joint_names=list(self._joint_names),
            positions=target,
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Log preemption and fail closed when activation is required."""
        claimed = frozenset(self._joint_names)
        if joints & claimed:
            logger.warning(
                "Gripper joints preempted",
                task=self._name,
                preempting_task=by_task,
                joints=sorted(joints & claimed),
            )
            if self._config.requires_activation:
                self.deactivate()

    def set_estop(self, estopped: bool) -> None:
        """Latch E-STOP and drop the target so it cannot replay."""
        with self._lock:
            self._estopped = estopped
            if estopped:
                self._target = None

    def set_position(self, values: list[float], t_now: float | None = None) -> bool:
        """Set per-joint targets in the adapter's native units."""
        if not self._validate_values("set_position", values, self._limits):
            return False
        return self._latch(list(values), t_now)

    def set_normalized(self, values: list[float], t_now: float | None = None) -> bool:
        """Set bounded per-joint openings; finite inputs saturate to 0.0-1.0."""
        if len(values) != len(self._joint_names):
            logger.warning(
                "Joint command rejected",
                task=self._name,
                method="set_normalized",
                reason="arity",
                expected=len(self._joint_names),
                got=len(values),
            )
            return False
        if any(not math.isfinite(value) for value in values):
            logger.warning(
                "Joint command rejected",
                task=self._name,
                method="set_normalized",
                reason="non-finite",
                values=values,
            )
            return False

        normalized = [min(_OPEN, max(_CLOSED, value)) for value in values]
        for name, value, bounded in zip(self._joint_names, values, normalized, strict=True):
            if value != bounded and name not in self._warned_normalized_clamps:
                logger.warning(
                    "Normalized gripper command saturated",
                    task=self._name,
                    joint_name=name,
                    value=value,
                    saturated=bounded,
                )
                self._warned_normalized_clamps.add(name)
        native = [
            lo + (hi - lo) * value for value, (lo, hi) in zip(normalized, self._limits, strict=True)
        ]
        return self._latch(native, t_now)

    def get_position(self) -> list[float] | None:
        """Measured positions in native units, from the tick snapshot."""
        with self._lock:
            if any(n not in self._measured for n in self._joint_names):
                return None
            return [self._measured[n] for n in self._joint_names]

    def get_normalized(self) -> list[float] | None:
        """Measured positions as 0.0-1.0 of travel; 0.0 closed, 1.0 open."""
        positions = self.get_position()
        if positions is None:
            return None
        normalized = [
            (p - lo) / (hi - lo) for p, (lo, hi) in zip(positions, self._limits, strict=True)
        ]
        for name, position, value in zip(self._joint_names, positions, normalized, strict=True):
            if not _CLOSED <= value <= _OPEN:
                logger.warning(
                    "Measured joint is outside declared limits",
                    task=self._name,
                    joint_name=name,
                    value=position,
                    normalized=value,
                )
        return normalized

    def on_gripper_command(self, msg: Float32, t_now: float) -> bool:
        """Apply one normalized opening to every joint owned by the task."""
        return self.set_normalized([float(msg.data)] * len(self._joint_names), t_now)

    def _validate_values(
        self, method: str, values: list[float], limits: list[tuple[float, float]]
    ) -> bool:
        if len(values) != len(self._joint_names):
            logger.warning(
                "Joint command rejected",
                task=self._name,
                method=method,
                reason="arity",
                expected=len(self._joint_names),
                got=len(values),
            )
            return False
        for name, value, (lo, hi) in zip(self._joint_names, values, limits, strict=True):
            if not math.isfinite(value) or not lo <= value <= hi:
                logger.warning(
                    "Joint command rejected",
                    task=self._name,
                    method=method,
                    joint_name=name,
                    value=value,
                    lower=lo,
                    upper=hi,
                )
                return False
        return True

    def _latch(self, target: list[float], t_now: float | None) -> bool:
        with self._lock:
            if self._estopped or not self._activated:
                return False
            self._target = target
            if t_now is None:
                self._stamp_pending = True
            else:
                self._target_set_at = t_now
                self._stamp_pending = False
        return True


class GripperControlTaskParams(BaseConfig):
    """Task-specific gripper parameters."""

    hold_duration: float = 0.0
    requires_activation: bool = False


def _resolve_limits(cfg: Any, hardware: Any) -> list[tuple[float, float]]:
    """Resolve configured joints against ordinary adapter limit arrays."""
    where = f"gripper task {cfg.name!r}"
    joint_names = list(cfg.joint_names)
    if not joint_names:
        raise ValueError(f"{where}: requires at least one joint")
    if len(set(joint_names)) != len(joint_names):
        raise ValueError(f"{where}: joint_names must not contain duplicates")
    resolved: list[tuple[float, float]] = []
    for joint_name in joint_names:
        owners = [
            connected
            for connected in (hardware or {}).values()
            if joint_name in connected.component.joints
        ]
        if not owners:
            raise ValueError(f"{where}: joint {joint_name!r} is not owned by coordinator hardware")
        if len(owners) > 1:
            raise ValueError(
                f"{where}: joint {joint_name!r} is owned by multiple hardware components"
            )
        connected = owners[0]
        component = connected.component
        index = component.joints.index(joint_name)
        limits = connected.get_limits()
        if limits is None:
            raise ValueError(
                f"{where}: hardware {component.hardware_id!r} does not declare joint limits"
            )
        arrays = (limits.position_lower, limits.position_upper, limits.velocity_max)
        if any(len(array) != len(component.joints) for array in arrays):
            raise ValueError(
                f"{where}: hardware {component.hardware_id!r} limits must contain "
                f"{len(component.joints)} entries"
            )
        lower = limits.position_lower[index]
        upper = limits.position_upper[index]
        if lower is None or upper is None:
            raise ValueError(f"{where}: joint {joint_name!r} has no declared position limits")
        resolved.append((float(lower), float(upper)))
    return resolved


def create_task(cfg: Any, hardware: Any) -> GripperControlTask:
    params = GripperControlTaskParams.model_validate(cfg.params)
    return GripperControlTask(
        cfg.name,
        GripperControlTaskConfig(
            joint_names=list(cfg.joint_names),
            priority=cfg.priority,
            hold_duration=params.hold_duration,
            requires_activation=params.requires_activation,
        ),
        limits=_resolve_limits(cfg, hardware),
    )
