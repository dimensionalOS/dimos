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

"""Gripper control task: sole owner of a device's gripper joints.

Converts normalized and sweep commands to the adapter's native units using
limits read at construction. See dimos/hardware/GRIPPER-SPEC.md.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING, Any

from dimos.control.components import split_joint_name
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
    from dimos.msgs.std_msgs.Bool import Bool
    from dimos.teleop.quest.quest_types import Buttons

logger = setup_logger()

# 0.0 = fully closed, 1.0 = fully open on every scalar API.
_CLOSED = 0.0
_OPEN = 1.0


@dataclass
class GripperControlTaskConfig:
    """Configuration for the gripper task.

    Attributes:
        joint_names: Gripper joints this task owns; must equal one component's
            gripper_joints, in order.
        priority: Priority for arbitration.
        hold_duration: Seconds to keep emitting a target; 0.0 holds forever.
        reference_pose: Grasp posture per joint, native units. Required for
            multi-joint grippers; a single jaw uses its closed limit.
        open_posture: Open end of a sweep; defaults to the upper limits.
        hand: Which controller trigger drives this gripper, if any.
        require_engagement: Accept the trigger only while that hand's primary
            button is held.
    """

    joint_names: list[str]
    priority: int = 10
    hold_duration: float = 0.0
    reference_pose: list[float] | None = None
    open_posture: list[float] | None = None
    hand: str | None = None
    require_engagement: bool = True


class GripperControlTask(BaseControlTask):
    """Command- and stream-driven owner of a device's gripper joints."""

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

        self._name = name
        self._config = config
        self._joint_names_list = list(config.joint_names)
        self._joint_names = frozenset(config.joint_names)
        self._n = len(self._joint_names_list)
        self._limits = list(limits)

        self._open_posture = self._resolve_open_posture(config)
        self._reference_pose = self._resolve_reference_pose(config)

        self._lock = threading.Lock()
        self._target: list[float] | None = None
        self._target_set_at: float = 0.0
        # Commands without the coordinator clock are stamped on the next tick.
        self._stamp_pending = False
        self._estopped = False
        self._measured: dict[str, float] = {}
        self._primary_down = False

        logger.info(
            "GripperControlTask initialized",
            task=name,
            joints=self._joint_names_list,
            limits=self._limits,
            hold_duration=config.hold_duration,
        )

    def _resolve_open_posture(self, config: GripperControlTaskConfig) -> list[float]:
        if config.open_posture is None:
            return [hi for _lo, hi in self._limits]
        if len(config.open_posture) != self._n:
            raise ValueError(
                f"GripperControlTask '{self._name}': open_posture has "
                f"{len(config.open_posture)} values for {self._n} joints"
            )
        return list(config.open_posture)

    def _resolve_reference_pose(self, config: GripperControlTaskConfig) -> list[float] | None:
        # A multi-joint gripper must declare its grasp: joint limits describe
        # travel, not grasping.
        if config.reference_pose is not None:
            if len(config.reference_pose) != self._n:
                raise ValueError(
                    f"GripperControlTask '{self._name}': reference_pose has "
                    f"{len(config.reference_pose)} values for {self._n} joints"
                )
            return list(config.reference_pose)
        if self._n == 1:
            return [self._limits[0][0]]
        return None

    def claim(self) -> ResourceClaim:
        """Declare resource requirements."""
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        """Always True; compute() decides what to emit so reads stay fresh."""
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Record the measured snapshot, then emit the target while holding."""
        measured = {
            name: pos
            for name in self._joint_names_list
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
            joint_names=self._joint_names_list,
            positions=target,
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Log preemption; this task is meant to be the sole claimant."""
        if joints & self._joint_names:
            logger.warning(
                "Gripper joints preempted",
                task=self._name,
                preempting_task=by_task,
                joints=sorted(joints & self._joint_names),
            )

    def set_estop(self, estopped: bool) -> None:
        """Latch E-STOP and drop the target so it cannot replay."""
        with self._lock:
            self._estopped = estopped
            if estopped:
                self._target = None

    def set_position(self, values: list[float], t_now: float | None = None) -> bool:
        """Set per-joint targets in the adapter's native units."""
        if len(values) != self._n:
            logger.warning(
                "set_position arity mismatch", task=self._name, expected=self._n, got=len(values)
            )
            return False
        clamped = [max(lo, min(hi, v)) for v, (lo, hi) in zip(values, self._limits, strict=True)]
        return self._latch(clamped, t_now)

    def set_normalized(self, values: list[float], t_now: float | None = None) -> bool:
        """Set per-joint targets as 0.0-1.0 of travel; 0.0 closed, 1.0 open."""
        if len(values) != self._n:
            logger.warning(
                "set_normalized arity mismatch", task=self._name, expected=self._n, got=len(values)
            )
            return False
        native = [
            lo + (hi - lo) * max(_CLOSED, min(_OPEN, v))
            for v, (lo, hi) in zip(values, self._limits, strict=True)
        ]
        return self._latch(native, t_now)

    def set_sweep(self, value: float, t_now: float | None = None) -> bool:
        """Interpolate between the reference (grasp) posture and fully open."""
        if self._reference_pose is None:
            logger.error(
                "set_sweep requires a configured reference_pose on a multi-joint gripper",
                task=self._name,
                joints=self._joint_names_list,
            )
            return False
        v = max(_CLOSED, min(_OPEN, value))
        target = [
            ref + (opn - ref) * v
            for ref, opn in zip(self._reference_pose, self._open_posture, strict=True)
        ]
        return self._latch(target, t_now)

    def set_reference_pose(self, values: list[float]) -> bool:
        """Replace the grasp posture at runtime."""
        if len(values) != self._n:
            logger.warning(
                "set_reference_pose arity mismatch",
                task=self._name,
                expected=self._n,
                got=len(values),
            )
            return False
        with self._lock:
            self._reference_pose = list(values)
        return True

    def get_position(self) -> list[float] | None:
        """Measured positions in native units, from the tick snapshot."""
        with self._lock:
            if any(n not in self._measured for n in self._joint_names_list):
                return None
            return [self._measured[n] for n in self._joint_names_list]

    def get_normalized(self) -> list[float] | None:
        """Measured positions as 0.0-1.0 of travel; 0.0 closed, 1.0 open."""
        positions = self.get_position()
        if positions is None:
            return None
        return [
            0.0 if hi == lo else max(0.0, min(1.0, (p - lo) / (hi - lo)))
            for p, (lo, hi) in zip(positions, self._limits, strict=True)
        ]

    def get_state(self) -> dict[str, Any]:
        """Report the task's target, limits, and status."""
        with self._lock:
            return {
                "task": self._name,
                "joints": list(self._joint_names_list),
                "limits": list(self._limits),
                "target": None if self._target is None else list(self._target),
                "holding": self._target is not None,
                "estopped": self._estopped,
                "reference_pose": None
                if self._reference_pose is None
                else list(self._reference_pose),
            }

    def on_gripper_command(self, msg: Bool, t_now: float) -> bool:
        """Handle an open/closed toggle (True = closed)."""
        return self.set_sweep(_CLOSED if msg.data else _OPEN, t_now)

    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> bool:
        """Handle the configured hand's analog trigger; squeezing closes."""
        hand = self._config.hand
        if hand not in ("left", "right"):
            return False

        is_left = hand == "left"
        primary = msg.left_primary if is_left else msg.right_primary
        trigger = msg.left_trigger_analog if is_left else msg.right_trigger_analog

        with self._lock:
            self._primary_down = bool(primary)
            if self._estopped:
                return False
            gated = self._config.require_engagement and not primary

        if gated:
            return False
        squeeze = max(_CLOSED, min(_OPEN, float(trigger)))
        return self.set_sweep(_OPEN - squeeze, t_now)

    def _latch(self, target: list[float], t_now: float | None) -> bool:
        with self._lock:
            if self._estopped:
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
    reference_pose: list[float] | None = None
    open_posture: list[float] | None = None
    hand: str | None = None
    require_engagement: bool = True


def _resolve_limits(cfg: Any, hardware: Any) -> list[tuple[float, float]]:
    """Resolve the gripper's limits from its component's adapter."""
    where = f"gripper task {cfg.name!r}"
    joint_names = list(cfg.joint_names)
    if not joint_names:
        raise ValueError(f"{where}: requires at least one joint")

    hardware_ids = {split_joint_name(n)[0] for n in joint_names}
    if len(hardware_ids) != 1:
        raise ValueError(
            f"{where}: claims joints across {sorted(hardware_ids)}; a gripper task "
            "spans exactly one device"
        )
    hardware_id = hardware_ids.pop()

    connected = (hardware or {}).get(hardware_id)
    if connected is None:
        raise ValueError(f"{where}: no hardware {hardware_id!r} registered with the coordinator")

    component = connected.component
    if joint_names != list(component.gripper_joints):
        raise ValueError(
            f"{where}: joint_names {joint_names} must equal {hardware_id!r}'s "
            f"gripper_joints {list(component.gripper_joints)}, in order"
        )

    # Trailing slice sized by the component's declared count, not array math.
    limits = connected.adapter.get_limits()
    split = len(component.all_joints) - component.gripper_dof
    lower = list(limits.position_lower)[split:]
    upper = list(limits.position_upper)[split:]
    if len(lower) != len(joint_names) or len(upper) != len(joint_names):
        raise ValueError(
            f"{where}: adapter {component.adapter_type!r} returned "
            f"{len(limits.position_lower)} limit entries; expected "
            f"{len(component.all_joints)} covering all joints"
        )
    return list(zip(lower, upper, strict=True))


def create_task(cfg: Any, hardware: Any) -> GripperControlTask:
    params = GripperControlTaskParams.model_validate(cfg.params)
    return GripperControlTask(
        cfg.name,
        GripperControlTaskConfig(
            joint_names=list(cfg.joint_names),
            priority=cfg.priority,
            hold_duration=params.hold_duration,
            reference_pose=params.reference_pose,
            open_posture=params.open_posture,
            hand=params.hand,
            require_engagement=params.require_engagement,
        ),
        limits=_resolve_limits(cfg, hardware),
    )
