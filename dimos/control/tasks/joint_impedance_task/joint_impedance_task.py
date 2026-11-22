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

"""Follow a leader arm softly, so the follower pushes with limited force when
something is in the way."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from typing import TYPE_CHECKING, Annotated, Protocol

from pydantic import Field

from dimos.control.contract.keys import Interface
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig
    from dimos.msgs.sensor_msgs.JointState import JointState

# A leader sending in this frame pauses with zero speeds and no positions.
_GUARDED_FRAME_ID = "dimos/guarded-joint"

_AtLeastZero = Annotated[float, Field(ge=0.0, allow_inf_nan=False)]
_AboveZero = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]


class JointImpedanceTaskParams(BaseConfig):
    """Settings for a ``joint_impedance`` task, from ``TaskConfig.params``.

    Each map is keyed by full joint name, e.g. "arm/joint1".

    Attributes:
        stiffness: How hard each joint is pulled toward the leader's position,
            in Nm/rad. Required for every joint. A value above the robot's own
            maximum is lowered to that maximum when sent.
        damping: How strongly each joint resists moving at a different speed
            from its target, in Nm·s/rad. Required for every joint. A value
            above the robot's own maximum is lowered to that maximum when sent.
        max_torque: The most each joint's spring may push, in Nm. Required for
            every joint. The target stays within ``max_torque / stiffness`` rad
            of where the joint is measured, so a blocked follower pushes no
            harder than this however far the leader moves. No effect at
            stiffness 0.
        velocity_limits: How fast each joint's target may move, in rad/s. If
            given, it must name every joint; otherwise 1 rad/s each. Too low and
            the follower lags behind the leader.
        send_velocity: Also send the target's speed. Without it the robot is
            told speed 0, so damping brakes the follower whenever it moves.
        timeout_s: Seconds without a leader message after which the task lets
            go, and the robot holds each joint where it is measured, at its own
            stiffness.
    """

    stiffness: dict[str, _AtLeastZero]
    damping: dict[str, _AtLeastZero]
    max_torque: dict[str, _AboveZero]
    velocity_limits: dict[str, _AboveZero] | None = None
    send_velocity: bool = True
    timeout_s: _AboveZero = 0.4


class ImpedanceSource(Protocol):
    """A robot driven through a connection module, as this task sees it."""

    @property
    def joint_command_interfaces(self) -> dict[str, frozenset[Interface]]: ...


class JointImpedanceTask(BaseControlTask):
    """Follow the leader's joint positions as a soft spring.

    Each joint's target moves toward the leader's position no faster than its
    velocity limit, and stays close enough to where the joint is measured that
    the spring never pushes harder than ``max_torque``. Gravity is left to the
    robot.

    Args:
        name: The task's name.
        joint_names: Full names of the joints it drives, e.g. ["arm/joint1"].
        priority: Which task wins a joint two tasks drive; higher wins.
        params: Its settings.

    Raises:
        ValueError: If a joint has no stiffness, damping or max torque, or no
            velocity limit when velocity limits are given.
    """

    def __init__(
        self,
        name: str,
        joint_names: Sequence[str],
        priority: int,
        params: JointImpedanceTaskParams,
    ) -> None:
        joints = list(joint_names)
        velocity_limits = params.velocity_limits or dict.fromkeys(joints, 1.0)
        for setting, values in (
            ("stiffness", params.stiffness),
            ("damping", params.damping),
            ("max_torque", params.max_torque),
            ("velocity_limits", velocity_limits),
        ):
            for joint in joints:
                if joint not in values:
                    raise ValueError(
                        f"joint_impedance task {name!r}: joint {joint!r} has no {setting}"
                    )
        self._name = name
        self._joint_names = joints
        self._claim = ResourceClaim(
            frozenset(joints), priority=priority, mode=ControlMode.IMPEDANCE
        )
        self._stiffness = {joint: params.stiffness[joint] for joint in joints}
        self._damping = {joint: params.damping[joint] for joint in joints}
        self._velocity_limits = {joint: velocity_limits[joint] for joint in joints}
        # How far each target may be from the measured position, in rad.
        self._reach = {
            joint: params.max_torque[joint] / params.stiffness[joint]
            if params.stiffness[joint] > 0.0
            else math.inf
            for joint in joints
        }
        self._send_velocity = params.send_velocity
        self._timeout_s = params.timeout_s
        # Joint -> the leader's latest position, in rad.
        self._goals: dict[str, float] = {}
        # Joint -> when its leader position expires, by the coordinator clock, in s.
        self._deadlines: dict[str, float] = {}
        # Joint -> the target sent on the last tick, in rad.
        self._targets: dict[str, float] = {}
        self._estopped = False

    def claim(self) -> ResourceClaim:
        return self._claim

    def is_active(self) -> bool:
        return bool(self._goals)

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """This tick's targets, or None when no joint has a leader position."""
        if not self._goals:
            return None
        dt = max(0.0, state.dt)
        names: list[str] = []
        positions: list[float] = []
        velocities: list[float] | None = [] if self._send_velocity else None
        for name in self._joint_names:
            goal = self._goals.get(name)
            if goal is None:
                continue
            if state.t_now >= self._deadlines[name]:
                self._forget(name)
                continue
            measured = state.joints.get_position(name)
            if measured is None or not math.isfinite(measured):
                # Start again from the measured position once it is back.
                self._targets.pop(name, None)
                continue
            previous = self._targets.get(name)
            start = measured if previous is None else previous
            step = self._velocity_limits[name] * dt
            target = start + max(-step, min(step, goal - start))
            # The spring pushes stiffness * (target - measured), so this keeps it within max_torque.
            reach = self._reach[name]
            target = max(measured - reach, min(measured + reach, target))
            self._targets[name] = target
            names.append(name)
            positions.append(target)
            if velocities is not None:
                velocities.append(
                    (target - previous) / dt if previous is not None and dt > 0.0 else 0.0
                )
        if not names:
            return None
        return JointCommandOutput(
            joint_names=names,
            positions=positions,
            velocities=velocities,
            stiffness=[self._stiffness[name] for name in names],
            damping=[self._damping[name] for name in names],
            mode=ControlMode.IMPEDANCE,
        )

    def on_joint_command(self, msg: JointState, t_now: float) -> bool:
        """Take the leader's latest joint positions.

        A message in the "dimos/guarded-joint" frame with zero speeds and no
        positions means the leader paused: the joints it names are let go now.

        Args:
            msg: Joint names and positions, in rad. Joints this task does not
                drive are ignored.
            t_now: When it arrived, by the coordinator clock, in s.

        Returns:
            Whether the message was used. A message with no joint of this
            task, duplicate names or a non-finite position is refused, and so
            is every position while the emergency stop is on.
        """
        if (
            msg.frame_id == _GUARDED_FRAME_ID
            and not msg.position
            and len(msg.name) == len(msg.velocity)
        ):
            if not msg.name or any(value != 0.0 for value in msg.velocity):
                return False
            for name in msg.name:
                self._forget(name)
            return True
        if self._estopped or not msg.position or len(msg.name) != len(msg.position):
            return False
        selected = [
            (name, position)
            for name, position in zip(msg.name, msg.position, strict=True)
            if name in self._claim.joints
        ]
        if (
            not selected
            or len({name for name, _ in selected}) != len(selected)
            or not all(math.isfinite(position) for _, position in selected)
        ):
            return False
        deadline = t_now + self._timeout_s
        for name, position in selected:
            self._goals[name] = position
            self._deadlines[name] = deadline
        return True

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Let go of the joints another task took; each starts again from
        where it is measured on the next leader message."""
        for name in joints:
            self._forget(name)

    def set_estop(self, estopped: bool) -> None:
        """Let go of every joint, and refuse leader positions until cleared."""
        self._estopped = estopped
        if estopped:
            self._goals.clear()
            self._deadlines.clear()
            self._targets.clear()

    def _forget(self, name: str) -> None:
        self._goals.pop(name, None)
        self._deadlines.pop(name, None)
        self._targets.pop(name, None)


def create_task(cfg: TaskConfig, connections: Mapping[str, ImpedanceSource]) -> JointImpedanceTask:
    """Build the task, refusing joints whose robot cannot take stiffness and damping.

    Raises:
        ValueError: If a joint belongs to no robot this coordinator drives,
            or its robot does not accept position, stiffness, damping and
            (with ``send_velocity``) velocity; or a setting is missing for a
            joint.
    """
    params = JointImpedanceTaskParams.model_validate(cfg.params)
    where = f"joint_impedance task {cfg.name!r}"
    accepted = {
        joint: interfaces
        for source in connections.values()
        for joint, interfaces in source.joint_command_interfaces.items()
    }
    needed = {Interface.POSITION, Interface.STIFFNESS, Interface.DAMPING}
    if params.send_velocity:
        needed.add(Interface.VELOCITY)
    for joint in cfg.joint_names:
        if joint not in accepted:
            raise ValueError(f"{where}: joint {joint!r} belongs to no robot driven here")
        missing = needed - accepted[joint]
        if missing:
            raise ValueError(
                f"{where}: joint {joint!r} does not accept "
                f"{', '.join(sorted(interface.value for interface in missing))}"
            )
    return JointImpedanceTask(cfg.name, cfg.joint_names, cfg.priority, params)
