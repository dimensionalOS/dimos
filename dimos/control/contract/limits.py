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

"""Which keys a robot accepts, and how far each may go.

A connection's per-joint interface declaration and robot model define one
key-to-Limits table. Only declared interfaces need bounds; a velocity-only
prismatic joint does not need a bounded position. ``validate_command`` checks
every command against this table.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import TYPE_CHECKING

from dimos.control.contract.keys import Interface, Key

if TYPE_CHECKING:
    from dimos.control.connection.connection_module import ConnectionConfig
    from dimos.robot.assets.model import JointDescription, RobotModel

#: The URDF joint types that can be driven. A fixed, floating or planar
#: joint cannot.
MOVABLE_JOINT_TYPES = frozenset({"revolute", "continuous", "prismatic"})

#: What a joint is told when it is held by stiffness and damping.
PD_INTERFACES: tuple[Interface, ...] = (
    Interface.POSITION,
    Interface.VELOCITY,
    Interface.EFFORT,
    Interface.KP,
    Interface.KD,
)


@dataclass(frozen=True, slots=True)
class Limits:
    """How far one command may go.

    A value past a bound is refused, or with ``clamp`` pulled back to the
    bound and sent, so a teleop stick pushed too far still drives a base at
    its top speed.

    Attributes:
        lo: The lowest allowed value, in the key's unit. ``None`` for no bound.
        hi: The highest allowed value, in the key's unit. ``None`` for no bound.
        clamp: Pull a value past a bound back to it instead of refusing the
            command. Needs both bounds.
    """

    lo: float | None = None
    hi: float | None = None
    clamp: bool = False

    def __post_init__(self) -> None:
        for bound in (self.lo, self.hi):
            if bound is not None and not math.isfinite(bound):
                raise ValueError(f"{self} has a non-finite bound")
        if self.lo is not None and self.hi is not None and self.lo > self.hi:
            raise ValueError(f"{self} has lo above hi")
        if self.clamp and (self.lo is None or self.hi is None):
            raise ValueError(f"{self} clamps but is not bounded on both sides")


def limits(config: ConnectionConfig) -> dict[str, Limits]:
    """Every key a robot accepts, with how far each may go, from its config.

    Only declared interfaces get keys, in model order. Declare PD_INTERFACES
    for joints driven by position, velocity, effort and gains together. Bounds
    come from the model; clamping is opt-in per key through ``clamp_commands``.

    Raises:
        ValueError: If joints are commanded without a model, no joint of the
            model moves, the model lacks a bound a command needs, or the
            config commands nothing.
    """
    source = config.source
    if not config.joint_commands:
        raise ValueError(f"{source!r} is told nothing: set joint_commands")
    if config.model is None:
        raise ValueError(f"{source!r} commands joints but has no model: set `model`")
    joints = {joint.name: joint for joint in movable_joints(config.model)}
    if not joints:
        raise ValueError(f"{source!r} has no joints to drive: none in its model moves")
    unknown = config.joint_commands.keys() - joints.keys()
    if unknown:
        raise ValueError(f"{source!r} declares joints absent from its model: {sorted(unknown)}")
    table: dict[str, Limits] = {}
    for name, joint in joints.items():
        if name not in config.joint_commands:
            continue
        interfaces = dict.fromkeys(config.joint_commands[name])
        for interface in interfaces:
            full_name = name if "/" in name else f"{source}/{name}"
            key = Key(f"{full_name}/{interface}")
            if key.source != source:
                raise ValueError(f"joint {name!r} belongs to another source than {source!r}")
            bound = model_limit(joint, interface)
            table[key] = replace(bound, clamp=True) if key in config.clamp_commands else bound
    if not table:
        raise ValueError(f"{source!r} is told nothing: set joint_commands")
    if config.clamp_commands - table.keys():
        raise ValueError("clamp_commands contains undeclared keys")
    return table


def movable_joints(model: RobotModel) -> tuple[JointDescription, ...]:
    """The joints of a robot model that can be driven, in the model's order.

    A revolute, continuous or prismatic joint moves; a fixed, floating or
    planar one is left out, and so is a ``mimic`` joint in the URDF, such as a
    gripper finger that follows its drive.
    """
    return tuple(
        joint
        for joint in model.load().joints
        if joint.type in MOVABLE_JOINT_TYPES and joint.mimic is None
    )


def model_limit(joint: JointDescription, interface: Interface) -> Limits:
    """How far a joint may be told to go on one interface, from its model.

    Position, velocity and effort come from the model's ``<limit>`` tag.
    Stiffness and damping (kp, kd) are any size from 0 up. A bound the model
    lacks is an error, never "no limit"; the one exception is the position
    of a joint that spins freely.

    Raises:
        ValueError: If the model lacks the bound, or a joint is not told
            ``interface``.
    """
    match interface:
        case Interface.POSITION:
            if joint.lower is not None and joint.upper is not None:
                return Limits(joint.lower, joint.upper)
            if joint.lower is not None or joint.upper is not None:
                raise ValueError(
                    f"joint {joint.name!r} has only half a position range in its model"
                )
            if joint.type != "continuous":
                raise ValueError(
                    f"joint {joint.name!r} is {joint.type} but has no position range in its "
                    f"model; only a joint that spins freely may go without one"
                )
            return Limits()
        case Interface.VELOCITY:
            bound = joint.velocity
        case Interface.EFFORT:
            bound = joint.effort
        case Interface.KP | Interface.KD:
            return Limits(0.0, None)
        case _:
            raise ValueError(f"a joint is not told {interface}; only {', '.join(PD_INTERFACES)}")
    if bound is None:
        raise ValueError(f"joint {joint.name!r} has no {interface} limit in its model")
    return Limits(-bound, bound)
