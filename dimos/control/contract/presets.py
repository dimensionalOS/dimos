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

"""Ready-made descriptions for the three kinds of hardware we drive.

Every robot driver has to describe itself before anything will talk to it.
Writing that out by hand is long and easy to get subtly wrong, so these three
functions build it in one call::

    manipulator_description("arm", JOINTS, model=XARM7_MODEL)

  ``manipulator_description``  an arm, optionally with a gripper
  ``pd_joint_description``     a body whose joints are held in place by
                               stiffness and damping, such as a humanoid
  ``twist_base_description``   something that drives around on the floor

Given the robot's model (its URDF, loaded as a ``RobotModel``), the arm and
body presets read each joint's kind and limits from it. Without one, every
joint is taken to be revolute, measured in radians, and limited only by
what is passed in ``limits``.

Each one checks its own result before handing it back, so a description built
this way is always a valid one.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from dimos.control.contract.description import (
    ControlDescription,
    Limits,
    Resource,
    ResourceKind,
)
from dimos.control.contract.keys import Interface, Key, Unit
from dimos.control.contract.validate import validate_description

if TYPE_CHECKING:
    from dimos.robot.assets.model import JointDescription, RobotModel

#: The unit of each joint interface, for a revolute joint and a prismatic
#: one. A gripper is the exception: it sets its own, because some are
#: measured in metres and some in a 0-to-1 fraction of fully open.
_REVOLUTE: Mapping[str, Unit] = {
    Interface.POSITION: Unit.RAD,
    Interface.VELOCITY: Unit.RAD_PER_S,
    Interface.EFFORT: Unit.NM,
    Interface.KP: Unit.UNITLESS,
    Interface.KD: Unit.UNITLESS,
}
_PRISMATIC: Mapping[str, Unit] = {
    Interface.POSITION: Unit.M,
    Interface.VELOCITY: Unit.M_PER_S,
    Interface.EFFORT: Unit.N,
    Interface.KP: Unit.UNITLESS,
    Interface.KD: Unit.UNITLESS,
}

_PD_STATE: tuple[Interface, ...] = (Interface.POSITION, Interface.VELOCITY, Interface.EFFORT)
_PD_COMMAND: tuple[Interface, ...] = (
    Interface.POSITION,
    Interface.VELOCITY,
    Interface.EFFORT,
    Interface.KP,
    Interface.KD,
)

_IMU_UNITS: Mapping[str, Unit] = {
    Interface.QX: Unit.UNITLESS,
    Interface.QY: Unit.UNITLESS,
    Interface.QZ: Unit.UNITLESS,
    Interface.QW: Unit.UNITLESS,
    Interface.GX: Unit.RAD_PER_S,
    Interface.GY: Unit.RAD_PER_S,
    Interface.GZ: Unit.RAD_PER_S,
    Interface.AX: Unit.M_PER_S2,
    Interface.AY: Unit.M_PER_S2,
    Interface.AZ: Unit.M_PER_S2,
}

#: What a base is told: how fast to drive forwards, sideways, and turn.
_BASE_TWIST: Mapping[str, Unit] = {
    Interface.VX: Unit.M_PER_S,
    Interface.VY: Unit.M_PER_S,
    Interface.WZ: Unit.RAD_PER_S,
}
#: Where a base reports itself as being, and which way it is facing.
_BASE_POSE: Mapping[str, Unit] = {
    Interface.X: Unit.M,
    Interface.Y: Unit.M,
    Interface.YAW: Unit.RAD,
}


@dataclass(frozen=True, slots=True, kw_only=True)
class GripperSpec:
    """A gripper to add to an arm.

    Grippers do not agree on how to measure how open they are: some report
    metres between the fingers, some a fraction from 0 to 1. So each one
    states its own unit and range. For a gripper with a single opening value.

    Attributes:
        name: What to call it, e.g. "gripper".
        unit: How its opening is measured.
        lo: The smallest value it accepts, in that unit.
        hi: The largest value it accepts, in that unit.
    """

    name: str = "gripper"
    unit: Unit
    lo: float
    hi: float


def imu_resource(name: str = "imu") -> Resource:
    """An orientation sensor, to hand to a preset in ``sensors``.

    It only reports: which way up it is (a quaternion), how fast it is
    turning (rad/s), and how hard it is accelerating (m/s^2).

    Args:
        name: What to call it, e.g. "imu" or "chest_imu".
    """
    return Resource(
        name=name,
        kind=ResourceKind.SENSOR,
        state_interfaces=tuple(_IMU_UNITS),
        units=_IMU_UNITS,
    )


def _model_limit(joint: JointDescription, interface: Interface) -> Limits | None:
    """The robot model's limit on one thing a joint is told, or ``None``.

    A missing bound is never read as "no limit", which would let the joint be
    driven anywhere. The one exception is the position of a joint that spins
    freely. Stiffness and damping have no limit in a URDF.
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
            return None
        case Interface.VELOCITY:
            bound = joint.velocity
        case Interface.EFFORT:
            bound = joint.effort
        case _:
            return None
    if bound is None:
        raise ValueError(f"joint {joint.name!r} has no {interface} limit in its model")
    return Limits(-bound, bound)


def _joints(
    source: str,
    joints: Sequence[str],
    state: Sequence[Interface],
    command: Sequence[Interface],
    model: RobotModel | None,
    limits: Mapping[str, Limits] | None,
) -> tuple[list[Resource], dict[str, Limits]]:
    """Build each joint and the limits on what it is told.

    With a model, each joint's units and limits come from it. Limits passed by
    hand are applied last, so they replace the model's for the same key.
    """
    loaded = model.load() if model is not None else None
    resources: list[Resource] = []
    found: dict[str, Limits] = {}
    for name in joints:
        kind_units = _REVOLUTE
        if loaded is not None:
            joint = loaded.get_joint(name)
            if joint is None:
                raise ValueError(f"joint {name!r} is not in the robot model")
            match joint.type:
                case "revolute" | "continuous":
                    kind_units = _REVOLUTE
                case "prismatic":
                    kind_units = _PRISMATIC
                case _:
                    # fixed, floating and planar joints
                    raise ValueError(
                        f"joint {name!r} is {joint.type} in the robot model, so it cannot be driven"
                    )
            for interface in command:
                limit = _model_limit(joint, interface)
                if limit is not None:
                    found[Key.of(source, name, interface)] = limit

        units: dict[str, Unit] = {}
        for interface in dict.fromkeys((*state, *command)):
            if interface not in kind_units:
                raise ValueError(
                    f"interface '{interface}' has no preset unit: build the Resource by hand "
                    f"or use one of {', '.join(kind_units)}"
                )
            units[interface] = kind_units[interface]
        resources.append(
            Resource(
                name=name,
                kind=ResourceKind.JOINT,
                state_interfaces=tuple(state),
                command_interfaces=tuple(command),
                units=units,
            )
        )
    return resources, found | dict(limits or {})


def manipulator_description(
    source: str,
    joints: Sequence[str],
    *,
    model: RobotModel | None = None,
    limits: Mapping[str, Limits] | None = None,
    state: Sequence[Interface] = (Interface.POSITION, Interface.VELOCITY, Interface.EFFORT),
    command: Sequence[Interface] = (Interface.POSITION, Interface.VELOCITY),
    gripper: GripperSpec | None = None,
    sensors: Sequence[Resource] = (),
    state_rate_hz: float = 100.0,
    deadman_timeout_s: float = 0.1,
) -> ControlDescription:
    """Describe an arm, optionally with a gripper on the end.

    A command past a limit is refused.

    Args:
        source: Short name for this piece of hardware. Every name it reports
            starts with it, e.g. "arm".
        joints: The arm's joint names, in the order the hardware lists them.
            With a model, each must be a joint in it.
        model: The arm's robot model. Each joint's type (revolute or prismatic,
            which sets its units) and its limits on everything it is told
            come from here. ``None`` to treat every joint as revolute, limited
            only by ``limits``.
        limits: Limits by full name, such as "arm/joint1/position". With a
            model, each one replaces the model's, e.g. to drive slower than
            the hardware allows.
        state: What each joint reports. Leave out anything it cannot measure,
            rather than having it report zeros that look like real readings.
        command: What each joint can be told: position, velocity, or both.
        gripper: A gripper to add, or ``None`` for none.
        sensors: Sensors to report alongside the joints, e.g.
            ``[imu_resource()]``.
        state_rate_hz: How many times a second the arm reports.
        deadman_timeout_s: How long it goes without a command, in seconds,
            before it stops itself.

    Returns:
        A complete, checked description of the arm.

    Raises:
        ValueError: If there are no joints, nothing to command, an interface
            whose unit this function does not know, or a model that lacks a
            joint or is unclear about one of its limits.
        DescriptionError: If the finished description breaks a rule.
    """
    if not joints:
        raise ValueError(f"manipulator {source!r} declares no joints")
    if not command:
        raise ValueError(f"manipulator {source!r} declares nothing to command")

    resources, all_limits = _joints(source, joints, state, command, model, limits)
    if gripper is not None:
        resources.append(
            Resource(
                name=gripper.name,
                kind=ResourceKind.JOINT,
                state_interfaces=(Interface.POSITION,),
                command_interfaces=(Interface.POSITION,),
                units={Interface.POSITION: gripper.unit},
            )
        )
        all_limits[Key.of(source, gripper.name, Interface.POSITION)] = Limits(
            gripper.lo, gripper.hi
        )

    description = ControlDescription(
        source=source,
        resources=(*resources, *sensors),
        limits=all_limits,
        state_rate_hz=state_rate_hz,
        deadman_timeout_s=deadman_timeout_s,
    )
    validate_description(description)
    return description


def pd_joint_description(
    source: str,
    joints: Sequence[str],
    *,
    model: RobotModel | None = None,
    limits: Mapping[str, Limits] | None = None,
    sensors: Sequence[Resource] = (),
    state_rate_hz: float = 500.0,
    deadman_timeout_s: float = 0.05,
) -> ControlDescription:
    """Describe a body whose joints are held by stiffness and damping.

    Used for humanoids and similar robots. Each joint is told a target
    position, speed and force, and two numbers that say how hard to pull
    towards them: ``kp`` (stiffness) and ``kd`` (damping). All five are sent
    in every command. A command past a limit is refused.

    Args:
        source: Short name for this piece of hardware, e.g. "g1".
        joints: Joint names, in the order the hardware lists them. The order
            is how commands line up with the right motors. With a model, each
            must be a joint in it.
        model: The body's robot model. Each joint's type (revolute or prismatic,
            which sets its units) and its position, speed and force limits
            come from here. ``None`` to treat every joint as revolute, limited
            only by ``limits``.
        limits: Limits by full name, such as "g1/left_knee/position". With a
            model, each one replaces the model's.
        sensors: Sensors to report alongside the joints, e.g.
            ``[imu_resource()]``.
        state_rate_hz: How many times a second the body reports.
        deadman_timeout_s: How long it goes without a command, in seconds,
            before it stops itself.

    Returns:
        A complete, checked description of the body.

    Raises:
        ValueError: If there are no joints, or a model that lacks a joint or
            is unclear about one of its limits.
        DescriptionError: If the finished description breaks a rule.
    """
    if not joints:
        raise ValueError(f"PD body {source!r} declares no joints")

    resources, all_limits = _joints(source, joints, _PD_STATE, _PD_COMMAND, model, limits)
    description = ControlDescription(
        source=source,
        resources=(*resources, *sensors),
        limits=all_limits,
        state_rate_hz=state_rate_hz,
        deadman_timeout_s=deadman_timeout_s,
    )
    validate_description(description)
    return description


def twist_base_description(
    source: str,
    *,
    limits: Mapping[str, Limits],
    odometry: bool = True,
    measured_velocity: bool = True,
    sensors: Sequence[Resource] = (),
    state_rate_hz: float = 50.0,
    deadman_timeout_s: float = 0.2,
) -> ControlDescription:
    """Describe something that drives around on the floor.

    The base is one part called "base", not a set of wheels. It is told how
    fast to drive forwards (``vx``, m/s), sideways (``vy``, m/s) and turn
    (``wz``, rad/s), all measured from the base itself: "forwards" means
    forwards whichever way it happens to be facing.

    A speed past its limit is pulled back to the limit and sent, rather than
    refused, so a teleop stick pushed too far still drives the base at its
    top speed. A base is the only hardware that does this.

    Args:
        source: Short name for this piece of hardware, e.g. "chassis".
        limits: The slowest and fastest it may be told to go on each axis, by
            full name such as "chassis/base/vx". Each needs both a lower and
            an upper bound. A base that cannot move sideways gives ``vy`` a
            limit of zero both ways.
        odometry: Whether it reports where it has got to: ``x`` and ``y`` in
            metres and ``yaw`` in radians.
        measured_velocity: Whether it can measure how fast it is going. False
            for one that can only repeat back what it was told, which is not
            a measurement.
        sensors: Sensors to report alongside the base, e.g.
            ``[imu_resource()]``.
        state_rate_hz: How many times a second the base reports.
        deadman_timeout_s: How long it goes without a command, in seconds,
            before it stops itself.

    Returns:
        A complete, checked description of the base.

    Raises:
        DescriptionError: If the finished description breaks a rule, such as
            a limit missing one of its bounds.
    """
    pose = _BASE_POSE if odometry else {}
    description = ControlDescription(
        source=source,
        resources=(
            Resource(
                name="base",
                kind=ResourceKind.BASE,
                state_interfaces=(*(_BASE_TWIST if measured_velocity else ()), *pose),
                command_interfaces=tuple(_BASE_TWIST),
                units={**_BASE_TWIST, **pose},
            ),
            *sensors,
        ),
        limits={key: replace(limit, clamp=True) for key, limit in limits.items()},
        state_rate_hz=state_rate_hz,
        deadman_timeout_s=deadman_timeout_s,
    )
    validate_description(description)
    return description
