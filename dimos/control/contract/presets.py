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

    manipulator_description("arm", JOINTS, limits=limits_from_urdf(URDF, ...))

  ``manipulator_description``  an arm, optionally with a gripper
  ``pd_joint_description``     a body whose joints are held in place by
                               stiffness and damping, such as a humanoid
  ``twist_base_description``   something that drives around on the floor

Each one checks its own result before handing it back, so a description built
this way is always a valid one.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, replace

from dimos.control.contract.description import (
    ControlDescription,
    Limits,
    Resource,
    ResourceKind,
)
from dimos.control.contract.keys import (
    AX,
    AY,
    AZ,
    EFFORT,
    GX,
    GY,
    GZ,
    KD,
    KP,
    POSITION,
    QW,
    QX,
    QY,
    QZ,
    VELOCITY,
    VX,
    VY,
    WZ,
    YAW,
    Key,
    Unit,
    X,
    Y,
)
from dimos.control.contract.validate import validate_description

#: The unit each joint interface is measured in. A gripper is the exception:
#: it sets its own, because some are measured in metres and some in a 0-to-1
#: fraction of fully open.
_JOINT_UNITS: Mapping[str, Unit] = {
    POSITION: Unit.RAD,
    VELOCITY: Unit.RAD_PER_S,
    EFFORT: Unit.NM,
    KP: Unit.UNITLESS,
    KD: Unit.UNITLESS,
}

_PD_STATE: tuple[str, ...] = (POSITION, VELOCITY, EFFORT)
_PD_COMMAND: tuple[str, ...] = (POSITION, VELOCITY, EFFORT, KP, KD)

_IMU_UNITS: Mapping[str, Unit] = {
    QX: Unit.UNITLESS,
    QY: Unit.UNITLESS,
    QZ: Unit.UNITLESS,
    QW: Unit.UNITLESS,
    GX: Unit.RAD_PER_S,
    GY: Unit.RAD_PER_S,
    GZ: Unit.RAD_PER_S,
    AX: Unit.M_PER_S2,
    AY: Unit.M_PER_S2,
    AZ: Unit.M_PER_S2,
}

#: What a base is told: how fast to drive forwards, sideways, and turn.
_BASE_TWIST: Mapping[str, Unit] = {VX: Unit.M_PER_S, VY: Unit.M_PER_S, WZ: Unit.RAD_PER_S}
#: Where a base reports itself as being, and which way it is facing.
_BASE_POSE: Mapping[str, Unit] = {X: Unit.M, Y: Unit.M, YAW: Unit.RAD}


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


def _joint_units(interfaces: Iterable[str]) -> dict[str, Unit]:
    """Look up the unit of each joint interface, refusing unfamiliar ones."""
    units: dict[str, Unit] = {}
    for iface in interfaces:
        unit = _JOINT_UNITS.get(iface)
        if unit is None:
            raise ValueError(
                f"interface {iface!r} has no preset unit: build the Resource by hand "
                f"or use one of {sorted(_JOINT_UNITS)}"
            )
        units[iface] = unit
    return units


def _limits_for(
    source: str, joints: Sequence[str], command: Sequence[str], limits: Mapping[str, Limits]
) -> dict[str, Limits]:
    """Keep the limits that apply to what these joints can be told to do.

    A robot model gives position, speed and force limits for every joint, but
    a limit is only meaningful on something the joint accepts as a command. A
    limit for one of these joints on a joint interface it does not accept is
    left out. Anything else, such as a limit naming a joint that does not
    exist, is kept so the final check reports it.
    """
    kept: dict[str, Limits] = {}
    for key, limit in limits.items():
        name = Key(key)
        unused = (
            name.source == source
            and name.resource in joints
            and name.interface in _JOINT_UNITS
            and name.interface not in command
        )
        if not unused:
            kept[key] = limit
    return kept


def manipulator_description(
    source: str,
    joints: Sequence[str],
    *,
    limits: Mapping[str, Limits],
    state: Sequence[str] = (POSITION, VELOCITY, EFFORT),
    command: Sequence[str] = (POSITION, VELOCITY),
    gripper: GripperSpec | None = None,
    state_rate_hz: float = 100.0,
    deadman_timeout_s: float = 0.1,
) -> ControlDescription:
    """Describe an arm, optionally with a gripper on the end.

    A command past a limit is refused.

    Args:
        source: Short name for this piece of hardware. Every name it reports
            starts with it, e.g. "arm".
        joints: The arm's joint names, in the order the hardware lists them.
        limits: How far each joint may go, by full name such as
            "arm/joint1/position". Usually from ``limits_from_urdf``. Limits
            on something these joints do not accept, such as effort for an
            arm driven by position, are ignored.
        state: What each joint reports. Leave out anything it cannot measure,
            rather than having it report zeros that look like real readings.
        command: What each joint can be told: position in radians, velocity
            in rad/s, or both.
        gripper: A gripper to add, or ``None`` for none.
        state_rate_hz: How many times a second the arm reports.
        deadman_timeout_s: How long it goes without a command, in seconds,
            before it stops itself.

    Returns:
        A complete, checked description of the arm.

    Raises:
        ValueError: If there are no joints, nothing to command, or an
            interface whose unit this function does not know.
        DescriptionError: If the finished description breaks a rule.
    """
    if not joints:
        raise ValueError(f"manipulator {source!r} declares no joints")
    if not command:
        raise ValueError(f"manipulator {source!r} declares nothing to command")

    units = _joint_units(dict.fromkeys((*state, *command)))
    resources = [
        Resource(
            name=joint,
            kind=ResourceKind.JOINT,
            state_interfaces=tuple(state),
            command_interfaces=tuple(command),
            units=units,
        )
        for joint in joints
    ]
    all_limits = _limits_for(source, joints, command, limits)
    if gripper is not None:
        resources.append(
            Resource(
                name=gripper.name,
                kind=ResourceKind.JOINT,
                state_interfaces=(POSITION,),
                command_interfaces=(POSITION,),
                units={POSITION: gripper.unit},
            )
        )
        all_limits[Key.of(source, gripper.name, POSITION)] = Limits(gripper.lo, gripper.hi)

    description = ControlDescription(
        source=source,
        resources=tuple(resources),
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
    limits: Mapping[str, Limits],
    imu: str | None = None,
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
            is how commands line up with the right motors.
        limits: How far each joint may go, by full name such as
            "g1/left_knee/position". Usually from ``limits_from_urdf``.
        imu: The name of an orientation sensor to report alongside the
            joints, e.g. "imu", or ``None`` for none. It only reports: which
            way up the body is, how fast it is turning, and how hard it is
            accelerating.
        state_rate_hz: How many times a second the body reports.
        deadman_timeout_s: How long it goes without a command, in seconds,
            before it stops itself.

    Returns:
        A complete, checked description of the body.

    Raises:
        ValueError: If there are no joints.
        DescriptionError: If the finished description breaks a rule.
    """
    if not joints:
        raise ValueError(f"PD body {source!r} declares no joints")

    units = _joint_units(_PD_COMMAND)
    resources = [
        Resource(
            name=joint,
            kind=ResourceKind.JOINT,
            state_interfaces=_PD_STATE,
            command_interfaces=_PD_COMMAND,
            units=units,
        )
        for joint in joints
    ]
    if imu is not None:
        resources.append(
            Resource(
                name=imu,
                kind=ResourceKind.SENSOR,
                state_interfaces=tuple(_IMU_UNITS),
                units=_IMU_UNITS,
            )
        )

    description = ControlDescription(
        source=source,
        resources=tuple(resources),
        limits=_limits_for(source, joints, _PD_COMMAND, limits),
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
        ),
        limits={key: replace(limit, clamp=True) for key, limit in limits.items()},
        state_rate_hz=state_rate_hz,
        deadman_timeout_s=deadman_timeout_s,
    )
    validate_description(description)
    return description
