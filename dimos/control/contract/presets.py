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

Every robot driver has to describe itself before anything will talk to it:
which parts it has, what each one can be told to do, in what units, within
what limits, and how it comes to a stop. Writing all that out by hand is long
and easy to get subtly wrong, so these three functions build it for you.

  ``manipulator_description``  an arm, optionally with a gripper
  ``pd_joint_description``     a body whose joints are held in place by
                               stiffness and damping, such as a humanoid
  ``twist_base_description``   something that drives around on the floor

Each one checks its own result before handing it back, so a description built
this way is always a valid one. Anything they do not cover by default can be
passed in as a keyword argument.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, replace

from dimos.control.contract.description import (
    ActivationPolicy,
    ControlDescription,
    Estop,
    EstopKind,
    EstopRecovery,
    LimitPolicy,
    Limits,
    ModeGroup,
    Omission,
    ProcessLoss,
    Resource,
    ResourceKind,
    SafeStop,
    SafeStopKind,
    Timing,
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
    PITCH,
    POSITION,
    QW,
    QX,
    QY,
    QZ,
    ROLL,
    VELOCITY,
    VX,
    VY,
    VZ,
    WX,
    WY,
    WZ,
    YAW,
    Unit,
    X,
    Y,
    Z,
    make_key,
)
from dimos.control.contract.validate import validate_description

#: The order joint interfaces are listed in, chosen to read the way people
#: describe a joint rather than alphabetically.
_JOINT_INTERFACE_ORDER: tuple[str, ...] = (POSITION, VELOCITY, EFFORT, KP, KD)

#: The unit each joint interface is measured in. A gripper is the exception:
#: it sets its own, because some are measured in metres and some in a 0-to-1
#: fraction of fully closed.
_JOINT_UNITS: Mapping[str, Unit] = {
    POSITION: Unit.RAD,
    VELOCITY: Unit.RAD_PER_S,
    EFFORT: Unit.NM,
    KP: Unit.UNITLESS,
    KD: Unit.UNITLESS,
}

_PD_STATE: tuple[str, ...] = (POSITION, VELOCITY, EFFORT)
_PD_COMMAND: tuple[str, ...] = (POSITION, VELOCITY, EFFORT, KP, KD)

_IMU_INTERFACES: tuple[str, ...] = (QX, QY, QZ, QW, GX, GY, GZ, AX, AY, AZ)
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

#: How fast a base may be told to move along or turn about each axis. A base
#: on the floor uses a few of these; something that flies uses all six.
_TWIST_UNITS: Mapping[str, Unit] = {
    VX: Unit.M_PER_S,
    VY: Unit.M_PER_S,
    VZ: Unit.M_PER_S,
    WX: Unit.RAD_PER_S,
    WY: Unit.RAD_PER_S,
    WZ: Unit.RAD_PER_S,
}

#: Where a base can report itself as being: its position, and which way it is
#: facing.
_POSE_UNITS: Mapping[str, Unit] = {
    X: Unit.M,
    Y: Unit.M,
    Z: Unit.M,
    ROLL: Unit.RAD,
    PITCH: Unit.RAD,
    YAW: Unit.RAD,
}

#: Moving along an axis changes the matching position; turning about an axis
#: changes the matching facing.
_POSITION_OF: Mapping[str, str] = {VX: X, VY: Y, VZ: Z}
_ORIENTATION_OF: Mapping[str, str] = {WX: ROLL, WY: PITCH, WZ: YAW}

#: Turning about an axis swaps the two directions at right angles to it. Turn
#: left, and what used to be "forwards" now points where "left" did.
_ROTATION_PLANE: Mapping[str, tuple[str, str]] = {
    WX: (VY, VZ),
    WY: (VZ, VX),
    WZ: (VX, VY),
}


def _pose_interfaces(axes: Sequence[str]) -> tuple[str, ...]:
    """Work out where a base can get to, given the ways it can move.

    Not simply one answer per axis. A base that can only drive forwards and
    turn cannot move sideways, but by turning first it can still reach any
    point on the floor, so its position has to be reported in both directions
    and not just one.

    The rule: start with the directions it can move in, then add any direction
    it can turn to face. Two independent ways of turning let it face any
    direction at all.

    Args:
        axes: The ways this base can be told to move.

    Returns:
        The parts of its position and facing it can report, in order.
    """
    linear = {axis for axis in axes if axis in _POSITION_OF}
    angular = {axis for axis in axes if axis in _ORIENTATION_OF}

    # Close the linear set under the available rotations, to a fixpoint: a
    # rotation reached through one plane can open another.
    changed = True
    while changed:
        changed = False
        for rotation in angular:
            plane = set(_ROTATION_PLANE[rotation])
            if (linear & plane) and not plane <= linear:
                linear |= plane
                changed = True

    # Any two independent rotation generators compose to reach every
    # orientation, so a base with two of them has all three pose terms.
    if len(angular) >= 2:
        orientation = set(_ORIENTATION_OF.values())
    else:
        orientation = {_ORIENTATION_OF[axis] for axis in angular}

    reached = {_POSITION_OF[axis] for axis in linear} | orientation
    return tuple(term for term in _POSE_UNITS if term in reached)


@dataclass(frozen=True, slots=True, kw_only=True)
class GripperSpec:
    """A gripper to add to an arm.

    Grippers do not agree on how to measure how open they are: some report
    metres between the fingers, some a fraction from 0 (shut) to 1 (open). So
    each one states its own unit and range here, rather than everything being
    forced into the same one.

    Attributes:
        name: What to call it, e.g. "gripper".
        unit: How its opening is measured.
        lo: Fully closed, in that unit.
        hi: Fully open, in that unit.
        policy: What to do with a command outside that range. REJECT turns it
            away; CLAMP pulls it back to the nearest end.
    """

    name: str = "gripper"
    unit: Unit
    lo: float
    hi: float
    policy: LimitPolicy = LimitPolicy.REJECT


@dataclass(frozen=True, slots=True, kw_only=True)
class ImuSpec:
    """An orientation sensor to add to a body.

    Reports which way the body is tilted, how fast it is rotating, and how
    hard it is accelerating. It only ever reports; nothing is sent to it.

    Attributes:
        name: What to call it, e.g. "imu".
        frame_id: Which part of the robot it is bolted to, so its readings can
            be related to everything else.
    """

    name: str = "imu"
    frame_id: str


def _ordered_interfaces(interfaces: Iterable[str]) -> tuple[str, ...]:
    """Put interfaces into a consistent order, so two equal sets read alike."""
    present = set(interfaces)
    known = tuple(i for i in _JOINT_INTERFACE_ORDER if i in present)
    return known + tuple(sorted(present - set(_JOINT_INTERFACE_ORDER)))


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


def _group_name(interfaces: Iterable[str]) -> str:
    """Name a group of interfaces after the things it drives.

    ``{position}`` becomes ``"position"``, and ``{position, velocity}``
    becomes ``"position+velocity"``. Building the name this way means two
    groups driving the same things end up with the same name, which is then
    caught as the duplicate it is.
    """
    return "+".join(sorted(interfaces))


def _broadcast(
    value: Mapping[str, float] | float, joints: Sequence[str], label: str
) -> dict[str, float]:
    """Spread one value across every joint, or check a per-joint table.

    Pass a single number to use it for all of them, or a table giving each
    joint its own. A table that misses a joint is an error rather than a
    default: the joint left out would be the one that goes slack.
    """
    if not isinstance(value, Mapping):
        return {joint: float(value) for joint in joints}
    missing = [joint for joint in joints if joint not in value]
    if missing:
        raise ValueError(f"{label} does not cover joint(s) {missing}")
    extra = sorted(set(value) - set(joints))
    if extra:
        raise ValueError(f"{label} names undeclared joint(s) {extra}")
    return {joint: float(value[joint]) for joint in joints}


def manipulator_description(
    source: str,
    joints: Sequence[str],
    *,
    limits: Mapping[str, Limits],
    state: Sequence[str] = (POSITION, VELOCITY, EFFORT),
    groups: Sequence[frozenset[str]] = (frozenset({POSITION}),),
    gripper: GripperSpec | None = None,
    safe_stop: SafeStop = SafeStop(kind=SafeStopKind.HOLD),
    estop: Estop = Estop(kind=EstopKind.HOLD, recovery=EstopRecovery.CLEAR),
    activation: ActivationPolicy = ActivationPolicy.DIRECT,
    timing: Timing = Timing(state_rate_hz=100.0, stale_timeout_s=0.05, watchdog_timeout_s=0.1),
    process_loss: ProcessLoss | Mapping[str, ProcessLoss] = ProcessLoss.UNKNOWN,
    omission: Mapping[str, Omission] | None = None,
    meta: Mapping[str, str] | None = None,
) -> ControlDescription:
    """Describe an arm, optionally with a gripper on the end.

    An arm can usually be driven in more than one way -- told where to go, or
    told how fast to move -- but only one way at a time, or the instructions
    would contradict each other. ``groups`` lists the ways this arm accepts,
    and only one of them can be in use at any moment.

    A gripper is treated as one more joint, except that it can be worked at
    the same time as the arm rather than instead of it, and it uses its own
    unit.

    Args:
        source: Short name for this piece of hardware. Every name it reports
            starts with it, e.g. "arm".
        joints: The arm's joint names, in the order the hardware lists them.
        limits: How far each joint may go, usually from ``limits_from_urdf``.
            The gripper's own limits are added from ``gripper``.
        state: What the arm can actually tell you about itself. Leave out
            anything it cannot measure, rather than having it report zeros
            that look like real readings.
        groups: The ways this arm can be driven, one set of interfaces each.
        gripper: A gripper to add, or ``None`` for none.
        safe_stop: How it comes to a stop when it is told to give up control.
            By default it stays where it is.
        estop: What it does when stopped in an emergency.
        activation: Whether a person has to confirm before it will move.
        timing: How often it reports, and how long to wait before deciding it
            has gone quiet.
        process_loss: What the hardware does if the program driving it dies.
            Leave as UNKNOWN until someone has actually tested it.
        meta: Any extra notes to carry along, as text.

    Returns:
        A complete, checked description of the arm.

    Raises:
        ValueError: If there are no joints, no groups, or an interface whose
            unit this function does not know.
        DescriptionError: If the finished description breaks a rule.
    """
    if not joints:
        raise ValueError(f"manipulator {source!r} declares no joints")
    if not groups:
        raise ValueError(f"manipulator {source!r} declares no mode groups")

    command = _ordered_interfaces(iface for group in groups for iface in group)
    state_interfaces = tuple(state)
    joint_units = _joint_units(dict.fromkeys(state_interfaces + command))
    resources = [
        Resource(
            name=joint,
            kind=ResourceKind.JOINT,
            state_interfaces=state_interfaces,
            command_interfaces=command,
            units=dict(joint_units),
        )
        for joint in joints
    ]
    mode_groups = [
        ModeGroup(
            name=_group_name(group),
            resources=tuple(joints),
            interfaces=frozenset(group),
            exclusive=True,
        )
        for group in groups
    ]

    all_limits = dict(limits)
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
        mode_groups.append(
            ModeGroup(
                name="gripper",
                resources=(gripper.name,),
                interfaces=frozenset({POSITION}),
                exclusive=False,
            )
        )
        all_limits[make_key(source, gripper.name, POSITION)] = Limits(
            gripper.lo, gripper.hi, gripper.policy
        )

    description = ControlDescription(
        source=source,
        resources=tuple(resources),
        limits=all_limits,
        mode_groups=tuple(mode_groups),
        omission=dict(omission or {}),
        safe_stop=safe_stop,
        estop=estop,
        activation_policy=activation,
        timing=timing,
        process_loss=process_loss,
        meta=dict(meta or {}),
    )
    validate_description(description)
    return description


def pd_joint_description(
    source: str,
    joints: Sequence[str],
    *,
    limits: Mapping[str, Limits],
    kp: Mapping[str, float] | float,
    kd: Mapping[str, float] | float,
    damp_kd: Mapping[str, float] | float,
    imu: ImuSpec | None = None,
    safe_stop: SafeStop | None = None,
    estop: Estop = Estop(kind=EstopKind.DISABLE, recovery=EstopRecovery.PREPARE_ARM_REQUIRED),
    activation: ActivationPolicy = ActivationPolicy.OPERATOR_CONFIRMED,
    timing: Timing = Timing(state_rate_hz=500.0, stale_timeout_s=0.02, watchdog_timeout_s=0.05),
    process_loss: ProcessLoss | Mapping[str, ProcessLoss] = ProcessLoss.UNPROTECTED,
    omission: Mapping[str, Omission] | None = None,
    meta: Mapping[str, str] | None = None,
) -> ControlDescription:
    """Describe a body whose joints are held by stiffness and damping.

    Used for humanoids and similar robots. Rather than being commanded to a
    position and getting there however it likes, each joint is given a target
    and two numbers that say how hard to pull towards it:

      kp  stiffness -- how strongly it pulls towards the target
      kd  damping   -- how strongly it resists moving at the wrong speed

    Those two are stored with the description rather than being sent with
    every command, so something that only knows where it wants the joints to
    go does not have to invent them.

    Args:
        source: Short name for this piece of hardware, e.g. "g1".
        joints: Joint names, in the order the hardware lists them. This order
            matters -- it is how commands line up with the right motors.
        limits: How far each joint may go, usually from ``limits_from_urdf``.
        kp: Stiffness. One number for every joint, or a table naming each.
        kd: Damping, same form.
        damp_kd: The damping used when the robot is told to stop and go limp,
            normally higher than ``kd``. Same form.
        imu: An orientation sensor to report alongside the joints, or ``None``.
        safe_stop: How it comes to a stop. By default it goes limp using
            ``damp_kd`` and sinks under its own weight.
        estop: What it does when stopped in an emergency. By default the
            motors are switched off, which is not recoverable without starting
            the robot up again.
        activation: Whether a person has to confirm before it will move.
        timing: How often it reports, and how long before it counts as quiet.
        process_loss: What the hardware does if the program driving it dies.
        omission: What it means when a command leaves something out. Some
            hardware reads a commanded speed of zero as "no instruction"
            rather than "hold still", and this is where that is declared.
        meta: Any extra notes to carry along, as text.

    Returns:
        A complete, checked description of the body.

    Raises:
        ValueError: If there are no joints, or a stiffness or damping table
            misses one.
        DescriptionError: If the finished description breaks a rule.
    """
    if not joints:
        raise ValueError(f"PD body {source!r} declares no joints")

    kp_table = _broadcast(kp, joints, "kp")
    kd_table = _broadcast(kd, joints, "kd")
    damp_table = _broadcast(damp_kd, joints, "damp_kd")

    units = _joint_units(_PD_COMMAND)
    resources = [
        Resource(
            name=joint,
            kind=ResourceKind.JOINT,
            state_interfaces=_PD_STATE,
            command_interfaces=_PD_COMMAND,
            units=dict(units),
        )
        for joint in joints
    ]

    all_meta = dict(meta or {})
    if imu is not None:
        resources.append(
            Resource(
                name=imu.name,
                kind=ResourceKind.SENSOR,
                state_interfaces=_IMU_INTERFACES,
                command_interfaces=(),
                units=dict(_IMU_UNITS),
            )
        )
        all_meta["imu_frame_id"] = imu.frame_id

    damping = {make_key(source, joint, KD): damp_table[joint] for joint in joints}
    if safe_stop is None:
        safe_stop = SafeStop(kind=SafeStopKind.DAMP, kd=damping)
    elif safe_stop.kind is SafeStopKind.DAMP and not safe_stop.kd:
        safe_stop = replace(safe_stop, kd=damping)

    description = ControlDescription(
        source=source,
        resources=tuple(resources),
        limits=dict(limits),
        mode_groups=(
            ModeGroup(
                name="pd",
                resources=tuple(joints),
                interfaces=frozenset(_PD_COMMAND),
                exclusive=True,
            ),
        ),
        omission=dict(omission or {}),
        initial_values={make_key(source, joint, KP): kp_table[joint] for joint in joints}
        | {make_key(source, joint, KD): kd_table[joint] for joint in joints},
        safe_stop=safe_stop,
        estop=estop,
        activation_policy=activation,
        timing=timing,
        process_loss=process_loss,
        meta=all_meta,
    )
    validate_description(description)
    return description


def twist_base_description(
    source: str,
    *,
    axes: Sequence[str] = (VX, VY, WZ),
    limits: Mapping[str, Limits],
    odometry: bool = True,
    measured_velocity: bool = True,
    safe_stop: SafeStop = SafeStop(kind=SafeStopKind.ZERO),
    estop: Estop = Estop(kind=EstopKind.ZERO, recovery=EstopRecovery.CLEAR),
    activation: ActivationPolicy = ActivationPolicy.DIRECT,
    timing: Timing = Timing(state_rate_hz=50.0, stale_timeout_s=0.2, watchdog_timeout_s=0.2),
    process_loss: ProcessLoss | Mapping[str, ProcessLoss] = ProcessLoss.UNKNOWN,
    meta: Mapping[str, str] | None = None,
) -> ControlDescription:
    """Describe something that drives around on the floor.

    The base is one thing, not a set of wheels or legs: it is told how fast to
    move and how fast to turn, and whatever is underneath works out the rest.

    It is commanded in its own terms -- forwards, sideways, turning -- rather
    than in terms of the room it is in. "Forwards" means forwards whichever
    way it happens to be facing.

    If nothing is steering it, it is told to stop rather than left to carry on
    at whatever it was last doing. Anything that wants it to keep moving has
    to keep saying so.

    Args:
        source: Short name for this piece of hardware, e.g. "chassis".
        axes: The ways it can be told to move. ``vx`` forwards, ``vy``
            sideways, ``wz`` turning, and ``vz``/``wx``/``wy`` for something
            that also flies. A base that cannot move sideways leaves out
            ``vy``.
        limits: The fastest it may be told to go along each axis.
        odometry: Whether it keeps track of where it has got to. What it can
            report follows from ``axes``: a base that can turn can reach
            anywhere on the floor, not just the line it drives along.
        measured_velocity: Whether it can actually measure how fast it is
            going. False for one that can only repeat back what it was told,
            which is not a measurement.
        safe_stop: How it comes to a stop. By default it is told to stop
            moving.
        estop: What it does when stopped in an emergency.
        activation: Whether a person has to confirm before it will move.
        timing: How often it reports, and how long before it counts as quiet.
        process_loss: What the hardware does if the program driving it dies.
        meta: Any extra notes to carry along, as text.

    Returns:
        A complete, checked description of the base.

    Raises:
        ValueError: If there are no axes, an axis is repeated, or an axis is
            not one of the six.
        DescriptionError: If the finished description breaks a rule.
    """
    if not axes:
        raise ValueError(f"base {source!r} declares no axes")
    unknown = [axis for axis in axes if axis not in _TWIST_UNITS]
    if unknown:
        raise ValueError(
            f"base {source!r} declares non-twist axes {unknown}: expected {sorted(_TWIST_UNITS)}"
        )
    repeated = sorted({axis for axis in axes if list(axes).count(axis) > 1})
    if repeated:
        raise ValueError(f"base {source!r} repeats axes {repeated}")

    units = {axis: _TWIST_UNITS[axis] for axis in axes}
    state_interfaces: tuple[str, ...] = tuple(axes) if measured_velocity else ()
    if odometry:
        pose = _pose_interfaces(axes)
        state_interfaces += pose
        units |= {term: _POSE_UNITS[term] for term in pose}

    description = ControlDescription(
        source=source,
        resources=(
            Resource(
                name="base",
                kind=ResourceKind.BASE,
                state_interfaces=state_interfaces,
                command_interfaces=tuple(axes),
                units=units,
            ),
        ),
        limits=dict(limits),
        mode_groups=(
            ModeGroup(
                name="twist",
                resources=("base",),
                interfaces=frozenset(axes),
                exclusive=True,
            ),
        ),
        safe_stop=safe_stop,
        estop=estop,
        activation_policy=activation,
        timing=timing,
        process_loss=process_loss,
        meta=dict(meta or {}),
    )
    validate_description(description)
    return description
