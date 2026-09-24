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

"""Descriptions for the three shapes hardware actually comes in.

A vendor's ``describe()`` hook is meant to be one call plus whatever its SDK
only learns at connect time. These functions are that call. They are sugar and
nothing else: every description they build could be written out by hand, and
the contract tests prove it by rebuilding the hand-written fixtures through
them.

  ``manipulator_description``  an arm: joints driven one mode group at a time,
                               optionally a gripper in a group of its own
  ``pd_joint_description``     a PD-controlled body: one impedance group over
                               every joint, gains seeded from the description
  ``twist_base_description``   a mobile base: one resource commanded as a body
                               twist, pose and velocity reported back

Each one validates before returning, so a preset can never hand out a
description the coordinator would reject later. Anything a preset does not
cover is a keyword away, and anything a keyword does not cover is a reason to
build the ``ControlDescription`` by hand rather than to grow a fourth preset.
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

#: Declaration order for joint interfaces. Alphabetical would read
#: ``effort, kd, kp, position, velocity``, which is nobody's mental model of a
#: joint; this is the order the fixtures and every vendor table already use.
_JOINT_INTERFACE_ORDER: tuple[str, ...] = (POSITION, VELOCITY, EFFORT, KP, KD)

#: The unit every preset joint interface is carried in. A gripper is the one
#: joint that escapes this table, which is exactly what makes it a gripper (D19).
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

#: The six-DOF base vocabulary: each commandable twist axis, the pose term it
#: integrates into, and the unit of each. A ground base declares the planar
#: subset and a free-flyer declares all six; neither is a different kind of
#: thing, which is why there is one table and not two presets.
_BASE_AXES: Mapping[str, tuple[str, Unit, Unit]] = {
    VX: (X, Unit.M_PER_S, Unit.M),
    VY: (Y, Unit.M_PER_S, Unit.M),
    VZ: (Z, Unit.M_PER_S, Unit.M),
    WX: (ROLL, Unit.RAD_PER_S, Unit.RAD),
    WY: (PITCH, Unit.RAD_PER_S, Unit.RAD),
    WZ: (YAW, Unit.RAD_PER_S, Unit.RAD),
}


@dataclass(frozen=True, slots=True, kw_only=True)
class GripperSpec:
    """A gripper to hang off a manipulator, in the unit its vendor speaks.

    Metres for an xArm, normalized for a Damiao. Forcing one on both breaks the
    other, so the unit is declared here and the gripper task reads it back off
    the description (D19).
    """

    name: str = "gripper"
    unit: Unit
    lo: float
    hi: float
    policy: LimitPolicy = LimitPolicy.REJECT


@dataclass(frozen=True, slots=True, kw_only=True)
class ImuSpec:
    """An IMU to hang off a PD body. State-only; never commanded."""

    name: str = "imu"
    frame_id: str


def _ordered_interfaces(interfaces: Iterable[str]) -> tuple[str, ...]:
    """The given interfaces in canonical order, unknown ones sorted at the end."""
    present = set(interfaces)
    known = tuple(i for i in _JOINT_INTERFACE_ORDER if i in present)
    return known + tuple(sorted(present - set(_JOINT_INTERFACE_ORDER)))


def _joint_units(interfaces: Iterable[str]) -> dict[str, Unit]:
    """Units for joint interfaces, refusing anything the preset cannot name."""
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
    """The name of a mode group, derived from what it drives.

    ``{position}`` is ``"position"`` and ``{position, velocity}`` is
    ``"position+velocity"``. Deriving the name means two groups that drive the
    same interfaces collide on it, which the validator then reports as the
    duplicate they are.
    """
    return "+".join(sorted(interfaces))


def _broadcast(
    value: Mapping[str, float] | float, joints: Sequence[str], label: str
) -> dict[str, float]:
    """A per-joint table from either one number for all of them or a full map.

    A partial mapping is an error rather than a default, because the joint
    somebody forgot is the one that gets no gain and falls over.
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
    """An arm: every joint the same, one mode group live at a time.

    Each entry of ``groups`` becomes one exclusive group over every arm joint,
    named after the interfaces it drives, and the joints command the union of
    them. Two groups is the xArm shape -- position or velocity, never both on
    one joint -- and one group is everything else.

    The gripper, if there is one, is a joint like any other except for its unit,
    and it lands in a non-exclusive group of its own so it stays commandable
    alongside whichever arm group is live.

    Args:
        source: Key prefix for this hardware. Also its watchdog and epoch scope.
        joints: Arm joint names, in the order the vendor orders them.
        limits: Per-key limits, normally from ``limits_from_urdf``. Passed
            through as given; the gripper's own limit is added from ``gripper``.
        state: Interfaces the hardware actually reports. An xArm reports no
            velocity, so it passes ``(position, effort)`` rather than publishing
            fabricated zeros.
        groups: One frozenset of interfaces per mode group.
        gripper: A gripper to add, or ``None``.
        safe_stop: How the arm comes to rest. Holding position by default.
        estop: How the arm drops authority.
        activation: Whether arming needs an operator step.
        timing: Rates and deadlines.
        process_loss: What happens if this process dies. UNKNOWN until somebody
            runs the bench checklist (D23).
        omission: Per-key overrides of what an unmentioned key means. This is
            how a vendor opts a key into UNSET (D11).
        meta: Free-form strings for the description view.

    Raises:
        ValueError: On no joints, no groups, or an interface with no preset unit.
        DescriptionError: If the result breaks any contract rule.
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
    """A PD-controlled body: one impedance group over every joint.

    Position, velocity, effort, kp and kd all travel together in one group, so
    there is no mode to switch and nothing to sequence. The gains ride in
    ``initial_values``: a position-only task that wins one of these joints gets
    the declared kp and kd through the retain-last rule rather than having to
    emit gains it has no opinion about (D12).

    ``damp_kd`` is the separate, higher gain table the damping safe stop runs
    at. It is required even when ``safe_stop`` is overridden, because a body
    that stops by going limp needs to know how limp.

    Args:
        source: Key prefix for this hardware.
        joints: Joint names, in the vendor's own order -- for a Unitree that is
            the order of the motor slots, so it is load-bearing.
        limits: Per-key limits, normally from ``limits_from_urdf``. A whole-body
            policy overshooting by a milliradian must not stall the robot, so
            these are usually CLAMP (D13).
        kp: Proportional gain: one number for every joint, or a full table keyed
            by joint name.
        kd: Derivative gain, same shape.
        damp_kd: Derivative gain for the damping safe stop, same shape.
        imu: An IMU to report alongside the joints, or ``None``.
        safe_stop: Overrides the damping stop built from ``damp_kd``. A DAMP
            policy passed without its own kd table is filled in from
            ``damp_kd``, so overriding one only to say where the robot ends up
            does not mean restating the gains.
        estop: How the body drops authority. Disabling the motors by default,
            which is not recoverable without a fresh bring-up.
        activation: Whether arming needs an operator step.
        timing: Rates and deadlines.
        process_loss: What happens if this process dies.
        omission: Per-key overrides. A G1 declares its velocity keys UNSET,
            because 0.0 reaches that firmware as VEL_STOP rather than as no
            command at all (D11).
        meta: Free-form strings. ``imu_frame_id`` is added when ``imu`` is given.

    Raises:
        ValueError: On no joints, or a gain table that misses one.
        DescriptionError: If the result breaks any contract rule.
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
    """A mobile base: one resource, commanded as a body-frame twist.

    There are no virtual joints. A base is a single resource carrying the axes
    it can actually drive, which is what a base task claims and already emits
    (D3). A differential drive declares ``(vx, wz)``, a holonomic chassis adds
    ``vy``, and a free-flyer declares all six.

    Omission is left at its default, so an axis nobody claims is driven to zero
    every cycle rather than coasting on the last thing it was told. A task that
    wants the base to hold keeps claiming it (D17).

    Args:
        source: Key prefix for this hardware.
        axes: Commandable twist axes, from ``vx vy vz wx wy wz``.
        limits: Per-axis limits. This is the velocity envelope, so a Go2
            entering rage mode re-describes with a different one.
        odometry: Whether the base reports the integrated pose of its axes.
        measured_velocity: Whether the base reports measured twist. False for a
            base that can only echo what it was commanded, which is not a
            measurement and must not be declared as one.
        safe_stop: How the base comes to rest. Zeroing by default.
        estop: How the base drops authority.
        activation: Whether arming needs an operator step.
        timing: Rates and deadlines.
        process_loss: What happens if this process dies.
        meta: Free-form strings, typically the odometry frame and whether this
            base's yaw is wrapped.

    Raises:
        ValueError: On no axes, a repeated axis, or an axis outside the six.
        DescriptionError: If the result breaks any contract rule.
    """
    if not axes:
        raise ValueError(f"base {source!r} declares no axes")
    unknown = [axis for axis in axes if axis not in _BASE_AXES]
    if unknown:
        raise ValueError(
            f"base {source!r} declares non-twist axes {unknown}: expected {sorted(_BASE_AXES)}"
        )
    repeated = sorted({axis for axis in axes if list(axes).count(axis) > 1})
    if repeated:
        raise ValueError(f"base {source!r} repeats axes {repeated}")

    units = {axis: _BASE_AXES[axis][1] for axis in axes}
    state_interfaces: tuple[str, ...] = tuple(axes) if measured_velocity else ()
    if odometry:
        pose = tuple(_BASE_AXES[axis][0] for axis in axes)
        state_interfaces += pose
        units |= {_BASE_AXES[axis][0]: _BASE_AXES[axis][2] for axis in axes}

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
