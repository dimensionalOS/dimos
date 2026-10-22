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

Every connection module has to describe itself before anything will talk to
it. Writing that out by hand is long and easy to get subtly wrong, so these
three functions build it in one call::

    manipulator_description(XARM7_MODEL, source="arm")

  ``manipulator_description``  an arm
  ``pd_joint_description``     a body whose joints are held in place by
                               stiffness and damping, such as a humanoid
  ``twist_base_description``   something that drives around on the floor

The arm and body presets read every joint that can move, its units and its
limits from the robot's model (its URDF, loaded as a ``RobotModel``). A
gripper is a joint of the model like any other. A robot with no URDF builds
its ``ControlDescription`` by hand.

A connection module's config says the same things as typed fields: a
``JointSpec``, ``PdSpec`` or ``BaseSpec`` for each thing the robot is told,
plus ``imu`` for an orientation sensor. ``control_description`` reads the
config and the robot model and builds the description::

    class Go2Config(ConnectionConfig):
        source: str = "go2"
        cmd_vel: BaseSpec = BaseSpec(range=Twist(Vector3(1.0, 0.5, 0.0), Vector3(0.0, 0.0, 2.0)))
        imu: bool = True

A spec only describes. The module itself subscribes to its ports, turns their
messages into values for ``command``, and publishes its readings.

Each builder checks its own result before handing it back, so a description
built this way is always a valid one.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import replace
import math
from typing import TYPE_CHECKING

from dimos.control.contract.convert import JOINT_STATE_FIELDS, MOTOR_INTERFACES
from dimos.control.contract.description import (
    ControlDescription,
    Limits,
    Resource,
    ResourceKind,
)
from dimos.control.contract.keys import Interface, Key, Unit
from dimos.control.contract.validate import validate_description
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.control.connection.connection_module import ConnectionConfig
    from dimos.robot.assets.model import JointDescription, RobotModel

#: The one part a base is: it is driven as a whole, not as wheels.
BASE_NAME = "base"

#: The unit of each joint interface, for a revolute joint and a prismatic one.
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
#: Units by the kind of joint a URDF says it is. These are the joints that
#: can move; any other kind (fixed, floating, planar) cannot be driven.
_UNITS_BY_URDF_TYPE: Mapping[str, Mapping[str, Unit]] = {
    "revolute": _REVOLUTE,
    "continuous": _REVOLUTE,
    "prismatic": _PRISMATIC,
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
BASE_TWIST_UNITS: Mapping[str, Unit] = {
    Interface.VX: Unit.M_PER_S,
    Interface.VY: Unit.M_PER_S,
    Interface.WZ: Unit.RAD_PER_S,
}
#: Where a base reports itself as being, and which way it is facing.
BASE_POSE_UNITS: Mapping[str, Unit] = {
    Interface.X: Unit.M,
    Interface.Y: Unit.M,
    Interface.YAW: Unit.RAD,
}


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


def model_joints(
    source: str,
    model: RobotModel,
    state: Sequence[Interface],
    command: Sequence[Interface],
) -> tuple[list[Resource], dict[str, Limits]]:
    """Every joint of the robot model that can move, with the model's limits on what it is told.

    Joints come in the model's order. A fixed, floating or planar joint is
    skipped.

    Args:
        source: The robot's name, the first part of each key.
        model: Where the joints, their units and their limits come from.
        state: What each joint reports.
        command: What each joint is told. Only these get limits.

    Raises:
        ValueError: If no joint can move, an interface has no preset unit, or
            the model is unclear about a limit a commanded joint needs.
    """
    loaded = model.load()
    joints = [joint for joint in loaded.joints if joint.type in _UNITS_BY_URDF_TYPE]
    if not joints:
        raise ValueError(f"{source!r} has no joints to drive")

    resources: list[Resource] = []
    limits: dict[str, Limits] = {}
    for joint in joints:
        kind_units = _UNITS_BY_URDF_TYPE[joint.type]
        units: dict[str, Unit] = {}
        for interface in dict.fromkeys((*state, *command)):
            if interface not in kind_units:
                raise ValueError(
                    f"interface '{interface}' has no preset unit: build the Resource by hand "
                    f"or use one of {', '.join(kind_units)}"
                )
            units[interface] = kind_units[interface]
        for interface in command:
            limit = _model_limit(joint, interface)
            if limit is not None:
                limits[Key.of(source, joint.name, interface)] = limit
        resources.append(
            Resource(
                name=joint.name,
                kind=ResourceKind.JOINT,
                state_interfaces=tuple(state),
                command_interfaces=tuple(command),
                units=units,
            )
        )
    return resources, limits


def manipulator_description(
    model: RobotModel,
    *,
    source: str,
    state: Sequence[Interface] = (Interface.POSITION, Interface.VELOCITY, Interface.EFFORT),
    command: Sequence[Interface] = (Interface.POSITION, Interface.VELOCITY),
    sensors: Sequence[Resource] = (),
    state_rate_hz: float = 100.0,
    deadman_timeout_s: float = 0.1,
) -> ControlDescription:
    """Describe an arm from its robot model.

    Every joint that can move is driven, in the model's order, within the
    model's limits; a gripper is one of them. A command past a limit is
    refused.

    Args:
        model: The arm's robot model.
        source: Short name for this arm, the first part of every name it
            reports, e.g. "arm". Two arms built from the same model need
            different names.
        state: What each joint reports. Leave out anything it cannot measure,
            rather than having it report zeros that look like real readings.
        command: What each joint can be told: position, velocity, or both.
        sensors: Sensors to report alongside the joints, e.g.
            ``[imu_resource()]``.
        state_rate_hz: How many times a second the arm reports.
        deadman_timeout_s: How long it goes without a command, in seconds,
            before it stops itself.

    Returns:
        A complete, checked description of the arm.

    Raises:
        ValueError: If nothing is commanded, no joint can move, an interface
            has no preset unit, or the model is unclear about a limit.
        DescriptionError: If the finished description breaks a rule.
    """
    if not command:
        raise ValueError(f"manipulator {source!r} declares nothing to command")

    resources, limits = model_joints(source, model, state, command)
    description = ControlDescription(
        source=source,
        resources=(*resources, *sensors),
        limits=limits,
        state_rate_hz=state_rate_hz,
        deadman_timeout_s=deadman_timeout_s,
    )
    validate_description(description)
    return description


def pd_joint_description(
    model: RobotModel,
    *,
    source: str,
    sensors: Sequence[Resource] = (),
    state_rate_hz: float = 500.0,
    deadman_timeout_s: float = 0.05,
) -> ControlDescription:
    """Describe a body whose joints are held by stiffness and damping, from its robot model.

    Used for humanoids and similar robots. Every joint that can move is told
    a target position, speed and force, and two numbers that say how hard to
    pull towards them: ``kp`` (stiffness) and ``kd`` (damping). All five are
    sent in every command. The joints come in the model's order, within the
    model's limits. A command past a limit is refused.

    Args:
        model: The body's robot model.
        source: Short name for this body, e.g. "g1".
        sensors: Sensors to report alongside the joints, e.g.
            ``[imu_resource()]``.
        state_rate_hz: How many times a second the body reports.
        deadman_timeout_s: How long it goes without a command, in seconds,
            before it stops itself.

    Returns:
        A complete, checked description of the body.

    Raises:
        ValueError: If no joint can move, or the model is unclear about a
            limit.
        DescriptionError: If the finished description breaks a rule.
    """
    resources, limits = model_joints(source, model, _PD_STATE, _PD_COMMAND)
    description = ControlDescription(
        source=source,
        resources=(*resources, *sensors),
        limits=limits,
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
    description = ControlDescription(
        source=source,
        resources=(_base_resource(odometry, measured_velocity), *sensors),
        limits={key: replace(limit, clamp=True) for key, limit in limits.items()},
        state_rate_hz=state_rate_hz,
        deadman_timeout_s=deadman_timeout_s,
    )
    validate_description(description)
    return description


def _base_resource(odometry: bool, measured_velocity: bool) -> Resource:
    """The base: told three speeds, reporting where it is and how fast it goes as asked."""
    pose = BASE_POSE_UNITS if odometry else {}
    return Resource(
        name=BASE_NAME,
        kind=ResourceKind.BASE,
        state_interfaces=(*(BASE_TWIST_UNITS if measured_velocity else ()), *pose),
        command_interfaces=tuple(BASE_TWIST_UNITS),
        units={**BASE_TWIST_UNITS, **pose},
    )


class Spec(BaseConfig):
    """What one thing the robot is told drives, and what that part reports. Use a subclass."""


class BaseSpec(Spec):
    """The base is told how fast to drive, and reports where it is and how fast it goes.

    It is told forwards (``vx``), sideways (``vy``) and turning (``wz``)
    speeds, measured from itself. A speed past its limit is pulled back to
    the limit and sent, not refused, so a stick pushed too far still drives
    the base at its top speed.

    Attributes:
        range: The fastest it may go each way: ``linear.x`` and ``linear.y``
            in m/s, ``angular.z`` in rad/s. Each is a size, 0 or more, that
            applies in both directions; 0 means it cannot move that way.
        odometry: Whether it reports where it has got to: ``x`` and ``y`` in
            metres and ``yaw`` in radians.
        measured_velocity: Whether it reports how fast it is going: ``vx``,
            ``vy`` and ``wz``. False for one that can only repeat back what
            it was told, which is not a measurement.
    """

    range: Twist
    odometry: bool = True
    measured_velocity: bool = True


class JointSpec(Spec):
    """Every joint of the model that can move is told one thing, and reports some.

    Joints come in the model's order, within the model's limits.

    Attributes:
        interface: What each joint is told: position, velocity or effort.
        reports: What each joint reports, out of position, velocity and
            effort. Leave out anything the hardware cannot measure.
    """

    interface: Interface = Interface.POSITION
    reports: tuple[Interface, ...] = JOINT_STATE_FIELDS


class PdSpec(Spec):
    """Every joint of the model that can move is told where to go and how stiffly to hold it.

    Each joint is told a target position, speed and effort, and the stiffness
    (``kp``) and damping (``kd``) to pull towards them with, all in every
    command, within the model's limits. Each reports its position, speed and
    effort.
    """


def specs(config: ConnectionConfig) -> dict[str, Spec]:
    """The spec fields of a config, by field name, in the order declared."""
    found: dict[str, Spec] = {}
    for name in type(config).model_fields:
        value = getattr(config, name)
        if isinstance(value, Spec):
            found[name] = value
    return found


def control_description(config: ConnectionConfig) -> ControlDescription:
    """Describe a robot from its connection module's config.

    A JointSpec or PdSpec drives every joint of the model that can move, in
    the model's order; a joint on two specs becomes one joint that takes and
    reports both. A BaseSpec adds the base, ``imu`` the sensor.

    Raises:
        ValueError: If a joint spec has no model to read, the config has two
            BaseSpecs or two PdSpecs, no joint of the model can move, the
            model is unclear about a limit, or nothing is described.
        DescriptionError: If the finished description breaks a rule.
    """
    source = config.source
    joints: dict[str, Resource] = {}
    limits: dict[str, Limits] = {}
    base_field: str | None = None
    pd_field: str | None = None
    base: Resource | None = None
    for field, spec in specs(config).items():
        if isinstance(spec, BaseSpec):
            if base_field is not None:
                raise ValueError(
                    f"{base_field!r} and {field!r} both drive the base; a module has one"
                )
            base_field = field
            base = _base_resource(spec.odometry, spec.measured_velocity)
            limits.update(_twist_limits(source, field, spec))
            continue
        if config.model is None:
            raise ValueError(
                f"config field {field!r} is a {type(spec).__name__}, which needs the robot "
                f"model: set `model` in the config"
            )
        if isinstance(spec, PdSpec):
            if pd_field is not None:
                raise ValueError(f"{pd_field!r} and {field!r} are both PdSpecs; a module has one")
            pd_field = field
            resources, spec_limits = model_joints(
                source, config.model, JOINT_STATE_FIELDS, MOTOR_INTERFACES
            )
        elif isinstance(spec, JointSpec):
            for interface in (spec.interface, *spec.reports):
                if interface not in JOINT_STATE_FIELDS:
                    raise ValueError(
                        f"{field!r}: a joint is told and reports "
                        f"{', '.join(JOINT_STATE_FIELDS)}, not {interface}"
                    )
            resources, spec_limits = model_joints(
                source, config.model, spec.reports, (spec.interface,)
            )
        else:
            raise TypeError(f"config field {field!r}: unknown spec {type(spec).__name__}")
        for resource in resources:
            _merge(joints, resource)
        limits.update(spec_limits)

    resources = list(joints.values())
    if base is not None:
        resources.append(base)
    if config.imu:
        resources.append(imu_resource())
    if not resources:
        raise ValueError(f"{source!r} describes nothing: no spec fields and no imu")

    description = ControlDescription(
        source=source,
        resources=tuple(resources),
        limits=limits,
        state_rate_hz=config.state_rate_hz,
        deadman_timeout_s=config.deadman_timeout_s,
    )
    validate_description(description)
    return description


def _twist_limits(source: str, field: str, spec: BaseSpec) -> dict[str, Limits]:
    tops = {
        Interface.VX: spec.range.linear.x,
        Interface.VY: spec.range.linear.y,
        Interface.WZ: spec.range.angular.z,
    }
    limits: dict[str, Limits] = {}
    for axis, top in tops.items():
        top = float(top)
        if not (math.isfinite(top) and top >= 0.0):
            raise ValueError(f"{field!r}: the range for {axis} must be 0 or more, got {top}")
        limits[Key.of(source, BASE_NAME, axis)] = Limits(-top, top, clamp=True)
    return limits


def _merge(into: dict[str, Resource], new: Resource) -> None:
    """Add a joint, or widen the one of that name to take and report both."""
    old = into.get(new.name)
    if old is None:
        into[new.name] = new
        return
    for interface, unit in new.units.items():
        if old.units.get(interface, unit) is not unit:
            raise ValueError(
                f"joint {new.name!r} is given two units for {interface}: "
                f"{old.units[interface].value} and {unit.value}"
            )
    into[new.name] = Resource(
        name=new.name,
        kind=old.kind,
        state_interfaces=tuple(dict.fromkeys((*old.state_interfaces, *new.state_interfaces))),
        command_interfaces=tuple(dict.fromkeys((*old.command_interfaces, *new.command_interfaces))),
        units={**old.units, **new.units},
    )
