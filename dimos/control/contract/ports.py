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

"""How a robot driver says what each of its command ports drives.

A driver's config has one field per command input port, named like the port
and typed with one of the three specs here. ``control_description`` reads
those fields, the robot model and the driver's output ports, and builds the
one ``ControlDescription`` the rest of the control stack works with::

    class Go2Config(ConnectionConfig):
        source: str = "go2"
        cmd_vel: TwistPort = TwistPort(range=Twist(Vector3(1.0, 0.5, 0.0), Vector3(0.0, 0.0, 2.0)))

    class Go2Connection(ConnectionModule):
        config: Go2Config
        cmd_vel: In[Twist]
        odom: Out[PoseStamped]

Readings need no spec. Each output port says what it reports by its message
type alone, under any name:

  ``Out[JointState]``   the position, speed and effort of every joint
  ``Out[PoseStamped]``  where the base is: x, y and yaw
  ``Out[Twist]``        how fast the base is going: vx, vy and wz
  ``Out[Imu]``          an orientation sensor called "imu"

A module reports each of these on one port at most, and has one base.
"""

from __future__ import annotations

from collections.abc import Mapping
import math
from typing import TYPE_CHECKING, ClassVar

from dimos.control.contract.convert import (
    JOINT_STATE_FIELDS,
    MOTOR_INTERFACES,
    POSE_INTERFACES,
    TWIST_INTERFACES,
)
from dimos.control.contract.description import (
    ControlDescription,
    Limits,
    Resource,
    ResourceKind,
)
from dimos.control.contract.keys import Interface, Key
from dimos.control.contract.presets import (
    BASE_POSE_UNITS,
    BASE_TWIST_UNITS,
    imu_resource,
    model_joints,
)
from dimos.control.contract.validate import validate_description
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.protocol.service.spec import BaseConfig

if TYPE_CHECKING:
    from dimos.control.connection.connection_module import ConnectionConfig

#: The one part a base is: it is driven as a whole, not as wheels.
BASE_NAME = "base"

#: The messages an output port may report readings with.
READING_MESSAGES: tuple[type, ...] = (JointState, PoseStamped, Twist, Imu)


class PortSpec(BaseConfig):
    """What one command input port drives. Use one of its subclasses."""

    message: ClassVar[type]
    """The message the input port of the same name must carry."""


class TwistPort(PortSpec):
    """A Twist input that drives the base.

    The base is told how fast to drive forwards (``linear.x``), sideways
    (``linear.y``) and turn (``angular.z``), measured from itself. A speed
    past its limit is pulled back to the limit and sent, not refused, so a
    stick pushed too far still drives the base at its top speed.

    Attributes:
        range: The fastest it may go each way: ``linear.x`` and ``linear.y``
            in m/s, ``angular.z`` in rad/s. Each is a size, 0 or more, that
            applies in both directions; 0 means it cannot move that way.
    """

    message: ClassVar[type] = Twist

    range: Twist


class JointPort(PortSpec):
    """A JointState input that tells the joints where to be or how fast to turn.

    It drives every joint of the robot model that can move, in the model's
    order, within the model's limits.

    Attributes:
        interface: Which JointState field the port reads: position, velocity
            or effort. The other fields of a message are ignored.
    """

    message: ClassVar[type] = JointState

    interface: Interface = Interface.POSITION


class MotorPort(PortSpec):
    """A MotorCommandArray input that tells the joints where to go and how
    stiffly to hold it.

    It drives every joint of the robot model that can move, within the
    model's limits. Each joint is told a target position, speed and effort,
    and the stiffness (``kp``) and damping (``kd``) to pull towards them
    with, all in every message. The message has no names: it lists the
    joints in the model's order.
    """

    message: ClassVar[type] = MotorCommandArray


def port_specs(config: ConnectionConfig) -> dict[str, PortSpec]:
    """The port spec fields of a config, by field name, in the order declared."""
    specs: dict[str, PortSpec] = {}
    for name in type(config).model_fields:
        value = getattr(config, name)
        if isinstance(value, PortSpec):
            specs[name] = value
    return specs


def reading_ports(outputs: Mapping[str, type]) -> dict[type, str]:
    """Which output port reports each kind of reading.

    Args:
        outputs: The module's output ports, name to message type. Ports of
            other message types are ignored.

    Raises:
        ValueError: If two ports carry the same reading message.
    """
    found: dict[type, str] = {}
    for name, message in outputs.items():
        if message not in READING_MESSAGES:
            continue
        if message in found:
            raise ValueError(
                f"output ports {found[message]!r} and {name!r} both report {message.__name__}; "
                f"a module reports each kind of reading on one port"
            )
        found[message] = name
    return found


def control_description(
    config: ConnectionConfig,
    inputs: Mapping[str, type],
    outputs: Mapping[str, type],
) -> ControlDescription:
    """Describe a robot from its connection module's config and ports.

    A joint port or motor port drives every joint of the model that can move,
    in the model's order. A joint driven by two ports becomes one joint that
    accepts both.

    Args:
        config: The module's config: the robot's name, model and rates, and a
            port spec field for each command input.
        inputs: The module's input ports, name to message type.
        outputs: The module's output ports, name to message type. Ports of
            other message types are ignored.

    Raises:
        ValueError: If a spec field has no input port of its name carrying its
            message, a joint port has no model to read, a module has two
            bases or two motor ports, two outputs report the same kind of
            reading, no joint of the model can move, or the model is unclear
            about a limit.
        DescriptionError: If the finished description breaks a rule.
    """
    source = config.source
    specs = port_specs(config)
    for field, spec in specs.items():
        _check_input(field, spec, inputs)
    reported = reading_ports(outputs)
    joint_state = JOINT_STATE_FIELDS if JointState in reported else ()

    joints: dict[str, Resource] = {}
    limits: dict[str, Limits] = {}
    base_field: str | None = None
    motor_field: str | None = None
    for field, spec in specs.items():
        if isinstance(spec, TwistPort):
            if base_field is not None:
                raise ValueError(
                    f"{base_field!r} and {field!r} both drive the base; a module has one"
                )
            base_field = field
            limits.update(_twist_limits(source, field, spec))
            continue
        if config.model is None:
            raise ValueError(
                f"config field {field!r} is a {type(spec).__name__}, which needs the robot "
                f"model: set `model` in the config"
            )
        if isinstance(spec, MotorPort):
            if motor_field is not None:
                raise ValueError(
                    f"{motor_field!r} and {field!r} are both motor ports; a module has one"
                )
            motor_field = field
            resources, port_limits = model_joints(
                source, config.model, joint_state, MOTOR_INTERFACES
            )
        elif isinstance(spec, JointPort):
            if spec.interface not in JOINT_STATE_FIELDS:
                raise ValueError(
                    f"{field!r}: a JointState carries {', '.join(JOINT_STATE_FIELDS)}, "
                    f"not {spec.interface}"
                )
            resources, port_limits = model_joints(
                source, config.model, joint_state, (spec.interface,)
            )
        else:
            raise TypeError(f"config field {field!r}: unknown port spec {type(spec).__name__}")
        for resource in resources:
            _merge(joints, resource)
        limits.update(port_limits)

    resources = list(joints.values())
    if JointState in reported and not resources:
        raise ValueError(
            f"output port {reported[JointState]!r} reports joints, but no JointPort or "
            f"MotorPort drives any"
        )
    base_state = (
        *(TWIST_INTERFACES if Twist in reported else ()),
        *(POSE_INTERFACES if PoseStamped in reported else ()),
    )
    base_commands = TWIST_INTERFACES if base_field is not None else ()
    if base_state or base_commands:
        units = {**BASE_TWIST_UNITS, **BASE_POSE_UNITS}
        resources.append(
            Resource(
                name=BASE_NAME,
                kind=ResourceKind.BASE,
                state_interfaces=base_state,
                command_interfaces=base_commands,
                units={i: units[i] for i in (*base_state, *base_commands)},
            )
        )
    if Imu in reported:
        resources.append(imu_resource())
    if not resources:
        raise ValueError(f"{source!r} has no command ports and reports nothing")

    description = ControlDescription(
        source=source,
        resources=tuple(resources),
        limits=limits,
        state_rate_hz=config.state_rate_hz,
        deadman_timeout_s=config.deadman_timeout_s,
    )
    validate_description(description)
    return description


def _check_input(field: str, spec: PortSpec, inputs: Mapping[str, type]) -> None:
    kind = type(spec).__name__
    carried = inputs.get(field)
    if carried is None:
        raise ValueError(
            f"config field {field!r} is a {kind}, but the module has no input port called "
            f"{field!r}; declare `{field}: In[{spec.message.__name__}]`"
        )
    if carried is not spec.message:
        raise ValueError(
            f"config field {field!r} is a {kind}, so input port {field!r} must carry "
            f"{spec.message.__name__}, not {getattr(carried, '__name__', carried)}"
        )


def _twist_limits(source: str, field: str, spec: TwistPort) -> dict[str, Limits]:
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
    """Add a joint, or widen the one of that name to accept and report both."""
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
