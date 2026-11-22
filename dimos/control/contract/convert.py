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

"""Turn the standard robot messages into named values, and back.

Connection modules and the coordinator talk in standard messages
(``JointState``, ``Imu``).
Inside, both work with named values such as ``{"arm/joint1/position": 0.3}``.
These functions convert between the two, and both sides use the same ones, so
they always agree on what each field means.

Each pair is ``<message>_to_values`` and ``<message>_from_values``. Names:

  - a joint is named in full, robot and part: ``"arm/joint1"``
  - a base and a sensor are named the same way: ``"chassis/base"``,
    ``"g1/imu"``
  - a value adds what it is about: ``"arm/joint1/position"``

Every ``_from_values`` needs each value its message carries, and raises
``ValueError`` naming any that are missing.
"""

from __future__ import annotations

from collections.abc import Collection, Mapping, Sequence
from typing import TYPE_CHECKING

from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.control.contract.limits import movable_joints
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from dimos.robot.assets.model import RobotModel

#: The JointState fields, each named after the interface it carries.
JOINT_STATE_FIELDS: tuple[Interface, ...] = (
    Interface.POSITION,
    Interface.VELOCITY,
    Interface.EFFORT,
)
#: What an Imu carries: orientation, turn rates, accelerations.
IMU_INTERFACES: tuple[Interface, ...] = (
    Interface.QX,
    Interface.QY,
    Interface.QZ,
    Interface.QW,
    Interface.GX,
    Interface.GY,
    Interface.GZ,
    Interface.AX,
    Interface.AY,
    Interface.AZ,
)


def motor_joints(model: RobotModel, source: str) -> list[str]:
    """Every joint of the model that moves, in model order, named in full,
    e.g. "g1/left_knee"."""
    return [
        joint.name if SEPARATOR in joint.name else f"{source}{SEPARATOR}{joint.name}"
        for joint in movable_joints(model)
    ]


def joint_state_to_values(msg: JointState, fields: Collection[str]) -> dict[str, float]:
    """Joint values from some of a JointState's fields.

    A field left empty in the message gives no values. A command input reads
    only its own field, e.g. ``position_command`` reads only ``position``.

    Args:
        msg: Joints named in full, e.g. "arm/joint1"; positions in rad or m,
            speeds in rad/s or m/s, efforts in Nm or N.
        fields: Which of "position", "velocity", "effort" to read.

    Raises:
        ValueError: If a field asked for is not a JointState field, or is
            filled but not one value per name.
    """
    values: dict[str, float] = {}
    for field in fields:
        if field not in JOINT_STATE_FIELDS:
            raise ValueError(f"JointState has no {field!r} field")
        column = getattr(msg, field)
        if not column:
            continue
        if len(column) != len(msg.name):
            raise ValueError(f"JointState has {len(msg.name)} names but {len(column)} {field}")
        values.update(
            (_key(name, field), float(v)) for name, v in zip(msg.name, column, strict=True)
        )
    return values


def joint_state_from_values(
    values: Mapping[str, float], joints: Sequence[str], ts: float | None = None
) -> JointState:
    """A JointState for some joints.

    Each of position, velocity and effort is filled when ``values`` has it
    for every joint, and left empty when it has it for none.

    Args:
        values: Named values, e.g. ``{"arm/joint1/position": 0.3}``.
        joints: Which joints, named in full, in the order to list them.
        ts: When the values were taken, in Unix seconds. Now if ``None``.

    Raises:
        ValueError: If a field is there for some joints and not others (send
            those in separate messages), or there is nothing to send.
    """
    columns: dict[str, list[float]] = {}
    for field in JOINT_STATE_FIELDS:
        present = [_key(joint, field) in values for joint in joints]
        if all(present):
            columns[field] = [values[_key(joint, field)] for joint in joints]
        elif any(present):
            missing = [j for j, p in zip(joints, present, strict=True) if not p]
            raise ValueError(f"{field} is missing for {missing} but not the other joints")
    if not columns:
        raise ValueError(f"no position, velocity or effort for {list(joints)}")
    return JointState(
        ts=ts,
        name=list(joints),
        position=columns.get(Interface.POSITION),
        velocity=columns.get(Interface.VELOCITY),
        effort=columns.get(Interface.EFFORT),
    )


def imu_to_values(imu: Imu, sensor: str) -> dict[str, float]:
    """An IMU's readings from an Imu message: orientation as a quaternion,
    turn rates in rad/s, accelerations in m/s^2."""
    q, g, a = imu.orientation, imu.angular_velocity, imu.linear_acceleration
    readings = (q.x, q.y, q.z, q.w, g.x, g.y, g.z, a.x, a.y, a.z)
    return {_key(sensor, i): float(v) for i, v in zip(IMU_INTERFACES, readings, strict=True)}


def imu_from_values(
    values: Mapping[str, float], sensor: str, ts: float | None = None, frame_id: str = ""
) -> Imu:
    """An Imu message from an IMU's ten readings (see ``imu_to_values``)."""
    qx, qy, qz, qw, gx, gy, gz, ax, ay, az = _take(values, sensor, IMU_INTERFACES)
    return Imu(
        orientation=Quaternion(qx, qy, qz, qw),
        angular_velocity=Vector3(gx, gy, gz),
        linear_acceleration=Vector3(ax, ay, az),
        frame_id=frame_id,
        ts=ts,
    )


def _key(part: str, interface: str) -> str:
    return f"{part}{SEPARATOR}{interface}"


def _take(values: Mapping[str, float], part: str, interfaces: Sequence[str]) -> list[float]:
    _require(values, [part], interfaces)
    return [values[_key(part, i)] for i in interfaces]


def _require(values: Mapping[str, float], parts: Sequence[str], interfaces: Sequence[str]) -> None:
    missing = [_key(p, i) for p in parts for i in interfaces if _key(p, i) not in values]
    if missing:
        raise ValueError(f"missing values {missing}")
