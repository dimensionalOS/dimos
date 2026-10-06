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

Robot drivers and the coordinator talk in standard messages (``Twist``,
``JointState``, ``MotorCommandArray``, ``PoseStamped``, ``Imu``). Inside,
both work with named values such as ``{"arm/joint1/position": 0.3}``. These
functions convert between the two, and both sides use the same ones, so they
always agree on what each field means.

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
import math

from dimos.control.contract.description import ControlDescription, ResourceKind
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
    SEPARATOR,
    VELOCITY,
    VX,
    VY,
    WZ,
    YAW,
    X,
    Y,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray

#: The command inputs a robot driver may declare, by port name, and the
#: message each carries. Each JointState input reads one field: the one it is
#: named after.
COMMAND_PORTS: Mapping[str, type] = {
    "position_command": JointState,
    "velocity_command": JointState,
    "motor_command": MotorCommandArray,
    "base_command": Twist,
}
#: The reading outputs a robot driver may declare, by port name, and the
#: message each carries.
STATE_PORTS: Mapping[str, type] = {
    "joint_state": JointState,
    "odom": PoseStamped,
    "base_velocity": Twist,
    "imu": Imu,
}

#: The JointState fields, each named after the interface it carries.
JOINT_STATE_FIELDS: tuple[str, ...] = (POSITION, VELOCITY, EFFORT)
#: What a MotorCommandArray carries for each joint, in its field order
#: q, dq, tau, kp, kd.
MOTOR_INTERFACES: tuple[str, ...] = (POSITION, VELOCITY, EFFORT, KP, KD)
#: What a Twist carries for a base: forwards, leftwards, turning.
TWIST_INTERFACES: tuple[str, ...] = (VX, VY, WZ)
#: What a PoseStamped carries for a base: where it is and which way it faces.
POSE_INTERFACES: tuple[str, ...] = (X, Y, YAW)
#: What an Imu carries: orientation, turn rates, accelerations.
IMU_INTERFACES: tuple[str, ...] = (QX, QY, QZ, QW, GX, GY, GZ, AX, AY, AZ)


def motor_joints(descriptions: Sequence[ControlDescription]) -> list[str]:
    """The joints a MotorCommandArray carries, in its order.

    Every joint that takes stiffness and damping (``kp`` and ``kd``), in the
    order the descriptions list them. The message has no names, so the sender
    and the driver must both build the order with this.

    Args:
        descriptions: Everything one driver module runs, in the order its
            ``describe_control`` returns them.
    """
    return [
        f"{d.source}{SEPARATOR}{r.name}"
        for d in descriptions
        for r in d.resources
        if r.kind is ResourceKind.JOINT and {KP, KD} <= set(r.command_interfaces)
    ]


def twist_to_values(twist: Twist, base: str) -> dict[str, float]:
    """A base's speeds from a Twist.

    Only forwards (``linear.x``), leftwards (``linear.y``) and turning
    (``angular.z``) are read; a base on the floor has no other axes.

    Args:
        twist: Speeds in m/s and rad/s, measured from the base itself.
        base: The base's full name, e.g. "chassis/base".
    """
    vx, vy, wz = twist.linear.x, twist.linear.y, twist.angular.z
    return {_key(base, VX): vx, _key(base, VY): vy, _key(base, WZ): wz}


def twist_from_values(values: Mapping[str, float], base: str) -> Twist:
    """A Twist from a base's ``vx``, ``vy`` and ``wz`` (m/s, m/s, rad/s): a
    speed to command, or one the base measured."""
    vx, vy, wz = _take(values, base, TWIST_INTERFACES)
    return Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz))


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
        position=columns.get(POSITION),
        velocity=columns.get(VELOCITY),
        effort=columns.get(EFFORT),
    )


def motor_command_to_values(msg: MotorCommandArray, joints: Sequence[str]) -> dict[str, float]:
    """Joint values from a MotorCommandArray.

    Args:
        msg: One entry per joint: target position ``q`` (rad), speed ``dq``
            (rad/s), extra torque ``tau`` (Nm), stiffness ``kp``, damping ``kd``.
        joints: The joint each entry is for, from ``motor_joints``.

    Raises:
        ValueError: If the message is not one entry per joint.
    """
    if msg.num_joints != len(joints) or len(msg.q) != len(joints):
        raise ValueError(f"MotorCommandArray has {len(msg.q)} joints, expected {len(joints)}")
    columns = (msg.q, msg.dq, msg.tau, msg.kp, msg.kd)
    return {
        _key(joint, interface): float(column[i])
        for i, joint in enumerate(joints)
        for interface, column in zip(MOTOR_INTERFACES, columns, strict=True)
    }


def motor_command_from_values(
    values: Mapping[str, float], joints: Sequence[str], ts: float | None = None
) -> MotorCommandArray:
    """A MotorCommandArray from each joint's position, velocity, effort, kp
    and kd, in the order of ``joints`` (from ``motor_joints``)."""
    _require(values, joints, MOTOR_INTERFACES)
    q, dq, tau, kp, kd = ([values[_key(j, i)] for j in joints] for i in MOTOR_INTERFACES)
    return MotorCommandArray(q=q, dq=dq, kp=kp, kd=kd, tau=tau, timestamp=ts)


def pose_to_values(pose: PoseStamped, base: str) -> dict[str, float]:
    """Where a base is (``x``, ``y`` in m) and which way it faces (``yaw`` in
    rad, 0 along x, growing to the left), from a PoseStamped."""
    q = pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return {_key(base, X): pose.position.x, _key(base, Y): pose.position.y, _key(base, YAW): yaw}


def pose_from_values(
    values: Mapping[str, float], base: str, ts: float | None = None, frame_id: str = ""
) -> PoseStamped:
    """A PoseStamped from a base's ``x``, ``y`` (m) and ``yaw`` (rad), flat on
    the floor."""
    x, y, yaw = _take(values, base, POSE_INTERFACES)
    orientation = Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    return PoseStamped(
        ts=ts, frame_id=frame_id, position=Vector3(x, y, 0.0), orientation=orientation
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
