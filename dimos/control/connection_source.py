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


from __future__ import annotations

from collections.abc import Callable, Collection, Mapping
from dataclasses import dataclass
import math
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.control.contract.convert import (
    IMU_INTERFACES,
    JOINT_STATE_FIELDS,
    POSE_INTERFACES,
    TWIST_INTERFACES,
    imu_to_values,
    joint_state_from_values,
    joint_state_to_values,
    pose_to_values,
    twist_from_values,
    twist_to_values,
)
from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.control.contract.limits import Limits, model_limit, movable_joints
from dimos.control.contract.sequence import Freshness
from dimos.control.task import JointStateSnapshot
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.robot.assets.model import RobotModel

logger = setup_logger()

Values = dict[str, float]

POSITION_COMMAND = "position_command"
VELOCITY_COMMAND = "velocity_command"
MOTOR_COMMAND = "motor_command"
CMD_VEL = "cmd_vel"
JOINT_STATE = "joint_state"
ODOM = "odom"
BASE_VELOCITY = "base_velocity"
IMU = "imu"

COMMAND_PORTS: Mapping[str, type] = {
    POSITION_COMMAND: JointState,
    VELOCITY_COMMAND: JointState,
    MOTOR_COMMAND: MotorCommandArray,
    CMD_VEL: Twist,
}
READING_PORTS: Mapping[str, type] = {
    JOINT_STATE: JointState,
    ODOM: PoseStamped,
    BASE_VELOCITY: TwistStamped,
    IMU: Imu,
}

_JOINT_PORTS = frozenset({POSITION_COMMAND, VELOCITY_COMMAND, MOTOR_COMMAND, JOINT_STATE})

_INTERFACE_BY_MODE: dict[ControlMode, str] = {
    ControlMode.POSITION: Interface.POSITION,
    ControlMode.SERVO_POSITION: Interface.POSITION,
    ControlMode.VELOCITY: Interface.VELOCITY,
    ControlMode.TORQUE: Interface.EFFORT,
}


@dataclass(frozen=True, slots=True)
class PortMessage:
    port: str
    message: Any


@dataclass(frozen=True, slots=True)
class _SensorReading:
    sensor: str
    reading: str
    key: str


@dataclass(frozen=True, slots=True)
class _Joint:
    name: str
    state_keys: dict[str, str]
    command_keys: dict[str, str]
    position_limit: Limits


class ConnectionSource:
    def __init__(
        self,
        robot: str,
        commands: Collection[str],
        readings: Collection[str],
        model: RobotModel | None,
        reading_timeout_s: float,
        *,
        zero_stiffness_joints: Collection[str] = (),
    ) -> None:
        if commands and not readings:
            raise ValueError(
                f"{robot!r} reports nothing, so the coordinator cannot tell it is there; "
                f"it needs a reading port, e.g. `{robot}_{JOINT_STATE}: In[JointState]`"
            )
        if MOTOR_COMMAND in commands:
            raise ValueError(
                f"{robot!r}: {robot}_{MOTOR_COMMAND} needs stiffness and damping, which the "
                "coordinator cannot send"
            )
        joints: Mapping[str, Limits] = {}
        if _JOINT_PORTS & {*commands, *readings}:
            if VELOCITY_COMMAND not in commands and not (
                POSITION_COMMAND in commands and JOINT_STATE in readings
            ):
                raise ValueError(
                    f"{robot!r}: its joints cannot be held when no task drives them; they need "
                    f"to take a velocity ({robot}_{VELOCITY_COMMAND}), or take a position "
                    f"({robot}_{POSITION_COMMAND}) they report ({robot}_{JOINT_STATE})"
                )
            if model is None:
                raise ValueError(
                    f"{robot!r} has joint ports but no model to read its joints from; set "
                    "`model` on the coordinator"
                )
            joints = _robot_joints(robot, model)

        self._robot = robot
        self._timeout = reading_timeout_s
        self._joint_ports: dict[str, str] = {}
        if POSITION_COMMAND in commands:
            self._joint_ports[Interface.POSITION] = POSITION_COMMAND
        if VELOCITY_COMMAND in commands:
            self._joint_ports[Interface.VELOCITY] = VELOCITY_COMMAND
        self._base_port = CMD_VEL if CMD_VEL in commands else None
        self._base = f"{robot}{SEPARATOR}base"
        self._joints = [
            _Joint(
                name=name,
                state_keys={i: f"{name}{SEPARATOR}{i}" for i in JOINT_STATE_FIELDS},
                command_keys={i: f"{name}{SEPARATOR}{i}" for i in self._joint_ports},
                position_limit=limit,
            )
            for name, limit in joints.items()
        ]
        unknown = frozenset(zero_stiffness_joints) - set(joints)
        if unknown:
            raise ValueError(
                f"{robot!r}: zero-stiffness joints are not in its model: {sorted(unknown)}"
            )
        self.zero_stiffness_joints = frozenset(zero_stiffness_joints)
        imu = f"{robot}{SEPARATOR}imu"
        self._sensors = (
            [_SensorReading(imu, i, f"{imu}{SEPARATOR}{i}") for i in IMU_INTERFACES]
            if IMU in readings
            else []
        )

        self._readers: dict[str, Callable[[Any], Values]] = {}
        self._needed: dict[str, list[tuple[str, ...]]] = {}
        own: set[str] = set()
        for port in readings:
            if port == JOINT_STATE:
                self._readers[port] = _read_joint_state
                groups = [tuple(joint.state_keys.values()) for joint in self._joints]
            elif port == ODOM:
                self._readers[port] = _reader(pose_to_values, self._base, robot)
                groups = [(f"{self._base}{SEPARATOR}{i}",) for i in POSE_INTERFACES]
            elif port == BASE_VELOCITY:
                self._readers[port] = _reader(twist_to_values, self._base, robot)
                groups = [(f"{self._base}{SEPARATOR}{i}",) for i in TWIST_INTERFACES]
            elif port == IMU:
                self._readers[port] = _reader(imu_to_values, imu, robot)
                groups = [(sensor.key,) for sensor in self._sensors]
            else:
                raise ValueError(
                    f"{robot!r}: {port!r} is not a kind of reading port; "
                    f"one of {', '.join(READING_PORTS)}"
                )
            self._needed[port] = groups
            own.update(key for group in groups for key in group)
        self._own_keys = frozenset(own)

        self._lock = threading.Lock()
        self._latest: Values = {}
        self._reported: frozenset[str] = frozenset()
        self._heard: set[str] = set()
        self._received_at = 0.0
        self._freshness = Freshness()
        self._resumed = False
        self._restart = False
        self._holds: dict[str, float] = {}
        self._last_warned = float("-inf")

    @property
    def robot(self) -> str:
        return self._robot

    @property
    def joint_names(self) -> list[str]:
        return [joint.name for joint in self._joints]

    @property
    def command_ports(self) -> list[str]:
        ports = list(self._joint_ports.values())
        if self._base_port is not None:
            ports.append(self._base_port)
        return ports

    @property
    def reading_ports(self) -> list[str]:
        return list(self._readers)

    @property
    def reading_timeout_s(self) -> float:
        return self._timeout

    def on_message(self, port: str, msg: Any) -> None:
        try:
            values = self._readers[port](msg)
        except ValueError as error:
            now = time.monotonic()
            if now - self._last_warned >= 1.0:
                self._last_warned = now
                logger.warning(f"Dropped a {port} message for {self._robot!r}: {error}")
            return
        mine = {key: value for key, value in values.items() if key in self._own_keys}
        if not mine:
            return
        now = time.monotonic()
        with self._lock:
            latest, reported = self._latest, self._reported
            last = self._freshness.last_receipt
            if self._restart or (last is not None and now - last > self._timeout):
                latest, reported = {}, frozenset()
                self._resumed = True
                self._restart = False
            latest = {**latest, **mine}
            if port not in reported and all(
                any(key in latest for key in group) for group in self._needed[port]
            ):
                reported = reported | {port}
                self._heard.add(port)
            self._latest, self._reported = latest, reported
            stamp = getattr(msg, "ts", None)
            self._received_at = (
                float(stamp) if isinstance(stamp, (int, float)) and stamp > 0 else time.time()
            )
            self._freshness.mark(now)

    def drop_holds(self) -> None:
        with self._lock:
            self._reported = frozenset()
            self._restart = True

    def ready_for_control(self, now: float | None = None) -> bool:
        if now is None:
            now = time.monotonic()
        with self._lock:
            return len(self._reported) == len(self._needed) and not self._freshness.is_stale(
                now, self._timeout
            )

    def unreported(self) -> list[str]:
        with self._lock:
            return [port for port in self._needed if port not in self._heard]

    def read_joints(self) -> JointStateSnapshot:
        latest = self._latest
        snapshot = JointStateSnapshot(timestamp=self._received_at)
        for joint in self._joints:
            for interface, into in (
                (Interface.POSITION, snapshot.joint_positions),
                (Interface.VELOCITY, snapshot.joint_velocities),
                (Interface.EFFORT, snapshot.joint_efforts),
            ):
                key = joint.state_keys[interface]
                if key in latest:
                    into[joint.name] = latest[key]
        return snapshot

    def read_sensors(self) -> dict[str, dict[str, float]]:
        latest = self._latest
        readings: dict[str, dict[str, float]] = {}
        for each in self._sensors:
            if each.key in latest:
                readings.setdefault(each.sensor, {})[each.reading] = latest[each.key]
        return readings

    def command(self, winners: Mapping[str, float], mode: ControlMode | None) -> Values:
        with self._lock:
            latest = self._latest
            if self._resumed:
                self._resumed = False
                self._holds.clear()
        interface = _INTERFACE_BY_MODE.get(mode) if mode is not None else None
        for joint in self._joints:
            if joint.name not in winners:
                continue
            if interface is None or interface not in joint.command_keys:
                raise ValueError(f"{joint.name} cannot be told its {interface}")
            value = winners[joint.name]
            limit = joint.position_limit
            if interface == Interface.POSITION and not _within(value, limit):
                raise ValueError(f"{joint.name} told {value}, outside {limit.lo}..{limit.hi}")
        values: Values = {}
        for joint in self._joints:
            if joint.name in winners:
                assert interface is not None
                value = winners[joint.name]
                values[joint.command_keys[interface]] = value
                if interface == Interface.POSITION:
                    self._holds[joint.name] = value
                else:
                    self._holds.pop(joint.name, None)
                continue
            velocity_key = joint.command_keys.get(Interface.VELOCITY)
            if mode is ControlMode.VELOCITY and velocity_key is not None:
                values[velocity_key] = 0.0
                self._holds.pop(joint.name, None)
                continue
            hold = self._hold(joint, latest)
            if hold is not None:
                values[joint.command_keys[Interface.POSITION]] = hold
            elif velocity_key is not None:
                values[velocity_key] = 0.0
        if self._base_port is not None:
            values.update((f"{self._base}{SEPARATOR}{i}", 0.0) for i in TWIST_INTERFACES)
        return values

    def command_messages(self, values: Mapping[str, float], ts: float) -> list[PortMessage]:
        messages: list[PortMessage] = []
        for interface, port in self._joint_ports.items():
            joints = [j.name for j in self._joints if f"{j.name}{SEPARATOR}{interface}" in values]
            if joints:
                messages.append(PortMessage(port, joint_state_from_values(values, joints, ts)))
        if self._base_port is not None:
            messages.append(PortMessage(self._base_port, twist_from_values(values, self._base)))
        return messages

    def _hold(self, joint: _Joint, latest: Values) -> float | None:
        if Interface.POSITION not in joint.command_keys:
            return None
        held = self._holds.get(joint.name)
        if held is None:
            measured_key = joint.state_keys[Interface.POSITION]
            if measured_key not in latest:
                return None
            held = latest[measured_key]
            self._holds[joint.name] = held
        limit = joint.position_limit
        if limit.lo is not None:
            held = max(held, limit.lo)
        if limit.hi is not None:
            held = min(held, limit.hi)
        return held


def _robot_joints(robot: str, model: RobotModel) -> dict[str, Limits]:
    prefix = f"{robot}{SEPARATOR}"
    joints = {
        joint.name: model_limit(joint, Interface.POSITION)
        for joint in movable_joints(model)
        if joint.name.startswith(prefix)
    }
    if not joints:
        raise ValueError(
            f"the model has no movable joint named {prefix}...; name {robot!r}'s joints after "
            "it, e.g. with `RobotModel.with_renamed_joints`"
        )
    return joints


def _read_joint_state(msg: Any) -> Values:
    return joint_state_to_values(msg, JOINT_STATE_FIELDS)


def _within(value: float, limit: Limits) -> bool:
    return (
        math.isfinite(value)
        and (limit.lo is None or value >= limit.lo)
        and (limit.hi is None or value <= limit.hi)
    )


def _reader(
    convert: Callable[[Any, str], Values], part: str, robot: str
) -> Callable[[Any], Values]:
    def read(msg: Any) -> Values:
        frame = getattr(msg, "frame_id", "")
        if SEPARATOR in frame and not frame.startswith(f"{robot}{SEPARATOR}"):
            raise ValueError(f"frame {frame!r} is another robot's")
        return convert(msg, part)

    return read
