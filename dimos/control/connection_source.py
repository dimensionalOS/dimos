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

"""The coordinator's record of one robot it drives through a connection module.

A connection module reports a robot's readings and takes its commands as
standard messages (``JointState``, ``TwistStamped``, ``PoseStamped``, ``Imu``)
on its ports. The coordinator keeps one ``ConnectionSource`` per robot it
finds among its own ports. It turns each reading into named values with the
shared converters in ``dimos.control.contract.convert``, keeps the latest, and
builds each tick's complete command and the messages that carry it, each
addressed to the kind of port it goes out on.
"""

from __future__ import annotations

from collections.abc import Callable, Collection, Mapping
from dataclasses import dataclass
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
from dimos.control.contract.presets import model_limit, movable_joints
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
    from dimos.control.contract.description import Limits
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

#: The kinds of port a coordinator sends a robot commands on, each with the
#: message it carries. A coordinator names such a port ``<robot>_<kind>``.
COMMAND_PORTS: Mapping[str, type] = {
    POSITION_COMMAND: JointState,
    VELOCITY_COMMAND: JointState,
    MOTOR_COMMAND: MotorCommandArray,
    CMD_VEL: Twist,
}
#: The kinds of port a coordinator reads a robot's readings from, each with
#: the message it carries. Named ``<robot>_<kind>`` too.
READING_PORTS: Mapping[str, type] = {
    JOINT_STATE: JointState,
    ODOM: PoseStamped,
    BASE_VELOCITY: TwistStamped,
    IMU: Imu,
}

#: The kinds of port that carry joints; a robot with one has joints to read from the model.
_JOINT_PORTS = frozenset({POSITION_COMMAND, VELOCITY_COMMAND, MOTOR_COMMAND, JOINT_STATE})

# Which part of a joint each task control mode drives.
_INTERFACE_BY_MODE: dict[ControlMode, str] = {
    ControlMode.POSITION: Interface.POSITION,
    ControlMode.SERVO_POSITION: Interface.POSITION,
    ControlMode.VELOCITY: Interface.VELOCITY,
    ControlMode.TORQUE: Interface.EFFORT,
}


@dataclass(frozen=True, slots=True)
class PortMessage:
    """One command message and the kind of port it goes out on.

    Attributes:
        port: The port kind, e.g. "cmd_vel"; the coordinator sends it on its
            ``<robot>_cmd_vel`` port.
        message: The message to send on it, e.g. a ``Twist``.
    """

    port: str
    message: Any


@dataclass(frozen=True, slots=True)
class _SensorReading:
    """One number a sensor reports.

    Attributes:
        sensor: The sensor's full name, e.g. "g1/imu".
        reading: What it measures, e.g. "qw".
        key: Its full key, e.g. "g1/imu/qw".
    """

    sensor: str
    reading: str
    key: str


@dataclass(frozen=True, slots=True)
class _Joint:
    """One joint's name, the keys it may report and accept, and its position limit."""

    name: str
    state_keys: dict[str, str]
    command_keys: dict[str, str]
    position_limit: Limits | None


class ConnectionSource:
    """One robot behind a connection module, as the coordinator's tick loop sees it.

    Joints are named by robot and part, e.g. "arm/joint1", the same names
    tasks claim. Each tick the coordinator sends the robot a complete
    command: what tasks asked for, and for every other joint:

    - in a tick where tasks drive this robot by velocity: velocity 0;
    - otherwise: hold its position. That is the last position a task sent,
      or, if none did, where the joint was when the hold began. A held
      position is kept inside the joint's position limit. A joint that
      cannot be held at a position it reports gets velocity 0 instead.

    A base is told 0 on every axis. Sensors are never told anything.

    Commands go out as one message per kind of port: a ``JointState`` on
    each joint port, a ``Twist`` on the base port. What tasks ask for is
    sent as is, even past a limit; the connection module then refuses that
    whole message.

    A robot whose readings stop for longer than ``reading_timeout_s`` is no
    longer commanded; its connection module's deadman halts it. When its
    readings come back, e.g. after the connection module restarted, it is
    held where it then is.

    Does no I/O: the coordinator feeds it messages and publishes what it
    builds.

    Args:
        robot: The robot's name, e.g. "arm": the first part of each of its
            joint names and frames.
        commands: The kinds of port the coordinator sends it commands on,
            from ``COMMAND_PORTS``, e.g. ``["position_command"]``.
        readings: The kinds of port the coordinator reads it from, from
            ``READING_PORTS``, e.g. ``["joint_state"]``.
        model: Where its joints and their position limits come from: the
            model's movable joints named ``<robot>/...``. ``None`` for a
            robot with no joint ports.
        reading_timeout_s: How old its newest reading may be, in seconds,
            before it is no longer commanded.

    Raises:
        ValueError: If the coordinator cannot drive the robot: it has command
            ports but reports nothing, so the coordinator cannot tell it is
            there; it has a motor port, which needs stiffness and damping the
            coordinator has no source for; its joints can neither be told a
            velocity nor held at a position they report (an arm left with
            effort 0 falls under gravity); or it has joint ports but no
            model, or no joints under its name in the model.
    """

    def __init__(
        self,
        robot: str,
        commands: Collection[str],
        readings: Collection[str],
        model: RobotModel | None,
        reading_timeout_s: float,
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
        joints: Mapping[str, Limits | None] = {}
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
        imu = f"{robot}{SEPARATOR}imu"
        self._sensors = (
            [_SensorReading(imu, i, f"{imu}{SEPARATOR}{i}") for i in IMU_INTERFACES]
            if IMU in readings
            else []
        )

        # Each reading port: how to read its messages, and the keys that show
        # it has reported in full, as groups of which one must be present
        # (for joints, any key of each joint).
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

        # Guards the readings and flags below, which readings change and the
        # tick loop reads.
        self._lock = threading.Lock()
        self._latest: Values = {}
        # The ports whose readings are all in _latest.
        self._reported: frozenset[str] = frozenset()
        # The ports that have ever reported in full.
        self._heard: set[str] = set()
        self._received_at = 0.0
        # When the last reading arrived, by time.monotonic().
        self._freshness = Freshness()
        # Readings came back after stopping for longer than the timeout. The
        # tick loop drops the holds when it sees this.
        self._resumed = False
        # Joint name -> the position it is held at. Only the tick loop touches it.
        self._holds: dict[str, float] = {}
        self._last_warned = float("-inf")

    @property
    def robot(self) -> str:
        """The robot's name, e.g. "arm"."""
        return self._robot

    @property
    def joint_names(self) -> list[str]:
        """Every joint, e.g. ["arm/joint1", "arm/joint2"]."""
        return [joint.name for joint in self._joints]

    @property
    def command_ports(self) -> list[str]:
        """The kinds of port commands go out on, e.g. ["position_command", "cmd_vel"]."""
        ports = list(self._joint_ports.values())
        if self._base_port is not None:
            ports.append(self._base_port)
        return ports

    @property
    def reading_ports(self) -> list[str]:
        """The kinds of port readings come in on, e.g. ["joint_state"]."""
        return list(self._readers)

    @property
    def reading_timeout_s(self) -> float:
        """How old the newest reading may be, in seconds, before the robot is no longer commanded."""
        return self._timeout

    def on_message(self, port: str, msg: Any) -> None:
        """Take one reading message from the robot's ``port``, e.g. "joint_state".

        Every message counts: a connection module may send several per tick,
        e.g. one for joints that report effort and one for a gripper that
        does not. Values for other robots, which share a topic with this one,
        are ignored. A message that cannot be read is dropped and logged at
        most once a second; if they keep coming, the robot's readings go
        stale and it is no longer commanded.
        """
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
            if last is not None and now - last > self._timeout:
                # Back after a silence: wait for a whole new reading, and take
                # the holds again from it.
                latest, reported = {}, frozenset()
                self._resumed = True
            # Replaced whole, so the tick loop never sees half of one update.
            latest = {**latest, **mine}
            if port not in reported and all(
                any(key in latest for key in group) for group in self._needed[port]
            ):
                reported = reported | {port}
                self._heard.add(port)
            self._latest, self._reported = latest, reported
            self._received_at = time.time()
            self._freshness.mark(now)

    def ready_for_control(self, now: float | None = None) -> bool:
        """Whether a command can be sent: every reading port has reported (a
        joint port, for every joint) and the newest reading is no older than
        ``reading_timeout_s``.

        Args:
            now: The current ``time.monotonic()``, in seconds. Defaults to now.
        """
        if now is None:
            now = time.monotonic()
        with self._lock:
            return len(self._reported) == len(self._needed) and not self._freshness.is_stale(
                now, self._timeout
            )

    def unreported(self) -> list[str]:
        """The reading ports nothing has ever arrived on, e.g. ["odom"]. A
        joint port counts until every joint has reported."""
        with self._lock:
            return [port for port in self._needed if port not in self._heard]

    def read_joints(self) -> JointStateSnapshot:
        """The joints' latest positions, velocities and efforts.

        A joint that does not report something is left out of that lookup,
        rather than given a zero that looks like a real reading.
        """
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
        """Each sensor's latest readings, e.g. ``{"g1/imu": {"qw": 1.0, ...}}``."""
        latest = self._latest
        readings: dict[str, dict[str, float]] = {}
        for each in self._sensors:
            if each.key in latest:
                readings.setdefault(each.sensor, {})[each.reading] = latest[each.key]
        return readings

    def command(self, winners: Mapping[str, float], mode: ControlMode | None) -> Values:
        """The complete command to send the robot this tick, by key.

        Args:
            winners: What tasks asked for, by joint name, e.g.
                ``{"arm/joint1": 0.3}``. Only this robot's joints.
            mode: How the winners drive their joints. ``None`` when no task
                drives this robot this tick.

        Raises:
            ValueError: If a task drives a joint in a way it does not accept,
                e.g. by velocity when it only takes a position. The robot then
                gets no command this tick.
        """
        with self._lock:
            latest = self._latest
            if self._resumed:
                self._resumed = False
                self._holds.clear()
        interface = _INTERFACE_BY_MODE.get(mode) if mode is not None else None
        values: Values = {}
        for joint in self._joints:
            if joint.name in winners:
                if interface is None or interface not in joint.command_keys:
                    raise ValueError(f"{joint.name} cannot be told its {interface}")
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
        """The messages that carry ``values``, each with the kind of port to send it on.

        One per port: a ``JointState`` on each joint port, of the joints told
        that port's kind of value, and a ``Twist`` on the base port.

        Args:
            values: A command from ``command``.
            ts: When it was made, in Unix seconds.
        """
        messages: list[PortMessage] = []
        for interface, port in self._joint_ports.items():
            joints = [j.name for j in self._joints if f"{j.name}{SEPARATOR}{interface}" in values]
            if joints:
                messages.append(PortMessage(port, joint_state_from_values(values, joints, ts)))
        if self._base_port is not None:
            messages.append(PortMessage(self._base_port, twist_from_values(values, self._base)))
        return messages

    def _hold(self, joint: _Joint, latest: Values) -> float | None:
        """Where to hold ``joint``, inside its position limit, or ``None`` if
        it cannot be told a position or its position is unknown.

        Args:
            joint: The joint to hold.
            latest: The robot's latest reading, to start a new hold from.
        """
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
        if limit is not None:
            if limit.lo is not None:
                held = max(held, limit.lo)
            if limit.hi is not None:
                held = min(held, limit.hi)
        return held


def _robot_joints(robot: str, model: RobotModel) -> dict[str, Limits | None]:
    """The model's movable joints named ``<robot>/...``, each with its position limit.

    Raises:
        ValueError: If there are none, or the model is unclear about a limit.
    """
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


def _reader(
    convert: Callable[[Any, str], Values], part: str, robot: str
) -> Callable[[Any], Values]:
    """``convert(msg, part)`` for a message in one of this robot's frames.

    Raises:
        ValueError: If the message's frame belongs to another robot: a frame
            with a namespace must start with the robot's name ("go2b/odom");
            one without a namespace ("odom") is taken as this robot's.
    """

    def read(msg: Any) -> Values:
        frame = getattr(msg, "frame_id", "")
        if SEPARATOR in frame and not frame.startswith(f"{robot}{SEPARATOR}"):
            raise ValueError(f"frame {frame!r} is another robot's")
        return convert(msg, part)

    return read
