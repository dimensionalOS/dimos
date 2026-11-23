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
standard messages (``JointState``, ``Imu``)
on its ports. The coordinator keeps one ``ConnectionSource`` per robot it
finds among its own ports. It turns each reading into named values with the
shared converters in ``dimos.control.contract.convert``, keeps the latest, and
builds each tick's complete command and the messages that carry it, each
addressed to the kind of port it goes out on.
"""

from __future__ import annotations

from collections.abc import Callable, Collection, Mapping
from dataclasses import dataclass
import math
import threading
import time
from typing import Any

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.convert import (
    IMU_INTERFACES,
    JOINT_STATE_FIELDS,
    imu_to_values,
    joint_state_from_values,
    joint_state_to_values,
)
from dimos.control.contract.keys import SEPARATOR, Interface, Key
from dimos.control.contract.limits import Limits, limits as limits_of, movable_joints
from dimos.control.group_command import (
    GroupCommandHandler,
    GroupCommandLease,
)
from dimos.control.task import JointStateSnapshot
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Values = dict[str, float]

POSITION_COMMAND = "position_command"
VELOCITY_COMMAND = "velocity_command"
MOTOR_COMMAND = "motor_command"
JOINT_STATE = "joint_state"
IMU = "imu"

#: The kinds of port a coordinator sends a robot commands on, each with the
#: message it carries. A coordinator names such a port ``<robot>_<kind>``.
COMMAND_PORTS: Mapping[str, type] = {
    POSITION_COMMAND: JointState,
    VELOCITY_COMMAND: JointState,
    MOTOR_COMMAND: MotorCommandArray,
}
#: The kinds of port a coordinator reads a robot's readings from, each with
#: the message it carries. Named ``<robot>_<kind>`` too.
READING_PORTS: Mapping[str, type] = {
    JOINT_STATE: JointState,
    IMU: Imu,
}

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
        port: The port kind, e.g. "velocity_command"; sent on
            ``<robot>_velocity_command``.
        message: The canonical command message to send.
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
    position_limit: Limits = Limits()


@dataclass(slots=True)
class _GroupOwner:
    task: str
    feedback_epoch: int
    issued: int = 0
    written: int = 0


class ConnectionSource:
    """One robot behind a connection module, as the coordinator's tick loop sees it.

    Each tick the robot gets a complete command: what tasks asked for, and
    for every other joint a hold at its last commanded or reported position,
    kept within its limit, or zero velocity if it cannot take a position.
    A robot whose newest
    reading is older than ``reading_timeout_s`` is not commanded, and is
    held where it is once readings return. Does no I/O.

    Args:
        config: The same model and per-joint interfaces supplied to the connection.
        commands: Port kinds it is commanded on, from ``COMMAND_PORTS``.
        readings: Port kinds it is read from, from ``READING_PORTS``.
        reading_timeout_s: How old its newest reading may be, in seconds.
        zero_stiffness_joints: Joints the connection commands with zero
            stiffness, so tasks such as hand guiding may follow them by hand.

    Raises:
        ValueError: If the coordinator cannot drive it: no reading port, a
            motor port, joints that can neither take a velocity nor be held
            at a position they report, or joint ports without a model.
    """

    def __init__(
        self,
        config: ConnectionConfig,
        commands: Collection[str],
        readings: Collection[str],
        reading_timeout_s: float,
        *,
        zero_stiffness_joints: Collection[str] = (),
        group_handlers: Mapping[str, GroupCommandHandler] | None = None,
        group_lock: threading.RLock | None = None,
        group_feedback_fields: frozenset[Interface] = frozenset(),
    ) -> None:
        robot = config.source
        if commands and not readings:
            raise ValueError(f"{robot!r} reports nothing; it needs a reading port")
        if MOTOR_COMMAND in commands:
            raise ValueError(
                f"{robot!r}: {robot}_{MOTOR_COMMAND} needs stiffness and damping, "
                "which the coordinator cannot send"
            )
        if config.group_commands and (
            JOINT_STATE not in readings
            or not math.isfinite(reading_timeout_s)
            or reading_timeout_s <= 0
        ):
            raise ValueError(f"{robot!r}: groups need joint_state and a finite positive timeout")
        self.group_joints: dict[str, frozenset[str]] = {}
        movable: dict[str, str] = {}
        if config.group_commands:
            if config.model is None:
                raise ValueError(f"{robot!r}: groups need a model")
            movable = {
                j.name: j.name if SEPARATOR in j.name else f"{robot}/{j.name}"
                for j in movable_joints(config.model)
            }
            for name, covered in config.group_commands.items():
                interface = f"{robot}/{name}"
                if not name.strip() or SEPARATOR in name or not covered or covered - movable.keys():
                    raise ValueError(
                        f"{interface!r}: expected a local name and movable model joints"
                    )
                joints = frozenset(movable[j] for j in covered)
                if any(not j.startswith(f"{robot}/") for j in joints):
                    raise ValueError(f"{interface!r}: covered joint belongs to another source")
                self.group_joints[interface] = joints
        if not group_feedback_fields <= frozenset(JOINT_STATE_FIELDS):
            raise ValueError("group feedback fields must be joint state fields")
        self._group_feedback_keys = {
            name: (
                frozenset(f"{joint}/{field}" for joint in joints for field in group_feedback_fields)
                if group_feedback_fields
                else joints
            )
            for name, joints in self.group_joints.items()
        }
        self._group_feedback_fields = group_feedback_fields
        self._group_handlers = dict(group_handlers or {})
        if self._group_handlers.keys() != self.group_joints.keys():
            raise ValueError(f"{robot!r}: bind one local handler per group")
        # Share the coordinator write/stop boundary. A local handler can
        # use its lease synchronously while the tick holds this lock.
        self._group_lock = group_lock if group_lock is not None else threading.RLock()
        self._group_owners: dict[str, _GroupOwner] = {}
        self._reading_epoch = 0
        self._group_joint_by_key: dict[str, str] = {}
        self._joint_receipts: dict[str, tuple[float, int]] = {}

        table = limits_of(config)
        self._robot = robot
        self._timeout = reading_timeout_s
        self._limits = table

        self._joint_ports = {
            interface: port
            for interface, port in (
                (Interface.POSITION, POSITION_COMMAND),
                (Interface.VELOCITY, VELOCITY_COMMAND),
            )
            if port in commands
        }

        keys: dict[str, dict[str, str]] = {}
        for raw_key in table:
            key = Key(raw_key)
            keys.setdefault(key.joint, {})[key.interface] = key
        if self.group_joints:
            covered = set().union(*self.group_joints.values())
            for name in movable.values():
                if name in covered:
                    keys.setdefault(name, {})
                    self._group_joint_by_key.update(
                        (f"{name}/{field}", f"{name}/{field}" if group_feedback_fields else name)
                        for field in group_feedback_fields or JOINT_STATE_FIELDS
                    )
        self._joints = []
        for name, command_keys in keys.items():
            for interface in command_keys:
                if interface not in self._joint_ports:
                    raise ValueError(f"{name}: declared {interface} has no command transport")
            if (
                command_keys
                and Interface.VELOCITY not in command_keys
                and not (Interface.POSITION in command_keys and JOINT_STATE in readings)
            ):
                raise ValueError(f"{name}: cannot be held; needs velocity or reported position")
            self._joints.append(
                _Joint(
                    name=name,
                    state_keys={i: f"{name}/{i}" for i in JOINT_STATE_FIELDS},
                    command_keys=command_keys,
                    position_limit=table.get(f"{name}/position", Limits()),
                )
            )
        unknown = frozenset(zero_stiffness_joints) - set(keys)
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

        # Each reading port: how to read its messages, and the keys that show
        # it has reported in full, as groups of which one must be present
        # (for joints, any key of each joint).
        self._readers: dict[str, Callable[[Any], Values]] = {}
        self._needed: dict[str, list[tuple[str, ...]]] = {}
        self._port_keys: dict[str, frozenset[str]] = {}
        for port in readings:
            if port == JOINT_STATE:
                self._readers[port] = _read_joint_state
                groups = [tuple(joint.state_keys.values()) for joint in self._joints]
            elif port == IMU:
                self._readers[port] = _reader(imu_to_values, imu, robot)
                groups = [(sensor.key,) for sensor in self._sensors]
            else:
                raise ValueError(
                    f"{robot!r}: {port!r} is not a kind of reading port; "
                    f"one of {', '.join(READING_PORTS)}"
                )
            self._needed[port] = groups
            self._port_keys[port] = frozenset(key for group in groups for key in group)

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
        self._port_receipts: dict[str, float] = {}
        # Readings came back after stopping for longer than the timeout, or
        # after drop_holds. The tick loop drops the holds when it sees this.
        self._resumed = False
        # Set by drop_holds: the next reading starts afresh, as after a silence.
        self._restart = False
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
    def joint_command_interfaces(self) -> dict[str, frozenset[Interface]]:
        """Declared command interfaces, independent of feedback freshness."""
        return {
            joint.name: frozenset(Interface(key) for key in joint.command_keys)
            for joint in self._joints
        }

    def position_limit(self, joint_name: str) -> Limits:
        """The declared position-command bounds for one fully qualified joint.

        Raises ValueError if the joint does not accept position commands.
        The returned limit is immutable and independent of feedback freshness.
        """
        key = f"{joint_name}/{Interface.POSITION}"
        if key not in self._limits:
            raise ValueError(f"{joint_name!r} has no declared position command")
        return self._limits[key]

    def supports_joint_mode(self, joint: str, mode: ControlMode) -> bool:
        """Whether the joint has a legacy interface for this mode."""
        return f"{joint}/{_INTERFACE_BY_MODE.get(mode)}" in self._limits

    @property
    def command_ports(self) -> list[str]:
        """The kinds of port commands go out on, e.g. ["position_command"]."""
        return list(self._joint_ports.values())

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
            if any(not math.isfinite(value) for value in values.values()):
                raise ValueError("reading contains non-finite values")
        except ValueError as error:
            now = time.monotonic()
            if now - self._last_warned >= 1.0:
                self._last_warned = now
                logger.warning(f"Dropped a {port} message for {self._robot!r}: {error}")
            return
        mine = {key: value for key, value in values.items() if key in self._port_keys[port]}
        if not mine:
            return
        with self._lock:
            now = time.monotonic()
            latest, reported = self._latest, self._reported
            if self._restart or any(
                now - receipt > self._timeout for receipt in self._port_receipts.values()
            ):
                # Back after a silence, or holds dropped: wait for a whole new
                # reading, and take the holds again from it.
                latest, reported = {}, frozenset()
                self._port_receipts.clear()
                self._reading_epoch += 1 + sum(gaps for _, gaps in self._joint_receipts.values())
                self._joint_receipts.clear()
                self._resumed = True
                self._restart = False
            if port == JOINT_STATE and self._group_joint_by_key:
                updated = {
                    self._group_joint_by_key[k]
                    for k in mine.keys() & self._group_joint_by_key.keys()
                }
                for joint in updated:
                    previous, epoch = self._joint_receipts.get(joint, (now, 0))
                    self._joint_receipts[joint] = now, epoch + (now - previous > self._timeout)
            # Replaced whole, so the tick loop never sees half of one update.
            latest = {**latest, **mine}
            if port not in reported and all(
                any(key in latest for key in group) for group in self._needed[port]
            ):
                reported = reported | {port}
                self._heard.add(port)
            self._latest, self._reported = latest, reported
            self._port_receipts[port] = now
            self._received_at = time.time()

    def drop_holds(self) -> None:
        """Send nothing until the robot reports again, then hold each joint
        where that new reading has it, not where it was last told to be."""
        with self._lock:
            self._reported = frozenset()
            self._restart = True
            self._reading_epoch += 1

    def retain_group_owners(self, winners: Mapping[str, str]) -> None:
        for interface, owner in list(self._group_owners.items()):
            if winners.get(interface) != owner.task:
                del self._group_owners[interface]

    def revoke_group_owners(self, task: str | None) -> None:
        for interface, owner in list(self._group_owners.items()):
            if task is None or owner.task == task:
                del self._group_owners[interface]

    def group_feedback_epoch(self, interface: str) -> int | None:
        """Return the group's continuity token only while its feedback is fresh."""
        with self._lock:
            return self._group_feedback_epoch(interface, time.monotonic())

    def group_feedback(self, interface: str) -> tuple[int, float, Values] | None:
        """Snapshot the required state fields with their oldest receipt."""
        if not self._group_feedback_fields:
            raise ValueError("group feedback fields were not declared")
        with self._lock:
            stamp = self._group_feedback_stamp(interface, time.monotonic())
            if stamp is None:
                return None
            epoch, oldest = stamp
            return epoch, oldest, {k: self._latest[k] for k in self._group_feedback_keys[interface]}

    def _group_feedback_epoch(self, interface: str, now: float) -> int | None:
        stamp = self._group_feedback_stamp(interface, now)
        return stamp[0] if stamp is not None else None

    def _group_feedback_stamp(self, interface: str, now: float) -> tuple[int, float] | None:
        if not self._ready(now):
            return None
        epoch, oldest = self._reading_epoch, now
        for key in self._group_feedback_keys[interface]:
            received, gaps = self._joint_receipts.get(key, (float("-inf"), 0))
            if now - received > self._timeout:
                return None
            epoch += gaps
            oldest = min(oldest, received)
        return epoch, oldest

    def route_group(self, interface: str, task: str, payload: object, feedback_epoch: int) -> None:
        """Offer a winning payload while the caller holds the shared write/stop lock."""
        with self._lock:
            now = time.monotonic()
            if feedback_epoch != self._group_feedback_epoch(interface, now):
                self._group_owners.pop(interface, None)
                return
            owner = self._group_owners.get(interface)
            if owner is None or owner.task != task or owner.feedback_epoch != feedback_epoch:
                owner = self._group_owners[interface] = _GroupOwner(task, feedback_epoch)
            expires_at = now + self._timeout
            owner.issued += 1
            sequence = owner.issued
            # Ownership changed independently of whether the legacy batch
            # on other joints passes validation this tick.
            for joint in self.group_joints[interface]:
                self._holds.pop(joint, None)

        def guard(write: Callable[[], None]) -> bool:
            with self._group_lock:
                with self._lock:
                    now = time.monotonic()
                    if (
                        self._group_owners.get(interface) is not owner
                        or sequence <= owner.written
                        or now >= expires_at
                        or owner.feedback_epoch != self._group_feedback_epoch(interface, now)
                    ):
                        return False
                    owner.written = sequence
                write()
                return True

        try:
            self._group_handlers[interface](payload, GroupCommandLease(expires_at, guard))
        except Exception:
            self.revoke_group_owners(task)
            raise

    def ready_for_control(self, now: float | None = None) -> bool:
        """Whether every required reading port has reported and is fresh.

        Joint ports need a reading for each declared joint; traffic on one
        port cannot refresh another port's cached state.

        Args:
            now: The current ``time.monotonic()``, in seconds. Defaults to now.
        """
        if now is None:
            now = time.monotonic()
        with self._lock:
            return self._ready(now)

    def _ready(self, now: float) -> bool:
        return len(self._reported) == len(self._needed) and all(
            now - self._port_receipts[port] <= self._timeout for port in self._needed
        )

    def unreported(self) -> list[str]:
        """The reading ports nothing has ever arrived on, e.g. ["joint_state"]. A
        joint port counts until every joint has reported."""
        with self._lock:
            return [port for port in self._needed if port not in self._heard]

    def read_joints(self) -> JointStateSnapshot:
        """The joints' latest positions, velocities and efforts.

        A joint that does not report something is left out of that lookup,
        rather than given a zero that looks like a real reading.
        """
        latest = self._latest
        snapshot = JointStateSnapshot(
            timestamp=self._received_at,
            joint_command_interfaces=self.joint_command_interfaces,
        )
        for joint in self._joints:
            for interface, into in (
                (Interface.POSITION, snapshot.joint_positions),
                (Interface.VELOCITY, snapshot.joint_velocities),
                (Interface.EFFORT, snapshot.joint_efforts),
            ):
                key = joint.state_keys.get(interface)
                if key is not None and key in latest:
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

    def command(
        self,
        winners: Mapping[str, tuple[float, ControlMode]],
        *,
        group_joints: Collection[str] = (),
    ) -> Values:
        """The complete command to send the robot this tick, by key.

        Args:
            winners: Each winning joint's value and control mode, e.g.
                ``{"arm/joint1": (0.3, ControlMode.POSITION)}``. Each interface
                must be authorized by the connection declaration.
            group_joints: Joints owned by group winners; never held or sent here.

        Raises:
            ValueError: If a task drives a joint in a way it does not accept,
                e.g. by velocity when it only takes a position, tells it a
                position outside its limit, or tells a joint a non-finite
                speed. The robot then gets no command this tick and its holds
                are left as they were.
        """
        interfaces = {name: _INTERFACE_BY_MODE.get(mode) for name, (_, mode) in winners.items()}
        # Check every task value first: the connection module refuses the
        # whole message if one is out of range, so no hold may change unless
        # all of them are sendable.
        joints = {joint.name: joint for joint in self._joints}
        for name, (value, _) in winners.items():
            if name not in joints:
                raise ValueError(f"Unknown joint {name}")
            joint = joints[name]
            interface = interfaces[name]
            if interface is None or interface not in joint.command_keys:
                raise ValueError(f"{joint.name} cannot be told its {interface}")
            if not math.isfinite(value):
                raise ValueError(f"{name} told {value}, not a finite speed")
            limit = self._limits[joint.command_keys[interface]]
            if not limit.clamp and not _within(value, limit):
                raise ValueError(f"{joint.name} told {value}, outside {limit.lo}..{limit.hi}")
        with self._lock:
            latest = self._latest
            if self._resumed:
                self._holds.clear()
            self._resumed = False
        values: Values = {}
        for joint in self._joints:
            if joint.name in group_joints:
                continue
            if joint.name in winners:
                interface = interfaces[joint.name]
                assert interface is not None
                value, _ = winners[joint.name]
                limit = self._limits[joint.command_keys[interface]]
                if limit.clamp:
                    assert limit.lo is not None and limit.hi is not None
                    value = max(limit.lo, min(value, limit.hi))
                values[joint.command_keys[interface]] = value
                if interface == Interface.POSITION:
                    self._holds[joint.name] = value
                else:
                    self._holds.pop(joint.name, None)
                continue
            velocity_key = joint.command_keys.get(Interface.VELOCITY)
            hold = self._hold(joint, latest)
            if hold is not None:
                values[joint.command_keys[Interface.POSITION]] = hold
            elif velocity_key is not None:
                values[velocity_key] = 0.0
        return values

    def command_messages(self, values: Mapping[str, float], ts: float) -> list[PortMessage]:
        """The messages that carry ``values``, each with the kind of port to send it on.

        One per port: a ``JointState`` on each joint port, of the joints told
        that port's kind of value.

        Args:
            values: A command from ``command``.
            ts: When it was made, in Unix seconds.
        """
        messages: list[PortMessage] = []
        for interface, port in self._joint_ports.items():
            joints = [
                j.name
                for j in self._joints
                if j.command_keys.get(interface) == f"{j.name}{SEPARATOR}{interface}"
                and j.command_keys[interface] in values
            ]
            if joints:
                messages.append(PortMessage(port, joint_state_from_values(values, joints, ts)))
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
        if limit.lo is not None:
            held = max(held, limit.lo)
        if limit.hi is not None:
            held = min(held, limit.hi)
        return held


def _read_joint_state(msg: Any) -> Values:
    return joint_state_to_values(msg, JOINT_STATE_FIELDS)


def _within(value: float, limit: Limits) -> bool:
    """Whether ``value`` is a finite number inside ``limit``'s bounds."""
    return (
        math.isfinite(value)
        and (limit.lo is None or value >= limit.lo)
        and (limit.hi is None or value <= limit.hi)
    )


def _reader(
    convert: Callable[[Any, Any], Values], part: Any, robot: str
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
