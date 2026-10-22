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
standard messages (``JointState``, ``Twist``, ``PoseStamped``, ``Imu``) on its
own ports. The coordinator keeps one ``ConnectionSource`` per robot. It says
which ports the robot needs, turns each reading into named values with the
shared converters in ``dimos.control.contract.convert``, keeps the latest, and
builds each tick's complete command and the messages that carry it.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
import math
import threading
import time
from typing import Any

from dimos.control.connection.connection_module import ConnectionDescription
from dimos.control.contract.convert import (
    IMU_INTERFACES,
    JOINT_STATE_FIELDS,
    POSE_INTERFACES,
    TWIST_INTERFACES,
    imu_to_values,
    joint_state_from_values,
    joint_state_to_values,
    motor_command_from_values,
    motor_joints,
    pose_to_values,
    twist_from_values,
    twist_to_values,
)
from dimos.control.contract.description import ControlDescription, Limits, ResourceKind
from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.control.contract.ports import JointPort, MotorPort, PortSpec, TwistPort
from dimos.control.task import JointStateSnapshot
from dimos.hardware.manipulators.spec import ControlMode
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Values = dict[str, float]

# Which part of a joint each task control mode drives.
_INTERFACE_BY_MODE: dict[ControlMode, str] = {
    ControlMode.POSITION: Interface.POSITION,
    ControlMode.SERVO_POSITION: Interface.POSITION,
    ControlMode.VELOCITY: Interface.VELOCITY,
    ControlMode.TORQUE: Interface.EFFORT,
}


@dataclass(frozen=True, slots=True)
class _Joint:
    """One joint's name and the keys it reports and accepts."""

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

    Commands go out as one message per kind: a position ``JointState``, a
    velocity ``JointState``, a ``Twist`` for the base. What tasks ask for is
    sent as is, even past a limit; the connection module then refuses that
    whole message.

    A robot whose readings stop for longer than its deadman timeout is no
    longer commanded; its connection's deadman halts it. When its readings
    come back, it is held where it then is.

    PD joints require explicit gains from the coordinator configuration. Their
    commands carry all five motor fields together, including while holding.
    When a joint has both a motor and a simpler joint port, the motor port owns
    it for this coordinator; it is never commanded twice in the same tick.

    Does no I/O: the coordinator feeds it messages and publishes what it
    builds.

    Args:
        contract: The robot description, resolved ports and session returned by
            its connection module's ``describe_control``.
        motor_gains: Explicit stiffness and damping in the motor port's order.

    Raises:
        ValueError: If a joint can neither be told a velocity nor held at a
            position it reports (an arm left with effort 0 falls under
            gravity), or takes stiffness and damping without configured gains.
    """

    def __init__(
        self, contract: ConnectionDescription, *, motor_gains: WholeBodyConfig | None = None
    ) -> None:
        description = contract.description
        unholdable = [
            r.name
            for r in description.resources
            if r.kind is ResourceKind.JOINT
            and r.command_interfaces
            and Interface.VELOCITY not in r.command_interfaces
            and not (
                Interface.POSITION in r.command_interfaces
                and Interface.POSITION in r.state_interfaces
            )
        ]
        if unholdable:
            raise ValueError(
                f"{description.source!r}: joint(s) {unholdable} cannot be held when no task "
                "drives them; each needs to take a velocity, or take a position it reports"
            )
        self._motor_joints = motor_joints([description])
        self._motor_gains: dict[str, tuple[float, float]] = {}
        if self._motor_joints:
            if motor_gains is None or motor_gains.kp is None or motor_gains.kd is None:
                raise ValueError(
                    f"{description.source!r}: stiffness and damping must be configured"
                )
            if len(motor_gains.kp) != len(self._motor_joints) or len(motor_gains.kd) != len(
                self._motor_joints
            ):
                raise ValueError(f"{description.source!r}: gains must match the PD joint order")
            if any(not math.isfinite(v) or v < 0 for v in (*motor_gains.kp, *motor_gains.kd)):
                raise ValueError(f"{description.source!r}: gains must be finite and nonnegative")
            self._motor_gains = {
                name: (kp, kd)
                for name, kp, kd in zip(
                    self._motor_joints, motor_gains.kp, motor_gains.kd, strict=True
                )
            }
        self.contract = contract
        self._description = description
        self._session_id = contract.session_id
        self._state_keys = frozenset(description.state_keys())
        self._joints: list[_Joint] = []
        self._sensors: list[tuple[str, str, str]] = []
        self._readers: dict[str, Callable[[Any], Values]] = {}
        self._base: str | None = None
        reported = contract.reading_ports
        for resource in description.resources:
            part = f"{description.source}{SEPARATOR}{resource.name}"
            state, command = set(resource.state_interfaces), set(resource.command_interfaces)
            if resource.kind is ResourceKind.JOINT:
                self._joints.append(
                    _Joint(
                        name=part,
                        state_keys={i: f"{part}{SEPARATOR}{i}" for i in resource.state_interfaces},
                        command_keys={
                            i: f"{part}{SEPARATOR}{i}" for i in resource.command_interfaces
                        },
                        position_limit=description.limits.get(
                            f"{part}{SEPARATOR}{Interface.POSITION}"
                        ),
                    )
                )
                if state:
                    self._readers[reported[JointState]] = _read_joint_state
            elif resource.kind is ResourceKind.BASE:
                if set(TWIST_INTERFACES) <= command:
                    self._base = part
                if set(POSE_INTERFACES) <= state:
                    self._readers[reported[PoseStamped]] = _reader(pose_to_values, part)
                if set(TWIST_INTERFACES) <= state:
                    self._readers[reported[Twist]] = _reader(twist_to_values, part)
            else:
                self._sensors += [(part, i, f"{part}{SEPARATOR}{i}") for i in sorted(state)]
                if set(IMU_INTERFACES) <= state:
                    self._readers[reported[Imu]] = _reader(imu_to_values, part)

        self._commands = self._plan_commands(contract.command_ports)

        # Guards the reading and the flags below, which readings change and
        # the tick loop reads.
        self._lock = threading.Lock()
        self._latest: Values = {}
        self._key_received: dict[str, float] = {}
        self._received_at = 0.0
        # The holds are out of date: readings came back after stopping for
        # longer than the deadman timeout, or drop_holds was called. The tick
        # loop drops the holds when it sees this.
        self._drop_holds = False
        # Described again, or holds dropped: send nothing until the robot
        # reports again.
        self._awaiting_reading = False
        # Joint name -> the position it is held at. Only the tick loop touches it.
        self._holds: dict[str, float] = {}
        self._last_warned = float("-inf")

    @property
    def hardware_id(self) -> str:
        """The robot's name, e.g. "arm"."""
        return self._description.source

    @property
    def description(self) -> ControlDescription:
        """What the robot reports and accepts."""
        return self._description

    @property
    def session_id(self) -> str:
        """The connection module's session, new each time it starts."""
        return self._session_id

    @property
    def joint_names(self) -> list[str]:
        """Every joint, e.g. ["arm/joint1", "arm/joint2"]."""
        return [joint.name for joint in self._joints]

    @property
    def command_ports(self) -> dict[str, type]:
        """The actual command port names and message types the coordinator uses."""
        return {name: spec.message for name, spec in self._commands.items()}

    @property
    def reading_ports(self) -> dict[str, type]:
        """The actual reading port names and message types."""
        return {name: message for message, name in self.contract.reading_ports.items()}

    def _plan_commands(self, specs: dict[str, PortSpec]) -> dict[str, PortSpec]:
        commands: dict[str, PortSpec] = {}
        routes: dict[tuple[str, Interface], str] = {}
        for name, spec in specs.items():
            if isinstance(spec, JointPort):
                assert spec.joints is not None  # Resolved by describe_control.
                joints = [*spec.joints, *([spec.gripper.name] if spec.gripper else [])]
                joints = [
                    j
                    for j in joints
                    if f"{self.hardware_id}{SEPARATOR}{j}" not in self._motor_gains
                ]
                if not joints:
                    continue
                for joint in joints:
                    key = (joint, spec.interface)
                    if key in routes:
                        raise ValueError(
                            f"{self.hardware_id!r}: {routes[key]!r} and {name!r} both command "
                            f"{joint}/{spec.interface}"
                        )
                    routes[key] = name
                spec = spec.model_copy(update={"joints": joints, "gripper": None})
            commands[name] = spec
        return commands

    def on_message(self, port: str, msg: Any) -> None:
        """Take one reading message from the robot's ``port``, e.g. "joint_state".

        Every message counts: a driver may send several per tick, e.g. one for
        joints that report effort and one for a gripper that does not. Values
        for other robots, which share a topic with this one, are ignored. A
        message that cannot be read is dropped and logged at most once a
        second; if they keep coming, the robot's readings go stale and it is
        no longer commanded.
        """
        try:
            values = self._readers[port](msg)
        except ValueError as error:
            now = time.monotonic()
            if now - self._last_warned >= 1.0:
                self._last_warned = now
                logger.warning(f"Dropped a {port} message for {self.hardware_id!r}: {error}")
            return
        mine = {key: value for key, value in values.items() if key in self._state_keys}
        if not mine:
            return
        now = time.monotonic()
        source_ts = float(getattr(msg, "ts", time.time()))
        age = time.time() - source_ts
        if not math.isfinite(age) or age < -self._description.deadman_timeout_s:
            return
        with self._lock:
            latest = self._latest
            if any(
                now - self._key_received[key] > self._description.deadman_timeout_s
                for key in mine
                if key in self._key_received
            ):
                # Back after a silence: wait for a whole new reading, and take
                # the holds again from it.
                latest = {}
                self._key_received = {}
                self._drop_holds = True
            # Replaced whole, so the tick loop never sees half of one update.
            self._latest = {**latest, **mine}
            self._key_received.update((key, now - max(0.0, age)) for key in mine)
            self._received_at = source_ts
            self._awaiting_reading = False

    def adopt(self, other: ConnectionSource) -> None:
        """Carry on from ``other``, an earlier record of the same robot from the
        same session with the same description: its reading and its holds.

        Commands wait for the robot's next reading.
        """
        with other._lock:
            latest, received_at = other._latest, other._received_at
            key_received = dict(other._key_received)
            drop_holds = other._drop_holds
        with self._lock:
            self._latest, self._received_at = latest, received_at
            self._key_received = key_received
            self._drop_holds = drop_holds
            self._awaiting_reading = True
        self._holds = dict(other._holds)

    def drop_holds(self) -> None:
        """Forget every held position, and send nothing until the robot reports
        again. The next command then holds each joint where that new reading
        has it, not where it was last told to be."""
        with self._lock:
            self._drop_holds = True
            self._awaiting_reading = True
            self._latest = {}
            self._key_received = {}

    def ready_for_control(self, now: float | None = None) -> bool:
        """Whether a command can be sent: everything the robot reports has
        arrived, it has reported since it was last described, and its last
        reading is no older than its deadman timeout.

        Args:
            now: The current ``time.monotonic()``, in seconds. Defaults to now.
        """
        if now is None:
            now = time.monotonic()
        with self._lock:
            return (
                not self._awaiting_reading
                and len(self._latest) == len(self._state_keys)
                and bool(self._key_received)
                and now - min(self._key_received.values()) <= self._description.deadman_timeout_s
            )

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
                key = joint.state_keys.get(interface)
                if key is not None and key in latest:
                    into[joint.name] = latest[key]
        return snapshot

    def read_sensors(self) -> dict[str, dict[str, float]]:
        """Each sensor's latest readings, e.g. ``{"g1/imu": {"qw": 1.0, ...}}``."""
        latest = self._latest
        readings: dict[str, dict[str, float]] = {}
        for sensor, reading, key in self._sensors:
            if key in latest:
                readings.setdefault(sensor, {})[reading] = latest[key]
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
            if self._drop_holds:
                self._drop_holds = False
                self._holds.clear()
        interface = _INTERFACE_BY_MODE.get(mode) if mode is not None else None
        values: Values = {}
        for joint in self._joints:
            if joint.name in self._motor_gains:
                kp, kd = self._motor_gains[joint.name]
                target = self._hold(joint, latest)
                if target is None:
                    raise ValueError(f"{joint.name}: cannot command PD before position feedback")
                velocity = effort = 0.0
                if joint.name in winners:
                    value = winners[joint.name]
                    if interface == Interface.POSITION:
                        target = value
                        self._holds[joint.name] = target
                    elif interface == Interface.VELOCITY:
                        kp, velocity = 0.0, value
                        self._holds.pop(joint.name, None)
                    elif interface == Interface.EFFORT:
                        kp, kd, effort = 0.0, 0.0, value
                        self._holds.pop(joint.name, None)
                    else:
                        raise ValueError(f"{joint.name} cannot be told its {interface}")
                values.update(
                    zip(
                        (
                            joint.command_keys[i]
                            for i in (
                                Interface.POSITION,
                                Interface.VELOCITY,
                                Interface.EFFORT,
                                Interface.KP,
                                Interface.KD,
                            )
                        ),
                        (target, velocity, effort, kp, kd),
                        strict=True,
                    )
                )
                continue
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
        if self._base is not None:
            values.update((f"{self._base}{SEPARATOR}{i}", 0.0) for i in TWIST_INTERFACES)
        return values

    def command_messages(self, values: Mapping[str, float], ts: float) -> list[tuple[str, Any]]:
        """The messages that carry ``values``, each with the driver's port name.

        One per kind: a position ``JointState`` for the joints told a position,
        a velocity ``JointState`` for those told a velocity, a ``Twist`` for
        the base.

        Args:
            values: A command from ``command``.
            ts: When it was made, in Unix seconds.
        """
        messages: list[tuple[str, Any]] = []
        for port, spec in self._commands.items():
            if isinstance(spec, JointPort):
                assert spec.joints is not None
                joints = [f"{self.hardware_id}{SEPARATOR}{j}" for j in spec.joints]
                joints = [
                    joint for joint in joints if f"{joint}{SEPARATOR}{spec.interface}" in values
                ]
                selected = {
                    f"{joint}{SEPARATOR}{spec.interface}": values[
                        f"{joint}{SEPARATOR}{spec.interface}"
                    ]
                    for joint in joints
                }
                if selected:
                    messages.append((port, joint_state_from_values(selected, joints, ts)))
            elif isinstance(spec, MotorPort):
                messages.append((port, motor_command_from_values(values, self._motor_joints, ts)))
            elif isinstance(spec, TwistPort):
                assert self._base is not None
                messages.append((port, twist_from_values(values, self._base)))
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
            measured_key = joint.state_keys.get(Interface.POSITION)
            if measured_key is None or measured_key not in latest:
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


def _read_joint_state(msg: Any) -> Values:
    return joint_state_to_values(msg, JOINT_STATE_FIELDS)


def _reader(convert: Callable[[Any, str], Values], part: str) -> Callable[[Any], Values]:
    """``convert(msg, part)`` as a function of the message alone."""
    return lambda msg: convert(msg, part)
