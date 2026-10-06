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

"""The base every robot driver is built on: one module per robot.

A driver subclasses ``ConnectionModule``, declares the standard ports it
uses, and fills in five methods for its hardware. The base turns incoming
messages into checked named values for ``write``, publishes the driver's
readings as messages, and halts the robot when commands stop coming. Anything
that publishes the standard messages can drive it; it does not need a
coordinator.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import asyncio
from collections.abc import Callable, Mapping
from dataclasses import dataclass
import threading
import time
from typing import Any, ClassVar
import uuid

from reactivex import operators as ops
from reactivex.abc import DisposableBase

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.contract.convert import (
    COMMAND_PORTS,
    IMU_INTERFACES,
    JOINT_STATE_FIELDS,
    MOTOR_INTERFACES,
    POSE_INTERFACES,
    STATE_PORTS,
    TWIST_INTERFACES,
    imu_from_values,
    joint_state_from_values,
    joint_state_to_values,
    motor_command_to_values,
    motor_joints,
    pose_from_values,
    twist_from_values,
    twist_to_values,
)
from dimos.control.contract.description import ControlDescription, ResourceKind
from dimos.control.contract.keys import POSITION, SEPARATOR, VELOCITY
from dimos.control.contract.validate import (
    FrameRejectedError,
    Rejected,
    validate_command,
    validate_description,
    validate_state,
)
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Values = dict[str, float]
#: One subscription to make: the input port, its converter and the
#: converter's second argument, and the robot whose values to keep.
_Listener = tuple[In[Any], Callable[[Any, Any], Values], Any, str]


@dataclass(frozen=True, slots=True)
class ConnectionDescription:
    """What ``ConnectionModule.describe_control`` returns.

    Attributes:
        session_id: New each time the module starts, so a restarted driver
            can be told apart from one that kept running. Empty before start.
        descriptions: One per robot the module runs.
    """

    session_id: str
    descriptions: tuple[ControlDescription, ...]


@dataclass(frozen=True, slots=True)
class ConnectionStatus:
    """A snapshot of one connection, returned by ``ConnectionModule.status``.

    Attributes:
        connected: ``connect`` succeeded and the module has not stopped.
        last_state_time: When readings were last published, in Unix seconds,
            or ``None`` if never.
        last_command_time: When a command was last accepted, in Unix seconds,
            or ``None`` if never.
        deadman_fired: Commands stopped arriving, the robot was halted, and no
            command has been accepted since.
        last_rejection: Why the last refused command was refused, or ``None``.
        last_error: The last failure in the driver (``write``, ``read_state``,
            ``halt``, or a reading that does not match the description), or
            ``None``.
    """

    connected: bool
    last_state_time: float | None
    last_command_time: float | None
    deadman_fired: bool
    last_rejection: str | None
    last_error: str | None


class ConnectionModule(Module, ABC):
    """A robot driver. Subclass it, declare its ports, fill in five methods.

    Ports: a driver declares the standard ports it uses, as on any module::

        position_command: In[JointState]
        joint_state: Out[JointState]

    The base finds them by name (``COMMAND_PORTS`` and ``STATE_PORTS`` in
    ``dimos.control.contract.convert``) and ignores every other port. A
    JointState names its joints in full ("arm/joint1"). A Twist, PoseStamped
    or Imu has no names, so it is for the one base or IMU the descriptions
    have; a MotorCommandArray lists the joints ``motor_joints`` gives, in that
    order. Start fails if a description names a command no declared input
    carries, or a reading no declared output carries.

    Starting it calls ``describe``, checks each description (raising on the
    first bad one, before touching the hardware), then calls ``connect``.

    Readings: a thread calls ``read_state`` at the fastest ``state_rate_hz``
    among the descriptions and publishes the result on the output ports.

    Commands: each message is turned into named values, and values for robots
    this module does not run are ignored. Each input keeps just its newest
    message per robot, so if ``write`` is slower than commands arrive, the
    ones in between are skipped. Values are checked with ``validate_command``
    (unknown keys, limits) before ``write`` sees them.

    Deadman: once a command has been accepted, if no other is accepted within
    the shortest ``deadman_timeout_s`` among the descriptions, ``halt`` is
    called once. The next accepted command starts it again.

    ``write`` and ``halt`` never run at the same time. Commands and the
    deadman share the module's event loop, so a ``write`` that hangs stops
    both: the deadman cannot fire and no later command is handled until it
    returns.

    To close the hardware connection when the module stops, override ``stop``
    and close it after calling ``super().stop()``, which halts the robot first.
    """

    dedicated_worker: ClassVar[bool] = True

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._session_id = ""
        self._descriptions: dict[str, ControlDescription] = {}
        self._deadman_s = 0.0
        # Guards the status fields and the stop flag.
        self._lock = threading.Lock()
        # Held around every write() and halt(), so they never overlap, and
        # around every change to _connected.
        self._hardware_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._stopping = False
        self._connected = False
        self._subscriptions: list[DisposableBase] = []
        # Per output: the robot it reports on, the port, and how to build a
        # message: build(values, target, unix_time) for target joints or part.
        self._outputs: list[tuple[str, Out[Any], Callable[[Values, Any, float], Any], Any]] = []
        self._state_thread: threading.Thread | None = None
        # Only touched on the module's event loop, where commands are handled.
        self._deadman_timer: asyncio.TimerHandle | None = None
        self._last_state_time: float | None = None
        self._last_command_time: float | None = None
        self._deadman_fired = False
        self._last_rejection: str | None = None
        self._last_error: str | None = None
        # When a rejection (True) or an error (False) was last logged.
        self._last_logged: dict[bool, float] = {}

    @abstractmethod
    def describe(self) -> ControlDescription | list[ControlDescription]:
        """Say what the robot is, usually with one preset from
        ``dimos.control.contract.presets`` built from ``self.config``.

        Runs before ``connect``, so it must not touch the hardware. Return a
        list when this module runs several robots, such as an arm and the base
        under it, each with its own ``source`` name.
        """

    @abstractmethod
    def connect(self) -> None:
        """Open the connection to the hardware. Must not make anything move."""

    @abstractmethod
    def read_state(self) -> Values | None:
        """The robot's latest readings, or ``None`` if nothing new has arrived.

        Keys are full keys such as ``"arm/joint1/position"``, each value in
        its key's unit, and every key a description lists must be present. A
        driver whose SDK delivers readings in a callback returns ``None`` here
        and calls ``publish_state`` from the callback instead.
        """

    @abstractmethod
    def write(self, values: Values) -> None:
        """Send one checked command to the hardware.

        Send and return, servo-style: never wait for a motion to finish; its
        progress shows up in the readings. Switching the hardware between ways
        of being driven (position, velocity) happens here, and any waiting a
        mode switch needs is the driver's own business. Raise if the hardware
        refuses: the module then logs it, calls ``halt`` and carries on.

        Args:
            values: The keys of one robot only, e.g.
                ``{"arm/joint1/position": 0.3}``, already within its limits.
        """

    @abstractmethod
    def halt(self) -> None:
        """Stop the robot moving now, e.g. hold its position or zero its speeds.

        Must be safe to call any number of times, including before any
        command. Called when commands stop arriving, after a failed ``write``,
        and when the module stops.
        """

    @rpc
    def start(self) -> None:
        super().start()
        self._session_id = uuid.uuid4().hex
        described = self.describe()
        descriptions = described if isinstance(described, list) else [described]
        for description in descriptions:
            validate_description(description)
        sources = [description.source for description in descriptions]
        if not sources or len(set(sources)) != len(sources):
            raise ValueError(f"{self._label}: robot names must be present and unique: {sources}")
        self._descriptions = {description.source: description for description in descriptions}
        self._deadman_s = min(description.deadman_timeout_s for description in descriptions)
        listeners = self._plan_ports()

        self.connect()
        self._connected = True
        self._subscriptions = [self._listen(*listener) for listener in listeners]
        self._state_thread = threading.Thread(
            target=self._state_loop, daemon=True, name=f"{self._label}-state"
        )
        self._state_thread.start()

    @rpc
    def stop(self) -> None:
        with self._lock:
            if self._stopping:
                return
            self._stopping = True
        # Commands are cut off before the final halt, so none can follow it.
        for subscription in self._subscriptions:
            subscription.dispose()
        self._stop_event.set()
        if self._state_thread is not None:
            self._state_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        with self._hardware_lock:
            if self._connected:
                self._call_halt()
            self._connected = False
        super().stop()

    @rpc
    def describe_control(self) -> ConnectionDescription:
        """This run's session ID, and one description per robot it runs."""
        return ConnectionDescription(self._session_id, tuple(self._descriptions.values()))

    @rpc
    def status(self) -> ConnectionStatus:
        """A snapshot of the connection. See ``ConnectionStatus``."""
        with self._lock:
            return ConnectionStatus(
                connected=self._connected,
                last_state_time=self._last_state_time,
                last_command_time=self._last_command_time,
                deadman_fired=self._deadman_fired,
                last_rejection=self._last_rejection,
                last_error=self._last_error,
            )

    def publish_state(self, values: Mapping[str, float]) -> None:
        """Publish one set of readings on the output ports.

        Each robot's readings must carry exactly the keys its description
        lists; a robot whose readings do not is left out and shows up in
        ``status().last_error``. Does nothing once the module is stopping.

        Args:
            values: Readings by full key, e.g. ``{"arm/joint1/position": 0.3}``,
                each in its key's unit. Keys of several robots may be mixed.
        """
        if self._stop_event.is_set():
            return
        now = time.time()
        by_source: dict[str, Values] = {}
        for key, value in values.items():
            by_source.setdefault(_source(key), {})[key] = value
        checked: Values = {}
        for source, readings in by_source.items():
            description = self._descriptions.get(source)
            if description is None:
                self._report(f"readings for {source!r}, which this module does not run")
                continue
            try:
                # validate_state takes a ControlValues; this one never leaves
                # the process.
                frame = ControlValues(source, now, 0, 0, list(readings), list(readings.values()))
                validate_state(description, frame)
            except (ValueError, FrameRejectedError) as error:
                self._report(f"bad readings for {source!r}: {error}")
                continue
            checked.update(readings)
        published = {_source(key) for key in checked}
        for source, port, build, target in self._outputs:
            if source in published:
                port.publish(build(checked, target, now))
        if checked:
            with self._lock:
                self._last_state_time = now

    def _plan_ports(self) -> list[_Listener]:
        """Match the declared ports to the descriptions.

        Fills in ``self._outputs`` and returns one subscription to make per
        input and robot.

        Raises:
            TypeError: If a standard port carries the wrong message.
            ValueError: If a description names a command no declared input
                carries or a reading no declared output carries, or a port
                for a message with no names has several parts to choose from.
        """
        descriptions = list(self._descriptions.values())
        inputs = {n: p for n, p in self.inputs.items() if n in COMMAND_PORTS}
        outputs = {n: p for n, p in self.outputs.items() if n in STATE_PORTS}
        for name, port in {**inputs, **outputs}.items():
            expected = {**COMMAND_PORTS, **STATE_PORTS}[name]
            if port.type is not expected:
                raise TypeError(f"{self._label}.{name} must carry {expected.__name__}")
        joints = _parts(descriptions, ResourceKind.JOINT)
        bases = _parts(descriptions, ResourceKind.BASE)
        motor = motor_joints(descriptions)

        # Per input: how to turn its message into values, the second argument
        # that takes, and every key it can carry.
        readers: dict[str, tuple[Callable[[Any, Any], Values], Any, set[str]]] = {
            "position_command": (
                joint_state_to_values,
                (POSITION,),
                _keys([j for j, cmd, _ in joints if POSITION in cmd], (POSITION,)),
            ),
            "velocity_command": (
                joint_state_to_values,
                (VELOCITY,),
                _keys([j for j, cmd, _ in joints if VELOCITY in cmd], (VELOCITY,)),
            ),
            "motor_command": (motor_command_to_values, motor, _keys(motor, MOTOR_INTERFACES)),
        }
        if "base_command" in inputs:
            base = self._only(
                [b for b, cmd, _ in bases if set(TWIST_INTERFACES) <= cmd], "base_command"
            )
            readers["base_command"] = (twist_to_values, base, _keys([base], TWIST_INTERFACES))
        carried = set().union(*(readers[name][2] for name in inputs))
        self._check_covered({k for d in descriptions for k in d.command_keys()} - carried, "input")
        listeners = [
            (port, readers[name][0], readers[name][1], source)
            for name, port in inputs.items()
            for source in self._descriptions
            if any(_source(key) == source for key in readers[name][2])
        ]

        published: set[str] = set()
        if "joint_state" in outputs:
            # One message per robot and set of fields: a JointState fills a
            # field for every joint it names or for none.
            groups: dict[tuple[str, tuple[str, ...]], list[str]] = {}
            for joint, _, state in joints:
                fields = tuple(f for f in JOINT_STATE_FIELDS if f in state)
                if fields:
                    groups.setdefault((_source(joint), fields), []).append(joint)
            for (source, fields), names in groups.items():
                published |= _keys(names, fields)
                self._outputs.append(
                    (source, outputs["joint_state"], joint_state_from_values, names)
                )
        if "odom" in outputs:
            odom = self._only([b for b, _, st in bases if set(POSE_INTERFACES) <= st], "odom")
            published |= _keys([odom], POSE_INTERFACES)
            self._outputs.append((_source(odom), outputs["odom"], pose_from_values, odom))
        if "base_velocity" in outputs:
            speed = self._only(
                [b for b, _, st in bases if set(TWIST_INTERFACES) <= st], "base_velocity"
            )
            published |= _keys([speed], TWIST_INTERFACES)
            self._outputs.append((_source(speed), outputs["base_velocity"], _measured_twist, speed))
        if "imu" in outputs:
            sensors = _parts(descriptions, ResourceKind.SENSOR)
            imu = self._only([s for s, _, st in sensors if set(IMU_INTERFACES) <= st], "imu")
            published |= _keys([imu], IMU_INTERFACES)
            self._outputs.append((_source(imu), outputs["imu"], imu_from_values, imu))
        self._check_covered({k for d in descriptions for k in d.state_keys()} - published, "output")
        return listeners

    def _listen(
        self, port: In[Any], convert: Callable[[Any, Any], Values], argument: Any, source: str
    ) -> DisposableBase:
        """Pass ``source``'s values from each message on ``port`` to ``_apply``,
        newest message only. ``convert(message, argument)`` gives the values.

        A JointState names its joints, so one naming none of ``source``'s is
        dropped before it can displace one that does. Handling runs on the
        module's event loop.
        """
        prefix = f"{source}{SEPARATOR}"

        def carries_source(msg: Any) -> bool:
            return not isinstance(msg, JointState) or any(n.startswith(prefix) for n in msg.name)

        async def handle(msg: Any) -> None:
            try:
                values = convert(msg, argument)
            except ValueError as error:
                self._report(f"{port.name}: {error}", rejection=True)
                return
            mine = {key: value for key, value in values.items() if key.startswith(prefix)}
            if mine:
                self._apply(source, mine)

        return self.process_observable(
            port.pure_observable().pipe(ops.filter(carries_source)), handle
        )

    def _apply(self, source: str, values: Values) -> None:
        try:
            # validate_command takes a ControlValues; this one never leaves the
            # process. Typed messages carry no sequence number, so order is
            # not checked.
            frame = ControlValues(source, None, 0, 0, list(values), list(values.values()))
        except ValueError as error:
            self._report(f"{source}: {error}", rejection=True)
            return
        result = validate_command(self._descriptions[source], frame, last_sequence=None)
        if isinstance(result, Rejected):
            self._report(f"{source}: {result.reason}: {result.detail}", rejection=True)
            return
        with self._lock:
            self._last_command_time = time.time()
            self._deadman_fired = False
        self._start_deadman()
        try:
            with self._hardware_lock:
                if not self._connected:
                    return
                self.write(result.values)
        except Exception as error:
            self._report(f"write for {source!r} failed: {type(error).__name__}: {error}")
            self._cancel_deadman()
            self._halt()

    def _start_deadman(self) -> None:
        self._cancel_deadman()
        loop = asyncio.get_running_loop()
        self._deadman_timer = loop.call_later(self._deadman_s, self._deadman_expired)

    def _cancel_deadman(self) -> None:
        if self._deadman_timer is not None:
            self._deadman_timer.cancel()
            self._deadman_timer = None

    def _deadman_expired(self) -> None:
        self._deadman_timer = None
        with self._lock:
            self._deadman_fired = True
        logger.warning(f"{self._label}: no command for {self._deadman_s} s, halting")
        self._halt()

    def _halt(self) -> None:
        with self._hardware_lock:
            if self._connected:
                self._call_halt()

    def _call_halt(self) -> None:
        try:
            self.halt()
        except Exception as error:
            self._report(f"halt failed: {type(error).__name__}: {error}")

    def _state_loop(self) -> None:
        period = 1.0 / max(d.state_rate_hz for d in self._descriptions.values())
        next_at = time.monotonic()
        while not self._stop_event.is_set():
            try:
                values = self.read_state()
                if values is not None:
                    self.publish_state(values)
            except Exception as error:
                self._report(f"read_state failed: {type(error).__name__}: {error}")
            # After falling behind, carry on from now rather than catching up.
            next_at = max(next_at + period, time.monotonic())
            self._stop_event.wait(next_at - time.monotonic())

    def _only(self, names: list[str], port: str) -> str:
        """The one robot part a port carries, for a message with no names."""
        if len(names) != 1:
            raise ValueError(f"{self._label}.{port} needs exactly one part to carry, found {names}")
        return names[0]

    def _check_covered(self, uncovered: set[str], direction: str) -> None:
        if uncovered:
            raise ValueError(
                f"{self._label}: no declared {direction} port carries {sorted(uncovered)}; "
                f"declare the port, or leave them out of the description"
            )

    def _report(self, message: str, *, rejection: bool = False) -> None:
        """Record a problem for ``status``, and log it unless it repeats the
        last one or one of its kind was logged in the past second."""
        now = time.monotonic()
        with self._lock:
            previous = self._last_rejection if rejection else self._last_error
            if rejection:
                self._last_rejection = message
            else:
                self._last_error = message
            quiet = (
                message == previous or now - self._last_logged.get(rejection, float("-inf")) < 1.0
            )
            if not quiet:
                self._last_logged[rejection] = now
        if not quiet:
            logger.warning(f"{self._label}: {message}")

    @property
    def _label(self) -> str:
        return type(self).__name__


def _source(key: str) -> str:
    """The robot a key or part name belongs to: "arm" for "arm/joint1"."""
    return key.split(SEPARATOR, 1)[0]


def _measured_twist(values: Values, base: str, ts: float) -> Twist:
    """A base's measured speed as a Twist, which carries no time."""
    return twist_from_values(values, base)


def _keys(parts: list[str], interfaces: tuple[str, ...]) -> set[str]:
    return {f"{part}{SEPARATOR}{interface}" for part in parts for interface in interfaces}


def _parts(
    descriptions: list[ControlDescription], kind: ResourceKind
) -> list[tuple[str, frozenset[str], frozenset[str]]]:
    """Each part of a kind, as its full name, what it accepts, what it reports."""
    return [
        (
            f"{d.source}{SEPARATOR}{r.name}",
            frozenset(r.command_interfaces),
            frozenset(r.state_interfaces),
        )
        for d in descriptions
        for r in d.resources
        if r.kind is kind
    ]
