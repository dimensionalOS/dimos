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

A driver subclasses ``ConnectionModule``, gives it a config that says what
each command port drives, declares its ports, and fills in four methods for
its hardware. The base turns incoming messages into checked named values for
``write``, publishes the driver's readings as messages, and halts the robot
when commands stop coming. Anything that publishes the standard messages can
drive it; it does not need a coordinator.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import asyncio
from collections.abc import Callable, Mapping
from dataclasses import dataclass
import functools
import threading
import time
from typing import Any, ClassVar
import uuid

from reactivex import operators as ops
from reactivex.abc import DisposableBase

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.contract.convert import (
    JOINT_STATE_FIELDS,
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
from dimos.control.contract.keys import SEPARATOR
from dimos.control.contract.ports import (
    JointPort,
    MotorPort,
    TwistPort,
    control_description,
    port_specs,
    reading_ports,
)
from dimos.control.contract.validate import (
    FrameRejectedError,
    Rejected,
    validate_command,
    validate_state,
)
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Values = dict[str, float]


@dataclass(frozen=True, slots=True)
class _Listener:
    """One input port to subscribe to, and how to turn its messages into values.

    Attributes:
        port: The input port.
        convert: Turns a message into named values, called as
            ``convert(message, argument)``.
        argument: The second argument ``convert`` needs, e.g. the base's name.
    """

    port: In[Any]
    convert: Callable[[Any, Any], Values]
    argument: Any


@dataclass(frozen=True, slots=True)
class _Output:
    """One output port to publish readings on.

    Attributes:
        port: The output port.
        build: Builds its message from the checked readings, called as
            ``build(values, ts=unix_time)``.
    """

    port: Out[Any]
    build: Callable[..., Any]


class ConnectionConfig(ModuleConfig):
    """Settings every robot driver has.

    A driver's config subclasses it and adds one field per command input
    port, named like the port and typed with a port spec from
    ``dimos.control.contract.ports``.

    Attributes:
        source: The robot's name, and the first part of every key, e.g. "go2".
        model: The robot's model (its URDF), which gives a joint port its
            joints, their units and their limits. ``None`` for a robot that
            has no joint ports.
        state_rate_hz: How many times a second ``read_state`` is called.
        deadman_timeout_s: How long the robot goes without a command, in
            seconds, before it is halted.
    """

    source: str
    model: RobotModel | None = None
    state_rate_hz: float = 100.0
    deadman_timeout_s: float = 0.1


@dataclass(frozen=True, slots=True)
class ConnectionStatus:
    """A snapshot of one connection, returned by ``ConnectionModule.status``.

    Attributes:
        session_id: New each time the module starts, so a reader can tell a
            restarted driver from one that kept running. Empty before start.
        connected: ``connect`` succeeded and the module has not stopped.
        last_state_time: When readings were last published, in Unix seconds,
            or ``None`` if never.
        last_command_time: When a command was last accepted, in Unix seconds,
            or ``None`` if never.
        deadman_fired: Commands stopped arriving, the robot was halted for
            it, and none has been accepted since.
        last_rejection: Why the last refused command was refused, or ``None``.
        last_error: The last failure in the driver (``write``, ``read_state``,
            ``halt``, or a reading that does not match the description), or
            ``None``.
    """

    session_id: str
    connected: bool
    last_state_time: float | None
    last_command_time: float | None
    deadman_fired: bool
    last_rejection: str | None
    last_error: str | None


class ConnectionModule(Module, ABC):
    """A robot driver. Subclass it, give it a config, declare its ports, fill in four methods.

    The config says what each command input drives, with one field per
    input port, named like the port::

        class ArmConfig(ConnectionConfig):
            source: str = "arm"
            model: RobotModel | None = ARM_MODEL
            position_command: JointPort = JointPort()

        class ArmConnection(ConnectionModule):
            config: ArmConfig
            position_command: In[JointState]
            joint_state: Out[JointState]

    Each such field needs an input port of the same name carrying its
    message (``TwistPort``: Twist, ``JointPort``: JointState, ``MotorPort``:
    MotorCommandArray). Output ports need no config: what each reports is set
    by its message type (see ``dimos.control.contract.ports``), under any
    name. Ports of other messages are the driver's own and left alone.

    Starting it builds ``description`` from the config and the ports, raising
    on anything wrong before touching the hardware, then calls ``connect``.

    Readings: a thread calls ``read_state`` ``state_rate_hz`` times a second
    and publishes what it returns on the output ports.

    Commands: each message is turned into named values, which are checked
    against ``description`` (unknown keys, limits) before ``write`` sees
    them. Each input keeps just its newest message, so if ``write`` is slower
    than commands arrive, the ones in between are skipped.

    Deadman: once a command has been accepted, if no other is accepted within
    ``deadman_timeout_s``, ``halt`` is called once. The next accepted command
    starts the deadman again.

    ``write`` and ``halt`` never run at the same time. Commands and the
    deadman share the module's event loop, so a ``write`` that hangs stops
    both: the deadman cannot fire and no later command is handled until it
    returns.

    Frames: an odometry pose is in ``odom``, an IMU in ``imu``, a measured
    base speed in ``base``. With several robots on one bus, give each module
    a ``frame_id_prefix`` (its robot's name) and its frames become
    ``go2b/odom`` and so on, so readers can tell the robots apart.

    To close the hardware connection when the module stops, override ``stop``
    and close it after calling ``super().stop()``, which halts the robot first.
    """

    config: ConnectionConfig

    dedicated_worker: ClassVar[bool] = True

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._session_id = ""
        self._description: ControlDescription | None = None
        # Guards the status fields and the stop flag.
        self._lock = threading.Lock()
        # Held around every write() and halt(), so they never overlap, and
        # around every change to _connected.
        self._hardware_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._stopping = False
        self._connected = False
        self._subscriptions: list[DisposableBase] = []
        self._outputs: list[_Output] = []
        self._state_thread: threading.Thread | None = None
        # Only touched on the module's event loop, where commands are handled.
        self._deadman: asyncio.TimerHandle | None = None
        self._last_state_time: float | None = None
        self._last_command_time: float | None = None
        self._deadman_fired = False
        self._last_rejection: str | None = None
        self._last_error: str | None = None
        # When a rejection (True) or an error (False) was last logged.
        self._last_logged: dict[bool, float] = {}
        # How often the state loop has run late since it last said so, and when.
        self._late = 0
        self._late_logged_at = float("-inf")

    @property
    def description(self) -> ControlDescription:
        """What this robot is, built once from the config and the ports.

        Raises ``ValueError`` or ``DescriptionError`` if they do not add up;
        see ``control_description``.
        """
        if self._description is None:
            self._description = control_description(
                self.config,
                {name: port.type for name, port in self.inputs.items()},
                {name: port.type for name, port in self.outputs.items()},
            )
        return self._description

    @abstractmethod
    def connect(self) -> None:
        """Open the connection to the hardware.

        It should not make anything move. Whether the SDK keeps to that is
        the driver's responsibility: if opening it can move the robot, the
        driver must stop that here.
        """

    @abstractmethod
    def read_state(self) -> Values:
        """The robot's latest readings, or an empty dict if nothing new has arrived.

        Keys are full keys such as ``"arm/joint1/position"``, each value in
        its key's unit, and every key the description lists must be present.
        A driver whose SDK delivers readings in a callback returns an empty
        dict here and calls ``publish_state`` from the callback instead.
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
            values: Named values from one message, e.g.
                ``{"arm/joint1/position": 0.3}``, already within their limits.
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
        description = self.description
        listeners = self._plan_inputs(description)
        self._outputs = self._plan_outputs(description)

        self.connect()
        self._connected = True
        self._subscriptions = [self._listen(listener) for listener in listeners]
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
    def status(self) -> ConnectionStatus:
        """A snapshot of the connection. See ``ConnectionStatus``."""
        with self._lock:
            return ConnectionStatus(
                session_id=self._session_id,
                connected=self._connected,
                last_state_time=self._last_state_time,
                last_command_time=self._last_command_time,
                deadman_fired=self._deadman_fired,
                last_rejection=self._last_rejection,
                last_error=self._last_error,
            )

    def publish_state(self, values: Mapping[str, float]) -> None:
        """Publish one set of readings on the output ports.

        The readings must carry exactly the keys the description lists; a set
        that does not is dropped and shows up in ``status().last_error``.
        Does nothing once the module is stopping.

        Args:
            values: Readings by full key, e.g. ``{"arm/joint1/position": 0.3}``,
                each in its key's unit.
        """
        if self._stop_event.is_set():
            return
        now = time.time()
        description = self.description
        try:
            # validate_state takes a ControlValues; this one never leaves the
            # process.
            frame = ControlValues(
                description.source, now, 0, 0, list(values), list(values.values())
            )
            checked = validate_state(description, frame)
        except (ValueError, FrameRejectedError) as error:
            self._report(f"bad readings: {error}")
            return
        for output in self._outputs:
            output.port.publish(output.build(checked, ts=now))
        with self._lock:
            self._last_state_time = now

    def _plan_inputs(self, description: ControlDescription) -> list[_Listener]:
        """One subscription per port spec in the config."""
        listeners: list[_Listener] = []
        for name, spec in port_specs(self.config).items():
            port = self.inputs[name]
            if isinstance(spec, TwistPort):
                base = self._part(description, ResourceKind.BASE)
                listeners.append(_Listener(port, twist_to_values, base))
            elif isinstance(spec, JointPort):
                listeners.append(_Listener(port, joint_state_to_values, (spec.interface,)))
            elif isinstance(spec, MotorPort):
                listeners.append(
                    _Listener(port, motor_command_to_values, motor_joints([description]))
                )
        return listeners

    def _plan_outputs(self, description: ControlDescription) -> list[_Output]:
        """How to build each output port's message from the readings."""
        outputs: list[_Output] = []
        reported = reading_ports({name: port.type for name, port in self.outputs.items()})
        if JointState in reported:
            # One message per set of fields: a JointState fills a field for
            # every joint it names or for none.
            groups: dict[tuple[str, ...], list[str]] = {}
            for resource in description.resources:
                if resource.kind is not ResourceKind.JOINT:
                    continue
                fields = tuple(f for f in JOINT_STATE_FIELDS if f in resource.state_interfaces)
                if fields:
                    name = f"{description.source}{SEPARATOR}{resource.name}"
                    groups.setdefault(fields, []).append(name)
            port = self.outputs[reported[JointState]]
            for joints in groups.values():
                outputs.append(
                    _Output(port, functools.partial(joint_state_from_values, joints=joints))
                )
        if PoseStamped in reported:
            base = self._part(description, ResourceKind.BASE)
            pose = functools.partial(pose_from_values, base=base, frame_id=self._frame("odom"))
            outputs.append(_Output(self.outputs[reported[PoseStamped]], pose))
        if TwistStamped in reported:
            base = self._part(description, ResourceKind.BASE)
            speed = functools.partial(_measured_twist, base, frame_id=self._frame("base"))
            outputs.append(_Output(self.outputs[reported[TwistStamped]], speed))
        if Imu in reported:
            imu = self._part(description, ResourceKind.SENSOR)
            imu_msg = functools.partial(imu_from_values, sensor=imu, frame_id=self._frame("imu"))
            outputs.append(_Output(self.outputs[reported[Imu]], imu_msg))
        return outputs

    def _listen(self, listener: _Listener) -> DisposableBase:
        """Pass this robot's values from each message on the listener's port to
        ``_apply``, newest message only.

        A JointState names its joints, so one naming none of this robot's is
        dropped before it can displace one that does. Handling runs on the
        module's event loop.
        """
        prefix = f"{self.config.source}{SEPARATOR}"

        def carries_mine(msg: Any) -> bool:
            return not isinstance(msg, JointState) or any(n.startswith(prefix) for n in msg.name)

        async def handle(msg: Any) -> None:
            try:
                values = listener.convert(msg, listener.argument)
            except ValueError as error:
                self._report(f"{listener.port.name}: {error}", rejection=True)
                return
            mine = {key: value for key, value in values.items() if key.startswith(prefix)}
            if mine:
                self._apply(mine)

        return self.process_observable(
            listener.port.pure_observable().pipe(ops.filter(carries_mine)), handle
        )

    def _apply(self, values: Values) -> None:
        description = self.description
        try:
            # validate_command takes a ControlValues; this one never leaves the
            # process. Typed messages carry no sequence number, so order is
            # not checked.
            frame = ControlValues(
                description.source, None, 0, 0, list(values), list(values.values())
            )
        except ValueError as error:
            self._report(str(error), rejection=True)
            return
        result = validate_command(description, frame, last_sequence=None)
        if isinstance(result, Rejected):
            self._report(f"{result.reason}: {result.detail}", rejection=True)
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
            self._report(f"write failed: {type(error).__name__}: {error}")
            self._cancel_deadman()
            self._halt()

    def _start_deadman(self) -> None:
        self._cancel_deadman()
        loop = asyncio.get_running_loop()
        self._deadman = loop.call_later(self.config.deadman_timeout_s, self._deadman_expired)

    def _cancel_deadman(self) -> None:
        if self._deadman is not None:
            self._deadman.cancel()
            self._deadman = None

    def _deadman_expired(self) -> None:
        self._deadman = None
        logger.warning(f"{self._label}: no command in {self.config.deadman_timeout_s} s, halting")
        self._halt()
        # Set only once halted, so a status that says it fired means the robot was told to stop.
        with self._lock:
            self._deadman_fired = True

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
        period = 1.0 / self.config.state_rate_hz
        next_at = time.monotonic()
        while not self._stop_event.is_set():
            try:
                values = self.read_state()
                if values:
                    self.publish_state(values)
            except Exception as error:
                self._report(f"read_state failed: {type(error).__name__}: {error}")
            next_at += period
            now = time.monotonic()
            if now > next_at:
                # A whole period behind: carry on from now rather than
                # catching up, and say so.
                self._note_late(now - next_at, period)
                next_at = now
            self._stop_event.wait(next_at - now)

    def _note_late(self, late_s: float, period_s: float) -> None:
        """Warn that readings ran late, at most once a second."""
        now = time.monotonic()
        self._late += 1
        if now - self._late_logged_at < 1.0:
            return
        logger.warning(
            f"{self._label}: readings ran late {self._late} time(s), the last by "
            f"{late_s * 1000:.1f} ms against a {period_s * 1000:.1f} ms period"
        )
        self._late = 0
        self._late_logged_at = now

    def _frame(self, name: str) -> str:
        """A reading's frame: ``name``, behind the config's ``frame_id_prefix`` if set."""
        prefix = self.config.frame_id_prefix
        return f"{prefix}/{name}" if prefix else name

    def _part(self, description: ControlDescription, kind: ResourceKind) -> str:
        """The full name of the robot's one part of a kind, e.g. "go2/base"."""
        for resource in description.resources:
            if resource.kind is kind:
                return f"{description.source}{SEPARATOR}{resource.name}"
        raise ValueError(f"{self._label}: the description has no {kind.value}")

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


def _measured_twist(
    base: str, values: Values, ts: float | None = None, *, frame_id: str
) -> TwistStamped:
    """A base's measured speed, stamped with when it was read and the base's frame."""
    twist = twist_from_values(values, base)
    return TwistStamped(
        ts if ts is not None else time.time(), frame_id, linear=twist.linear, angular=twist.angular
    )
