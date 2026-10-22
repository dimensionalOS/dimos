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

"""The base every connection module is built on: one module per robot.

A connection module subclasses ``ConnectionModule``, gives it a config that
says what the robot is told, and fills in ``connect``, ``write`` and
``halt``. It declares its own ports, hands their messages to ``command`` as
named values (``dimos.control.contract.convert``), and publishes its
readings however suits its SDK.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import asyncio
from dataclasses import dataclass
import threading
import time
from typing import Any, ClassVar
import uuid

from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.control.contract.limits import Limits, limits as limits_of
from dimos.control.contract.validate import Rejected, validate_command
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.assets.model import RobotModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Values = dict[str, float]


class ConnectionConfig(ModuleConfig):
    """Settings every connection module has: what the robot is told, and when it is halted.

    Attributes:
        source: The robot's name, and the first part of every key, e.g. "go2".
        model: The robot's model (its URDF): its joints, their order and their
            limits. Needed when joints are commanded.
        joint_commands: What every joint that moves accepts: position,
            velocity, effort.
        pd: The joints take position, velocity, effort, kp and kd in one
            message, as a humanoid's do.
        base_range: The fastest the base may be told to go, either way:
            ``linear.x`` and ``linear.y`` in m/s, ``angular.z`` in rad/s. A
            speed past it is clamped. ``None`` for a robot with no base.
        deadman_timeout_s: How long the robot goes without a command, in
            seconds, before it is halted.
    """

    source: str
    model: RobotModel | None = None
    joint_commands: tuple[Interface, ...] = ()
    pd: bool = False
    base_range: Twist | None = None
    deadman_timeout_s: float = 0.1


@dataclass(frozen=True, slots=True)
class ConnectionStatus:
    """A snapshot of one connection, returned by ``ConnectionModule.status``.

    Attributes:
        session_id: New each time the module starts, so a reader can tell a
            restarted module from one that kept running. Empty before start.
        connected: ``connect`` succeeded and the module has not stopped.
        last_command_time: When a command was last accepted, in Unix seconds,
            or ``None`` if never.
        deadman_fired: Commands stopped arriving, the robot was halted for
            it, and none has been accepted since.
        last_rejection: Why the last refused command was refused, or ``None``.
        last_error: The last failure in ``write`` or ``halt``, or ``None``.
    """

    session_id: str
    connected: bool
    last_command_time: float | None
    deadman_fired: bool
    last_rejection: str | None
    last_error: str | None


class ConnectionModule(Module, ABC):
    """One robot: commands go through ``command``, and it halts when they stop.

    Subclass it, give it a config, declare the ports, fill in ``connect``,
    ``write`` and ``halt``, and pass each command port's messages to
    ``command`` as named values::

        class ArmConfig(ConnectionConfig):
            source: str = "arm"
            model: RobotModel | None = ARM_MODEL
            joint_commands: tuple[Interface, ...] = (Interface.POSITION,)

        class ArmConnection(ConnectionModule):
            config: ArmConfig
            position_command: In[JointState]
            joint_state: Out[JointState]

            @rpc
            def start(self) -> None:
                super().start()
                self.register_disposable(Disposable(self.position_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.POSITION,)))
                )))

    Readings are the module's own: publish them from wherever the SDK
    delivers them, stamped with ``frame``. Once a command is accepted, if no
    other is within ``deadman_timeout_s``, ``halt`` is called once. To close
    the hardware, override ``stop`` and close it after ``super().stop()``.
    """

    config: ConnectionConfig

    dedicated_worker: ClassVar[bool] = True

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._session_id = ""
        self._limits: dict[str, Limits] | None = None
        # Guards the status fields and the stop flag.
        self._lock = threading.Lock()
        # Held around every write() and halt(), so they never overlap, and
        # around every change to _connected.
        self._hardware_lock = threading.Lock()
        self._stopping = False
        self._connected = False
        # Only touched on the module's event loop.
        self._deadman: asyncio.TimerHandle | None = None
        # Counts accepted commands; a deadman timer only halts if no command
        # was accepted after the one that armed it.
        self._command_seq = 0
        self._last_command_time: float | None = None
        self._deadman_fired = False
        self._last_rejection: str | None = None
        self._last_error: str | None = None
        # When a rejection (True) or an error (False) was last logged.
        self._last_logged: dict[bool, float] = {}

    @property
    def limits(self) -> dict[str, Limits]:
        """Every key this robot accepts and how far each may go, built once from the config."""
        if self._limits is None:
            self._limits = limits_of(self.config)
        return self._limits

    @abstractmethod
    def connect(self) -> None:
        """Open the connection to the hardware without making anything move."""

    @abstractmethod
    def write(self, values: Values) -> None:
        """Send one checked command to the hardware and return at once.

        Never wait for a motion to finish. Raise if the hardware refuses:
        the module then records it, calls ``halt`` and carries on.

        Args:
            values: Named values from one command, e.g.
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
        # A bad config fails here, before the hardware is touched.
        self._limits = limits_of(self.config)
        self.connect()
        with self._hardware_lock:
            self._connected = True

    @rpc
    def stop(self) -> None:
        with self._lock:
            if self._stopping:
                return
            self._stopping = True
        with self._hardware_lock:
            if self._connected:
                self._call_halt()
            self._connected = False
        self._on_loop(self._disarm_deadman)
        super().stop()

    @rpc
    def status(self) -> ConnectionStatus:
        """A snapshot of the connection. See ``ConnectionStatus``."""
        with self._lock:
            return ConnectionStatus(
                session_id=self._session_id,
                connected=self._connected,
                last_command_time=self._last_command_time,
                deadman_fired=self._deadman_fired,
                last_rejection=self._last_rejection,
                last_error=self._last_error,
            )

    def command(self, values: Values) -> None:
        """Check one command against the limits table and send it to the hardware.

        Call it from a port subscription, on any thread. A refused command
        (an unknown key, a value past a limit that does not clamp) is dropped
        and shown in ``status()``; an accepted one restarts the deadman and
        goes to ``write``. If ``write`` raises, the robot is halted.

        Args:
            values: Named values in each key's unit, e.g.
                ``{"arm/joint1/position": 0.3}``. Keys of other robots are
                skipped; a command with none of this robot's does nothing.
        """
        prefix = f"{self.config.source}{SEPARATOR}"
        own = {key: value for key, value in values.items() if key.startswith(prefix)}
        if not own:
            return
        result = validate_command(own, self.limits)
        if isinstance(result, Rejected):
            self._report(f"{result.reason}: {result.detail}", rejection=True)
            return
        with self._hardware_lock:
            if not self._connected:
                return
            with self._lock:
                self._last_command_time = time.time()
                self._deadman_fired = False
                self._command_seq += 1
                seq = self._command_seq
            self._on_loop(lambda: self._arm_deadman(seq))
            try:
                self.write(result.values)
            except Exception as error:
                self._report(f"write failed: {type(error).__name__}: {error}")
                self._on_loop(self._disarm_deadman)
                self._call_halt()

    def frame(self, name: str) -> str:
        """The frame to stamp a reading with: ``name`` behind the config's
        ``frame_id_prefix``, e.g. "go2b/odom", or just "odom" without one."""
        prefix = self.config.frame_id_prefix
        return f"{prefix}/{name}" if prefix else name

    def _on_loop(self, step: Any) -> None:
        """Run a deadman step on the module's event loop, from any thread, in order."""
        if self._loop is not None:
            self._loop.call_soon_threadsafe(step)

    def _arm_deadman(self, seq: int) -> None:
        self._disarm_deadman()
        self._deadman = asyncio.get_running_loop().call_later(
            self.config.deadman_timeout_s, self._deadman_expired, seq
        )

    def _disarm_deadman(self) -> None:
        if self._deadman is not None:
            self._deadman.cancel()
            self._deadman = None

    def _deadman_expired(self, seq: int) -> None:
        self._deadman = None
        with self._hardware_lock:
            with self._lock:
                # A newer command was accepted while this timer was due: its
                # own timer is armed or queued, so this one has nothing to say.
                if seq != self._command_seq or not self._connected:
                    return
            logger.warning(
                f"{self._label}: no command in {self.config.deadman_timeout_s} s, halting"
            )
            self._call_halt()
            # Set only once halted, so a status that says it fired means the
            # robot was told to stop.
            with self._lock:
                self._deadman_fired = True

    def _call_halt(self) -> None:
        try:
            self.halt()
        except Exception as error:
            self._report(f"halt failed: {type(error).__name__}: {error}")

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
