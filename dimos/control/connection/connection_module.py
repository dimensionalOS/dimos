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
named values (``dimos.control.contract.convert``), and publishes canonical
joint readings. Vendor geometry and frame conversion stay inside that implementation.

Every connection also serves the arm config contract
(``dimos.control.connection.arm_config``): ``config_keys`` and ``config_get``
say what it can change and how its arms are driven now; with
``config_sessions`` on (calibration blueprints only) ``config_set``,
``config_reset`` and ``config_command`` let one calibration tool own the arm
in a config session and change how it is driven. A driver that can change
something overrides the underscore hooks (``_config_keys``, ``_config_state``,
``_config_apply``, ``_config_undo``, ``_config_parkable``, ``_config_fault``);
by default it can change nothing.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import asyncio
from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
import numbers
import threading
import time
from typing import Any, ClassVar, final
import uuid

from pydantic import BaseModel

from dimos.control.connection.arm_config import (
    DEFAULT_CONFIG_LEASE_S,
    MAX_CONFIG_LEASE_S,
    ArmConfigState,
    CommandResult,
    ConfigApplied,
    ConfigKeys,
    ConfigPatch,
    ConfigScope,
)
from dimos.control.contract.keys import SEPARATOR, Interface
from dimos.control.contract.limits import Limits, limits as limits_of
from dimos.control.contract.validate import Rejected, validate_command
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.robot.assets.model import RobotModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Values = dict[str, float]

#: How long ``_await_config_park`` waits for the calibration tool to park the
#: arm and end its config session before the stop goes on, seconds.
CONFIG_PARK_WAIT_S = 10.0
#: How long ``_await_config_park`` waits for a calibration tool that is about
#: to park (no session yet, or one that just ended) to begin its park
#: session, seconds.
CONFIG_PARK_GRACE_S = 1.0


class ConnectionConfig(ModuleConfig):
    """Settings every connection module has: what the robot is told, and when it is halted.

    Attributes:
        source: The robot's name, and the first part of every key, e.g. "go2".
        model: The robot's model (its URDF): its joints, their order and their
            limits. Needed when joints are commanded.
        joint_commands: Model joint name to the interfaces it accepts. Only
            listed joints are commanded; planar axes are ordinary joints.
        deadman_timeout_s: How long the robot goes without a command, in
            seconds, before it is halted.
        config_sessions: Serve ``config_set``, ``config_reset`` and
            ``config_command``, which change how the energized arm is driven.
            Only a calibration blueprint turns this on; without it they raise.
    """

    source: str
    model: RobotModel | None = None
    joint_commands: dict[str, tuple[Interface, ...]] = {}
    deadman_timeout_s: float = 0.1
    config_sessions: bool = False


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
        config_session_id: The live config session (``config_set``), or
            ``None``.
    """

    session_id: str
    connected: bool
    last_command_time: float | None
    deadman_fired: bool
    last_rejection: str | None
    last_error: str | None
    config_session_id: str | None = None


@dataclass(slots=True)
class _ConfigSession:
    """A calibration tool's hold on the arm, until ``deadline`` (``time.monotonic``)."""

    session_id: str
    lease_s: float
    deadline: float
    # Position commands dropped while the session lasts.
    dropped: int = 0


def _real(name: str, value: Any) -> float:
    """``value`` as a float; a bool, a string or anything else not a number raises ``TypeError``."""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise TypeError(f"{name} must be a number, got {value!r}")
    return float(value)


def _flag(name: str, value: Any) -> bool:
    """``value`` if it is a real bool; anything else, the string "false" included, raises."""
    if not isinstance(value, bool):
        raise TypeError(f"{name} must be True or False, got {value!r}")
    return value


def _patch_of(patch: Any) -> ConfigPatch:
    """``patch`` validated strictly: the RPC codec rebuilds models without validating them."""
    if isinstance(patch, BaseModel):
        data: Any = patch.model_dump(warnings=False)
    elif isinstance(patch, Mapping):
        data = dict(patch)
    else:
        raise TypeError(f"patch must be a ConfigPatch, got {patch!r}")
    return ConfigPatch.model_validate(data, strict=True)


class ConnectionModule(Module, ABC):
    """One robot: commands go through ``command``, and it halts when they stop.

    Subclass it, give it a config, declare the ports, fill in ``connect``,
    ``write`` and ``halt``, and pass each command port's messages to
    ``command`` as named values::

        class ArmConfig(ConnectionConfig):
            source: str = "arm"
            model: RobotModel | None = ARM_MODEL
            joint_commands: dict[str, tuple[Interface, ...]] = {"joint1": (Interface.POSITION,)}

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

    The ``config_*`` RPCs are the arm config contract and are final; a driver
    fills in the underscore hooks instead, which are called with
    ``_hardware_lock`` held. A config session (``config_set`` with scope
    "session") belongs to one calibration tool: while it lasts every other
    position command is dropped, and it ends, undoing its live changes
    (``_config_undo``), on ``config_reset``, when its lease passes without a
    call that carries its id (``config_set`` with its ``session_id``,
    ``config_command``), when the robot halts (the command deadman included),
    when a ``config_*`` call raises, and when the module stops. When the undo
    raises the robot is halted, and when that fails too ``_config_fault`` is
    called. A driver that can fault ends it with ``_end_config_session``, and
    one that serves sessions calls ``_await_config_park`` first in its own
    ``stop``.
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
        # Guarded by _lock, and changed only under _hardware_lock as well: the
        # config session, why the last one ended, and when (time.monotonic()).
        self._config_session: _ConfigSession | None = None
        self._config_ended = ""
        self._config_ended_at: float | None = None
        # Only touched on the module's event loop: when the session's lease is up.
        self._config_lease: asyncio.TimerHandle | None = None

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
            self._end_config_session("the connection stopped")
            self._connected = False
        self._on_loop(self._disarm_deadman)
        self._on_loop(self._disarm_config_lease)
        super().stop()

    @rpc
    def status(self) -> ConnectionStatus:
        """A snapshot of the connection. See ``ConnectionStatus``."""
        with self._lock:
            session = self._config_session
            return ConnectionStatus(
                session_id=self._session_id,
                connected=self._connected,
                last_command_time=self._last_command_time,
                deadman_fired=self._deadman_fired,
                last_rejection=self._last_rejection,
                last_error=self._last_error,
                config_session_id=None if session is None else session.session_id,
            )

    def command(self, values: Values) -> None:
        """Check one command against the limits table and send it to the hardware.

        Call it from a port subscription, on any thread. A refused command
        (an unknown key, a value past a limit that does not clamp) is dropped
        and shown in ``status()``; an accepted one restarts the deadman and
        goes to ``write``. If ``write`` raises, the robot is halted. While a
        config session lasts, commands are dropped: the calibration tool owns
        the arm and moves it with ``config_command``.

        Args:
            values: Named values in each key's unit, e.g.
                ``{"arm/joint1/position": 0.3}``. Keys of other robots are
                skipped; a command with none of this robot's does nothing.
        """
        self._command(values, None)

    def _command(self, values: Values, session_id: str | None) -> CommandResult:
        """Check one command against the limits and write it, restarting the deadman.

        With ``session_id`` ``None`` it is a position command: a refused one
        is shown in ``status()``, and while a config session lasts it is
        dropped and counted. Otherwise it is ``config_command``'s: a refused
        one raises ``ValueError``, and it is written only while the session
        it names lasts. Both are decided under ``_hardware_lock``, which a
        session's start and end also hold, so no command slips past one.
        """
        prefix = f"{self.config.source}{SEPARATOR}"
        own = {key: value for key, value in values.items() if key.startswith(prefix)}
        if not own:
            return CommandResult(ok=False, error=f"no keys of {self.config.source!r}")
        result = validate_command(own, self.limits)
        if isinstance(result, Rejected):
            message = f"{result.reason}: {result.detail}"
            if session_id is not None:
                raise ValueError(message)
            self._report(message, rejection=True)
            return CommandResult(ok=False, error=message)
        with self._hardware_lock:
            if not self._connected:
                if session_id is not None:
                    raise RuntimeError(f"{self._label} is not running")
                return CommandResult(ok=False, error=f"{self._label} is not running")
            if session_id is None:
                if self._dropped_for_config_session():
                    return CommandResult(ok=False, error="dropped: a config session owns the arm")
            elif self._check_config_session(session_id) is None:
                raise RuntimeError(
                    "no config session is live; config_set with scope 'session' starts one"
                )
            with self._lock:
                self._last_command_time = time.time()
                self._deadman_fired = False
                self._command_seq += 1
                seq = self._command_seq
            self._on_loop(lambda: self._arm_deadman(seq))
            try:
                self.write(result.values)
            except Exception as error:
                message = f"write failed: {type(error).__name__}: {error}"
                self._report(message)
                self._on_loop(self._disarm_deadman)
                self._call_halt()
                if session_id is not None:
                    message += "; the arm holds and the config session ended"
                return CommandResult(ok=False, error=message)
        return CommandResult(ok=True)

    @rpc
    @final
    def config_keys(self) -> ConfigKeys:
        """What this connection lets a calibration tool read and change. See ``ConfigKeys``.

        Answers whether or not config sessions are served. It carries no
        session id, so it renews no session's lease.
        """
        with self._config_call("config_keys", gated=False):
            with self._hardware_lock:
                keys = self._config_keys()
            return keys.model_copy(
                update={
                    "source": self.config.source,
                    "sessions": self.config.config_sessions,
                    "max_lease_s": MAX_CONFIG_LEASE_S,
                }
            )

    @rpc
    @final
    def config_get(self) -> dict[str, ArmConfigState]:
        """How each arm is driven now, by arm id. See ``ArmConfigState``.

        Answers whether or not config sessions are served. It carries no
        session id, so it renews no session's lease.
        """
        with self._config_call("config_get", gated=False):
            with self._hardware_lock:
                with self._lock:
                    session = self._config_session
                states = self._config_state()
            session_id = None if session is None else session.session_id
            return {
                arm_id: state.model_copy(update={"session_id": session_id})
                for arm_id, state in states.items()
            }

    @rpc
    @final
    def config_set(
        self,
        patch: ConfigPatch,
        scope: ConfigScope = "session",
        force: bool = False,
        apply_now: bool = True,
        session_id: str | None = None,
        lease_s: float = DEFAULT_CONFIG_LEASE_S,
    ) -> ConfigApplied:
        """Change how one arm is driven: for the config session, or saved to its file.

        With scope "session" and no session live, this starts the one config
        session (an empty patch only starts it): until it ends every other
        position command is dropped, and its live changes are undone when it
        ends. While one is live, every ``config_set`` must carry its
        ``session_id``. With scope "persist" the patch is merged into the
        arm's calibration file and applied from it; ``force`` applies it even
        when a joint's gravity torque would step too far at this pose, and
        ``apply_now`` False only writes the file. What the connection cannot
        change is listed in ``unsupported``, unchanged. For calibration tools
        only; refused unless ``config_sessions``.

        Args:
            patch: What to change; see ``ConfigPatch``.
            scope: "session" or "persist".
            force: Persist even past the gravity-step guard.
            apply_now: Persist: apply the saved file at once.
            session_id: The live session, as the call that started it returned.
            lease_s: The lease of a session this call starts, above 0 and at
                most ``MAX_CONFIG_LEASE_S`` seconds.
        """
        with self._config_call("config_set"):
            checked = _patch_of(patch)
            if scope not in ("session", "persist"):
                raise ValueError(f"scope must be 'session' or 'persist', got {scope!r}")
            force = _flag("force", force)
            apply_now = _flag("apply_now", apply_now)
            lease = _real("lease_s", lease_s)
            if not 0.0 < lease <= MAX_CONFIG_LEASE_S:
                raise ValueError(
                    f"lease_s must be above 0 and at most {MAX_CONFIG_LEASE_S:g} s, got {lease_s}"
                )
            with self._hardware_lock:
                if not self._connected:
                    raise RuntimeError(f"{self._label} is not running")
                session = self._check_config_session(session_id)
                if session is None and scope == "session":
                    session = self._begin_config_session(lease)
                applied = self._config_apply(checked, scope, force, apply_now)
            if session is None:
                return applied
            return applied.model_copy(
                update={"session_id": session.session_id, "lease_s": session.lease_s}
            )

    @rpc
    @final
    def config_reset(self, keep_session: bool = False) -> None:
        """Undo every live change and end the config session, unless ``keep_session``.

        Needs no session id, so any thread may end the session; without one
        it does nothing. For the same reason it renews no session's lease. A
        saved calibration stays as the arm runs it: no file is read again.
        Refused unless ``config_sessions``.
        """
        with self._config_call("config_reset"):
            keep = _flag("keep_session", keep_session)
            with self._hardware_lock:
                if not keep:
                    self._end_config_session("config_reset", expected=True)
                    return
                undone = self._config_undo()
            logger.info(f"{self._label}: config reset, the session goes on: {undone}")

    @rpc
    @final
    def config_command(self, targets: dict[str, float], session_id: str) -> CommandResult:
        """Send position targets by joint name inside the config session, as a command would.

        The targets are checked against the joint limits and restart the
        deadman; stop sending and the robot halts, which ends the session. A
        refused command raises ``ValueError``; one the hardware did not take
        returns ``ok`` False with the reason. Nothing limits how far one
        command jumps, so step the targets at the rate the arm may move.
        Refused unless ``config_sessions``.
        """
        with self._config_call("config_command"):
            if not isinstance(targets, dict):
                raise TypeError(
                    f"targets must be a dict of joint name to position, got {targets!r}"
                )
            if not isinstance(session_id, str) or not session_id:
                raise RuntimeError("config_command needs the session_id config_set returned")
            prefix = f"{self.config.source}{SEPARATOR}"
            suffix = f"{SEPARATOR}{Interface.POSITION.value}"
            known = {
                key.removeprefix(prefix).removesuffix(suffix)
                for key in self.limits
                if key.startswith(prefix) and key.endswith(suffix)
            }
            unknown = sorted(set(targets) - known)
            if unknown:
                raise ValueError(f"not position-commanded joints of {self._label}: {unknown}")
            values = {
                f"{prefix}{name}{suffix}": _real(name, value) for name, value in targets.items()
            }
            return self._command(values, session_id)

    def _config_keys(self) -> ConfigKeys:
        """What this driver lets a calibration tool read and change; by default nothing.

        Called with ``_hardware_lock`` held. ``config_keys`` fills in
        ``source``, ``sessions`` and ``max_lease_s``.
        """
        return ConfigKeys(source=self.config.source, sessions=self.config.config_sessions)

    def _config_state(self) -> dict[str, ArmConfigState]:
        """How each arm is driven now, by arm id; by default no arm. ``_hardware_lock`` held."""
        return {}

    def _config_apply(
        self, patch: ConfigPatch, scope: ConfigScope, force: bool, apply_now: bool
    ) -> ConfigApplied:
        """Apply a validated patch in ``scope``; by default every key is unsupported.

        Called with ``_hardware_lock`` held, inside the config session for
        scope "session". Raise to refuse the patch as a whole: that ends the
        session and undoes it.
        """
        keys = patch.keys()
        return ConfigApplied(
            arm_id=patch.arm_id,
            scope=scope,
            applied=ConfigPatch(arm_id=patch.arm_id),
            unsupported=keys,
            why=dict.fromkeys(keys, f"{self._label} cannot change it"),
            applied_now=False,
        )

    def _config_undo(self) -> str:
        """Undo every live change of the config session; says what, in a few words.

        Called with ``_hardware_lock`` held when a session ends and on
        ``config_reset``. By default there is nothing to undo.
        """
        return "nothing to undo"

    def _config_parkable(self) -> bool:
        """Whether the arm can still be parked, so a stop waits for it. ``_hardware_lock`` held."""
        return self._connected

    @contextmanager
    def _config_call(self, name: str, *, gated: bool = True) -> Iterator[None]:
        """One ``config_*`` call: refused unless ``config_sessions`` when ``gated``;
        raising ends the config session."""
        if gated and not self.config.config_sessions:
            raise RuntimeError(
                f"{self._label}: config sessions are off; only a connection deployed for "
                f"calibration (config_sessions=True) serves {name}"
            )
        try:
            yield
        except Exception as error:
            with self._hardware_lock:
                self._end_config_session(f"{name} raised {type(error).__name__}: {error}")
            raise

    def _check_config_session(self, session_id: str | None) -> _ConfigSession | None:
        """The live config session, renewed, once ``session_id`` names it.

        The one place a lease is renewed: only a call that carries the live
        session's id keeps it. ``None`` without one and without an id. A
        missing or foreign id for
        a live session raises, and so does an id once its session ended.
        Call with ``_hardware_lock`` held.
        """
        if session_id is not None and not isinstance(session_id, str):
            raise TypeError(f"session_id must be a string or None, got {session_id!r}")
        with self._lock:
            session = self._config_session
            ended = self._config_ended
            if session is not None and session_id == session.session_id:
                session.deadline = time.monotonic() + session.lease_s
                return session
        if session is not None:
            if session_id is None:
                raise RuntimeError(
                    "a config session is live; pass the session_id the config_set that "
                    "started it returned"
                )
            raise RuntimeError(f"{session_id[:8]!r} is not the live config session")
        if session_id is None:
            return None
        raise RuntimeError(
            f"the config session ended ({ended})" if ended else "no config session is live"
        )

    def _begin_config_session(self, lease_s: float) -> _ConfigSession:
        """Start the config session. Call with ``_hardware_lock`` held and none live.

        The deadman a position command armed before it is disarmed: those
        commands are dropped now, so their pause must not halt the robot and
        end the session. The session's own ``config_command`` arms it again.
        """
        session = _ConfigSession(uuid.uuid4().hex, lease_s, time.monotonic() + lease_s)
        with self._lock:
            self._config_session = session
            # A timer already due sees a newer command and does nothing.
            self._command_seq += 1
        self._on_loop(self._disarm_deadman)
        self._on_loop(self._arm_config_lease)
        logger.info(
            f"{self._label}: config session {session.session_id[:8]} started, lease "
            f"{lease_s:g} s; position commands are dropped until it ends"
        )
        return session

    def _end_config_session(
        self, reason: str, *, expected: bool = False, lapsed: bool = False
    ) -> bool:
        """End the config session, if any (with ``lapsed``, only one past its lease).

        Undoes its live changes (``_config_undo``) and logs one line: why,
        what was undone and how many position commands were dropped. Call
        with ``_hardware_lock`` held. Returns whether a session ended.
        """
        with self._lock:
            session = self._config_session
            if session is None or (lapsed and time.monotonic() <= session.deadline):
                return False
            if lapsed:
                reason = f"{reason} of {session.lease_s:g} s"
            self._config_session = None
            self._config_ended = reason
            self._config_ended_at = time.monotonic()
        self._on_loop(self._disarm_config_lease)
        try:
            undone = self._config_undo()
        except Exception as error:
            self._report(f"config session end failed: {type(error).__name__}: {error}")
            undone = (
                f"undoing it failed: {type(error).__name__}: {error}; "
                f"{self._halt_after_failed_undo(error)}"
            )
        message = (
            f"{self._label}: config session {session.session_id[:8]} ended ({reason}): "
            f"{undone}; {session.dropped} position commands dropped"
        )
        if expected:
            logger.info(message)
        else:
            logger.warning(message)
        return True

    def _halt_after_failed_undo(self, error: Exception) -> str:
        """After ``_config_undo`` raised: halt the robot, and fault when that fails too.

        The robot may still run the session's changes, a floating joint say,
        so it is held where it is (``halt``); when the halt raises as well,
        ``_config_fault`` latches the driver's fault. Says what was done.
        Call with ``_hardware_lock`` held.
        """
        try:
            self.halt()
        except Exception as halt_error:
            halted = f"{type(halt_error).__name__}: {halt_error}"
            self._report(f"halt failed: {halted}")
            self._config_fault(
                f"undoing the config session failed ({type(error).__name__}: {error}) and "
                f"halting failed ({halted})"
            )
            return f"halting failed too ({halted}); the connection faulted"
        return "the robot was halted"

    def _config_fault(self, reason: str) -> None:
        """Undoing a config session and halting the robot both failed; latch a fault.

        The robot may be left running the session's changes with nothing to
        keep them safe. A driver that can fault latches it here and makes the
        robot safe, e.g. de-energizes it; by default the failure is only
        reported. Called with ``_hardware_lock`` held, the session already
        ended.
        """

    def _dropped_for_config_session(self) -> bool:
        """Whether a position command is dropped for the config session; warns once per session.

        Call with ``_hardware_lock`` held.
        """
        with self._lock:
            session = self._config_session
            if session is None:
                return False
            session.dropped += 1
            first = session.dropped == 1
        if first:
            logger.warning(
                f"{self._label}: dropping position commands while a config session owns the arm"
            )
        return True

    def _await_config_park(self) -> None:
        """Before stopping, let the calibration tool park the arm and end its config session.

        The modules of a run stop in parallel, so a calibration tool may still
        be backing the arm out home when its connection is told to stop, and
        cutting the motors then would drop the arm where it is. A driver that
        serves config sessions calls this first in its own ``stop``, before
        its readings stop; ``stop`` here never does. Without
        ``config_sessions`` it returns at once. Waits while a session lasts,
        and for ``CONFIG_PARK_GRACE_S`` after the stop began or the last
        session ended for one about to begin, at most ``CONFIG_PARK_WAIT_S``
        in all, and not once the arm cannot be parked (``_config_parkable``).
        The deadman and the session lease run as usual meanwhile.
        """
        if not self.config.config_sessions:
            return
        started = time.monotonic()
        deadline = started + CONFIG_PARK_WAIT_S
        logged = False
        while True:
            with self._hardware_lock:
                parkable = self._config_parkable()
            with self._lock:
                active = self._config_session is not None
                ended_at = self._config_ended_at
            if not parkable:
                return
            now = time.monotonic()
            if not active:
                since = max(started, ended_at if ended_at is not None else started)
                if now - since >= CONFIG_PARK_GRACE_S:
                    return
            elif not logged:
                logger.warning(
                    f"{self._label}: stopping; waiting up to {CONFIG_PARK_WAIT_S:g} s for the "
                    "calibration tool to park the arm and end its config session"
                )
                logged = True
            if now >= deadline:
                logger.error(
                    f"{self._label}: the calibration tool did not end its config session within "
                    f"{CONFIG_PARK_WAIT_S:g} s; stopping with the arm where it is"
                )
                return
            time.sleep(0.02)

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

    def _arm_config_lease(self) -> None:
        """On the loop: end the config session once its lease is up, unless renewed."""
        self._disarm_config_lease()
        with self._lock:
            session = self._config_session
            if session is None:
                return
            delay = max(0.0, session.deadline - time.monotonic())
        self._config_lease = asyncio.get_running_loop().call_later(delay, self._config_lease_due)

    def _disarm_config_lease(self) -> None:
        if self._config_lease is not None:
            self._config_lease.cancel()
            self._config_lease = None

    def _config_lease_due(self) -> None:
        self._config_lease = None
        with self._hardware_lock:
            self._end_config_session("no config call within its lease", lapsed=True)
        # Renewed meanwhile, or a new session began: wait for its deadline.
        self._arm_config_lease()

    def _call_halt(self) -> None:
        # A halt ends the config session first: its live changes are undone,
        # then the robot holds.
        self._end_config_session("the connection halted")
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
