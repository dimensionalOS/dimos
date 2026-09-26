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

"""The part of every robot driver that is the same for every robot.

A driver for a robot is one module. Inside it, the driver writes only what is
particular to its hardware -- how to connect, how to read the joints, how to
send a command -- as a handful of small methods called hooks. It then creates a
``ConnectedHardware``, which does everything else:

  - publishes the robot's readings, after checking them
  - checks every incoming command and refuses anything unsafe or out of date
  - switches the hardware between ways of being driven when a command needs it
  - stops the robot, the way its description says, when commands stop
    arriving, readings stop arriving, the hardware reports a fault, or anyone
    asks it to
  - keeps track of whether the robot is allowed to move, and reports all of it

A typical driver::

    class XArmConnection(Module, ConnectionRpcMixin):
        @rpc
        def start(self) -> None:
            super().start()
            self.hw = ConnectedHardware(self, hooks=self, state_mode="poll")
            self.register_hardware(self.hw)
            self.hw.start()

        def connect(self) -> None: ...
        def describe(self) -> ControlDescription: ...
        def read_state(self) -> tuple[dict[str, float], float] | None: ...
        def write(self, frame: Frame) -> None: ...
        def shutdown(self) -> None: ...

Build it in ``start()``, not ``__init__``: modules are copied to their worker
process before they start, and the threads and locks in here cannot be copied.
"""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
from functools import partial
import threading
import time
from typing import Any, Literal, Protocol

from dimos.control.connection.safety import (
    damp_groups,
    estop_as_stop_kind,
    stop_values,
)
from dimos.control.connection.status import ConnectionStatus, LifecycleAck, LifecycleState
from dimos.control.contract.description import (
    ControlDescription,
    EstopKind,
    Omission,
    SafeStopKind,
)
from dimos.control.contract.sequence import Freshness
from dimos.control.contract.validate import (
    CommandBatch,
    FrameRejectedError,
    Rejected,
    validate_command,
    validate_description,
    validate_state,
)
from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

StateMode = Literal["poll", "push"]

#: Consecutive failed writes that stop the robot.
MAX_WRITE_FAILURES = 3

#: Answers to recent lifecycle requests kept for when a request is retried.
_ACK_MEMORY = 64


class WriteRejectedError(Exception):
    """Raise from a driver's ``write`` when the hardware refused a command.

    Three in a row stop the robot.
    """


@dataclass(frozen=True, slots=True)
class Frame:
    """One complete command, checked and ready to send to the hardware.

    Attributes:
        source: Which piece of hardware this is for, e.g. "arm".
        values: Every number to send, by full name, e.g.
            ``{"arm/joint1/position": 0.3}``. Names the robot marks as
            "leave unset" may be missing.
        active_groups: The ways of driving the hardware this command uses,
            e.g. ``{"position", "gripper"}``.
        epoch: The number the robot was armed under. For a frame sent while
            stopping, that of the last command obeyed, or 0 if none.
        sequence: The command's number within that arming. For a frame sent
            while stopping, that of the last command obeyed, or 0 if none.
    """

    source: str
    values: dict[str, float]
    active_groups: frozenset[str]
    epoch: int
    sequence: int

    def ordered(self, resources: Sequence[str], interface: str) -> list[float]:
        """The values of one interface across several parts, in the order given.

        Args:
            resources: Part names, e.g. ``["joint1", "joint2"]``.
            interface: What to read on each, e.g. ``"position"``.

        Raises:
            KeyError: If any of them is missing. Use ``get`` for values the
                robot allows to be left unset.
        """
        try:
            return [self.values[f"{self.source}/{name}/{interface}"] for name in resources]
        except KeyError as missing:
            raise KeyError(
                f"frame has no {missing.args[0]!r}; use get() for values that may be left unset"
            ) from None

    def get(self, key: str, default: float | None = None) -> float | None:
        """One value by full name, or ``default`` if it is not in this frame."""
        return self.values.get(key, default)


class Hooks(Protocol):
    """What a driver writes for its hardware. Usually the module itself.

    Required:
        connect: Open the connection to the hardware. Must not make anything
            move. If it raises, close whatever it opened first: ``shutdown``
            is only called once ``connect`` has succeeded.
        describe: Say what the hardware is, usually with one preset call. The
            ``epoch`` on what it returns is ignored.
        write: Send one ``Frame`` to the hardware. Raise ``WriteRejectedError`` if
            the hardware refuses it.
        shutdown: Close the connection.

    And one of:
        read_state() -> tuple[dict[str, float], float] | None: return the
            latest readings by full name, with when they were taken in
            seconds, or ``None`` if nothing new has arrived. Called at the
            description's ``state_rate_hz`` when ``state_mode="poll"``.
        or, with ``state_mode="push"``, call ``hw.ingest_state(values, ts)``
            from the driver's own thread whenever readings arrive.

    Optional, used when present:
        set_native(group: str) -> None: switch the hardware into one way of
            being driven, e.g. "velocity". Called before the first command
            that needs it. Raise on failure.
        on_prepare_arm() -> None: get ready to move, e.g. stand up. Raise on
            failure.
        on_commit_arm() -> None: the last step before obeying commands, e.g.
            enabling the motors. Raise on failure.
        on_safe_stop() -> None: the driver's own part of stopping. Called
            after any stop command has been sent, and may be called while a
            ``write`` is still in progress.
        on_estop() -> None: carry out an emergency stop, without going
            through ``write``. Required when the description's estop kind is
            DISABLE or VENDOR. Also used when a safe stop cannot be sent
            because a ``write`` has hung.
        fault() -> str | None: why the hardware is not working, or ``None``
            if it is fine. Checked continually, so it must return at once.
        clear_fault(latch: str) -> None: recover after a stop. ``latch`` is
            "safe_stop" or "estop".
        status_extras() -> dict[str, Any]: anything else worth showing.
            Values must pickle.
    """

    def connect(self) -> None: ...
    def describe(self) -> ControlDescription: ...
    def write(self, frame: Frame) -> None: ...
    def shutdown(self) -> None: ...


class _Publisher(Protocol):
    def publish(self, msg: ControlValues) -> None: ...


class _Subscriber(Protocol):
    def subscribe(self, cb: Callable[[ControlValues], Any]) -> Callable[[], None]: ...


class ControlPorts(Protocol):
    """A module with the two ports a robot driver talks on."""

    @property
    def control_state(self) -> _Publisher: ...
    @property
    def control_command(self) -> _Subscriber: ...


def missing_command_keys(desc: ControlDescription, batch: CommandBatch) -> list[str]:
    """What a checked command leaves out that a complete one must contain.

    Complete means every part of the robot that can be commanded is driven in
    exactly one way, and every value that way of driving needs is present.
    Values the robot allows to be left unset may be missing.

    An arm that can be driven by position or by speed is complete with all its
    positions, or with all its speeds, but not with a mix and not with both.

    Args:
        desc: The robot's description.
        batch: A command that has already passed ``validate_command``.

    Returns:
        The full names of missing values, plus ``"<source>/<part>"`` for any
        part the command does not drive at all. Empty when complete.
    """
    groups = {group.name: group for group in desc.mode_groups}
    driven: set[str] = set()
    missing: list[str] = []
    for name in sorted(batch.active_groups):
        group = groups[name]
        for resource_name in group.resources:
            resource = desc.resource(resource_name)
            if resource is None:
                continue
            driven.add(resource_name)
            for interface in resource.command_interfaces:
                if interface not in group.interfaces:
                    continue
                key = f"{desc.source}/{resource_name}/{interface}"
                if key not in batch.values and desc.omission_of(key) is not Omission.UNSET:
                    missing.append(key)
    for resource in desc.resources:
        if resource.name in driven or not resource.command_interfaces:
            continue
        keys = [f"{desc.source}/{resource.name}/{i}" for i in resource.command_interfaces]
        if any(desc.omission_of(key) is not Omission.UNSET for key in keys):
            missing.append(f"{desc.source}/{resource.name}")
    return missing


class ConnectedHardware:
    """Runs one piece of hardware on behalf of its driver.

    Args:
        module: The driver's module. Its ``control_state`` port carries the
            readings out; its ``control_command`` port brings commands in. A
            module that runs two pieces of hardware, such as a body and the
            wheels under it, creates one of these for each on the same ports.
        hooks: The driver's hardware-specific methods. See ``Hooks``.
        state_mode: "poll" to have ``read_state`` called on a timer, or "push"
            if the driver calls ``ingest_state`` itself.
        clock: Returns the current time in seconds, never going backwards.
            Only replaced in tests.
        description_epoch: The number to give the first description. Defaults
            to the current time in nanoseconds, so a driver that restarts
            always publishes a number it has not used before, and anything
            holding its old description knows to fetch the new one.
    """

    def __init__(
        self,
        module: ControlPorts,
        hooks: Hooks,
        *,
        state_mode: StateMode,
        clock: Callable[[], float] = time.monotonic,
        description_epoch: int | None = None,
    ) -> None:
        if state_mode not in ("poll", "push"):
            raise ValueError(f"state_mode must be 'poll' or 'push', got {state_mode!r}")
        self._module = module
        self._hooks = hooks
        self._state_mode: StateMode = state_mode
        self._clock = clock
        self._description_epoch = time.time_ns() if description_epoch is None else description_epoch

        # Held briefly around every read or change of what follows. Never held
        # while calling into the driver.
        self._lock = threading.RLock()
        # Serialises the slow lifecycle requests. Stops never wait on it.
        self._lifecycle_lock = threading.Lock()
        # Serialises commands, so they are handled in order.
        self._command_lock = threading.Lock()
        # Serialises calls to the driver's write().
        self._write_lock = threading.Lock()
        self._stopping = threading.Event()
        self._threads: list[threading.Thread] = []
        self._unsubscribe: Callable[[], None] | None = None
        self._started = False
        self._stopped = False

        self._desc: ControlDescription | None = None
        self._prefix = ""
        self._state = LifecycleState.STANDBY
        self._epoch: int | None = None
        self._last_sequence: int | None = None
        self._last_accepted: Frame | None = None
        # The command being written right now, if any. A stop is based on it,
        # since the hardware may be about to have it.
        self._in_flight: Frame | None = None
        # Driver calls that overran their time limit and are still running.
        self._overrunning = 0
        self._confirmed: frozenset[str] = frozenset()
        self._fault: str | None = None
        self._write_failures = 0
        self._state_freshness = Freshness()
        self._command_freshness = Freshness()
        self._measured: dict[str, float] = {}
        self._state_sequence = 0
        self._stop_started_at: float | None = None
        self._last_stop_emit: float | None = None
        self._rejections: dict[str, int] = {}
        self._last_log: dict[str, float] = {}
        self._acks: OrderedDict[tuple[str, int], LifecycleAck] = OrderedDict()

    @property
    def source(self) -> str:
        """Which piece of hardware this is, e.g. "arm"."""
        return self._description().source

    @property
    def state(self) -> LifecycleState:
        """Where it is in the lifecycle."""
        with self._lock:
            return self._state

    def start(self, *, background: bool = True) -> None:
        """Connect, check the description, and start listening.

        Args:
            background: Run the reading and supervising loops on their own
                threads. Pass False to run nothing in the background and call
                ``poll_state`` and ``supervise`` yourself, e.g. to step a
                simulator in lockstep.

        If anything fails after ``connect`` has succeeded, the driver's
        ``shutdown`` is called before the error is raised.

        Raises:
            DescriptionError: If the description breaks any rule.
            ValueError: If the description asks for something the hooks cannot
                do, such as an emergency stop only the driver can carry out
                with no ``on_estop`` hook.
        """
        if self._started:
            raise RuntimeError("ConnectedHardware.start() called twice")
        self._started = True
        self._hooks.connect()
        try:
            desc = replace(self._hooks.describe(), epoch=self._description_epoch)
            validate_description(desc)
            self._check_hooks(desc)
            self._log_unenforced(desc)
            with self._lock:
                self._desc = desc
                self._prefix = f"{desc.source}/"
                self._state = LifecycleState.STANDBY
            self._unsubscribe = self._module.control_command.subscribe(self.on_command)
            if background:
                self._spawn(self._supervise_loop, "supervise")
                if self._state_mode == "poll":
                    self._spawn(self._poll_loop, "poll")
        except BaseException:
            # Connected but unusable: close the connection before giving up.
            with self._lock:
                self._stopped = True
            self._stopping.set()
            if self._unsubscribe is not None:
                self._unsubscribe()
            try:
                self._hooks.shutdown()
            except Exception:
                logger.exception("driver shutdown after a failed start also failed")
            raise

    def stop(self) -> None:
        """Stop the robot if it is moving, then disconnect. Safe to call twice."""
        with self._lock:
            if self._stopped or not self._started:
                self._stopped = True
                return
            self._stopped = True
            armed = self._state is LifecycleState.ARMED
        if armed:
            try:
                self._run_safe_stop("shutdown")
            except Exception:
                logger.exception(f"{self._prefix} safe stop during shutdown failed")
        self._stopping.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        if self._unsubscribe is not None:
            try:
                self._unsubscribe()
            except Exception:
                logger.exception(f"{self._prefix} unsubscribing failed")
        try:
            self._hooks.shutdown()
        except Exception:
            logger.exception(f"{self._prefix} driver shutdown failed")

    def describe_control(self) -> ControlDescription:
        """The checked description currently in force."""
        return self._description()

    def redescribe(self) -> ControlDescription:
        """Ask the driver to describe the hardware again, e.g. after a setting
        that changes its speed limits.

        Only allowed while in STANDBY, so nothing is being commanded under the
        old description. The new one gets the next description number.

        Raises:
            RuntimeError: If not in STANDBY.
            DescriptionError: If the new description breaks any rule. The old
                one stays in force.
        """
        with self._lifecycle_lock:
            with self._lock:
                if self._state is not LifecycleState.STANDBY:
                    raise RuntimeError(f"can only re-describe in standby, not {self._state.value}")
                epoch = self._description_epoch + 1
            desc = replace(self._hooks.describe(), epoch=epoch)
            validate_description(desc)
            self._check_hooks(desc)
            with self._lock:
                self._description_epoch = epoch
                self._desc = desc
                self._prefix = f"{desc.source}/"
            return desc

    def status(self) -> ConnectionStatus:
        """A snapshot of the hardware and of how well it is being driven."""
        extras: dict[str, Any] = {}
        hook = getattr(self._hooks, "status_extras", None)
        if hook is not None:
            try:
                extras = dict(hook())
            except Exception as error:
                extras = {"status_extras_error": f"{type(error).__name__}: {error}"}
        desc = self._description()
        now = self._clock()
        with self._lock:
            return ConnectionStatus(
                source=desc.source,
                state=self._state,
                epoch=self._epoch,
                description_epoch=self._description_epoch,
                last_accepted_sequence=self._last_sequence,
                confirmed_groups=self._confirmed,
                fault=self._fault,
                state_fresh=not self._state_freshness.is_stale(now, desc.timing.stale_timeout_s),
                command_fresh=not self._command_freshness.is_stale(
                    now, desc.timing.watchdog_timeout_s
                ),
                rejections=tuple(sorted(self._rejections.items())),
                extras=extras,
            )

    def poll_state(self) -> None:
        """Take one reading from the driver and publish it.

        The background loop calls this at the description's ``state_rate_hz``
        when ``state_mode`` is "poll".
        """
        read = getattr(self._hooks, "read_state", None)
        if read is None:
            return
        try:
            sample = read()
        except Exception as error:
            self._reject("read_failed", f"{type(error).__name__}: {error}")
            return
        if sample is not None:
            values, ts = sample
            self.ingest_state(values, ts)

    def ingest_state(self, values: Mapping[str, float], ts: float) -> None:
        """Check one set of readings and publish it.

        A set that does not contain exactly the readings the description
        promises, or that contains a value that is not a finite number, is
        counted and dropped, never published.

        Readings that arrive before ``start`` has finished are dropped. A
        driver's own reading thread often begins inside ``connect``, before
        there is a description to check them against.

        Args:
            values: Every reading, by full name, e.g.
                ``{"arm/joint1/position": 0.3}``.
            ts: When the readings were taken, in seconds.
        """
        desc = self._desc
        if desc is None:
            return
        with self._lock:
            sequence = self._state_sequence
            self._state_sequence += 1
        try:
            frame = ControlValues(
                source=desc.source,
                source_ts=ts,
                epoch=desc.epoch,
                sequence=sequence,
                interface_names=list(values),
                values=[float(v) for v in values.values()],
            )
            checked = validate_state(desc, frame)
        except (FrameRejectedError, ValueError, TypeError) as error:
            self._reject("bad_state", str(error))
            return
        with self._lock:
            self._measured = checked
            self._state_freshness.mark(self._clock())
        self._module.control_state.publish(frame)

    def on_command(self, frame: ControlValues) -> None:
        """Handle one command from the ``control_command`` port.

        A command with nothing for this hardware is ignored: it is for another
        piece of hardware on the same port. Otherwise the command is obeyed
        only if the hardware is armed and the command is current, in order,
        within limits, complete, and drives each part in only one way.
        Anything else is counted by reason and dropped whole.
        """
        with self._command_lock:
            self._handle_command(frame)

    def supervise(self) -> None:
        """Check once that it is safe to carry on, and keep a stopped robot
        stopped.

        While armed, stops the robot if the hardware reports a fault, if
        readings have stopped arriving, or if commands have. While stopped,
        re-sends the stop command at the description's ``state_rate_hz``.
        The background loop calls this several times per watchdog period.
        """
        desc = self._description()
        timing = desc.timing
        fault = self._poll_fault()
        now = self._clock()
        with self._lock:
            state = self._state
            state_stale = self._state_freshness.is_stale(now, timing.stale_timeout_s)
            command_stale = self._command_freshness.is_stale(now, timing.watchdog_timeout_s)
            emit_due = (
                self._last_stop_emit is None
                or now - self._last_stop_emit >= 1.0 / timing.state_rate_hz
            )
        if state is LifecycleState.ARMED:
            if fault is not None:
                self._run_safe_stop(fault)
            elif state_stale:
                self._run_safe_stop("state_stale")
            elif command_stale:
                self._run_safe_stop("watchdog")
            return
        if state_stale:
            self._log_throttled("state_stale", f"{desc.source}: no fresh readings")
        if emit_due and self._stop_kind(state) is not None:
            self._emit_stop_frame()

    def prepare_arm(self, operation_id: int) -> LifecycleAck:
        """Get the hardware ready to move, e.g. stand it up.

        Only from STANDBY. Runs the driver's ``on_prepare_arm``, bounded by the
        description's ``prepare_arm_timeout_s``. On success the hardware is
        PREPARED and waits for ``commit_arm``. If the driver fails or takes
        too long, the hardware is SAFE_STOPPED, since it may be part-way
        through moving.

        Args:
            operation_id: A number for this request, different for every new
                one. Sending the same number again returns the first answer
                instead of a "wrong state" refusal, so a retry is safe.
        """
        cached = self._remembered("prepare_arm", operation_id)
        if cached is not None:
            return cached
        with self._lifecycle_lock:
            with self._lock:
                if self._state is not LifecycleState.STANDBY:
                    return self._ack(operation_id, False, f"cannot prepare in {self._state.value}")
                self._state = LifecycleState.PREPARING
            timeout = self._description().timing.prepare_arm_timeout_s
            ok, why = self._run_hook("on_prepare_arm", timeout)
            interrupted = self._interrupted(operation_id, "prepare")
            if interrupted is not None:
                return interrupted
            if not ok:
                self._run_safe_stop(f"prepare_arm: {why}")
                return self._ack(operation_id, False, why)
            with self._lock:
                self._state = LifecycleState.PREPARED
            return self._remember("prepare_arm", self._ack(operation_id, True))

    def commit_arm(self, operation_id: int, epoch: int) -> LifecycleAck:
        """Start obeying commands, under the given arming number.

        Only from PREPARED, only while readings are arriving on time, and only
        if the hardware reports no fault. Runs the driver's ``on_commit_arm``.
        From then on only commands carrying ``epoch`` are obeyed, and the
        first one must arrive within the description's ``watchdog_timeout_s``.

        Args:
            operation_id: A number for this request.
            epoch: The arming number commands will carry. Zero or more.
        """
        cached = self._remembered("commit_arm", operation_id)
        if cached is not None:
            return cached
        if epoch < 0:
            return self._ack(operation_id, False, f"epoch must be zero or more, got {epoch}")
        with self._lifecycle_lock:
            timing = self._description().timing
            with self._lock:
                if self._state is not LifecycleState.PREPARED:
                    return self._ack(operation_id, False, f"cannot commit in {self._state.value}")
                if self._state_freshness.is_stale(self._clock(), timing.stale_timeout_s):
                    return self._ack(operation_id, False, "no fresh readings from the hardware")
            fault = self._poll_fault()
            if fault is not None:
                return self._ack(operation_id, False, f"hardware fault: {fault}")
            ok, why = self._run_hook("on_commit_arm", timing.hook_timeout_s)
            interrupted = self._interrupted(operation_id, "commit")
            if interrupted is not None:
                return interrupted
            if not ok:
                self._run_safe_stop(f"commit_arm: {why}")
                return self._ack(operation_id, False, why)
            with self._lock:
                self._state = LifecycleState.ARMED
                self._epoch = epoch
                self._last_sequence = None
                self._last_accepted = None
                self._confirmed = frozenset()
                self._write_failures = 0
                self._fault = None
                self._command_freshness.mark(self._clock())
            return self._remember("commit_arm", self._ack(operation_id, True))

    def abort_arm(self, operation_id: int) -> LifecycleAck:
        """Give up on arming and go back to STANDBY. Only before ARMED.

        Does not undo what ``on_prepare_arm`` did; a robot that stood up stays
        standing. To stop an armed robot, use ``safe_stop``.
        """
        cached = self._remembered("abort_arm", operation_id)
        if cached is not None:
            return cached
        with self._lifecycle_lock, self._lock:
            if self._state not in (LifecycleState.PREPARING, LifecycleState.PREPARED):
                return self._ack(operation_id, False, f"cannot abort in {self._state.value}")
            self._state = LifecycleState.STANDBY
            return self._remember("abort_arm", self._ack(operation_id, True))

    def safe_stop(self, operation_id: int, reason: str = "requested") -> LifecycleAck:
        """Stop the robot the way its description asks, and stay stopped.

        Works from any state and never waits for another request to finish.
        Asking again while already stopped changes nothing and succeeds.

        If the stop cannot be sent -- a write to the hardware has hung -- the
        driver's own emergency stop is used instead, and the answer shows
        ESTOPPED. With no such hook the robot is still marked stopped, but the
        answer is not ``ok``: nothing is known to have reached the hardware.

        Args:
            operation_id: A number for this request.
            reason: Why, for the status report.
        """
        with self._lock:
            state = self._state
        if state is LifecycleState.ESTOPPED:
            return self._ack(operation_id, True, "already emergency-stopped")
        if state is LifecycleState.SAFE_STOPPED:
            return self._ack(operation_id, True, "already stopped")
        problem = self._run_safe_stop(reason)
        return self._ack(operation_id, problem is None, problem or "")

    def estop(self, operation_id: int, reason: str = "requested") -> LifecycleAck:
        """Stop the robot in an emergency, and stay stopped.

        Always acts, even if already stopped, and takes over from any other
        stop. Never waits for another request to finish. The answer is not
        ``ok`` if the stop could not be carried out.

        Args:
            operation_id: A number for this request.
            reason: Why, for the status report.
        """
        problem = self._run_estop(reason)
        return self._ack(operation_id, problem is None, problem or "")

    def clear_safe_stop(self, operation_id: int) -> LifecycleAck:
        """Release a stop, back to STANDBY. The robot does not move until it is
        prepared and committed again.

        Refused while the hardware still reports a fault, or while a driver
        call that ran out of time is still running and could yet act.
        """
        return self._clear(operation_id, LifecycleState.SAFE_STOPPED, "safe_stop")

    def clear_estop(self, operation_id: int) -> LifecycleAck:
        """Release an emergency stop, back to STANDBY. The robot does not move
        until it is prepared and committed again.

        Refused while the hardware still reports a fault, or while a driver
        call that ran out of time is still running and could yet act.
        """
        return self._clear(operation_id, LifecycleState.ESTOPPED, "estop")

    def _handle_command(self, frame: ControlValues) -> None:
        prefix = self._prefix
        if not any(name.startswith(prefix) for name in frame.interface_names):
            return
        desc = self._description()
        with self._lock:
            if self._state is not LifecycleState.ARMED or self._epoch is None:
                self._reject("not_armed", f"command while {self._state.value}")
                return
            epoch = self._epoch
            last_sequence = self._last_sequence
            confirmed = self._confirmed
        result = validate_command(desc, frame, current_epoch=epoch, last_sequence=last_sequence)
        if isinstance(result, Rejected):
            self._reject(result.reason, result.detail)
            return
        missing = missing_command_keys(desc, result)
        if missing:
            self._reject("incomplete", f"missing {missing}")
            return
        with self._lock:
            if self._epoch != epoch:
                return
            # Taken now, so a replay of this command is refused whether or not
            # the write below succeeds.
            self._last_sequence = frame.sequence
        if not self._switch_groups(desc, result.active_groups, confirmed, epoch):
            return
        out = Frame(desc.source, result.values, result.active_groups, epoch, frame.sequence)
        error: str | None = None
        with self._write_lock:
            with self._lock:
                if self._state is not LifecycleState.ARMED or self._epoch != epoch:
                    return
                self._in_flight = out
            try:
                self._hooks.write(out)
            except Exception as failure:
                error = f"{type(failure).__name__}: {failure}"
        with self._lock:
            self._in_flight = None
            if error is None:
                # Whatever happened meanwhile, this is what the hardware has.
                self._last_accepted = out
            still_armed = self._state is LifecycleState.ARMED and self._epoch == epoch
        if not still_armed:
            # Stopped while this command was on its way. Send the stop again,
            # so that it, not this command, is the last thing sent.
            self._reassert_stop()
            return
        if error is not None:
            with self._lock:
                self._write_failures += 1
                failures = self._write_failures
            self._reject("write_failed", error)
            if failures >= MAX_WRITE_FAILURES:
                self._run_safe_stop("write_failed")
            return
        with self._lock:
            self._write_failures = 0
            self._command_freshness.mark(self._clock())

    def _switch_groups(
        self,
        desc: ControlDescription,
        active: frozenset[str],
        confirmed: frozenset[str],
        epoch: int,
    ) -> bool:
        """Switch the hardware into the ways of driving a command needs.

        Only the ones that exclude others need switching; the rest are always
        available. Returns whether the command may now be sent.
        """
        exclusive = {group.name for group in desc.mode_groups if group.exclusive}
        entering = sorted((active - confirmed) & exclusive)
        set_native = getattr(self._hooks, "set_native", None)
        if set_native is not None:
            for group in entering:
                ok, why = self._run_bounded(
                    partial(set_native, group),
                    desc.timing.hook_timeout_s,
                    label=f"set_native({group})",
                    activates=True,
                )
                if self.state is LifecycleState.ESTOPPED:
                    self._assert_estop()
                    return False
                if not ok:
                    self._run_safe_stop(f"set_native({group}): {why}")
                    return False
        with self._lock:
            if self._state is not LifecycleState.ARMED or self._epoch != epoch:
                return False
            self._confirmed = active
        return True

    def _run_safe_stop(self, reason: str) -> str | None:
        """Latch SAFE_STOPPED and stop the hardware.

        Returns why the stop may not have reached the hardware, or ``None`` if
        it did, or if there was nothing moving to stop.
        """
        with self._lock:
            if self._state in (LifecycleState.SAFE_STOPPED, LifecycleState.ESTOPPED):
                return None
            # Latch first, so no command can slip in behind the stop.
            self._state = LifecycleState.SAFE_STOPPED
            self._epoch = None
            self._fault = reason
            self._stop_started_at = self._clock()
            self._last_stop_emit = None
        logger.warning(f"{self._prefix} safe stop: {reason}")
        problem = self._stop_hardware()
        if problem is None:
            return None
        if getattr(self._hooks, "on_estop", None) is not None:
            return self._run_estop(f"{reason}; {problem}")
        with self._lock:
            self._fault = f"{reason}; {problem}"
        logger.error(f"{self._prefix} safe stop may not have reached the hardware: {problem}")
        return problem

    def _stop_hardware(self) -> str | None:
        """Send the safe-stop command and run the driver's own stop.

        Returns why the stop may not have reached the hardware, or ``None``.
        """
        sent = self._emit_stop_frame()
        ok, why = self._run_hook("on_safe_stop", self._description().timing.hook_timeout_s)
        if not ok:
            self._reject("on_safe_stop_failed", why)
        if self._description().safe_stop.kind is SafeStopKind.VENDOR:
            return None if ok else f"on_safe_stop: {why}"
        if sent is False:
            return "stop not sent: a write to the hardware did not finish"
        return None

    def _run_estop(self, reason: str) -> str | None:
        """Latch ESTOPPED and stop the hardware.

        Returns why the stop may not have reached the hardware, or ``None``.
        """
        with self._lock:
            if self._state is not LifecycleState.ESTOPPED:
                self._fault = reason
                self._stop_started_at = self._clock()
                self._last_stop_emit = None
            self._state = LifecycleState.ESTOPPED
            self._epoch = None
        logger.warning(f"{self._prefix} emergency stop: {reason}")
        problem = self._assert_estop()
        if problem is not None:
            logger.error(
                f"{self._prefix} emergency stop may not have reached the hardware: {problem}"
            )
        return problem

    def _assert_estop(self) -> str | None:
        if getattr(self._hooks, "on_estop", None) is not None:
            ok, why = self._run_hook("on_estop", self._description().timing.hook_timeout_s)
            return None if ok else f"on_estop: {why}"
        if self._emit_stop_frame() is False:
            return "stop not sent: a write to the hardware did not finish"
        return None

    def _reassert_stop(self) -> None:
        """Stop the hardware again, the way the current latch says. For when
        something may have undone the stop: a write or driver call that was
        still running when the robot was stopped, and finished afterwards."""
        state = self.state
        if state is LifecycleState.ESTOPPED:
            self._assert_estop()
        elif state is LifecycleState.SAFE_STOPPED:
            self._stop_hardware()

    def _stop_kind(self, state: LifecycleState) -> SafeStopKind | None:
        """How a robot in this state is being held stopped by this object, or
        ``None`` if it is not stopped or its driver handles the stop."""
        desc = self._description()
        if state is LifecycleState.SAFE_STOPPED:
            kind = desc.safe_stop.kind
        elif state is LifecycleState.ESTOPPED and getattr(self._hooks, "on_estop", None) is None:
            kind = estop_as_stop_kind(desc.estop.kind)
        else:
            return None
        return None if kind is SafeStopKind.VENDOR else kind

    def _emit_stop_frame(self) -> bool | None:
        """Send the command that holds the robot stopped.

        Returns True if it was sent, False if it could not be, and ``None``
        if there is nothing to send: the driver handles the stop itself, or
        nothing was ever commanded.
        """
        desc = self._description()
        now = self._clock()
        with self._lock:
            kind = self._stop_kind(self._state)
            last = self._in_flight or self._last_accepted
            measured = dict(self._measured)
            elapsed = now - (self._stop_started_at if self._stop_started_at is not None else now)
            self._last_stop_emit = now
        if kind is None:
            return None
        values = stop_values(desc, kind, last.values if last else None, measured, elapsed_s=elapsed)
        if values is None:
            return None
        frame = Frame(
            source=desc.source,
            values=values,
            active_groups=last.active_groups if last else damp_groups(desc),
            epoch=last.epoch if last else 0,
            sequence=last.sequence if last else 0,
        )
        # A write that has hung must not hold up a stop for ever. The caller
        # falls back on the driver's emergency stop, and the stop is sent
        # again when the hung write returns.
        if not self._write_lock.acquire(timeout=desc.timing.hook_timeout_s):
            self._reject("stop_write_blocked", "a write is still in progress")
            return False
        try:
            self._hooks.write(frame)
        except Exception as error:
            self._reject("stop_write_failed", f"{type(error).__name__}: {error}")
            return False
        finally:
            self._write_lock.release()
        return True

    def _clear(self, operation_id: int, latched: LifecycleState, latch: str) -> LifecycleAck:
        name = f"clear_{latch}"
        cached = self._remembered(name, operation_id)
        if cached is not None:
            return cached
        with self._lifecycle_lock:
            with self._lock:
                if self._state is not latched:
                    return self._ack(
                        operation_id, False, f"not {latched.value}, but {self._state.value}"
                    )
                if self._overrunning:
                    return self._ack(
                        operation_id, False, "a driver call that timed out is still running"
                    )
            fault = self._poll_fault()
            if fault is not None:
                return self._ack(operation_id, False, f"hardware fault: {fault}")
            clear_fault = getattr(self._hooks, "clear_fault", None)
            if clear_fault is not None:
                ok, why = self._run_bounded(
                    partial(clear_fault, latch),
                    self._description().timing.hook_timeout_s,
                    label="clear_fault",
                    activates=True,
                )
                if not ok:
                    return self._ack(operation_id, False, f"clear_fault: {why}")
            with self._lock:
                if self._state is not latched:
                    return self._ack(operation_id, False, f"stopped again: {self._state.value}")
                self._state = LifecycleState.STANDBY
                self._fault = None
                self._last_accepted = None
                self._stop_started_at = None
                self._last_stop_emit = None
            return self._remember(name, self._ack(operation_id, True))

    def _interrupted(self, operation_id: int, step: str) -> LifecycleAck | None:
        """After a slow hook returns: if the robot was stopped meanwhile, the
        request failed. An emergency stop is asserted again, in case the hook
        undid it."""
        with self._lock:
            state = self._state
        if state is LifecycleState.ESTOPPED:
            self._assert_estop()
            return self._ack(operation_id, False, f"emergency-stopped during {step}")
        if state is LifecycleState.SAFE_STOPPED:
            return self._ack(operation_id, False, f"stopped during {step}")
        return None

    def _poll_fault(self) -> str | None:
        hook = getattr(self._hooks, "fault", None)
        if hook is None:
            return None
        try:
            fault = hook()
        except Exception as error:
            return f"fault() raised {type(error).__name__}: {error}"
        return None if fault is None else str(fault)

    def _run_hook(self, name: str, timeout_s: float) -> tuple[bool, str]:
        hook = getattr(self._hooks, name, None)
        if hook is None:
            return True, ""
        activates = name in ("on_prepare_arm", "on_commit_arm")
        return self._run_bounded(hook, timeout_s, label=name, activates=activates)

    def _run_bounded(
        self,
        call: Callable[[], Any],
        timeout_s: float,
        *,
        label: str,
        activates: bool = False,
    ) -> tuple[bool, str]:
        """Call a driver hook on a thread of its own and give up waiting after
        ``timeout_s`` seconds.

        Giving up does not stop the hook: Python cannot cancel a thread. So
        while it keeps running, no stop can be cleared, and if it was one that
        can make the hardware move (``activates``) and it finishes after all,
        the stop is sent again in case the hook undid it.
        """
        guard = threading.Lock()
        outcome: dict[str, Any] = {"done": False, "abandoned": False, "error": None}

        def target() -> None:
            try:
                call()
            except BaseException as error:
                outcome["error"] = error
            with guard:
                outcome["done"] = True
                abandoned = outcome["abandoned"]
            if abandoned:
                self._after_overrun(label, activates)

        thread = threading.Thread(target=target, daemon=True, name=f"{self._prefix}{label}")
        thread.start()
        thread.join(timeout_s)
        with guard:
            if not outcome["done"]:
                outcome["abandoned"] = True
                with self._lock:
                    self._overrunning += 1
                return False, "hook_timeout"
        error = outcome["error"]
        if error is not None:
            return False, f"{type(error).__name__}: {error}"
        return True, ""

    def _after_overrun(self, label: str, activates: bool) -> None:
        """A hook that had been given up on has finished."""
        try:
            logger.warning(f"{self._prefix} {label} finished after timing out")
            if activates:
                if self.state in (LifecycleState.SAFE_STOPPED, LifecycleState.ESTOPPED):
                    self._reassert_stop()
                else:
                    self._run_safe_stop(f"{label} finished after timing out")
        finally:
            with self._lock:
                self._overrunning -= 1

    def _check_hooks(self, desc: ControlDescription) -> None:
        problems: list[str] = []
        if self._state_mode == "poll" and getattr(self._hooks, "read_state", None) is None:
            problems.append('state_mode "poll" needs a read_state hook')
        if (
            desc.safe_stop.kind is SafeStopKind.VENDOR
            and getattr(self._hooks, "on_safe_stop", None) is None
        ):
            problems.append("safe_stop kind VENDOR needs an on_safe_stop hook")
        if getattr(self._hooks, "on_estop", None) is None:
            if desc.estop.kind in (EstopKind.DISABLE, EstopKind.VENDOR):
                problems.append(f"estop kind {desc.estop.kind.name} needs an on_estop hook")
            if desc.estop.kind is EstopKind.DAMP and not desc.safe_stop.kd:
                problems.append("estop kind DAMP needs a damping table in safe_stop.kd")
        if problems:
            raise ValueError(f"{desc.source}: " + "; ".join(problems))

    def _log_unenforced(self, desc: ControlDescription) -> None:
        declared = [
            name
            for name, value in (
                ("covered_resources", desc.covered_resources),
                ("available_after", desc.available_after),
                ("shutdown_motion", desc.shutdown_motion),
            )
            if value
        ]
        if declared:
            logger.info(f"{desc.source}: {', '.join(declared)} declared but not yet enforced")

    def _description(self) -> ControlDescription:
        desc = self._desc
        if desc is None:
            raise RuntimeError("ConnectedHardware has not been started")
        return desc

    def _ack(self, operation_id: int, ok: bool, reason: str = "") -> LifecycleAck:
        with self._lock:
            state = self._state
        return LifecycleAck(operation_id, self._description().source, state, ok, reason)

    def _remembered(self, name: str, operation_id: int) -> LifecycleAck | None:
        with self._lock:
            return self._acks.get((name, operation_id))

    def _remember(self, name: str, ack: LifecycleAck) -> LifecycleAck:
        with self._lock:
            self._acks[(name, ack.operation_id)] = ack
            while len(self._acks) > _ACK_MEMORY:
                self._acks.popitem(last=False)
        return ack

    def _reject(self, reason: str, detail: str) -> None:
        with self._lock:
            self._rejections[reason] = self._rejections.get(reason, 0) + 1
        self._log_throttled(reason, f"{self._prefix} {reason}: {detail}")

    def _log_throttled(self, key: str, message: str) -> None:
        now = self._clock()
        with self._lock:
            last = self._last_log.get(key)
            if last is not None and now - last < 1.0:
                return
            self._last_log[key] = now
        logger.warning(message)

    def _spawn(self, target: Callable[[], None], name: str) -> None:
        thread = threading.Thread(target=target, daemon=True, name=f"{self._prefix}{name}")
        self._threads.append(thread)
        thread.start()

    def _poll_loop(self) -> None:
        period = 1.0 / self._description().timing.state_rate_hz
        self._paced(self.poll_state, period)

    def _supervise_loop(self) -> None:
        timing = self._description().timing
        period = min(1.0 / timing.state_rate_hz, timing.watchdog_timeout_s / 4.0)
        self._paced(self.supervise, period)

    def _paced(self, step: Callable[[], None], period: float) -> None:
        """Call ``step`` every ``period`` seconds until stopped, without
        drifting, and without bursting to catch up after falling behind."""
        next_at = self._clock()
        while not self._stopping.is_set():
            try:
                step()
            except Exception:
                logger.exception(f"{self._prefix} background step failed")
            next_at += period
            delay = next_at - self._clock()
            if delay < 0:
                next_at = self._clock()
                delay = 0.0
            self._stopping.wait(delay)
