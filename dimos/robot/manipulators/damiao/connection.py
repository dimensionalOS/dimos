# Copyright 2026 Dimensional Inc.
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

"""The DimOS side of a Damiao robot: ports, limits, timing and faults.

The motor layer lives in :mod:`dimos.robot.manipulators.damiao.arm` and the
robot's data in :mod:`dimos.robot.manipulators.damiao.robot`. This module only
translates streams into targets, publishes readings at a fixed rate, and
latches a fault when feedback breaks.
"""

from __future__ import annotations

from abc import abstractmethod
import threading
import time
from typing import Any, ClassVar
import uuid

from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import limits as limits_of, movable_joints
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao.arm import ArmReading, DamiaoArm
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def zero_stiffness_joints(robot: DamiaoRobot, *, teach: bool, source: str = "arm") -> list[str]:
    """Joints a teaching connection commands with zero stiffness, in joint order.

    Hand-guided tasks use this declaration to prove the deployment will not
    fight the operator: every arm joint plus the jaws that follow the hand.
    Names carry the connection's robot name, e.g. ``arm/yam_joint1``.
    """
    if not teach:
        return []
    names = [
        *robot.arm_joint_names,
        *(gripper.joint for gripper in robot.grippers if gripper.name in robot.passive_grippers),
    ]
    return [f"{source}/{name}" for name in names]


class DamiaoConnectionConfig(ConnectionConfig):
    teach: bool = False
    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)
    deadman_timeout_s: float = Field(default=0.1, gt=0, allow_inf_nan=False)
    #: How old a target may be and still be written, in seconds, checked
    #: against its local receipt and, when the command carries one, its
    #: source timestamp. Defaults to ``deadman_timeout_s``: a target older
    #: than the silence that halts the robot is never executed.
    command_max_age_s: float | None = Field(default=None, gt=0, allow_inf_nan=False)

    @model_validator(mode="after")
    def _position_only(self) -> Self:
        if any(interfaces != (Interface.POSITION,) for interfaces in self.joint_commands.values()):
            raise ValueError("Damiao connections accept position commands only")
        return self


class DamiaoConnection(ConnectionModule):
    """Own one Damiao robot, its measured states and position targets."""

    #: The robot this connection deploys: motors, gains, grippers and models.
    robot: ClassVar[DamiaoRobot]
    config: DamiaoConnectionConfig
    position_command: In[JointState]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        kwargs.setdefault(
            "joint_commands", {name: (Interface.POSITION,) for name in self.robot.joint_names}
        )
        super().__init__(**kwargs)
        self._arm: DamiaoArm | None = None
        self._fault: str | None = None
        self._state_frame = ""
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None
        # The one pending target, by key: value, local receipt (monotonic)
        # and source timestamp (wall clock, if the command carried one).
        # Bounded by the number of command keys; the owner thread writes the
        # newest value of each key, never a FIFO backlog.
        self._pending_lock = threading.Lock()
        self._pending: dict[str, tuple[float, float, float | None]] = {}
        # Most keys ever pending at once, for observability and tests.
        self._pending_max = 0
        # Bumped by every invalidation, so a target taken just before a stop
        # or fault cannot start an SDK write after it.
        self._generation = 0
        # The generation of the target ``_take_fresh`` returned last.
        self._taken_generation = 0
        # Wakes the owner for new targets, safety requests and shutdown.
        self._owner_wake = threading.Event()
        # A due deadman halt, handed to the owner as (deadman, seq).
        self._halt_request: tuple[bool, int] | None = None
        self._disable_request = False
        self._hardware_closed = False
        self._close_error: Exception | None = None

    @property
    def joint_names(self) -> list[str]:
        return list(self.robot.joint_names)

    @property
    def arm_joints(self) -> dict[str, list[str]]:
        return self.robot.arm_joints

    @property
    def gripper_joints(self) -> dict[str, str]:
        return self.robot.gripper_joints

    @abstractmethod
    def can_ports(self) -> list[str]:
        """Return one explicitly configured CAN selector per robot bus, in order."""

    def connect(self) -> None:
        """Validate the deployed model and topology, then open the buses."""
        assert self.config.model is not None
        if {joint.name for joint in movable_joints(self.config.model)} != set(self.joint_names):
            raise ValueError("connection model joints must match the vendor topology")
        self._state_frame = self.frame(self.config.model.load().root_link)
        ports = self.can_ports()
        if len(ports) != len(self.robot.buses) or any(not port.strip() for port in ports):
            raise ValueError("CAN selectors must cover exactly the robot buses")
        if len(set(ports)) != len(ports):
            raise ValueError("CAN selectors must be distinct")
        arm = DamiaoArm(self.robot, ports, teach=self.config.teach)
        arm.connect()
        self._arm = arm
        self._read_state()

    @rpc
    def start(self) -> None:
        # One instance, one life. The inherited Module lifecycle disposes
        # the event loop, RPC and transports in stop(), so a started
        # connection is never started again: deploy a new module instance
        # instead. The guard runs before Module.start, any state/session
        # reset or any hardware open, so a re-entry while a previous SDK
        # owner is still alive - or after it, with cleanup done - can
        # neither reset the bookkeeping that owner relies on, nor open a
        # second device behind its back, nor replay old targets. The
        # previous owner keeps the hardware, the error status stays, and
        # its deferred close still finishes on its own thread. A bare
        # connect() before the first start stays legal.
        reader = self._reader
        if self._module_closed or reader is not None:
            detail = (
                "the previous SDK owner is still alive"
                if reader is not None and reader.is_alive()
                else "this instance was already started"
            )
            raise RuntimeError(
                f"{type(self).__name__} is not restartable ({detail}); deploy a new module instance"
            )
        try:
            Module.start(self)
            self._session_id = uuid.uuid4().hex
            self._limits = limits_of(self.config)
            self._invalidate_pending()
            self._pending_max = 0
            self._fault = None
            self._hardware_closed = False
            self._close_error = None
            self._halt_request = None
            self._disable_request = False
            self.connect()
            with self._hardware_lock:
                arm = self._arm
                assert arm is not None
                logger.info(
                    "Enabling robot and calibrating grippers",
                    connection=type(self).__name__,
                    source=self.config.source,
                )
                arm.enable()
                self._read_state()
                self.register_disposable(
                    Disposable(self.position_command.subscribe(self._on_position))
                )
                self._reader = threading.Thread(
                    target=self._read_loop, daemon=True, name=f"{self.config.source}-read"
                )
                self._reader.start()
                self._connected = True
                logger.info(
                    "Robot connection ready",
                    connection=type(self).__name__,
                    source=self.config.source,
                )
        except Exception:
            try:
                self.stop()
            except Exception as cleanup_error:
                self._report(
                    f"startup cleanup failed: {type(cleanup_error).__name__}: {cleanup_error}"
                )
            raise

    @rpc
    def stop(self) -> None:
        # Mark the stop before anything that can wait on the owner: every
        # caller must be able to observe the lifecycle change immediately,
        # even while the owner is parked inside an admission check or an SDK
        # call. The owner itself is released by the same flag and closes the
        # hardware once its current call returns.
        #
        # Contract: a normal return means the stop was *requested* and
        # bounded, not that the hardware is already halted and closed. When
        # the SDK owner was still busy (in a call, a publish or a disable)
        # past the join budget, this returns with the device still owned by
        # that thread: ``status().connected`` stays True and
        # ``status().last_error`` reports the incomplete cleanup until the
        # owner finishes closing. The instance is then disposed - ``start``
        # raises RuntimeError - so no second SDK owner can ever be created
        # on it; deploy a new module instance for a fresh connection.
        self._stop_reading.set()
        self._owner_wake.set()
        super().stop()

    def _shutdown_hardware(self) -> None:
        """Leave the close with the live SDK owner, under a bound.

        The owner halts and disables the hardware and then exits, so nothing
        is closed from another thread while it is inside an SDK call. If it
        is still alive after the join budget - inside an SDK call, or merely
        publishing - this only invalidates pending work, reports the timeout
        and returns: the hardware lock is deliberately *not* taken here, even
        when it happens to be free. Taking it on the stop caller would run
        halt/disable on the wrong thread; taking it with a timeout is not a
        bounded teardown either, because the SDK call behind it is not
        bounded. The owner keeps device ownership and closes as soon as it
        regains control. Only when no owner is running is this thread the
        sole SDK user, and then it closes directly under a bounded lock.
        """
        # Invalidate before waiting anything: everything accepted before the
        # stop is dropped, and the generation advances so a target the owner
        # has just taken cannot be written after this point.
        with self._pending_lock:
            self._pending = {}
            self._generation += 1
        self._owner_wake.set()
        reader = self._reader
        if reader is None or reader is threading.current_thread() or not reader.is_alive():
            self._close_bounded()
            if self._close_error is not None:
                raise self._close_error
            return
        reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if reader.is_alive():
            self._report(
                f"{self.config.source} reader is still inside the SDK after the shutdown "
                "timeout; leaving the close to the owner"
            )
            return
        if self._close_error is not None:
            raise self._close_error

    def _close_bounded(self) -> None:
        """Close from this thread after a bounded wait for the hardware lock."""
        if self._hardware_lock.acquire(timeout=DEFAULT_THREAD_JOIN_TIMEOUT):
            try:
                self._close_hardware()
            except Exception as error:
                self._close_error = error
                self._report(f"shutdown failed: {type(error).__name__}: {error}")
            finally:
                self._hardware_lock.release()
        else:
            self._report(f"{self.config.source} hardware lock busy, leaving the close to the owner")

    def _on_position(self, msg: JointState) -> None:
        try:
            values = joint_state_to_values(msg, (Interface.POSITION,))
        except ValueError as error:
            self._report(str(error), rejection=True)
            return
        self.command(values, ts=msg.ts)

    def _accept(self, values: Values, *, ts: float | None = None) -> None:
        """Merge the command into the one pending target and wake the owner.

        The callback never touches the SDK or waits for the hardware lock:
        an accepted periodic setpoint is merged by key (a complete setpoint
        overwrites every pending key, a partial update only its own), the
        owning thread is woken, and the callback returns. No deadman is
        restarted here: authorisation happens only when the owner actually
        writes a target, so traffic that is already stale, or that expires
        before it can be written, neither moves the robot nor keeps the
        deadman quiet. That is the enqueue-versus-write boundary: acceptance
        is a merge, execution is a write.
        Commands for a disconnected, faulted or stopping robot are dropped
        as before.
        """
        if (
            self._stopping
            or self._stop_reading.is_set()
            or not self._connected
            or self._fault is not None
        ):
            return
        max_age = self._max_age()
        if ts is not None and time.time() - ts > max_age:
            self._report(
                f"expired command dropped before write (older than {max_age:.3f} s"
                " by source timestamp)",
                rejection=True,
            )
            return
        receipt = time.monotonic()
        with self._pending_lock:
            # Re-check under the lock that also guards invalidation, so a
            # stop or fault racing this callback cannot leave work behind.
            if self._stopping or self._stop_reading.is_set() or self._fault is not None:
                return
            for key, value in values.items():
                self._pending[key] = (value, receipt, ts)
            self._pending_max = max(self._pending_max, len(self._pending))
        self._owner_wake.set()

    def _max_age(self) -> float:
        return self.config.command_max_age_s or self.config.deadman_timeout_s

    def _has_pending(self) -> bool:
        with self._pending_lock:
            return bool(self._pending)

    def _take_fresh(self) -> dict[str, tuple[float, float, float | None]]:
        """Pop the pending target, keeping only keys still fresh enough to write.

        A key is fresh when its age by local receipt and, when the command
        carried one, by source timestamp is within ``command_max_age_s`` (the
        deadman budget by default). Ages are measured from the original
        receipt, never re-stamped at dequeue. The source timestamp is the
        publisher's wall clock, so that check assumes the commander and this
        connection share a host or synchronized clocks; local receipt alone
        cannot see how long a command queued upstream. Expired keys are
        dropped and reported instead of executed, and an expiry never
        restarts the deadman.
        """
        with self._pending_lock:
            pending, self._pending = self._pending, {}
            # Remember which invalidation generation this batch belongs to,
            # so a stop or fault arriving while it is in hand drops it.
            self._taken_generation = self._generation
        if not pending:
            return {}
        fresh, expired = self._fresh(pending)
        if expired:
            self._report_expiry(expired)
        return fresh

    def _fresh(
        self, pending: dict[str, tuple[float, float, float | None]]
    ) -> tuple[dict[str, tuple[float, float, float | None]], int]:
        max_age = self._max_age()
        now_mono = time.monotonic()
        now_wall = time.time()
        fresh: dict[str, tuple[float, float, float | None]] = {}
        expired = 0
        for key, (value, receipt, ts) in pending.items():
            if now_mono - receipt > max_age or (ts is not None and now_wall - ts > max_age):
                expired += 1
            else:
                fresh[key] = (value, receipt, ts)
        return fresh, expired

    def _report_expiry(self, count: int) -> None:
        self._report(
            f"{count} pending target(s) expired before write (older than {self._max_age():.3f} s)",
            rejection=True,
        )

    def _flush_pending(self) -> None:
        """Write the newest fresh pending target once, on the owning thread.

        Execution admission is one linearisation point: the final freshness
        check, the invalidation generation and the lifecycle flags are all
        read in the same ``_pending_lock`` critical section, and every
        invalidation publishes its flag *before* it takes that lock. So a
        stop or fault either commits before this check (everything taken is
        dropped) or after it (the write is already admitted and, like any
        started SDK call, is not preemptible). The lock is never held across
        the SDK call itself, so a blocked write cannot block a stop.
        """
        targets = self._take_fresh()
        if not targets:
            return
        generation = self._taken_generation
        with self._hardware_lock:
            if self._arm is None:
                return
            with self._pending_lock:
                # Final age check at the execution boundary: a target that
                # went stale between the dequeue and the SDK call is not
                # written.
                fresh, expired = self._fresh(targets)
                # Admission: the eligibility snapshot taken at dequeue must
                # still be current, and no stop/fault/disable may have been
                # published while this target was in hand.
                if (
                    not fresh
                    or generation != self._generation
                    or self._stopping
                    or self._stop_reading.is_set()
                    or self._fault is not None
                    or not self._connected
                ):
                    if expired:
                        self._report_expiry(expired)
                    return
            if expired:
                self._report_expiry(expired)
            authorized_at = max(receipt for _, receipt, _ in fresh.values())
            try:
                self.write({key: value for key, (value, _, _) in fresh.items()})
            except Exception as error:
                self._report(f"write failed: {type(error).__name__}: {error}")
                self._on_loop(self._disarm_deadman)
                self._call_halt()
            else:
                # The target reached the hardware: only now does it authorise
                # the deadman, and with the instant it became eligible (its
                # receipt), not the instant this slow call returned. A write
                # that completes after that authority expired must not grant
                # a fresh timeout. A stop that landed during the call owns the
                # lifecycle now, so do not resurrect authority after it.
                with self._pending_lock:
                    stopped = self._stopping or self._stop_reading.is_set()
                if not stopped:
                    self._record_command(authorized_at=authorized_at)

    def _invalidate_pending(self) -> None:
        """Drop every pending target and wake the owner, so nothing accepted
        before a stop, fault or disable is written after it."""
        with self._pending_lock:
            self._pending = {}
            self._generation += 1
        self._owner_wake.set()

    def _defer_halt(self, seq: int, *, deadman: bool) -> bool:
        """Queue a due halt for the owner thread instead of halting here."""
        reader = self._reader
        if reader is None or not reader.is_alive() or reader is threading.current_thread():
            return False
        with self._lock:
            self._halt_request = (deadman, seq)
        self._invalidate_pending()
        self._owner_wake.set()
        return True

    def _drain_owner_requests(self) -> None:
        """Run safety actions that other threads handed to the owner."""
        with self._lock:
            halt = self._halt_request
            self._halt_request = None
            disable = self._disable_request
            self._disable_request = False
        if disable:
            with self._hardware_lock:
                self._disable_arm()
        if halt is not None:
            deadman, seq = halt
            self._halt_if_current(seq, deadman=deadman)

    def _request_disable(self) -> None:
        """De-energize on the owner; fall back only when no owner runs."""
        reader = self._reader
        if reader is None:
            # Before the owner starts, the caller is the only SDK user, and a
            # fault during start may already hold the hardware lock.
            self._disable_arm()
            return
        if reader.is_alive() and reader is not threading.current_thread():
            with self._lock:
                self._disable_request = True
            self._owner_wake.set()
            return
        if reader is threading.current_thread():
            # The owner already holds the hardware lock in its read path.
            self._disable_arm()
            return
        with self._hardware_lock:
            self._disable_arm()

    def _disable_arm(self) -> None:
        arm = self._arm
        if arm is None:
            return
        try:
            arm.disable()
        except Exception as disable_error:
            self._report(f"{self._fault}; disable failed: {disable_error}")

    def _close_hardware(self) -> None:
        """Halt and de-energize once, in that order. The caller holds the lock."""
        if self._hardware_closed:
            return
        self._hardware_closed = True
        arm = self._arm
        try:
            try:
                # Halt while the arm is still installed: clearing ``_arm``
                # first would make ``halt()`` a no-op and skip the hold that
                # shutdown promises before de-energising.
                if self._connected:
                    self._call_halt()
            finally:
                if arm is not None:
                    arm.disable()
        finally:
            self._arm = None
            self._connected = False
            self._fault = None

    def _owner_close(self) -> None:
        try:
            with self._hardware_lock:
                self._close_hardware()
        except Exception as error:
            self._close_error = error
            self._report(f"shutdown failed: {type(error).__name__}: {error}")

    def write(self, values: Values) -> None:
        if self._fault is not None:
            raise RuntimeError(self._fault)
        arm = self._arm
        if arm is None:
            raise RuntimeError("Damiao robot is inactive")
        prefix = f"{self.config.source}/"
        arm.command(
            {
                key.removeprefix(prefix).removesuffix("/position"): value
                for key, value in values.items()
            }
        )

    def halt(self) -> None:
        if self._fault is not None or self._arm is None:
            return
        self._arm.hold()

    def _read_state(self) -> ArmReading:
        try:
            arm = self._arm
            if arm is None:
                raise RuntimeError("Damiao robot is disconnected")
            return arm.read()
        except Exception as error:
            if self._fault is None:
                self._feedback_failed(error)
            raise

    def _feedback_failed(self, error: Exception) -> None:
        with self._lock:
            self._fault = f"feedback failed: {type(error).__name__}: {error}"
            reader = self._reader
            owner = (
                reader is not None
                and reader.is_alive()
                and reader is not threading.current_thread()
            )
            if owner:
                # Queued in the same critical section as the fault, so the
                # owner cannot see the fault and exit before the disable.
                self._disable_request = True
        self._invalidate_pending()
        self._report(self._fault)
        if owner:
            self._owner_wake.set()
        else:
            self._request_disable()

    def _read_loop(self) -> None:
        """The one runtime owner of the SDK: write the newest target, read, repeat.

        Every runtime SDK call (target writes, scheduled reads, safety holds
        and fault disables) runs here, so no other thread competes with the
        reader for the hardware lock or closes the hardware out from under a
        call in progress. An accepted command wakes the write at once instead
        of waiting for the next sampling tick; the read still runs at its
        configured rate, and because one flush drains the whole pending
        target, a command flood can neither starve the read nor spin the
        loop. A stop, fault or due deadman is a tiny request this thread
        drains between SDK calls; it is never executed on the event loop.
        """
        period = 1.0 / self.config.state_rate_hz
        next_read = time.monotonic()
        while True:
            if self._stop_reading.is_set():
                self._owner_close()
                return
            self._drain_owner_requests()
            if self._stop_reading.is_set():
                self._owner_close()
                return
            if self._fault is not None:
                # A fault disables the arm; leave the owner rather than keep
                # reading and publishing from a faulted robot.
                return
            if self._has_pending():
                self._flush_pending()
            else:
                self._owner_wake.wait(max(0.0, next_read - time.monotonic()))
                self._owner_wake.clear()
                self._drain_owner_requests()
                if self._stop_reading.is_set():
                    self._owner_close()
                    return
                if self._fault is not None:
                    return
                self._flush_pending()
            started = time.monotonic()
            if started < next_read:
                continue
            next_read = started + period
            try:
                with self._hardware_lock:
                    if self._arm is None or not self._connected:
                        return
                    reading = self._read_state()
                if self._stop_reading.is_set():
                    self._owner_close()
                    return
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        frame_id=self._state_frame,
                        name=[f"{self.config.source}/{name}" for name in self.joint_names],
                        position=reading.position,
                        velocity=reading.velocity,
                        effort=reading.effort,
                    )
                )
            except Exception as error:
                with self._hardware_lock:
                    if self._fault is None:
                        self._feedback_failed(error)
                # A fault raised elsewhere may have queued the disable; run
                # anything left before this owner steps away.
                self._drain_owner_requests()
                return
