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
    joint_commands: tuple[Interface, ...] = (Interface.POSITION,)
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
        if self.joint_commands != (Interface.POSITION,) or self.pd or self.base_range is not None:
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
        # Set by an accepted command to wake the owning thread at once.
        self._write_wake = threading.Event()

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
        try:
            Module.start(self)
            self._session_id = uuid.uuid4().hex
            self._limits = limits_of(self.config)
            self._invalidate_pending()
            self._pending_max = 0
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
        self._stop_reading.set()
        self._invalidate_pending()
        if (
            self._reader is not None
            and self._reader.is_alive()
            and self._reader is not threading.current_thread()
        ):
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        try:
            try:
                super().stop()
            finally:
                with self._hardware_lock:
                    arm, self._arm = self._arm, None
                    try:
                        if arm is not None:
                            arm.disable()
                    finally:
                        self._fault = None
        finally:
            if (
                self._reader is not None
                and self._reader.is_alive()
                and self._reader is not threading.current_thread()
            ):
                self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
                if self._reader.is_alive():
                    error = f"{self.config.source} reader did not stop before the shutdown timeout"
                    self._report(error)
                    raise RuntimeError(error)

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
        owning thread is woken, and the callback returns. Acceptance, and
        with it the deadman restart, still happens here; the actual SDK
        write is asynchronous from the caller's point of view and only ever
        carries the newest pending value of each key, not a FIFO backlog.
        Commands for a disconnected or faulted robot are dropped as before.
        """
        if not self._connected or self._fault is not None:
            return
        self._record_command()
        receipt = time.monotonic()
        with self._pending_lock:
            for key, value in values.items():
                self._pending[key] = (value, receipt, ts)
            self._pending_max = max(self._pending_max, len(self._pending))
        self._write_wake.set()

    def _has_pending(self) -> bool:
        with self._pending_lock:
            return bool(self._pending)

    def _take_fresh(self) -> Values:
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
        if not pending:
            return {}
        max_age = self.config.command_max_age_s or self.config.deadman_timeout_s
        now_mono = time.monotonic()
        now_wall = time.time()
        fresh: Values = {}
        expired = 0
        for key, (value, receipt, ts) in pending.items():
            if now_mono - receipt > max_age or (ts is not None and now_wall - ts > max_age):
                expired += 1
            else:
                fresh[key] = value
        if expired:
            self._report(
                f"{expired} pending target(s) expired before write (older than {max_age:.3f} s)",
                rejection=True,
            )
        return fresh

    def _flush_pending(self) -> None:
        """Write the newest fresh pending target once, on the owning thread."""
        values = self._take_fresh()
        if not values:
            return
        with self._hardware_lock:
            if self._arm is None or not self._connected:
                return
            try:
                self.write(values)
            except Exception as error:
                self._report(f"write failed: {type(error).__name__}: {error}")
                self._on_loop(self._disarm_deadman)
                self._call_halt()

    def _invalidate_pending(self) -> None:
        """Drop every pending target and wake the owner, so nothing accepted
        before a stop, fault or disable is written after it."""
        with self._pending_lock:
            self._pending = {}
        self._write_wake.set()

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
        self._fault = f"feedback failed: {type(error).__name__}: {error}"
        self._invalidate_pending()
        self._report(self._fault)
        arm = self._arm
        if arm is not None:
            try:
                arm.disable()
            except Exception as disable_error:
                self._report(f"{self._fault}; disable failed: {disable_error}")

    def _read_loop(self) -> None:
        """The one runtime owner of the SDK: write the newest target, read, repeat.

        Every runtime SDK call (target writes, scheduled reads, and through
        the hardware lock any safety halt) is serialized on this thread, so
        no write thread competes with the reader for the lock. An accepted
        command wakes the write at once instead of waiting for the next
        sampling tick; the read still runs at its configured rate, and
        because one flush drains the whole pending target, a command flood
        can neither starve the read nor spin the loop.
        """
        period = 1.0 / self.config.state_rate_hz
        next_read = time.monotonic()
        while not self._stop_reading.is_set():
            if not self._has_pending():
                self._write_wake.wait(max(0.0, next_read - time.monotonic()))
                self._write_wake.clear()
            if self._stop_reading.is_set():
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
                return
