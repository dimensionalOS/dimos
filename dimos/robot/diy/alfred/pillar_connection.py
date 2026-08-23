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

"""Alfred pillar serial connection.

This module owns every pillar-specific detail: firmware commands, serial
framing, units, limits, homing state, and command coalescing. The control
coordinator reaches it through the generic ``transport_lcm`` whole-body
adapter using one linear joint, ``pillar/lift``, expressed in metres.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import math
import re
import threading
from threading import Thread
import time
from typing import Any, Protocol, cast

from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.components import HardwareComponent, HardwareType
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PILLAR_HARDWARE_ID = "pillar"
PILLAR_LIFT_JOINT = "pillar/lift"
PILLAR_DEFAULT_DEVICE_PATH = "/dev/ttyUSB0"

# The active Nano firmware uses the top switch as zero and positive motion as
# upward, so every normal position lies below zero.
PILLAR_MIN_POSITION_M = -0.500
PILLAR_MAX_POSITION_M = -0.002
PILLAR_HOME_POSITION_M = -0.050
PILLAR_BAUD_RATE = 115_200
PILLAR_STEP_RESOLUTION_M = 1.0 / (80.0 * 1000.0)

_POSITION_RE = re.compile(r"(?:^| )pos (?P<mm>-?\d+(?:\.\d+)?) mm \((?P<steps>-?\d+) steps\)$")
_STATE_RE = re.compile(r"^state (?P<state>IDLE|MOVING|BRAKE-WAIT)(?: |$)")
_LIMIT_STATUS_RE = re.compile(
    r"^top limit: D\d+ (?P<level>HIGH|LOW) -> (?P<state>TRIGGERED|clear)(?: |$)"
)


class SerialPort(Protocol):
    """Subset of ``pyserial.Serial`` used by the pillar driver."""

    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def flush(self) -> None: ...
    def close(self) -> None: ...


class SerialPortFactory(Protocol):
    def __call__(
        self,
        *,
        port: str,
        baudrate: int,
        timeout: float,
        write_timeout: float,
    ) -> SerialPort: ...


def _open_serial(
    *,
    port: str,
    baudrate: int,
    timeout: float,
    write_timeout: float,
) -> SerialPort:
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError(
            "Alfred pillar support requires pyserial; install the 'alfred' extra"
        ) from exc

    return cast(
        "SerialPort",
        serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=write_timeout,
        ),
    )


class PillarPhase(str, Enum):
    UNKNOWN = "unknown"
    IDLE = "idle"
    COMMAND_PENDING = "command_pending"
    BRAKE_WAIT = "brake_wait"
    MOVING = "moving"
    HOMING = "homing"
    STOPPING = "stopping"
    FAULT = "fault"


@dataclass(frozen=True)
class PillarFeedback:
    """Latest open-loop position reported by the Nano firmware."""

    position_m: float
    received_at: float


@dataclass(frozen=True)
class PillarStatus:
    connected: bool
    ready: bool
    phase: PillarPhase
    homed: bool
    position_m: float | None
    pending_target_m: float | None
    active_target_m: float | None
    motion_active: bool
    top_limit_triggered: bool | None
    fault: str | None
    last_line: str | None


class PillarSerialDriver:
    """Line-oriented driver for the active Alfred pillar Nano firmware."""

    def __init__(
        self,
        *,
        device_path: str,
        baud_rate: int = PILLAR_BAUD_RATE,
        serial_timeout_s: float = 0.05,
        write_timeout_s: float = 0.5,
        boot_wait_s: float = 2.0,
        status_poll_interval_s: float = 0.5,
        feedback_stale_after_s: float = 2.0,
        shutdown_stop_timeout_s: float = 2.0,
        command_ack_timeout_s: float = 1.0,
        home_timeout_s: float = 75.0,
        serial_factory: SerialPortFactory = _open_serial,
        feedback_callback: Callable[[PillarFeedback], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._device_path = device_path
        self._baud_rate = baud_rate
        self._serial_timeout_s = serial_timeout_s
        self._write_timeout_s = write_timeout_s
        self._boot_wait_s = boot_wait_s
        self._status_poll_interval_s = status_poll_interval_s
        self._feedback_stale_after_s = feedback_stale_after_s
        self._shutdown_stop_timeout_s = shutdown_stop_timeout_s
        self._command_ack_timeout_s = command_ack_timeout_s
        self._home_timeout_s = home_timeout_s
        self._serial_factory = serial_factory
        self._feedback_callback = feedback_callback
        self._clock = clock

        self._state_lock = threading.RLock()
        self._state_changed = threading.Condition(self._state_lock)
        self._command_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._serial: SerialPort | None = None
        self._reader_thread: Thread | None = None
        self._line_buffer = bytearray()

        self._connected = False
        self._disconnecting = False
        self._ready_at = math.inf
        self._next_status_poll = math.inf
        self._phase = PillarPhase.UNKNOWN
        self._reported_phase = PillarPhase.UNKNOWN
        self._homed = False
        self._homing_in_progress = False
        self._stop_requested = False
        self._command_ack_deadline = math.inf
        self._home_deadline = math.inf
        self._position_m: float | None = None
        self._last_feedback_at: float | None = None
        self._pending_target_m: float | None = None
        self._active_target_m: float | None = None
        self._command_in_flight = False
        self._motion_active = False
        self._top_limit_triggered: bool | None = None
        self._fault: str | None = None
        self._last_line: str | None = None
        self._saw_boot_banner = False

    def connect(self) -> None:
        with self._state_lock:
            if self._connected:
                return

        port = self._serial_factory(
            port=self._device_path,
            baudrate=self._baud_rate,
            timeout=self._serial_timeout_s,
            write_timeout=self._write_timeout_s,
        )
        now = self._clock()
        with self._state_lock:
            self._serial = port
            self._connected = True
            self._disconnecting = False
            self._ready_at = now + self._boot_wait_s
            self._next_status_poll = self._ready_at
            self._phase = PillarPhase.UNKNOWN
            self._reported_phase = PillarPhase.UNKNOWN
            self._homed = False
            self._homing_in_progress = False
            self._stop_requested = False
            self._command_ack_deadline = math.inf
            self._home_deadline = math.inf
            self._position_m = None
            self._last_feedback_at = None
            self._pending_target_m = None
            self._active_target_m = None
            self._command_in_flight = False
            self._motion_active = False
            self._top_limit_triggered = None
            self._fault = None
            self._last_line = None
            self._saw_boot_banner = False
            self._line_buffer.clear()
            self._stop_event.clear()
            self._state_changed.notify_all()

        self._reader_thread = Thread(
            target=self._run,
            name="alfred-pillar-serial",
            daemon=True,
        )
        self._reader_thread.start()
        logger.info(
            "Connected to Alfred pillar",
            device_path=self._device_path,
            baud_rate=self._baud_rate,
        )

    def disconnect(self) -> None:
        sent_stop = False
        with self._command_lock:
            with self._state_lock:
                if self._disconnecting:
                    return
                port = self._serial
                was_connected = self._connected
                self._disconnecting = True
                motion_active = was_connected and self._motion_active
                self._pending_target_m = None
                self._active_target_m = None
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                self._home_deadline = math.inf
                if motion_active:
                    self._stop_requested = True
                    self._phase = PillarPhase.STOPPING
                    if self._homing_in_progress:
                        self._homed = False
                        self._fault = "homing aborted; home is required"
                self._state_changed.notify_all()

            if motion_active:
                try:
                    sent_stop = self._send_line("x", allow_disconnecting=True)
                except Exception:
                    logger.exception("Failed to request pillar stop during disconnect")

        if sent_stop and not self._wait_for_motion_end(self._shutdown_stop_timeout_s):
            logger.error(
                "Pillar did not acknowledge its ramped stop before disconnect; "
                "hardware may still be moving"
            )

        self._stop_event.set()
        if port is not None:
            try:
                port.close()
            except Exception:
                logger.exception("Failed to close pillar serial port")

        thread = self._reader_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if thread.is_alive():
                logger.warning("Pillar serial reader did not stop before timeout")

        with self._state_lock:
            self._serial = None
            self._reader_thread = None
            self._connected = False
            self._disconnecting = False
            self._ready_at = math.inf
            self._next_status_poll = math.inf
            self._phase = PillarPhase.UNKNOWN
            self._reported_phase = PillarPhase.UNKNOWN
            self._homed = False
            self._homing_in_progress = False
            self._stop_requested = False
            self._command_ack_deadline = math.inf
            self._home_deadline = math.inf
            self._pending_target_m = None
            self._active_target_m = None
            self._command_in_flight = False
            self._motion_active = False
            self._state_changed.notify_all()

        if was_connected:
            logger.info("Disconnected from Alfred pillar")

    def is_connected(self) -> bool:
        with self._state_lock:
            return self._connected

    def status(self) -> PillarStatus:
        now = self._clock()
        with self._state_lock:
            return PillarStatus(
                connected=self._connected,
                ready=self._connected and now >= self._ready_at,
                phase=self._phase,
                homed=self._homed,
                position_m=self._position_m,
                pending_target_m=self._pending_target_m,
                active_target_m=self._active_target_m,
                motion_active=self._motion_active,
                top_limit_triggered=self._top_limit_triggered,
                fault=self._fault,
                last_line=self._last_line,
            )

    def latest_feedback(self) -> PillarFeedback | None:
        now = self._clock()
        with self._state_lock:
            if (
                not self._connected
                or not self._homed
                or self._position_m is None
                or self._last_feedback_at is None
                or now - self._last_feedback_at > self._feedback_stale_after_s
            ):
                return None
            return PillarFeedback(self._position_m, self._last_feedback_at)

    def home(self) -> bool:
        now = self._clock()
        with self._command_lock:
            with self._state_lock:
                if not self._connected or self._disconnecting or now < self._ready_at:
                    logger.warning("Pillar is not ready to home")
                    return False
                if (
                    self._last_feedback_at is None
                    or now - self._last_feedback_at > self._feedback_stale_after_s
                    or self._reported_phase is not PillarPhase.IDLE
                ):
                    logger.warning("Pillar needs fresh idle feedback before homing")
                    return False
                if (
                    self._motion_active
                    or self._command_in_flight
                    or self._homing_in_progress
                    or self._phase in {PillarPhase.UNKNOWN, PillarPhase.STOPPING}
                ):
                    logger.warning(f"Pillar cannot home while state is {self._phase.value}")
                    return False
                self._homing_in_progress = True
                self._homed = False
                self._phase = PillarPhase.HOMING
                self._fault = None
                self._pending_target_m = None
                self._active_target_m = None
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                self._motion_active = True
                self._stop_requested = False
                self._home_deadline = now + self._home_timeout_s
                self._state_changed.notify_all()

            if self._send_line("home"):
                logger.info("Pillar homing started")
                return True

            with self._state_lock:
                self._homing_in_progress = False
                self._motion_active = False
                self._home_deadline = math.inf
                self._phase = PillarPhase.FAULT
                self._state_changed.notify_all()
            return False

    def stop_motion(self) -> bool:
        with self._command_lock:
            with self._state_lock:
                if not self._connected or self._disconnecting:
                    return False
                was_homing = self._homing_in_progress
                self._pending_target_m = None
                self._active_target_m = None
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                if self._motion_active:
                    self._stop_requested = True
                    self._phase = PillarPhase.STOPPING
                if was_homing:
                    self._homed = False
                    self._fault = "homing aborted; home is required"
                self._state_changed.notify_all()

            return self._send_line("x")

    def queue_position(self, position_m: float) -> bool:
        if not math.isfinite(position_m):
            logger.warning(f"Rejected non-finite pillar target: {position_m}")
            return False
        if not PILLAR_MIN_POSITION_M <= position_m <= PILLAR_MAX_POSITION_M:
            logger.warning(
                f"Rejected pillar target {position_m:.6f} m; valid range is "
                f"[{PILLAR_MIN_POSITION_M:.3f}, {PILLAR_MAX_POSITION_M:.3f}] m"
            )
            return False

        now = self._clock()
        with self._state_lock:
            if (
                not self._connected
                or self._disconnecting
                or not self._homed
                or self._homing_in_progress
            ):
                return False
            if self._fault is not None or self._phase in {
                PillarPhase.UNKNOWN,
                PillarPhase.STOPPING,
                PillarPhase.FAULT,
            }:
                return False
            if self._active_target_m is not None and math.isclose(
                position_m,
                self._active_target_m,
                abs_tol=PILLAR_STEP_RESOLUTION_M / 2.0,
            ):
                # Latest-wins: returning to the active target cancels any
                # different target that arrived while the firmware was busy.
                self._pending_target_m = None
                return True
            if self._pending_target_m is not None and math.isclose(
                position_m,
                self._pending_target_m,
                abs_tol=PILLAR_STEP_RESOLUTION_M / 2.0,
            ):
                return True
            if self._motion_active or self._command_in_flight:
                # The Nano cannot retarget an active point-to-point move. Keep
                # only the newest target and dispatch it after the terminal
                # position report, which also refreshes feedback.
                self._pending_target_m = position_m
                return True
            if (
                self._last_feedback_at is None
                or now - self._last_feedback_at > self._feedback_stale_after_s
            ):
                logger.warning("Rejected pillar target because feedback is stale")
                return False
            if self._position_m is not None and math.isclose(
                position_m,
                self._position_m,
                abs_tol=PILLAR_STEP_RESOLUTION_M / 2.0,
            ):
                self._pending_target_m = None
                return True
            self._pending_target_m = position_m
        return True

    def _run(self) -> None:
        while not self._stop_event.is_set():
            with self._state_lock:
                port = self._serial
            if port is None:
                return

            try:
                chunk = port.read(256)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._set_fault(f"serial read failed: {exc}", disconnected=True)
                return

            try:
                if chunk:
                    self._feed_bytes(chunk)
                self._service()
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._set_fault(f"serial protocol processing failed: {exc}", disconnected=True)
                return

    def _service(self) -> None:
        now = self._clock()
        self._check_deadlines(now)
        # A pending goal takes precedence over an idle status poll. The Nano
        # generates step pulses in its main loop, so its verbose `p` response
        # must never be requested while motion is active.
        self._dispatch_pending_target()
        should_poll = False
        with self._state_lock:
            if (
                self._connected
                and not self._disconnecting
                and not self._motion_active
                and not self._command_in_flight
                and not self._homing_in_progress
                and now >= self._next_status_poll
            ):
                should_poll = True
                self._next_status_poll = now + self._status_poll_interval_s
        if should_poll:
            self._send_line("p")

    def _feed_bytes(self, data: bytes) -> None:
        for byte in data:
            if byte in (ord("\n"), ord("\r")):
                if self._line_buffer:
                    line = self._line_buffer.decode("ascii", errors="replace")
                    self._line_buffer.clear()
                    self._process_line(line)
            elif len(self._line_buffer) < 512:
                self._line_buffer.append(byte)
            else:
                self._line_buffer.clear()

    def _process_line(self, raw_line: str) -> None:
        line = raw_line.strip()
        if not line:
            return

        now = self._clock()
        emit_feedback = False
        with self._state_lock:
            self._last_line = line

            position_match = _POSITION_RE.search(line)
            if position_match is not None:
                self._position_m = float(position_match.group("mm")) / 1000.0
                self._last_feedback_at = now
                emit_feedback = self._homed

            state_match = _STATE_RE.match(line)
            if state_match is not None:
                reported_phase = {
                    "IDLE": PillarPhase.IDLE,
                    "MOVING": PillarPhase.MOVING,
                    "BRAKE-WAIT": PillarPhase.BRAKE_WAIT,
                }[state_match.group("state")]
                self._reported_phase = reported_phase
                if reported_phase is PillarPhase.IDLE:
                    if not self._command_in_flight and not self._homing_in_progress:
                        self._motion_active = False
                        self._active_target_m = None
                        self._stop_requested = False
                else:
                    self._motion_active = True
                    if self._command_in_flight:
                        self._command_in_flight = False
                        self._command_ack_deadline = math.inf
                if (
                    not self._homing_in_progress
                    and not self._command_in_flight
                    and self._fault is None
                ):
                    self._phase = reported_phase

            limit_match = _LIMIT_STATUS_RE.match(line)
            if limit_match is not None:
                self._top_limit_triggered = limit_match.group("state") == "TRIGGERED"

            if line == "openFT pillar bench test":
                rebooted_while_running = self._saw_boot_banner or self._homed
                self._saw_boot_banner = True
                self._homed = False
                self._homing_in_progress = False
                self._phase = PillarPhase.UNKNOWN
                self._reported_phase = PillarPhase.UNKNOWN
                self._pending_target_m = None
                self._active_target_m = None
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                self._home_deadline = math.inf
                self._stop_requested = False
                self._motion_active = False
                if rebooted_while_running:
                    self._fault = "pillar controller rebooted; home is required"
                    self._phase = PillarPhase.FAULT
            elif line.startswith("HOMED, soft limits "):
                # Firmware sets its reference before the final -50 mm park.
                # Expose homed only after the definitive completion token.
                if not self._homing_in_progress:
                    self._homed = True
                    emit_feedback = self._position_m is not None
            elif line.startswith("NOT HOMED "):
                self._homed = False
            elif line == "HOMED - at start position":
                self._homed = True
                self._homing_in_progress = False
                self._phase = PillarPhase.IDLE
                self._fault = None
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                self._home_deadline = math.inf
                self._stop_requested = False
                self._motion_active = False
                emit_feedback = self._position_m is not None
            elif line.startswith("HOMING FAILED: "):
                self._homed = False
                self._homing_in_progress = False
                self._phase = PillarPhase.FAULT
                self._fault = line
                self._command_ack_deadline = math.inf
                self._home_deadline = math.inf
                self._stop_requested = False
                self._motion_active = False
            elif line == "homing aborted":
                self._homed = False
                self._homing_in_progress = False
                self._home_deadline = math.inf
                self._fault = "homing aborted; home is required"
                if self._stop_requested:
                    self._phase = PillarPhase.STOPPING
                    self._motion_active = True
                else:
                    self._phase = PillarPhase.FAULT
                    self._motion_active = False
            elif line.startswith("move "):
                self._motion_active = True
                if not self._homing_in_progress:
                    self._phase = (
                        PillarPhase.FAULT if self._fault is not None else PillarPhase.MOVING
                    )
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
            elif line.startswith("busy - "):
                if self._homing_in_progress:
                    self._homing_in_progress = False
                    self._home_deadline = math.inf
                    self._homed = False
                    self._pending_target_m = None
                    self._active_target_m = None
                    self._command_in_flight = False
                    self._command_ack_deadline = math.inf
                    self._motion_active = True
                    self._phase = PillarPhase.FAULT
                    self._fault = "home rejected: firmware busy"
                else:
                    self._restore_active_target_locked()
                    self._motion_active = True
                if not self._homing_in_progress and self._fault is None:
                    self._phase = PillarPhase.MOVING
            elif line.startswith("not homed - "):
                self._homed = False
                self._motion_active = False
                self._restore_active_target_locked()
                self._phase = PillarPhase.FAULT if self._fault is not None else PillarPhase.IDLE
            elif line in {"zero move", "already at soft limit"}:
                self._phase = PillarPhase.FAULT if self._fault is not None else PillarPhase.IDLE
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                self._active_target_m = None
                self._stop_requested = False
                self._motion_active = False
            elif line.startswith("refusing "):
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                self._active_target_m = None
                self._stop_requested = False
                self._motion_active = False
                if "top limit TRIGGERED" in line:
                    self._homed = False
                    self._phase = PillarPhase.FAULT
                    self._fault = line
                elif self._fault is None:
                    self._phase = PillarPhase.IDLE
            elif line.startswith("warning: brake is engaged"):
                self._fault = line
                self._phase = PillarPhase.FAULT
            elif line.startswith("top limit -> "):
                self._top_limit_triggered = line.endswith("TRIGGERED")

            if line.startswith("TOP LIMIT hit - stopped.") and not self._homing_in_progress:
                self._homed = False
                self._phase = PillarPhase.FAULT
                self._fault = "top limit hit outside homing; home is required"
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                self._home_deadline = math.inf
                self._active_target_m = None
                self._stop_requested = False
                self._motion_active = False
            elif line.startswith("done. pos "):
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                self._active_target_m = None
                if not self._homing_in_progress and not self._stop_requested:
                    self._motion_active = False
                    self._phase = PillarPhase.FAULT if self._fault is not None else PillarPhase.IDLE
            elif line.startswith("stopped. pos "):
                was_homing = self._homing_in_progress
                self._homing_in_progress = False
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                self._home_deadline = math.inf
                self._active_target_m = None
                self._stop_requested = False
                self._motion_active = False
                if was_homing:
                    self._homed = False
                    self._fault = "homing aborted; home is required"
                self._phase = PillarPhase.FAULT if self._fault is not None else PillarPhase.IDLE
            elif line in {"cancelled", "idle"}:
                was_homing = self._homing_in_progress
                self._homing_in_progress = False
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                self._home_deadline = math.inf
                self._active_target_m = None
                self._stop_requested = False
                self._motion_active = False
                if was_homing:
                    self._homed = False
                    self._fault = "homing aborted; home is required"
                self._phase = PillarPhase.FAULT if self._fault is not None else PillarPhase.IDLE

            self._state_changed.notify_all()

        if emit_feedback:
            self._emit_feedback()
        self._dispatch_pending_target()

    def _check_deadlines(self, now: float) -> None:
        faults: list[str] = []
        with self._state_lock:
            if not self._connected or self._disconnecting:
                return

            if self._command_in_flight and now >= self._command_ack_deadline:
                message = "pillar position command was not acknowledged"
                self._pending_target_m = None
                self._active_target_m = None
                self._command_in_flight = False
                self._command_ack_deadline = math.inf
                # Without an acknowledgement, assume the Nano may have accepted
                # the command until a later status report proves it is idle.
                self._motion_active = True
                self._phase = PillarPhase.FAULT
                self._fault = message
                faults.append(message)

            if self._homing_in_progress and now >= self._home_deadline:
                message = "pillar homing timed out"
                self._homed = False
                self._homing_in_progress = False
                self._home_deadline = math.inf
                # Homing has several internal idle/move transitions, so a stale
                # state line cannot prove that physical motion has ended.
                self._motion_active = True
                self._phase = PillarPhase.FAULT
                self._fault = message
                faults.append(message)

            if faults:
                self._state_changed.notify_all()

        for message in faults:
            logger.error(f"Alfred pillar fault: {message}")

    def _restore_active_target_locked(self) -> None:
        if self._pending_target_m is None and self._active_target_m is not None:
            self._pending_target_m = self._active_target_m
        self._active_target_m = None
        self._command_in_flight = False
        self._command_ack_deadline = math.inf
        self._motion_active = False

    def _dispatch_pending_target(self) -> None:
        with self._command_lock:
            with self._state_lock:
                target = self._pending_target_m
                if (
                    target is None
                    or not self._connected
                    or self._disconnecting
                    or self._stop_event.is_set()
                    or not self._homed
                    or self._homing_in_progress
                    or self._fault is not None
                    or self._phase is not PillarPhase.IDLE
                    or self._command_in_flight
                    or self._motion_active
                ):
                    return

                if self._position_m is not None and math.isclose(
                    target,
                    self._position_m,
                    abs_tol=PILLAR_STEP_RESOLUTION_M / 2.0,
                ):
                    self._pending_target_m = None
                    return

                self._pending_target_m = None
                self._active_target_m = target
                self._command_in_flight = True
                self._command_ack_deadline = self._clock() + self._command_ack_timeout_s
                self._motion_active = True
                self._phase = PillarPhase.COMMAND_PENDING
                self._state_changed.notify_all()

            command = f"g {target * 1000.0:.3f}"
            if not self._send_line(command):
                with self._state_lock:
                    self._restore_active_target_locked()
                    self._state_changed.notify_all()

    def _send_line(self, command: str, *, allow_disconnecting: bool = False) -> bool:
        payload = f"{command}\n".encode("ascii")
        with self._write_lock:
            with self._state_lock:
                port = self._serial
                connected = self._connected
                disconnecting = self._disconnecting
            if not connected or port is None or (disconnecting and not allow_disconnecting):
                return False
            try:
                written = port.write(payload)
                port.flush()
            except Exception as exc:
                self._set_fault(f"serial write failed: {exc}", disconnected=True)
                return False
        if written != len(payload):
            self._set_fault(
                f"short serial write: {written}/{len(payload)} bytes", disconnected=True
            )
            return False
        if command != "p":
            logger.info("Sent Alfred pillar firmware command", command=command)
        return True

    def _wait_for_motion_end(self, timeout_s: float) -> bool:
        with self._state_changed:
            return self._state_changed.wait_for(
                lambda: not self._motion_active or not self._connected,
                timeout=max(0.0, timeout_s),
            )

    def _emit_feedback(self) -> None:
        callback = self._feedback_callback
        if callback is None:
            return
        with self._state_lock:
            if not self._homed or self._position_m is None or self._last_feedback_at is None:
                return
            feedback = PillarFeedback(self._position_m, self._last_feedback_at)
        try:
            callback(feedback)
        except Exception:
            logger.exception("Failed to publish Alfred pillar feedback")

    def _set_fault(self, message: str, *, disconnected: bool = False) -> None:
        with self._state_lock:
            self._fault = message
            self._phase = PillarPhase.FAULT
            if disconnected:
                self._connected = False
            self._state_changed.notify_all()
        logger.error(f"Alfred pillar fault: {message}")


class PillarConnectionConfig(ModuleConfig):
    device_path: str | None = None
    baud_rate: int = Field(default=PILLAR_BAUD_RATE, gt=0)
    serial_timeout_s: float = Field(default=0.05, gt=0.0)
    write_timeout_s: float = Field(default=0.5, gt=0.0)
    boot_wait_s: float = Field(default=2.0, ge=0.0)
    status_poll_interval_s: float = Field(default=0.5, gt=0.0)
    feedback_stale_after_s: float = Field(default=2.0, gt=0.0)
    shutdown_stop_timeout_s: float = Field(default=2.0, ge=0.0)
    command_ack_timeout_s: float = Field(default=1.0, gt=0.0)
    home_timeout_s: float = Field(default=75.0, gt=0.0)


class PillarConnection(Module):
    """Robot-side pillar module bridged through generic whole-body streams."""

    config: PillarConnectionConfig

    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._driver: PillarSerialDriver | None = None

    @rpc
    def start(self) -> None:
        device_path = (
            self.config.device_path or self.config.g.device_path or PILLAR_DEFAULT_DEVICE_PATH
        )
        driver = PillarSerialDriver(
            device_path=device_path,
            baud_rate=self.config.baud_rate,
            serial_timeout_s=self.config.serial_timeout_s,
            write_timeout_s=self.config.write_timeout_s,
            boot_wait_s=self.config.boot_wait_s,
            status_poll_interval_s=self.config.status_poll_interval_s,
            feedback_stale_after_s=self.config.feedback_stale_after_s,
            shutdown_stop_timeout_s=self.config.shutdown_stop_timeout_s,
            command_ack_timeout_s=self.config.command_ack_timeout_s,
            home_timeout_s=self.config.home_timeout_s,
            feedback_callback=self._publish_feedback,
        )
        driver.connect()
        self._driver = driver
        try:
            super().start()
        except Exception:
            self._driver = None
            driver.disconnect()
            raise

    @rpc
    def stop(self) -> None:
        driver = self._driver
        self._driver = None
        if driver is not None:
            driver.disconnect()
        super().stop()

    async def handle_motor_command(self, msg: MotorCommandArray) -> None:
        if msg.num_joints != 1 or len(msg.q) != 1:
            logger.warning(f"Expected one pillar motor command, got {msg.num_joints}")
            return
        driver = self._driver
        if driver is not None:
            driver.queue_position(msg.q[0])

    @rpc
    def home(self) -> bool:
        """Start the firmware's explicit homing sequence."""
        driver = self._driver
        return driver.home() if driver is not None else False

    @rpc
    def set_position(self, position_m: float) -> bool:
        """Queue an absolute pillar position in metres."""
        driver = self._driver
        return driver.queue_position(position_m) if driver is not None else False

    @rpc
    def stop_motion(self) -> bool:
        """Request the firmware's ramped stop and discard any queued target."""
        driver = self._driver
        return driver.stop_motion() if driver is not None else False

    @rpc
    def get_status(self) -> dict[str, Any]:
        """Return current connection, homing, position, and fault status."""
        driver = self._driver
        if driver is None:
            status = PillarStatus(
                connected=False,
                ready=False,
                phase=PillarPhase.UNKNOWN,
                homed=False,
                position_m=None,
                pending_target_m=None,
                active_target_m=None,
                motion_active=False,
                top_limit_triggered=None,
                fault=None,
                last_line=None,
            )
        else:
            status = driver.status()
        return {
            "connected": status.connected,
            "ready": status.ready,
            "phase": status.phase.value,
            "homed": status.homed,
            "position_m": status.position_m,
            "pending_target_m": status.pending_target_m,
            "active_target_m": status.active_target_m,
            "motion_active": status.motion_active,
            "top_limit_triggered": status.top_limit_triggered,
            "fault": status.fault,
            "last_line": status.last_line,
        }

    def _publish_feedback(self, feedback: PillarFeedback) -> None:
        self.motor_states.publish(
            JointState(
                ts=time.time(),
                frame_id=PILLAR_HARDWARE_ID,
                name=[PILLAR_LIFT_JOINT],
                position=[feedback.position_m],
                velocity=[0.0],
                effort=[0.0],
            )
        )


def pillar_hardware() -> HardwareComponent:
    """Build Alfred's one-joint pillar component using the generic adapter."""
    return HardwareComponent(
        hardware_id=PILLAR_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=[PILLAR_LIFT_JOINT],
        adapter_type="transport_lcm",
        auto_enable=False,
        wb_config=WholeBodyConfig(kp=(0.0,), kd=(0.0,)),
    )
