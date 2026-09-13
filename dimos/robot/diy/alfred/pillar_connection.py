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
framing, units, limits, homing state, and command pacing. The control
coordinator reaches it through the generic ``transport_lcm`` whole-body
adapter using one linear joint, ``pillar/lift``, expressed in metres.

The wire protocol is the ``pillar_driver`` host interface. Every line the
board emits falls in exactly one bucket, which is what lets the reader be a
demultiplexer rather than a parser:

    <STATUS,<ms>,<code>>      asynchronous event, can arrive at any moment
    <<ms>,<joint>,<metres>>   telemetry, only while ``set rate`` is non-zero
    ok  /  ok <key>=<value>   reply, exactly one per command line
    err <code>                reply, exactly one per command line
    anything else             human prose, silenced by ``set echo 0``

Three firmware properties shape the design:

* A command's ``ok`` means *accepted*, not finished. ``home`` answers in about
  4 ms and completes tens of seconds later, so ``move_done`` and its siblings
  are the only signals that mean arrived. Nothing here blocks on motion.
* Motion commands preempt each other and never answer ``busy``. A new target
  is sent straight away; there is no queue behind an active move.
* Opening the port resets the board, and ``echo``/``rate`` do not survive a
  reset. Session setup is re-applied on every ``ready`` event, not just once.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import math
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

# The name the firmware answers to in joint frames. dimos uses ``pillar/lift``
# everywhere above this file; the two names only meet on the wire.
PILLAR_FIRMWARE_JOINT = "pillar_platform_joint"

# Zero is the top limit switch trip point and up is positive, so the whole
# working range is negative. These mirror the firmware's own soft limits.
PILLAR_MIN_POSITION_M = -0.500
PILLAR_MAX_POSITION_M = -0.002
PILLAR_HOME_POSITION_M = -0.050

PILLAR_BAUD_RATE = 115_200
PILLAR_STEPS_PER_MM = 320.0  # 1600 steps/rev over a 5 mm ball screw lead
PILLAR_STEP_RESOLUTION_M = 1.0 / (PILLAR_STEPS_PER_MM * 1000.0)
# A longer line is discarded whole by the firmware, not truncated.
PILLAR_MAX_LINE_LENGTH = 63

_STATUS_PREFIX = "<STATUS,"

# Events that end a motion. Only ``move_done`` means the target was reached,
# which is why every wait for motion is paired with a deadline.
_MOTION_END_EVENTS = frozenset(
    {"move_done", "limit_hit", "position_lost", "homing_failed", "homing_aborted"}
)

# Rejections that mean "already there". Nothing moved and nothing is wrong.
_BENIGN_ERRORS = frozenset({"zero_move", "at_soft_limit"})


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
    MOVING = "moving"
    HOMING = "homing"
    STOPPING = "stopping"
    FAULT = "fault"


@dataclass(frozen=True)
class PillarFeedback:
    """Latest open-loop position reported by the firmware's telemetry stream."""

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
    last_event: str | None
    last_line: str | None


@dataclass
class _PendingReply:
    """One command line awaiting its single ``ok``/``err``."""

    line: str
    done: threading.Event
    value: str | None = None
    error: str | None = None


def _is_motion_line(line: str) -> bool:
    """True for commands that start the rail moving, joint frames included."""
    if line.startswith("<"):
        return True
    return line.split(" ", 1)[0].lower() in {"move", "jog", "steps", "revs"}


class PillarSerialDriver:
    """Line-oriented driver for the ``pillar_driver`` host interface."""

    def __init__(
        self,
        *,
        device_path: str,
        baud_rate: int = PILLAR_BAUD_RATE,
        serial_timeout_s: float = 0.05,
        write_timeout_s: float = 0.5,
        boot_wait_s: float = 2.5,
        telemetry_rate_hz: float = 50.0,
        speed_mm_s: float | None = None,
        accel_mm_s2: float | None = None,
        min_command_interval_s: float = 0.02,
        feedback_stale_after_s: float = 1.0,
        shutdown_stop_timeout_s: float = 2.0,
        reply_timeout_s: float = 1.0,
        move_timeout_s: float = 30.0,
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
        self._telemetry_rate_hz = telemetry_rate_hz
        self._speed_mm_s = speed_mm_s
        self._accel_mm_s2 = accel_mm_s2
        self._min_command_interval_s = min_command_interval_s
        self._feedback_stale_after_s = feedback_stale_after_s
        self._shutdown_stop_timeout_s = shutdown_stop_timeout_s
        self._reply_timeout_s = reply_timeout_s
        self._move_timeout_s = move_timeout_s
        self._home_timeout_s = home_timeout_s
        self._serial_factory = serial_factory
        self._feedback_callback = feedback_callback
        self._clock = clock

        self._state_lock = threading.RLock()
        self._state_changed = threading.Condition(self._state_lock)
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._serial: SerialPort | None = None
        self._reader_thread: Thread | None = None
        self._line_buffer = bytearray()
        self._pending_replies: deque[_PendingReply] = deque()

        self._reset_state()

    def _reset_state(self) -> None:
        """Every field the firmware can invalidate, in one place."""
        with self._state_lock:
            self._connected = False
            self._disconnecting = False
            self._session_ready = False
            self._ready_at = math.inf
            self._phase = PillarPhase.UNKNOWN
            self._homed = False
            self._homing = False
            self._stop_requested = False
            self._motion_active = False
            self._home_deadline = math.inf
            self._motion_deadline = math.inf
            self._position_m: float | None = None
            self._last_feedback_at: float | None = None
            self._pending_target_m: float | None = None
            self._active_target_m: float | None = None
            self._next_command_at = 0.0
            self._top_limit_triggered: bool | None = None
            self._fault: str | None = None
            self._last_event: str | None = None
            self._last_line: str | None = None

    # ------------------------------------------------------------------ life cycle

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
            self._reset_state()
            self._serial = port
            self._connected = True
            # Opening the port pulls DTR and resets the board. Anything sent
            # before the reset finishes is lost, so session setup waits.
            self._ready_at = now + self._boot_wait_s
            self._line_buffer.clear()
            self._pending_replies.clear()
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
        with self._state_lock:
            if self._disconnecting:
                return
            port = self._serial
            was_connected = self._connected
            self._disconnecting = True
            motion_active = was_connected and self._motion_active
            self._pending_target_m = None
            self._active_target_m = None
            if motion_active:
                self._stop_requested = True
                self._phase = PillarPhase.STOPPING
            self._state_changed.notify_all()

        if motion_active:
            try:
                stopped = self._command("stop", allow_disconnecting=True) is not None
            except Exception:
                logger.exception("Failed to request pillar stop during disconnect")
                stopped = False
            if stopped and not self._wait_for_motion_end(self._shutdown_stop_timeout_s):
                logger.error(
                    "Pillar did not settle after its ramped stop before disconnect; "
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
            self._abandon_pending_replies()
            self._reset_state()
            self._serial = None
            self._reader_thread = None
            self._state_changed.notify_all()

        if was_connected:
            logger.info("Disconnected from Alfred pillar")

    def is_connected(self) -> bool:
        with self._state_lock:
            return self._connected

    # ------------------------------------------------------------------ queries

    def status(self) -> PillarStatus:
        with self._state_lock:
            return PillarStatus(
                connected=self._connected,
                ready=self._connected and self._session_ready,
                phase=self._phase,
                homed=self._homed,
                position_m=self._position_m,
                pending_target_m=self._pending_target_m,
                active_target_m=self._active_target_m,
                motion_active=self._motion_active,
                top_limit_triggered=self._top_limit_triggered,
                fault=self._fault,
                last_event=self._last_event,
                last_line=self._last_line,
            )

    def latest_feedback(self) -> PillarFeedback | None:
        now = self._clock()
        with self._state_lock:
            if (
                not self._connected
                or self._position_m is None
                or self._last_feedback_at is None
                or now - self._last_feedback_at > self._feedback_stale_after_s
            ):
                return None
            return PillarFeedback(self._position_m, self._last_feedback_at)

    # ------------------------------------------------------------------ commands

    def home(self) -> bool:
        """Start homing. Returns once the firmware has *accepted* the command."""
        now = self._clock()
        with self._state_lock:
            if not self._connected or self._disconnecting or not self._session_ready:
                logger.warning("Pillar is not ready to home")
                return False
            if self._homing or self._motion_active:
                logger.warning(f"Pillar cannot home while state is {self._phase.value}")
                return False
            self._homing = True
            self._homed = False
            self._motion_active = True
            self._stop_requested = False
            self._phase = PillarPhase.HOMING
            self._fault = None
            self._pending_target_m = None
            self._active_target_m = None
            self._home_deadline = now + self._home_timeout_s
            self._state_changed.notify_all()

        reply = self._command("home")
        if reply is not None and reply.error is None:
            logger.info("Pillar homing started")
            return True

        with self._state_lock:
            self._homing = False
            self._motion_active = False
            self._home_deadline = math.inf
            if self._fault is None:
                self._phase = PillarPhase.IDLE
            self._state_changed.notify_all()
        return False

    def stop_motion(self) -> bool:
        """Request the firmware's ramped stop and drop any queued target."""
        now = self._clock()
        with self._state_lock:
            if not self._connected or self._disconnecting:
                return False
            self._pending_target_m = None
            self._active_target_m = None
            if self._motion_active:
                self._stop_requested = True
                self._phase = PillarPhase.STOPPING
                # `stop` ramps down rather than halting, and only homing has a
                # dedicated abort event, so bound the settle with the move
                # deadline as well.
                self._motion_deadline = now + self._move_timeout_s
            self._state_changed.notify_all()

        return self._command("stop") is not None

    def queue_position(self, position_m: float) -> bool:
        """Accept an absolute target in metres. Latest target wins."""
        if not math.isfinite(position_m):
            logger.warning(f"Rejected non-finite pillar target: {position_m}")
            return False
        if not PILLAR_MIN_POSITION_M <= position_m <= PILLAR_MAX_POSITION_M:
            logger.warning(
                f"Rejected pillar target {position_m:.6f} m; valid range is "
                f"[{PILLAR_MIN_POSITION_M:.3f}, {PILLAR_MAX_POSITION_M:.3f}] m"
            )
            return False

        with self._state_lock:
            if (
                not self._connected
                or self._disconnecting
                or not self._session_ready
                or not self._homed
                or self._homing
                or self._fault is not None
                or self._phase in {PillarPhase.UNKNOWN, PillarPhase.STOPPING}
            ):
                return False
            self._pending_target_m = position_m
        self._dispatch_pending_target()
        return True

    # ------------------------------------------------------------------ reader loop

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
        self._begin_session(now)
        self._check_deadlines(now)
        self._dispatch_pending_target()

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
        with self._state_lock:
            self._last_line = line

        if line.startswith(_STATUS_PREFIX) and line.endswith(">"):
            self._on_event(line)
        elif line.startswith("<") and line.endswith(">"):
            self._on_telemetry(line)
        elif line == "ok":
            self._resolve_reply(value=None, error=None, raw=line)
        elif line.startswith("ok "):
            self._resolve_reply(value=line[3:].strip(), error=None, raw=line)
        elif line.startswith("err "):
            self._resolve_reply(value=None, error=line[4:].split()[0], raw=line)
        else:
            # Prose. `set echo 0` silences most of it; `help` and `get status`
            # still emit it by design, and this module asks for neither.
            logger.debug(f"Pillar prose: {line}")
        self._dispatch_pending_target()

    # ------------------------------------------------------------------ line handlers

    def _on_telemetry(self, line: str) -> None:
        fields = line[1:-1].split(",")
        if len(fields) != 3:
            logger.warning(f"Pillar telemetry frame is malformed: {line!r}")
            return
        _, name, value = fields
        if name != PILLAR_FIRMWARE_JOINT:
            logger.warning(f"Pillar telemetry names an unexpected joint: {name!r}")
            return
        try:
            position_m = float(value)
        except ValueError:
            logger.warning(f"Pillar telemetry carries a non-numeric position: {line!r}")
            return

        now = self._clock()
        with self._state_lock:
            self._position_m = position_m
            self._last_feedback_at = now
            self._state_changed.notify_all()
        # Published whether or not the rail is homed. On boot the firmware
        # restores its position from EEPROM, which is a good enough hint to
        # draw the robot and plan the arms against; it is not good enough to
        # move against, and `queue_position` is what enforces that. Withholding
        # it instead would drop `pillar/lift` from the coordinator's merged
        # state and take both arms down with it.
        self._emit_feedback()

    def _on_event(self, line: str) -> None:
        body = line[len(_STATUS_PREFIX) : -1]
        _, _, code = body.partition(",")
        code = code.strip()
        if not code:
            logger.warning(f"Pillar event is malformed: {line!r}")
            return

        now = self._clock()
        with self._state_lock:
            self._last_event = code

            if code == "ready":
                rebooted = self._session_ready or self._homed
                self._homed = False
                self._homing = False
                self._motion_active = False
                self._stop_requested = False
                self._pending_target_m = None
                self._active_target_m = None
                self._home_deadline = math.inf
                self._motion_deadline = math.inf
                self._phase = PillarPhase.UNKNOWN
                # `echo` and `rate` do not survive a reset; re-apply them.
                self._session_ready = False
                self._ready_at = now
                if rebooted:
                    self._fault = "pillar controller rebooted; home is required"
                    self._phase = PillarPhase.FAULT
            elif code == "homed":
                self._homed = True
                self._homing = False
                self._motion_active = False
                self._stop_requested = False
                self._home_deadline = math.inf
                self._motion_deadline = math.inf
                self._fault = None
                self._phase = PillarPhase.IDLE
            elif code == "homing_failed":
                self._homed = False
                self._homing = False
                self._motion_active = False
                self._home_deadline = math.inf
                self._motion_deadline = math.inf
                self._phase = PillarPhase.FAULT
                self._fault = "homing failed; check switch, wiring and direction"
            elif code == "homing_aborted":
                self._homed = False
                self._homing = False
                self._motion_active = False
                self._home_deadline = math.inf
                self._motion_deadline = math.inf
                self._fault = "homing aborted; home is required"
                self._phase = PillarPhase.FAULT
            elif code == "move_done":
                self._motion_active = False
                self._stop_requested = False
                self._active_target_m = None
                self._motion_deadline = math.inf
                if not self._homing:
                    self._phase = PillarPhase.FAULT if self._fault is not None else PillarPhase.IDLE
            elif code == "limit_hit":
                # The rail stopped on the top switch. The position reference is
                # no longer trustworthy; `position_lost` normally follows.
                self._top_limit_triggered = True
                self._homed = False
                self._motion_active = False
                self._stop_requested = False
                self._active_target_m = None
                self._pending_target_m = None
                self._motion_deadline = math.inf
                self._phase = PillarPhase.FAULT
                self._fault = "top limit hit; home is required"
            elif code == "position_lost":
                self._homed = False
                self._motion_active = False
                self._active_target_m = None
                self._pending_target_m = None
                self._motion_deadline = math.inf
                self._phase = PillarPhase.FAULT
                self._fault = "position reference lost; home is required"
            elif code == "pos_drift":
                # Homing disagreed with the stored position by more than the
                # firmware's tolerance. Homing still completes; a slipping
                # brake is the reason worth chasing.
                logger.error("Pillar position drifted while powered down; check the brake")
            elif code == "limit_triggered":
                self._top_limit_triggered = True
            elif code == "limit_clear":
                self._top_limit_triggered = False
            else:
                logger.warning(f"Pillar reported an unknown event: {code!r}")

            self._state_changed.notify_all()

        if code in {"homing_failed", "homing_aborted", "limit_hit", "position_lost"}:
            logger.error(f"Alfred pillar fault: {self.status().fault}")

    def _resolve_reply(self, *, value: str | None, error: str | None, raw: str) -> None:
        with self._state_lock:
            pending = self._pending_replies.popleft() if self._pending_replies else None
        if pending is None:
            logger.warning(f"Pillar sent a reply with no command outstanding: {raw!r}")
            return
        pending.value = value
        pending.error = error
        if error is not None:
            self._apply_error(pending.line, error)
        pending.done.set()

    def _apply_error(self, line: str, code: str) -> None:
        """Undo the optimistic state a rejected command left behind."""
        motion = _is_motion_line(line)
        with self._state_lock:
            if motion:
                self._motion_active = False
                self._active_target_m = None
                self._motion_deadline = math.inf

            if code in _BENIGN_ERRORS:
                # Already at the target. Treat it as an arrival, not a fault.
                if motion and not self._homing and self._fault is None:
                    self._phase = PillarPhase.IDLE
                self._state_changed.notify_all()
                return

            if code == "not_homed":
                self._homed = False
                self._fault = "firmware reports not homed; home is required"
                self._phase = PillarPhase.FAULT
            elif code == "busy":
                # Only home/zero/save/set invert can see this; motion preempts.
                logger.warning(f"Pillar rejected {line!r}: firmware is busy")
            elif code == "limit_blocked":
                self._top_limit_triggered = True
                self._fault = "up move refused: top limit is triggered"
                self._phase = PillarPhase.FAULT
            else:
                # needs_arg, bad_arg, bad_frame, too_far, unknown_* and
                # line_too_long all mean this module sent something wrong.
                self._fault = f"firmware rejected {line!r}: {code}"
                self._phase = PillarPhase.FAULT
            self._state_changed.notify_all()
        logger.error(f"Pillar rejected {line!r} with err {code}")

    # ------------------------------------------------------------------ scheduling

    def _begin_session(self, now: float) -> None:
        with self._state_lock:
            if (
                self._session_ready
                or not self._connected
                or self._disconnecting
                or now < self._ready_at
            ):
                return
            self._session_ready = True
            self._ready_at = math.inf
            if self._phase is PillarPhase.UNKNOWN and self._fault is None:
                self._phase = PillarPhase.IDLE
            rate = self._telemetry_rate_hz
            speed = self._speed_mm_s
            accel = self._accel_mm_s2
            self._state_changed.notify_all()

        # Fire-and-forget: these run on the reader thread, which is the thread
        # that would have to resolve a blocking wait.
        self._write_line("set echo 0")
        self._write_line(f"set rate {rate:g}")
        if speed is not None:
            self._write_line(f"set speed {speed:g}")
        if accel is not None:
            self._write_line(f"set accel {accel:g}")
        logger.info("Pillar session established", telemetry_rate_hz=rate)

    def _dispatch_pending_target(self) -> None:
        now = self._clock()
        with self._state_lock:
            target = self._pending_target_m
            if (
                target is None
                or not self._connected
                or self._disconnecting
                or self._stop_event.is_set()
                or not self._session_ready
                or not self._homed
                or self._homing
                or self._fault is not None
                or self._phase is PillarPhase.STOPPING
            ):
                return
            if now < self._next_command_at:
                # Pace the link. The target stays pending and the newest one
                # wins, so reversals ride one ramp instead of several.
                return
            if self._active_target_m is not None and math.isclose(
                target, self._active_target_m, abs_tol=PILLAR_STEP_RESOLUTION_M / 2.0
            ):
                self._pending_target_m = None
                return

            self._pending_target_m = None
            self._active_target_m = target
            self._next_command_at = now + self._min_command_interval_s
            self._motion_active = True
            self._motion_deadline = now + self._move_timeout_s
            if self._phase is not PillarPhase.MOVING:
                self._phase = PillarPhase.MOVING
            self._state_changed.notify_all()

        if self._write_line(f"<0,{PILLAR_FIRMWARE_JOINT},{target:.6f}>") is None:
            with self._state_lock:
                self._active_target_m = None
                self._motion_active = False
                self._motion_deadline = math.inf
                self._state_changed.notify_all()

    def _check_deadlines(self, now: float) -> None:
        faults: list[str] = []
        with self._state_lock:
            if not self._connected or self._disconnecting:
                return

            if self._homing and now >= self._home_deadline:
                message = "pillar homing timed out"
                self._homed = False
                self._homing = False
                self._home_deadline = math.inf
                self._motion_deadline = math.inf
                # Homing has several internal phases, so silence does not prove
                # the rail has stopped. Keep motion latched until told otherwise.
                self._motion_active = True
                self._phase = PillarPhase.FAULT
                self._fault = message
                faults.append(message)

            if self._motion_active and not self._homing and now >= self._motion_deadline:
                message = "pillar move did not report move_done in time"
                self._motion_deadline = math.inf
                self._pending_target_m = None
                self._active_target_m = None
                self._phase = PillarPhase.FAULT
                self._fault = message
                faults.append(message)

            if faults:
                self._state_changed.notify_all()

        for message in faults:
            logger.error(f"Alfred pillar fault: {message}")

    # ------------------------------------------------------------------ transport

    def _write_line(self, line: str, *, allow_disconnecting: bool = False) -> _PendingReply | None:
        if len(line) > PILLAR_MAX_LINE_LENGTH:
            self._set_fault(
                f"refusing to send a {len(line)}-char line; the firmware discards "
                f"anything over {PILLAR_MAX_LINE_LENGTH}"
            )
            return None

        payload = f"{line}\n".encode("ascii")
        pending = _PendingReply(line, threading.Event())
        with self._write_lock:
            with self._state_lock:
                port = self._serial
                if (
                    not self._connected
                    or port is None
                    or (self._disconnecting and not allow_disconnecting)
                ):
                    return None
                # Replies come back in order, one per line, so a FIFO is all the
                # matching this needs.
                self._pending_replies.append(pending)
            try:
                written = port.write(payload)
                port.flush()
            except Exception as exc:
                self._drop_pending(pending)
                self._set_fault(f"serial write failed: {exc}", disconnected=True)
                return None

        if written != len(payload):
            self._drop_pending(pending)
            self._set_fault(
                f"short serial write: {written}/{len(payload)} bytes", disconnected=True
            )
            return None
        logger.debug(f"Sent Alfred pillar firmware command: {line}")
        return pending

    def _command(
        self,
        line: str,
        *,
        timeout_s: float | None = None,
        allow_disconnecting: bool = False,
    ) -> _PendingReply | None:
        """Send and wait for the single reply. Never call from the reader thread."""
        pending = self._write_line(line, allow_disconnecting=allow_disconnecting)
        if pending is None:
            return None
        timeout = self._reply_timeout_s if timeout_s is None else timeout_s
        if not pending.done.wait(timeout=timeout):
            self._drop_pending(pending)
            logger.error(f"Pillar did not reply to {line!r} within {timeout:.3f} s")
            return None
        return None if pending.error is not None else pending

    def _drop_pending(self, pending: _PendingReply) -> None:
        with self._state_lock:
            try:
                self._pending_replies.remove(pending)
            except ValueError:
                pass
        pending.done.set()

    def _abandon_pending_replies(self) -> None:
        with self._state_lock:
            pending = list(self._pending_replies)
            self._pending_replies.clear()
        for item in pending:
            item.done.set()

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
            if self._position_m is None or self._last_feedback_at is None:
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
        if disconnected:
            self._abandon_pending_replies()
        logger.error(f"Alfred pillar fault: {message}")


class PillarConnectionConfig(ModuleConfig):
    device_path: str | None = None
    baud_rate: int = Field(default=PILLAR_BAUD_RATE, gt=0)
    serial_timeout_s: float = Field(default=0.05, gt=0.0)
    write_timeout_s: float = Field(default=0.5, gt=0.0)
    # Opening the port resets the board; nothing sent before this lands.
    boot_wait_s: float = Field(default=2.5, ge=0.0)
    # Telemetry is off at boot by design. 50 Hz costs about 20% of the link.
    telemetry_rate_hz: float = Field(default=50.0, gt=0.0, le=100.0)
    speed_mm_s: float | None = Field(default=None, gt=0.0)
    accel_mm_s2: float | None = Field(default=None, gt=0.0)
    min_command_interval_s: float = Field(default=0.02, ge=0.0)
    feedback_stale_after_s: float = Field(default=1.0, gt=0.0)
    shutdown_stop_timeout_s: float = Field(default=2.0, ge=0.0)
    reply_timeout_s: float = Field(default=1.0, gt=0.0)
    move_timeout_s: float = Field(default=30.0, gt=0.0)
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
            telemetry_rate_hz=self.config.telemetry_rate_hz,
            speed_mm_s=self.config.speed_mm_s,
            accel_mm_s2=self.config.accel_mm_s2,
            min_command_interval_s=self.config.min_command_interval_s,
            feedback_stale_after_s=self.config.feedback_stale_after_s,
            shutdown_stop_timeout_s=self.config.shutdown_stop_timeout_s,
            reply_timeout_s=self.config.reply_timeout_s,
            move_timeout_s=self.config.move_timeout_s,
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
        """Start the firmware's homing sequence. Completion arrives later."""
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
                last_event=None,
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
            "last_event": status.last_event,
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
