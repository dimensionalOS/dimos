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

"""The pillar's Nano, in-process: the ``pillar_driver`` serial protocol on a simulated rail.

``PillarConnection(device_path="sim")`` opens a ``PillarEmulator`` where it would open the
CH340, so the module, its RPCs (``home``, ``set_position``, ``stop_motion``, ``get_status``)
and its streams run unchanged: the coordinator reaches the lift exactly as on the robot.

What the far end of the wire does (``PillarFirmware``, driven by an explicit clock):

* opening the port boots the board: a line of prose, then ``<STATUS,<ms>,ready>``; it is
  unhomed, at its restored position, with echo on and telemetry off;
* ``set echo|rate|speed|accel <value>`` answer ``ok``; once ``rate`` is non-zero, telemetry
  frames ``<<ms>,pillar_platform_joint,<metres>>`` stream at that rate;
* ``home`` answers ``ok`` at once, drives up to the top switch (``limit_triggered``), backs
  off to the parking height (``limit_clear``) and reports ``homed``;
* a joint frame ``<0,pillar_platform_joint,<metres>>`` answers ``ok`` and ramps there on a
  trapezoid at ``speed`` and ``accel`` (a reversal rides the ramp down first), then reports
  ``move_done``; a newer frame preempts it;
* ``stop`` answers ``ok`` and ramps down where it is, then ``move_done`` (``homing_aborted``
  mid-home);
* the rejections the connection handles: ``not_homed``, ``zero_move``, ``at_soft_limit``,
  ``too_far``, ``busy``, ``bad_frame``, ``bad_arg``, ``line_too_long``, ``unknown_cmd``.
"""

from __future__ import annotations

from enum import Enum
import math
import threading
import time

from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_FIRMWARE_JOINT,
    PILLAR_HOME_POSITION_M,
    PILLAR_MAX_LINE_LENGTH,
    PILLAR_MAX_POSITION_M,
    PILLAR_MIN_POSITION_M,
    PILLAR_STEP_RESOLUTION_M,
)

# The firmware's defaults: a reversal at 40 mm/s rides v^2/2a = 2.7 mm of ramp.
FIRMWARE_SPEED_MM_S = 40.0
FIRMWARE_ACCEL_MM_S2 = 300.0
# The Nano's bootloader runs before the sketch says ready; the connection waits 2.5 s.
FIRMWARE_BOOT_S = 0.5
FIRMWARE_MAX_RATE_HZ = 100.0
# The top limit switch trips at the rail's zero.
TOP_SWITCH_M = 0.0
# Integration step for the rail: a ramp's shape, not the telemetry rate.
_SUBSTEP_S = 0.001


class _Motion(str, Enum):
    IDLE = "idle"
    MOVE = "move"
    HOME_UP = "home_up"
    HOME_DOWN = "home_down"
    STOP = "stop"


class PillarFirmware:
    """The board's state machine: command lines in, reply and event lines out.

    ``receive`` takes one line the host wrote and returns what the board sends back (events
    that fell due first, then the line's single reply); ``advance`` moves the rail to ``now``
    and returns the events and telemetry that fell due. Both take the time explicitly.
    """

    def __init__(
        self,
        *,
        now: float,
        position_m: float = PILLAR_HOME_POSITION_M,
        boot_s: float = FIRMWARE_BOOT_S,
        speed_mm_s: float = FIRMWARE_SPEED_MM_S,
        accel_mm_s2: float = FIRMWARE_ACCEL_MM_S2,
    ) -> None:
        self._opened_at = now
        self._boot_at = now + boot_s
        self._clock = now
        self._booted = False
        self._position = position_m
        self._velocity = 0.0
        self._speed = speed_mm_s / 1000.0
        self._accel = accel_mm_s2 / 1000.0
        self._echo = True
        self._rate_hz = 0.0
        self._next_frame_at = math.inf
        self._homed = False
        self._motion = _Motion.IDLE
        self._target = position_m
        self._stopping_home = False
        self._switch = position_m >= TOP_SWITCH_M

    @property
    def position_m(self) -> float:
        return self._position

    @property
    def homed(self) -> bool:
        return self._homed

    @property
    def moving(self) -> bool:
        return self._motion is not _Motion.IDLE

    def advance(self, now: float) -> list[str]:
        out: list[str] = []
        if not self._booted:
            if now < self._boot_at:
                self._clock = now
                return out
            self._booted = True
            self._clock = self._boot_at
            out += ["pillar_driver (emulated)", self._event("ready")]
        while self._clock < now:
            step = min(_SUBSTEP_S, now - self._clock)
            self._clock += step
            out += self._step(step)
        if self._rate_hz > 0.0 and now >= self._next_frame_at:
            out.append(f"<{self._ms()},{PILLAR_FIRMWARE_JOINT},{self._position:.6f}>")
            period = 1.0 / self._rate_hz
            self._next_frame_at += period
            if self._next_frame_at <= now:
                self._next_frame_at = now + period
        return out

    def receive(self, line: str, now: float) -> list[str]:
        out = self.advance(now)
        if not self._booted:
            # Lost in the reset, as on the board: nothing answers before ready.
            return out
        line = line.strip()
        if not line:
            return out
        if self._echo:
            out.append(f"> {line}")
        out.append(self._reply(line))
        return out

    # ------------------------------------------------------------------ commands

    def _reply(self, line: str) -> str:
        if len(line) > PILLAR_MAX_LINE_LENGTH:
            return "err line_too_long"
        if line.startswith("<"):
            return self._joint_frame(line)
        words = line.split()
        command = words[0].lower()
        if command == "set":
            return self._set(words[1:])
        if command == "home" and len(words) == 1:
            if self._motion is not _Motion.IDLE:
                return "err busy"
            self._homed = False
            self._go(_Motion.HOME_UP, TOP_SWITCH_M)
            return "ok"
        if command == "stop" and len(words) == 1:
            if self._motion is not _Motion.IDLE:
                self._stopping_home = self._motion in (_Motion.HOME_UP, _Motion.HOME_DOWN)
                stop_at = self._position + self._velocity * abs(self._velocity) / (2 * self._accel)
                self._go(_Motion.STOP, stop_at)
            return "ok"
        return f"err unknown_cmd {command}"

    def _set(self, args: list[str]) -> str:
        if len(args) != 2:
            return "err needs_arg"
        key, raw = args[0].lower(), args[1]
        try:
            value = float(raw)
        except ValueError:
            return "err bad_arg"
        if key == "echo" and value in (0.0, 1.0):
            self._echo = value == 1.0
        elif key == "rate" and 0.0 <= value <= FIRMWARE_MAX_RATE_HZ:
            self._rate_hz = value
            self._next_frame_at = self._clock if value > 0.0 else math.inf
        elif key == "speed" and value > 0.0:
            self._speed = value / 1000.0
        elif key == "accel" and value > 0.0:
            self._accel = value / 1000.0
        elif key in ("echo", "rate", "speed", "accel"):
            return "err bad_arg"
        else:
            return f"err unknown_key {key}"
        return "ok"

    def _joint_frame(self, line: str) -> str:
        fields = line[1:-1].split(",") if line.endswith(">") else []
        if len(fields) != 3 or fields[1] != PILLAR_FIRMWARE_JOINT:
            return "err bad_frame"
        try:
            target = float(fields[2])
        except ValueError:
            return "err bad_frame"
        if not math.isfinite(target):
            return "err bad_frame"
        if not self._homed or self._motion in (_Motion.HOME_UP, _Motion.HOME_DOWN):
            return "err not_homed"
        if not PILLAR_MIN_POSITION_M <= target <= PILLAR_MAX_POSITION_M:
            limit = (
                PILLAR_MAX_POSITION_M if target > PILLAR_MAX_POSITION_M else PILLAR_MIN_POSITION_M
            )
            at_limit = abs(self._position - limit) < PILLAR_STEP_RESOLUTION_M
            return "err at_soft_limit" if at_limit else "err too_far"
        if self._motion is _Motion.IDLE and abs(target - self._position) < PILLAR_STEP_RESOLUTION_M:
            return "err zero_move"
        self._go(_Motion.MOVE, target)
        return "ok"

    # ------------------------------------------------------------------ the rail

    def _go(self, motion: _Motion, target: float) -> None:
        self._motion = motion
        self._target = target

    def _step(self, dt: float) -> list[str]:
        """One substep of the trapezoid toward the target, and what arriving there means."""
        if self._motion is _Motion.IDLE:
            return []
        gap = self._target - self._position
        reachable = math.sqrt(2.0 * self._accel * abs(gap))
        wanted = math.copysign(min(self._speed, reachable), gap)
        change = max(-self._accel * dt, min(self._accel * dt, wanted - self._velocity))
        self._velocity += change
        position = self._position + self._velocity * dt
        arrived = gap == 0.0 or (self._target - position) * gap <= 0.0
        self._position = self._target if arrived else position
        events = self._switch_events()
        if arrived:
            self._velocity = 0.0
            events += self._arrive()
        return events

    def _switch_events(self) -> list[str]:
        pressed = self._position >= TOP_SWITCH_M - PILLAR_STEP_RESOLUTION_M
        if pressed == self._switch:
            return []
        self._switch = pressed
        return [self._event("limit_triggered" if pressed else "limit_clear")]

    def _arrive(self) -> list[str]:
        motion, self._motion = self._motion, _Motion.IDLE
        if motion is _Motion.HOME_UP:
            # The switch is the reference: zero here, then back off to the parking height.
            self._go(_Motion.HOME_DOWN, PILLAR_HOME_POSITION_M)
            return []
        if motion is _Motion.HOME_DOWN:
            self._homed = True
            return [self._event("homed")]
        if motion is _Motion.STOP and self._stopping_home:
            self._stopping_home = False
            return [self._event("homing_aborted")]
        return [self._event("move_done")]

    def _event(self, code: str) -> str:
        return f"<STATUS,{self._ms()},{code}>"

    def _ms(self) -> int:
        return round((self._clock - self._opened_at) * 1000.0)


class PillarEmulator:
    """A serial port with ``PillarFirmware`` on the far end: pyserial's ``read``, ``write``,
    ``flush`` and ``close``, the board's clock running in a thread of its own."""

    def __init__(
        self,
        *,
        timeout: float,
        tick_s: float = 0.005,
        boot_s: float = FIRMWARE_BOOT_S,
        position_m: float = PILLAR_HOME_POSITION_M,
    ) -> None:
        self.timeout = timeout
        self._tick_s = tick_s
        self._firmware = PillarFirmware(now=time.monotonic(), boot_s=boot_s, position_m=position_m)
        self._firmware_lock = threading.Lock()
        self._received = bytearray()
        self._pending = bytearray()
        self._ready = threading.Condition()
        self._closed = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="alfred-pillar-emulator", daemon=True
        )
        self._thread.start()

    @property
    def is_open(self) -> bool:
        return not self._closed.is_set()

    def read(self, size: int = 1) -> bytes:
        deadline = time.monotonic() + self.timeout
        with self._ready:
            while not self._pending:
                if self._closed.is_set():
                    raise OSError("emulated pillar port is closed")
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return b""
                self._ready.wait(remaining)
            chunk = bytes(self._pending[:size])
            del self._pending[:size]
            return chunk

    def write(self, data: bytes) -> int:
        if self._closed.is_set():
            raise OSError("emulated pillar port is closed")
        # Sent under the board's lock, like everything it says: a serial line keeps order.
        with self._firmware_lock:
            self._received.extend(data)
            while (end := self._received.find(b"\n")) >= 0:
                line = self._received[:end].decode("ascii", errors="replace")
                del self._received[: end + 1]
                self._send(self._firmware.receive(line, time.monotonic()))
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self._closed.set()
        with self._ready:
            self._ready.notify_all()
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._closed.wait(self._tick_s):
            with self._firmware_lock:
                self._send(self._firmware.advance(time.monotonic()))

    def _send(self, lines: list[str]) -> None:
        if not lines:
            return
        with self._ready:
            for line in lines:
                self._pending.extend(f"{line}\r\n".encode("ascii"))
            self._ready.notify_all()
