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

"""Headless terminal teleop for an already-running DimOS robot stack."""

from __future__ import annotations

import contextlib
import curses
from dataclasses import dataclass
import sys
import time
from typing import Any

from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

DEFAULT_LINEAR_SPEED = 0.15
DEFAULT_ANGULAR_SPEED = 0.3
DEFAULT_DEADMAN_TIMEOUT = 0.2
DEFAULT_RATE_HZ = 20.0

_ENTER_KEYS = (10, 13, curses.KEY_ENTER)
_QUIT_KEYS = (3, 27)  # Ctrl+C (when returned by curses), Escape


def _zero_twist() -> Twist:
    return Twist(linear=Vector3(), angular=Vector3())


def _key_character(key: int) -> str | None:
    if 0 <= key < 256:
        return chr(key).lower()
    return None


def _movement_for_key(
    key: int, linear_speed: float, angular_speed: float
) -> tuple[Twist, str] | None:
    key_character = _key_character(key)
    if key in (curses.KEY_UP,) or key_character == "w":
        return (
            Twist(linear=Vector3(linear_speed, 0.0, 0.0), angular=Vector3()),
            "FORWARD",
        )
    if key in (curses.KEY_DOWN,) or key_character == "s":
        return (
            Twist(linear=Vector3(-linear_speed, 0.0, 0.0), angular=Vector3()),
            "BACKWARD",
        )
    if key in (curses.KEY_LEFT,) or key_character == "a":
        return (
            Twist(linear=Vector3(), angular=Vector3(0.0, 0.0, angular_speed)),
            "TURN LEFT",
        )
    if key in (curses.KEY_RIGHT,) or key_character == "d":
        return (
            Twist(linear=Vector3(), angular=Vector3(0.0, 0.0, -angular_speed)),
            "TURN RIGHT",
        )
    if key_character == "q":
        return (
            Twist(linear=Vector3(0.0, linear_speed, 0.0), angular=Vector3()),
            "STRAFE LEFT",
        )
    if key_character == "e":
        return (
            Twist(linear=Vector3(0.0, -linear_speed, 0.0), angular=Vector3()),
            "STRAFE RIGHT",
        )
    return None


@dataclass
class TerminalTeleopState:
    """State machine implementing engagement and a keypress-refreshed command lease."""

    linear_speed: float = DEFAULT_LINEAR_SPEED
    angular_speed: float = DEFAULT_ANGULAR_SPEED
    deadman_timeout: float = DEFAULT_DEADMAN_TIMEOUT
    engaged: bool = False
    ever_engaged: bool = False
    should_quit: bool = False
    action: str = "DISENGAGED — press Enter"
    _active_command: Twist | None = None
    _active_until: float = 0.0
    _stop_pending: bool = False

    def handle_key(self, key: int, now: float) -> None:
        """Update teleop state for one curses key code."""
        if key in _QUIT_KEYS:
            self._disengage("EXITING")
            self.should_quit = True
            return

        if key == ord(" "):
            self._disengage("STOPPED — press Enter to re-engage")
            return

        if key in _ENTER_KEYS:
            self.engaged = True
            self.ever_engaged = True
            self.action = "ENGAGED — tap or hold a movement key"
            return

        if not self.engaged:
            return

        movement = _movement_for_key(key, self.linear_speed, self.angular_speed)
        if movement is None:
            return

        self._active_command, self.action = movement
        self._active_until = now + self.deadman_timeout

    def next_command(self, now: float) -> Twist | None:
        """Return the command to publish this tick, or None while idle."""
        if self._stop_pending:
            self._stop_pending = False
            return _zero_twist()

        if self._active_command is None:
            return None

        if now <= self._active_until:
            return self._active_command

        self._active_command = None
        self.action = "READY — movement command expired"
        return _zero_twist()

    def _disengage(self, action: str) -> None:
        self.engaged = False
        self._active_command = None
        self._active_until = 0.0
        self._stop_pending = True
        self.action = action


def _safe_add_line(screen: Any, row: int, text: str, attribute: int = 0) -> None:
    height, width = screen.getmaxyx()
    if row >= height or width <= 1:
        return
    with contextlib.suppress(curses.error):
        screen.addnstr(row, 0, text, width - 1, attribute)


def _draw(screen: Any, state: TerminalTeleopState, topic: str) -> None:
    screen.erase()
    _safe_add_line(screen, 0, "DimOS terminal cmd_vel teleop", curses.A_BOLD)
    _safe_add_line(screen, 2, f"Topic: {topic}")
    _safe_add_line(
        screen,
        3,
        f"Speed: {state.linear_speed:.2f} m/s, {state.angular_speed:.2f} rad/s",
    )
    _safe_add_line(screen, 5, "Enter      engage")
    _safe_add_line(screen, 6, "W/S or ↑/↓  forward/backward")
    _safe_add_line(screen, 7, "A/D or ←/→  turn left/right")
    _safe_add_line(screen, 8, "Q/E        strafe left/right")
    _safe_add_line(screen, 9, "Space      STOP and disengage")
    _safe_add_line(screen, 10, "Esc        stop and quit")
    _safe_add_line(
        screen,
        12,
        f"Dead-man: motion expires {state.deadman_timeout:.2f}s after the last key event.",
    )
    status_attribute = curses.A_BOLD if state.engaged else curses.A_REVERSE
    _safe_add_line(screen, 14, f"Status: {state.action}", status_attribute)
    screen.refresh()


def _terminal_loop(
    screen: Any,
    state: TerminalTeleopState,
    transport: PubSubTransport[Twist],
    topic: str,
    rate_hz: float,
) -> None:
    with contextlib.suppress(curses.error):
        curses.curs_set(0)
    screen.keypad(True)
    screen.timeout(max(1, round(1000.0 / rate_hz)))

    while not state.should_quit:
        _draw(screen, state, topic)
        key = screen.getch()
        now = time.monotonic()
        if key != -1:
            state.handle_key(key, now)

        command = state.next_command(now)
        if command is not None:
            transport.broadcast(None, command)


def run_terminal_teleop(
    *,
    topic: str = "cmd_vel",
    linear_speed: float = DEFAULT_LINEAR_SPEED,
    angular_speed: float = DEFAULT_ANGULAR_SPEED,
    deadman_timeout: float = DEFAULT_DEADMAN_TIMEOUT,
    rate_hz: float = DEFAULT_RATE_HZ,
) -> None:
    """Run terminal teleop and publish Twist into an existing DimOS stack."""
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise RuntimeError("terminal teleop requires an interactive TTY (for SSH, use ssh -t)")

    state = TerminalTeleopState(
        linear_speed=linear_speed,
        angular_speed=angular_speed,
        deadman_timeout=deadman_timeout,
    )
    transport = make_transport(topic, Twist)
    transport.start()
    try:
        curses.wrapper(_terminal_loop, state, transport, topic, rate_hz)
    except KeyboardInterrupt:
        pass
    finally:
        if state.ever_engaged:
            transport.broadcast(None, _zero_twist())
        transport.stop()
