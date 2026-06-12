#!/usr/bin/env python3
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

"""Terminal (TUI) keyboard teleop. Drop-in alternative to ``KeyboardTeleop``.

Unlike the pygame version this needs no display/window: it reads keypresses
straight from the controlling terminal and renders a status panel with ANSI
escapes. Same ``tele_cmd_vel`` Twist output and the same constructor knobs, so it
can be swapped into a blueprint wherever ``KeyboardTeleop`` is used.

Terminals can't report key-release, only key-repeat, so "hold to move" is
emulated: a movement key stays active for ``key_timeout`` seconds after its
last repeat. Holding a key refreshes it continuously (smooth motion); on
release the robot coasts to a stop within ``key_timeout``.

Controls:
    W/S: forward / back
    A/D: turn left / right
    Q/E: strafe left / right
    Shift+key (tap): toggle turbo on/off
    1: sit (lie down)    2: stand (and re-enter walk mode)
    SPACE: emergency stop
    ESC: quit

Sit/stand are driven through optional ``on_sit`` / ``on_stand`` callbacks so the
module stays robot-agnostic; the caller wires them to the robot's RPCs.
"""

from collections.abc import Callable
import os
import select
import sys
import termios
import threading
import time
import tty
from typing import Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_LINEAR_SPEED: float = 0.3  # m/s
DEFAULT_ANGULAR_SPEED: float = 0.6  # rad/s
DEFAULT_BOOST_MULTIPLIER: float = 2.0
DEFAULT_SLOW_MULTIPLIER: float = 0.5
DEFAULT_KEY_TIMEOUT: float = 0.4  # s a key stays "held" after its last repeat

_CONTROL_RATE_HZ = 50
# Min seconds between turbo toggles; debounces terminal key-repeat so holding
# Shift+key flips turbo once, not every repeat.
_TURBO_TOGGLE_COOLDOWN = 0.5

# ANSI helpers.
_ENTER_ALT = "\033[?1049h"  # switch to alternate screen buffer
_EXIT_ALT = "\033[?1049l"  # restore the normal screen (and its scrollback)
_HIDE_CURSOR = "\033[?25l"
_SHOW_CURSOR = "\033[?25h"
_CLEAR_SCREEN = "\033[2J"
_CLEAR_BELOW = "\033[J"  # clear from cursor to end of screen
_CURSOR_HOME = "\033[H"
_CLEAR_LINE = "\033[K"
_RESET = "\033[0m"
_CYAN = "\033[36m"
_DIM = "\033[2m"
_GREEN = "\033[32m"
_RED = "\033[31m"

_BOX_WIDTH = 46  # visible width of the panel's inner content area

# Movement keys -> (linear_x, linear_y, angular_z) unit contribution.
_KEY_MOTION: dict[str, tuple[float, float, float]] = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "q": (0.0, 1.0, 0.0),
    "e": (0.0, -1.0, 0.0),
    "a": (0.0, 0.0, 1.0),
    "d": (0.0, 0.0, -1.0),
}


class KeyboardTeleopTUI(Module):
    """Terminal-based keyboard control. Outputs Twist on tele_cmd_vel."""

    tele_cmd_vel: Out[Twist]

    _stop_event: threading.Event
    _thread: threading.Thread | None = None

    def __init__(
        self,
        linear_speed: float = DEFAULT_LINEAR_SPEED,
        angular_speed: float = DEFAULT_ANGULAR_SPEED,
        boost_multiplier: float = DEFAULT_BOOST_MULTIPLIER,
        slow_multiplier: float = DEFAULT_SLOW_MULTIPLIER,
        key_timeout: float = DEFAULT_KEY_TIMEOUT,
        publish_only_when_active: bool = False,
        on_sit: Callable[[], Any] | None = None,
        on_stand: Callable[[], Any] | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.boost_multiplier = boost_multiplier
        self.slow_multiplier = slow_multiplier
        self.key_timeout = key_timeout
        # Discrete robot actions, run on a background thread so the control loop
        # stays responsive. on_stand should also re-enter the locomotion mode so
        # the robot can walk again (e.g. standup() then balance_stand()).
        self.on_sit = on_sit
        self.on_stand = on_stand
        # When True, only publish while a movement key is held; on
        # release publish a single zero Twist (stop) then go silent.
        # Lets the teleop coexist with another /tele_cmd_vel publisher
        # instead of flooding zeros.
        self.publish_only_when_active = publish_only_when_active
        self._was_active = False
        # key char -> monotonic timestamp of its most recent press.
        self._last_press: dict[str, float] = {}
        # Turbo (boost) is a toggle now, not press-and-hold.
        self._turbo = False
        self._last_turbo_toggle = 0.0
        # Discrete-action state: name of the in-flight action ("sitting"/
        # "standing") or None. Guarded so only one runs at a time.
        self._action_lock = threading.Lock()
        self._action_status: str | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._last_press.clear()
        self._turbo = False
        self._last_turbo_toggle = 0.0
        self._thread = threading.Thread(target=self._tui_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._publish_stop()
        self._stop_event.set()
        if self._thread is None:
            raise RuntimeError("Cannot stop: thread was never started")
        self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _publish_stop(self) -> None:
        stop_twist = Twist()
        stop_twist.linear = Vector3(0, 0, 0)
        stop_twist.angular = Vector3(0, 0, 0)
        self.tele_cmd_vel.publish(stop_twist)

    def _tui_loop(self) -> None:
        if not sys.stdin.isatty():
            logger.warning(
                "KeyboardTeleopTUI: stdin is not a TTY; no keyboard input will be "
                "read. Run this module in the foreground of an interactive terminal."
            )
            self._stop_event.wait()
            return

        fd = sys.stdin.fileno()
        old_attrs = termios.tcgetattr(fd)
        period = 1.0 / _CONTROL_RATE_HZ
        # Render on the alternate screen buffer so the panel owns the terminal,
        # isolated from the coordinator/worker log spam (which keeps going to the
        # log file). The full repaint each frame wipes any stray writes that leak
        # through. Leaving the buffer on exit restores the user's normal screen.
        sys.stdout.write(_ENTER_ALT + _HIDE_CURSOR + _CLEAR_SCREEN)
        sys.stdout.flush()
        try:
            # cbreak leaves ISIG on, so Ctrl+C still works as usual.
            tty.setcbreak(fd)
            while not self._stop_event.is_set():
                ready, _, _ = select.select([sys.stdin], [], [], period)
                if ready:
                    data = os.read(fd, 64).decode("utf-8", errors="ignore")
                    if not self._handle_input(data):
                        break

                twist = self._compute_twist()
                self._maybe_publish(twist)
                self._render(twist)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
            sys.stdout.write(_SHOW_CURSOR + _EXIT_ALT)
            sys.stdout.flush()

    def _handle_input(self, data: str) -> bool:
        """Process a raw read. Returns False to request quit."""
        now = time.monotonic()
        for ch in data:
            if ch == " ":
                # Emergency stop: drop all held keys and send a zero Twist.
                self._last_press.clear()
                self._publish_stop()
                logger.warning("EMERGENCY STOP!")
            elif ch == "\x1b":
                # Lone ESC quits; ESC-sequences (arrow keys) are ignored here.
                if len(data) == 1:
                    return False
            elif ch == "1":
                self._trigger_action("sitting", self.on_sit)
            elif ch == "2":
                self._trigger_action("standing", self.on_stand)
            else:
                lower = ch.lower()
                if lower in _KEY_MOTION:
                    self._last_press[lower] = now
                    # Uppercase (Shift) taps toggle turbo; cooldown debounces
                    # terminal key-repeat so a held key flips it only once.
                    if ch.isupper() and now - self._last_turbo_toggle > _TURBO_TOGGLE_COOLDOWN:
                        self._turbo = not self._turbo
                        self._last_turbo_toggle = now
        return True

    def _trigger_action(self, name: str, fn: Callable[[], Any] | None) -> None:
        """Run a discrete robot action (sit/stand) on a background thread.

        Movement is suppressed while it runs; re-triggers are ignored until it
        finishes so key-repeat or impatient taps can't stack commands.
        """
        if fn is None:
            return
        with self._action_lock:
            if self._action_status is not None:
                return
            self._action_status = name
        self._last_press.clear()  # stop driving while the robot changes posture

        def run() -> None:
            try:
                fn()
            except Exception:
                logger.exception(f"KeyboardTeleopTUI: {name} action failed")
            finally:
                with self._action_lock:
                    self._action_status = None

        threading.Thread(target=run, daemon=True).start()

    def _compute_twist(self) -> Twist:
        now = time.monotonic()
        twist = Twist()
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)

        # Hold still while a sit/stand action is in flight.
        if self._action_status is not None:
            return twist

        for key, (fx, fy, fz) in _KEY_MOTION.items():
            if now - self._last_press.get(key, float("-inf")) <= self.key_timeout:
                twist.linear.x += fx * self.linear_speed
                twist.linear.y += fy * self.linear_speed
                twist.angular.z += fz * self.angular_speed

        multiplier = self.boost_multiplier if self._turbo else 1.0
        twist.linear.x *= multiplier
        twist.linear.y *= multiplier
        twist.angular.z *= multiplier
        return twist

    def _maybe_publish(self, twist: Twist) -> None:
        if self.publish_only_when_active:
            active = twist.linear.x != 0 or twist.linear.y != 0 or twist.angular.z != 0
            # Publish while active; publish exactly one zero on the
            # active->idle transition (clean stop); then stay silent.
            if active or self._was_active:
                self.tele_cmd_vel.publish(twist)
            self._was_active = active
        else:
            self.tele_cmd_vel.publish(twist)

    def _render(self, twist: Twist) -> None:
        now = time.monotonic()
        moving = twist.linear.x != 0 or twist.linear.y != 0 or twist.angular.z != 0
        action = self._action_status
        held = sorted(
            key.upper()
            for key in _KEY_MOTION
            if now - self._last_press.get(key, float("-inf")) <= self.key_timeout
        )
        status_plain = "● MOVING" if moving else "● IDLE"
        status_color = _RED if moving else _GREEN
        turbo_txt = f"   [TURBO {self.boost_multiplier:g}x]" if self._turbo else ""
        action_line = f"▶ {action}…" if action else ""

        # (plain_text, color) rows. Padding is computed from the *plain* text so
        # invisible ANSI color codes don't throw off the box alignment.
        rows: list[tuple[str, str]] = [
            (f"{status_plain}{turbo_txt}", status_color),
            (action_line, _CYAN if action else ""),
            ("", ""),
            (f"Linear X (Fwd/Back) : {twist.linear.x:+.2f} m/s", ""),
            (f"Linear Y (Strafe)   : {twist.linear.y:+.2f} m/s", ""),
            (f"Angular Z (Turn)    : {twist.angular.z:+.2f} rad/s", ""),
            ("", ""),
            (f"Held: {', '.join(held) if held else '-'}", ""),
            ("", ""),
            ("WS: move    AD: turn    QE: strafe", _DIM),
            ("Shift+key (tap): toggle turbo", _DIM),
            ("1: sit    2: stand", _DIM),
            ("SPACE: e-stop    ESC: quit", _DIM),
        ]

        w = _BOX_WIDTH
        title = " Keyboard Teleop (TUI) "
        top = "┌" + title + "─" * (w + 2 - len(title)) + "┐"
        bottom = "└" + "─" * (w + 2) + "┘"

        lines = [f"{_CYAN}{top}{_RESET}"]
        for plain, color in rows:
            text = plain[:w]
            pad = " " * (w - len(text))
            inner = f"{color}{text}{_RESET}{pad}" if color else f"{text}{pad}"
            lines.append(f"{_CYAN}│{_RESET} {inner} {_CYAN}│{_RESET}")
        lines.append(f"{_CYAN}{bottom}{_RESET}")

        out = (
            _CURSOR_HOME + "\r\n".join(line + _CLEAR_LINE for line in lines) + "\r\n" + _CLEAR_BELOW
        )
        sys.stdout.write(out)
        sys.stdout.flush()
