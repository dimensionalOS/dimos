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

import os
import sys
import threading
import time
from typing import Any

import pygame

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT

# Gate event codes published on KeyboardTeleop.operator_command for tools that need
# operator-confirmation per step. Defined in a dependency-free module so offline
# consumers (e.g. the benchmark scorer) don't pull pygame just to read them;
# re-exported here for back-compat with `from keyboard_teleop import GATE_*`.
from dimos.control.benchmarking.gate import GATE_ADVANCE, GATE_QUIT, GATE_SKIP
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Force X11 driver on Linux to avoid OpenGL threading issues. macOS has no X11
# driver (SDL uses cocoa); forcing x11 there makes pygame.display fail outright.
if sys.platform.startswith("linux"):
    os.environ["SDL_VIDEODRIVER"] = "x11"

DEFAULT_LINEAR_SPEED: float = 0.5  # m/s
DEFAULT_ANGULAR_SPEED: float = 0.8  # rad/s
DEFAULT_BOOST_MULTIPLIER: float = 2.0
DEFAULT_SLOW_MULTIPLIER: float = 0.5

_WINDOW_WIDTH = 520
_WINDOW_HEIGHT = 340
_CONTROL_RATE_HZ = 50
_ESTOP_FLASH_S = 0.6
_FONT_NAMES = "dejavusans,liberationsans,arial"

_BG = (17, 19, 23)
_PANEL = (26, 29, 35)
_CAP = (38, 42, 51)
_ACCENT = (0, 170, 255)
_TEXT = (230, 233, 238)
_DIM = (120, 126, 138)
_RED = (235, 72, 72)
_GREEN = (56, 200, 120)

# Order of Joy.buttons on the joystick stream (1 = held).
JOY_BUTTONS = ("w", "a", "s", "d", "q", "e", "shift", "ctrl", "space")


class KeyboardTeleop(Module):
    """Pygame-based keyboard control. Outputs Twist on cmd_vel.

    Also emits operator gate events on ``operator_command: Out[Int8]`` for
    tools that need to pause for operator confirmation between steps (e.g.
    the one-terminal Go2 benchmark blueprint). Three keys: ``ENTER`` ->
    advance, ``K`` -> skip, ``Backspace`` -> quit. Existing blueprints that
    don't wire the ``operator_command`` port are unaffected — the events
    publish into a stream nobody listens to.
    """

    # pygame.display supports one window per process; multi-robot blueprints
    # run one teleop per robot, so each instance needs its own worker.
    dedicated_worker = True

    cmd_vel: Out[Twist]
    # Held keys as Joy.buttons in JOY_BUTTONS order, published on every change.
    joystick: Out[Joy]
    operator_command: Out[Int8]
    # Reference-governor corridor half-width (m). Number keys 0-9 map
    # to 0.0–0.9 m so an operator can dial precision live during a run.
    e_max: Out[Float32]

    _stop_event: threading.Event
    _keys_held: set[int] | None = None
    _thread: threading.Thread | None = None
    _screen: pygame.Surface | None = None
    _clock: pygame.time.Clock | None = None
    _fonts: dict[tuple[int, bool], pygame.font.Font] | None = None

    def __init__(
        self,
        linear_speed: float = DEFAULT_LINEAR_SPEED,
        angular_speed: float = DEFAULT_ANGULAR_SPEED,
        boost_multiplier: float = DEFAULT_BOOST_MULTIPLIER,
        slow_multiplier: float = DEFAULT_SLOW_MULTIPLIER,
        publish_only_when_active: bool = True,
        disable_movement: bool = False,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.boost_multiplier = boost_multiplier
        self.slow_multiplier = slow_multiplier
        # When True, only publish while a movement key is held; on
        # release publish a single zero Twist (stop) then go silent.
        # Lets the teleop coexist with another /cmd_vel publisher
        # (e.g. the SI / benchmark tools) instead of flooding zeros.
        self.publish_only_when_active = publish_only_when_active
        # When True, WASD/QE movement keys are no-ops and the window is a
        # pure 0-9 e_max slider. Used by blueprints that drive cmd_vel
        # from another source (e.g. nav-stack-driven precision controller)
        # but still want the operator's live e_max input.
        self.disable_movement = disable_movement
        self._was_active = False
        self._last_buttons: list[int] | None = None
        self._estop_at = 0.0
        # Namespaced instances (e.g. "robot0/keyboardteleop") get their own
        # window title so multi-robot teleop windows are distinguishable.
        self._window_title = self.config.instance_name or "Keyboard Teleop"

    @rpc
    def start(self) -> None:
        super().start()

        self._keys_held = set()
        self._stop_event.clear()

        self._thread = threading.Thread(target=self._pygame_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        stop_twist = Twist()
        stop_twist.linear = Vector3(0, 0, 0)
        stop_twist.angular = Vector3(0, 0, 0)
        self.cmd_vel.publish(stop_twist)

        self._stop_event.set()

        if self._thread is None:
            raise RuntimeError("Cannot stop: thread was never started")
        self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)

        super().stop()

    def _pygame_loop(self) -> None:
        if self._keys_held is None:
            raise RuntimeError("_keys_held not initialized")

        pygame.init()
        self._screen = pygame.display.set_mode((_WINDOW_WIDTH, _WINDOW_HEIGHT), pygame.SWSURFACE)
        pygame.display.set_caption(self._window_title)
        self._clock = pygame.time.Clock()
        self._fonts = {}

        while not self._stop_event.is_set():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._stop_event.set()
                elif event.type == pygame.KEYDOWN:
                    self._keys_held.add(event.key)

                    if event.key == pygame.K_SPACE:
                        # Emergency stop - clear all keys and send zero twist
                        self._keys_held.clear()
                        stop_twist = Twist()
                        stop_twist.linear = Vector3(0, 0, 0)
                        stop_twist.angular = Vector3(0, 0, 0)
                        self.cmd_vel.publish(stop_twist)
                        self._estop_at = time.monotonic()
                        logger.warning("EMERGENCY STOP!")
                    elif event.key == pygame.K_ESCAPE:
                        # ESC quits
                        self._stop_event.set()
                    elif event.key == pygame.K_RETURN:
                        self.operator_command.publish(Int8(GATE_ADVANCE))
                    elif event.key == pygame.K_k:
                        self.operator_command.publish(Int8(GATE_SKIP))
                    elif event.key == pygame.K_BACKSPACE:
                        self.operator_command.publish(Int8(GATE_QUIT))
                    elif pygame.K_0 <= event.key <= pygame.K_9:
                        # 0 → 0.0 m, 1 → 0.1 m, …, 9 → 0.9 m corridor half-width.
                        self.e_max.publish(Float32(data=(event.key - pygame.K_0) * 0.1))

                elif event.type == pygame.KEYUP:
                    self._keys_held.discard(event.key)

            # Generate Twist message from held keys
            twist = Twist()
            twist.linear = Vector3(0, 0, 0)
            twist.angular = Vector3(0, 0, 0)

            # Movement keys (WASD/QE) — guarded by disable_movement so the
            # window can run as a pure e_max slider (0-9 keys stay live in
            # the KEYDOWN handler above).
            if not self.disable_movement:
                # Forward/backward (W/S)
                if pygame.K_w in self._keys_held:
                    twist.linear.x = self.linear_speed
                if pygame.K_s in self._keys_held:
                    twist.linear.x = -self.linear_speed

                # Strafe left/right (Q/E)
                if pygame.K_q in self._keys_held:
                    twist.linear.y = self.linear_speed
                if pygame.K_e in self._keys_held:
                    twist.linear.y = -self.linear_speed

                # Turning (A/D)
                if pygame.K_a in self._keys_held:
                    twist.angular.z = self.angular_speed
                if pygame.K_d in self._keys_held:
                    twist.angular.z = -self.angular_speed

            # Apply speed modifiers (Shift = boost, Ctrl = slow)
            speed_multiplier = 1.0
            if pygame.K_LSHIFT in self._keys_held or pygame.K_RSHIFT in self._keys_held:
                speed_multiplier = self.boost_multiplier
            elif pygame.K_LCTRL in self._keys_held or pygame.K_RCTRL in self._keys_held:
                speed_multiplier = self.slow_multiplier

            twist.linear.x *= speed_multiplier
            twist.linear.y *= speed_multiplier
            twist.angular.z *= speed_multiplier

            if self.publish_only_when_active:
                active = twist.linear.x != 0 or twist.linear.y != 0 or twist.angular.z != 0
                # Publish while active; publish exactly one zero on the
                # active->idle transition (clean stop); then stay silent
                # so a co-publisher owns /cmd_vel.
                if active or self._was_active:
                    self.cmd_vel.publish(twist)
                self._was_active = active
            else:
                self.cmd_vel.publish(twist)

            self._publish_joystick()
            self._update_display(twist)

            # Maintain control loop rate
            if self._clock is None:
                raise RuntimeError("_clock not initialized")
            self._clock.tick(_CONTROL_RATE_HZ)

        pygame.quit()

    def _publish_joystick(self) -> None:
        pressed = pygame.key.get_pressed()
        mods = pygame.key.get_mods()
        buttons = [
            int(pressed[pygame.K_w]),
            int(pressed[pygame.K_a]),
            int(pressed[pygame.K_s]),
            int(pressed[pygame.K_d]),
            int(pressed[pygame.K_q]),
            int(pressed[pygame.K_e]),
            int(bool(mods & pygame.KMOD_SHIFT)),
            int(bool(mods & pygame.KMOD_CTRL)),
            int(pressed[pygame.K_SPACE]),
        ]
        if buttons != self._last_buttons:
            self.joystick.publish(Joy(buttons=buttons))
            self._last_buttons = buttons

    def _font(self, size: int, bold: bool = False) -> pygame.font.Font:
        if self._fonts is None:
            raise RuntimeError("Not initialized correctly")
        key = (size, bold)
        if key not in self._fonts:
            self._fonts[key] = pygame.font.SysFont(_FONT_NAMES, size, bold=bold)
        return self._fonts[key]

    def _keycap(
        self,
        rect: pygame.Rect,
        label: str,
        held: bool,
        sub: str = "",
        color: tuple[int, int, int] = _ACCENT,
    ) -> None:
        if self._screen is None:
            raise RuntimeError("Not initialized correctly")
        pygame.draw.rect(self._screen, color if held else _CAP, rect, border_radius=10)
        text = self._font(20, True).render(label, True, _BG if held else _TEXT)
        self._screen.blit(
            text, text.get_rect(center=(rect.centerx, rect.centery - (6 if sub else 0)))
        )
        if sub:
            text = self._font(11).render(sub, True, _BG if held else _DIM)
            self._screen.blit(text, text.get_rect(center=(rect.centerx, rect.centery + 14)))

    def _update_display(self, twist: Twist) -> None:
        if self._screen is None or self._keys_held is None:
            raise RuntimeError("Not initialized correctly")
        screen = self._screen
        screen.fill(_BG)

        pressed = pygame.key.get_pressed()
        mods = pygame.key.get_mods()
        moving = twist.linear.x != 0 or twist.linear.y != 0 or twist.angular.z != 0
        estop = time.monotonic() - self._estop_at < _ESTOP_FLASH_S

        screen.blit(self._font(16, True).render(self._window_title.upper(), True, _TEXT), (20, 18))
        status, color = (
            ("E-STOP", _RED) if estop else ("DRIVING", _RED) if moving else ("IDLE", _GREEN)
        )
        pill = pygame.Rect(_WINDOW_WIDTH - 120, 14, 100, 26)
        pygame.draw.rect(screen, color, pill, border_radius=13)
        text = self._font(13, True).render(status, True, _BG)
        screen.blit(text, text.get_rect(center=pill.center))

        movement = not self.disable_movement
        rows = [
            [(pygame.K_q, "Q", "strafe"), (pygame.K_w, "W", "fwd"), (pygame.K_e, "E", "strafe")],
            [(pygame.K_a, "A", "turn"), (pygame.K_s, "S", "back"), (pygame.K_d, "D", "turn")],
        ]
        for r, row in enumerate(rows):
            for c, (key, label, sub) in enumerate(row):
                rect = pygame.Rect(20 + c * 60, 60 + r * 60, 52, 52)
                self._keycap(rect, label, movement and bool(pressed[key]), sub)
        boost, slow = f"boost {self.boost_multiplier:g}x", f"slow {self.slow_multiplier:g}x"
        self._keycap(pygame.Rect(20, 180, 82, 52), "Shift", bool(mods & pygame.KMOD_SHIFT), boost)
        self._keycap(pygame.Rect(110, 180, 82, 52), "Ctrl", bool(mods & pygame.KMOD_CTRL), slow)
        self._keycap(pygame.Rect(20, 240, 172, 52), "Space", estop, "e-stop", _RED)

        px = 220
        pygame.draw.rect(screen, _PANEL, (px, 60, _WINDOW_WIDTH - px - 20, 232), border_radius=12)
        top_linear = self.linear_speed * self.boost_multiplier
        top_angular = self.angular_speed * self.boost_multiplier
        axes = [
            ("forward", twist.linear.x, top_linear, "m/s"),
            ("strafe", twist.linear.y, top_linear, "m/s"),
            ("yaw", twist.angular.z, top_angular, "rad/s"),
        ]
        for i, (name, value, top, unit) in enumerate(axes):
            y = 84 + i * 66
            screen.blit(self._font(13).render(name.upper(), True, _DIM), (px + 16, y))
            text = self._font(18, True).render(f"{value:+.2f} {unit}", True, _TEXT)
            screen.blit(text, (_WINDOW_WIDTH - 36 - text.get_width(), y - 3))
            bar = pygame.Rect(px + 16, y + 26, _WINDOW_WIDTH - px - 52, 8)
            pygame.draw.rect(screen, _CAP, bar, border_radius=4)
            width = int(bar.width / 2 * max(-1.0, min(1.0, value / top))) if top else 0
            fill = pygame.Rect(min(bar.centerx, bar.centerx + width), bar.y, abs(width), 8)
            pygame.draw.rect(screen, _ACCENT, fill, border_radius=4)
            pygame.draw.line(screen, _DIM, (bar.centerx, bar.y - 3), (bar.centerx, bar.bottom + 2))

        help_text = "Esc quit  ·  Enter advance  ·  K skip  ·  Bksp quit tool  ·  0-9 e_max"
        if self.disable_movement:
            help_text = "Movement off (e_max mode)  ·  " + help_text
        screen.blit(self._font(12).render(help_text, True, _DIM), (20, _WINDOW_HEIGHT - 28))

        pygame.display.flip()
