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
import threading
from typing import Any

import pygame

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Force X11 driver to avoid OpenGL threading issues
os.environ["SDL_VIDEODRIVER"] = "x11"

DEFAULT_LINEAR_SPEED: float = 0.5  # m/s
DEFAULT_ANGULAR_SPEED: float = 0.8  # rad/s
DEFAULT_BOOST_MULTIPLIER: float = 2.0
DEFAULT_SLOW_MULTIPLIER: float = 0.5

_WINDOW_WIDTH = 500
_WINDOW_HEIGHT = 400
_FONT_SIZE = 24
_CONTROL_RATE_HZ = 50
_BG_COLOR = (30, 30, 30)
_HELP_TEXT_COLOR = (150, 150, 150)
_INDICATOR_RADIUS = 15


class KeyboardTeleop(Module):
    """Pygame-based keyboard control. Outputs Twist on /tele_cmd_vel; speeds tunable via constructor."""

    tele_cmd_vel: Out[Twist]  # Standard velocity commands

    _stop_event: threading.Event
    _keys_held: set[int] | None = None
    _thread: threading.Thread | None = None
    _screen: pygame.Surface | None = None
    _clock: pygame.time.Clock | None = None
    _font: pygame.font.Font | None = None

    def __init__(
        self,
        linear_speed: float = DEFAULT_LINEAR_SPEED,
        angular_speed: float = DEFAULT_ANGULAR_SPEED,
        boost_multiplier: float = DEFAULT_BOOST_MULTIPLIER,
        slow_multiplier: float = DEFAULT_SLOW_MULTIPLIER,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.boost_multiplier = boost_multiplier
        self.slow_multiplier = slow_multiplier

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
        self.tele_cmd_vel.publish(stop_twist)

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
        pygame.display.set_caption("Keyboard Teleop")
        self._clock = pygame.time.Clock()
        self._font = pygame.font.Font(None, _FONT_SIZE)

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
                        self.tele_cmd_vel.publish(stop_twist)
                        logger.warning("EMERGENCY STOP!")
                    elif event.key == pygame.K_ESCAPE:
                        # ESC quits
                        self._stop_event.set()

                elif event.type == pygame.KEYUP:
                    self._keys_held.discard(event.key)

            # Generate Twist message from held keys
            twist = Twist()
            twist.linear = Vector3(0, 0, 0)
            twist.angular = Vector3(0, 0, 0)

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

            self.tele_cmd_vel.publish(twist)

            self._update_display(twist)

            # Maintain control loop rate
            if self._clock is None:
                raise RuntimeError("_clock not initialized")
            self._clock.tick(_CONTROL_RATE_HZ)

        pygame.quit()

    def _update_display(self, twist: Twist) -> None:
        if self._screen is None or self._font is None or self._keys_held is None:
            raise RuntimeError("Not initialized correctly")

        self._screen.fill(_BG_COLOR)

        y_pos = 20

        # Determine active speed multiplier
        speed_mult_text = ""
        if pygame.K_LSHIFT in self._keys_held or pygame.K_RSHIFT in self._keys_held:
            speed_mult_text = f" [BOOST {self.boost_multiplier:g}x]"
        elif pygame.K_LCTRL in self._keys_held or pygame.K_RCTRL in self._keys_held:
            speed_mult_text = f" [SLOW {self.slow_multiplier:g}x]"

        texts = [
            "Keyboard Teleop" + speed_mult_text,
            "",
            f"Linear X (Forward/Back): {twist.linear.x:+.2f} m/s",
            f"Linear Y (Strafe L/R): {twist.linear.y:+.2f} m/s",
            f"Angular Z (Turn L/R): {twist.angular.z:+.2f} rad/s",
            "",
            "Keys: " + ", ".join([pygame.key.name(k).upper() for k in self._keys_held if k < 256]),
        ]

        for text in texts:
            if text:
                color = (0, 255, 255) if text.startswith("Keyboard Teleop") else (255, 255, 255)
                surf = self._font.render(text, True, color)
                self._screen.blit(surf, (20, y_pos))
            y_pos += 30

        if twist.linear.x != 0 or twist.linear.y != 0 or twist.angular.z != 0:
            pygame.draw.circle(self._screen, (255, 0, 0), (450, 30), _INDICATOR_RADIUS)
        else:
            pygame.draw.circle(self._screen, (0, 255, 0), (450, 30), _INDICATOR_RADIUS)

        y_pos = 280
        help_texts = [
            "WS: Move | AD: Turn | QE: Strafe",
            "Shift: Boost | Ctrl: Slow",
            "Space: E-Stop | ESC: Quit",
        ]
        for text in help_texts:
            surf = self._font.render(text, True, _HELP_TEXT_COLOR)
            self._screen.blit(surf, (20, y_pos))
            y_pos += 25

        pygame.display.flip()
