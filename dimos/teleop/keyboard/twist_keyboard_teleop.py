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

"""Mobile base keyboard teleop module publishing Twist on cmd_vel.

Pygame-based WASD keyboard control for mobile robots. Reads keyboard input
in a background thread and publishes Twist velocity commands at 50 Hz.

This is for mobile base teleoperation (holonomic drive). For cartesian arm
teleoperation, see keyboard_teleop_module.py in this directory.

Keyboard controls:
    W/S: Forward/backward (linear.x)
    Q/E: Strafe left/right (linear.y)
    A/D: Turn left/right (angular.z)
    Shift: 2x speed boost
    Ctrl: 0.5x speed reduction
    Space: Emergency stop (clears all motion, logs warning)
    ESC: Quit
"""

from dataclasses import dataclass
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
_BACKGROUND_COLOR = (30, 30, 30)
_HELP_TEXT_COLOR = (150, 150, 150)
_INDICATOR_RADIUS = 15


@dataclass(frozen=True)
class KeyboardState:
    """Snapshot of keyboard input state for a single control-loop tick."""

    forward: bool = False
    backward: bool = False
    strafe_left: bool = False
    strafe_right: bool = False
    turn_left: bool = False
    turn_right: bool = False
    boost: bool = False
    slow: bool = False


@dataclass
class VelocityConfig:
    """Velocity parameters for keyboard teleop."""

    linear_speed: float = DEFAULT_LINEAR_SPEED
    angular_speed: float = DEFAULT_ANGULAR_SPEED
    boost_multiplier: float = DEFAULT_BOOST_MULTIPLIER
    slow_multiplier: float = DEFAULT_SLOW_MULTIPLIER


def keyboard_state_to_twist(state: KeyboardState, cfg: VelocityConfig) -> Twist:
    """Convert keyboard state to a Twist velocity command.

    Pure function: no pygame, no I/O, no logging.

    When opposing keys are held simultaneously (e.g. W+S), the negative
    direction wins (backward, strafe-right, turn-right) because its
    assignment executes second.  Boost takes priority over slow when both
    modifiers are held.
    """
    twist = Twist()
    twist.linear = Vector3(0, 0, 0)
    twist.angular = Vector3(0, 0, 0)

    if state.forward:
        twist.linear.x = cfg.linear_speed
    if state.backward:
        twist.linear.x = -cfg.linear_speed

    if state.strafe_left:
        twist.linear.y = cfg.linear_speed
    if state.strafe_right:
        twist.linear.y = -cfg.linear_speed

    if state.turn_left:
        twist.angular.z = cfg.angular_speed
    if state.turn_right:
        twist.angular.z = -cfg.angular_speed

    speed_multiplier = 1.0
    if state.boost:
        speed_multiplier = cfg.boost_multiplier
    elif state.slow:
        speed_multiplier = cfg.slow_multiplier

    twist.linear.x *= speed_multiplier
    twist.linear.y *= speed_multiplier
    twist.angular.z *= speed_multiplier

    return twist


class KeyboardTeleop(Module):
    """Pygame-based keyboard control for mobile bases. Outputs Twist on cmd_vel.

    Runs a pygame window in a background thread that captures WASD/QE key
    input, converts it to a Twist via keyboard_state_to_twist(), and
    publishes on cmd_vel at 50 Hz.

    Keyboard controls:
        W/S: Forward/backward (linear.x)
        Q/E: Strafe left/right (linear.y)
        A/D: Turn left/right (angular.z)
        Shift: Speed boost (default 2x)
        Ctrl: Speed reduction (default 0.5x)
        Space: Emergency stop
        ESC: Quit

    Modes:
        publish_only_when_active=False (default):
            Publishes every tick, including zero Twist when idle.
        publish_only_when_active=True:
            Only publishes while a movement key is held. On the
            active-to-idle transition, publishes a single zero Twist
            (clean stop) then goes silent, allowing a co-publisher
            to own /cmd_vel.
    """

    cmd_vel: Out[Twist]

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
        publish_only_when_active: bool = False,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._velocity_cfg = VelocityConfig(
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            boost_multiplier=boost_multiplier,
            slow_multiplier=slow_multiplier,
        )
        self.publish_only_when_active = publish_only_when_active
        self._was_active = False

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
                        self._keys_held.clear()
                        stop_twist = Twist()
                        stop_twist.linear = Vector3(0, 0, 0)
                        stop_twist.angular = Vector3(0, 0, 0)
                        self.cmd_vel.publish(stop_twist)
                        logger.warning("EMERGENCY STOP!")
                    elif event.key == pygame.K_ESCAPE:
                        self._stop_event.set()

                elif event.type == pygame.KEYUP:
                    self._keys_held.discard(event.key)

            state = KeyboardState(
                forward=pygame.K_w in self._keys_held,
                backward=pygame.K_s in self._keys_held,
                strafe_left=pygame.K_q in self._keys_held,
                strafe_right=pygame.K_e in self._keys_held,
                turn_left=pygame.K_a in self._keys_held,
                turn_right=pygame.K_d in self._keys_held,
                boost=(pygame.K_LSHIFT in self._keys_held or pygame.K_RSHIFT in self._keys_held),
                slow=(pygame.K_LCTRL in self._keys_held or pygame.K_RCTRL in self._keys_held),
            )
            twist = keyboard_state_to_twist(state, self._velocity_cfg)

            if self.publish_only_when_active:
                active = twist.linear.x != 0 or twist.linear.y != 0 or twist.angular.z != 0
                if active or self._was_active:
                    self.cmd_vel.publish(twist)
                self._was_active = active
            else:
                self.cmd_vel.publish(twist)

            self._update_display(twist)

            if self._clock is None:
                raise RuntimeError("_clock not initialized")
            self._clock.tick(_CONTROL_RATE_HZ)

        pygame.quit()

    def _update_display(self, twist: Twist) -> None:
        if self._screen is None or self._font is None or self._keys_held is None:
            raise RuntimeError("Not initialized correctly")

        self._screen.fill(_BACKGROUND_COLOR)

        y_pos = 20

        speed_mult_text = ""
        if pygame.K_LSHIFT in self._keys_held or pygame.K_RSHIFT in self._keys_held:
            speed_mult_text = f" [BOOST {self._velocity_cfg.boost_multiplier:g}x]"
        elif pygame.K_LCTRL in self._keys_held or pygame.K_RCTRL in self._keys_held:
            speed_mult_text = f" [SLOW {self._velocity_cfg.slow_multiplier:g}x]"

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
