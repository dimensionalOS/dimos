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

"""Keyboard-based EEF twist teleop module for arm teleoperation.

Wraps a pygame UI as a DimOS Module so it can be composed with coordinator
blueprints via autoconnect.

Keyboard controls:
    W/S: +X/-X (forward/backward)
    A/D: +Y/-Y (left/right)
    Q/E: +Z/-Z (up/down)
    R/F: +Roll/-Roll
    T/G: +Pitch/-Pitch
    Y/H: +Yaw/-Yaw
    [: Open gripper
    ]: Close gripper
    ESC: Quit
"""

from __future__ import annotations

import os
import threading
from typing import TYPE_CHECKING, Any

try:
    import pygame
except ImportError:
    pygame = None  # type: ignore[assignment]

if TYPE_CHECKING:
    from pygame.key import _ScancodeWrapper

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Force X11 driver to avoid OpenGL threading issues
os.environ["SDL_VIDEODRIVER"] = "x11"

# Default jog speeds
DEFAULT_LINEAR_SPEED = 0.05  # m/s
DEFAULT_ANGULAR_SPEED = 0.5  # rad/s
# Normalized gripper command values.
GRIPPER_OPEN_POSITION = 1.0
GRIPPER_CLOSED_POSITION = 0.0
# TODO: Improve gripper handling.
GRIPPER_JOINT_NAME = "arm/gripper"

TwistVector = tuple[float, float, float]


class KeyboardTeleopConfig(ModuleConfig):
    task_name: str = EEF_TWIST_TASK_NAME
    linear_speed: float = DEFAULT_LINEAR_SPEED
    angular_speed: float = DEFAULT_ANGULAR_SPEED
    gripper_open_position: float = GRIPPER_OPEN_POSITION


def _motion_key_codes() -> frozenset[int]:
    if pygame is None:
        return frozenset()
    return frozenset(
        (
            pygame.K_w,
            pygame.K_s,
            pygame.K_a,
            pygame.K_d,
            pygame.K_q,
            pygame.K_e,
            pygame.K_r,
            pygame.K_f,
            pygame.K_t,
            pygame.K_g,
            pygame.K_y,
            pygame.K_h,
        )
    )


def _gripper_key_codes() -> tuple[int, int]:
    """Return pygame's bracket key codes without relying on stub attributes."""
    if pygame is None:
        return (-1, -1)
    return pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET


class KeyboardTeleopModule(Module):
    """Pygame-based spatial EEF twist keyboard teleop as a DimOS Module.

    Publishes routed TwistStamped commands for EEFTwistTask.
    """

    config: KeyboardTeleopConfig

    coordinator_ee_twist_command: Out[TwistStamped]
    joint_command: Out[JointState]

    _stop_event: threading.Event
    _thread: threading.Thread | None = None
    _gripper_position: float | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._gripper_position = None

    @rpc
    def start(self) -> None:
        if pygame is None:
            raise ImportError(
                "pygame is required for keyboard teleop. Install it with: pip install pygame"
            )
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._pygame_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _pygame_loop(self) -> None:
        task_name = self.config.task_name

        pygame.init()
        screen = pygame.display.set_mode((600, 400), pygame.SWSURFACE)
        pygame.display.set_caption(f"Keyboard Teleop — {task_name}")
        font = pygame.font.Font(None, 28)
        clock = pygame.time.Clock()
        held_motion_keys: set[int] = set()
        was_moving = False

        while not self._stop_event.is_set():
            for event in pygame.event.get():
                if self._handle_pygame_event(event, held_motion_keys, task_name):
                    self._stop_event.set()

            linear, angular = _twist_from_keys(
                held_motion_keys,
                linear_speed=self.config.linear_speed,
                angular_speed=self.config.angular_speed,
            )
            linear_x, linear_y, linear_z = linear
            angular_x, angular_y, angular_z = angular

            is_moving = any(value != 0.0 for value in (*linear, *angular))
            if is_moving or was_moving:
                self._publish_twist(
                    task_name,
                    linear=linear,
                    angular=angular,
                )
                was_moving = is_moving

            # Draw UI
            screen.fill((30, 30, 30))
            y_pos = 20

            title = font.render(f"Keyboard Teleop — {task_name}", True, (255, 255, 255))
            screen.blit(title, (20, y_pos))
            y_pos += 40

            twist_text = f"Linear twist: X={linear_x:.3f}  Y={linear_y:.3f}  Z={linear_z:.3f} m/s"
            screen.blit(font.render(twist_text, True, (100, 255, 100)), (20, y_pos))
            y_pos += 30

            angular_text = (
                f"Angular twist: R={angular_x:.3f}  P={angular_y:.3f}  Y={angular_z:.3f} rad/s"
            )
            screen.blit(font.render(angular_text, True, (100, 200, 255)), (20, y_pos))
            y_pos += 40

            controls = [
                ("W/S", "+X/-X (forward/back)"),
                ("A/D", "+Y/-Y (left/right)"),
                ("Q/E", "+Z/-Z (up/down)"),
                ("R/F", "+Roll/-Roll"),
                ("T/G", "+Pitch/-Pitch"),
                ("Y/H", "+Yaw/-Yaw"),
                ("[/]", "Open/close gripper"),
                ("ESC", "Quit"),
            ]
            for key, desc in controls:
                screen.blit(font.render(f"{key}: {desc}", True, (180, 180, 180)), (20, y_pos))
                y_pos += 25

            pygame.display.flip()
            clock.tick(50)

        self._publish_twist(task_name)
        pygame.quit()

    def _handle_pygame_event(
        self,
        event: Any,
        held_motion_keys: set[int],
        task_name: str,
    ) -> bool:
        """Apply one pygame event and synchronously stop motion on KEYUP."""
        if pygame is None:
            return False
        if event.type == pygame.QUIT:
            return True
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                return True
            left_bracket, right_bracket = _gripper_key_codes()
            if event.key in _motion_key_codes():
                held_motion_keys.add(event.key)
            elif event.key == left_bracket:
                self._set_gripper_position(self.config.gripper_open_position)
            elif event.key == right_bracket:
                self._set_gripper_position(GRIPPER_CLOSED_POSITION)
        elif event.type == pygame.KEYUP and event.key in _motion_key_codes():
            held_motion_keys.discard(event.key)
            linear, angular = _twist_from_keys(
                held_motion_keys,
                linear_speed=self.config.linear_speed,
                angular_speed=self.config.angular_speed,
            )
            self._publish_twist(task_name, linear=linear, angular=angular)
        return False

    def _publish_twist(
        self,
        task_name: str,
        *,
        linear: TwistVector = (0.0, 0.0, 0.0),
        angular: TwistVector = (0.0, 0.0, 0.0),
    ) -> None:
        self.coordinator_ee_twist_command.publish(
            TwistStamped(frame_id=task_name, linear=list(linear), angular=list(angular))
        )

    def _set_gripper_position(self, position: float) -> None:
        """Latch and publish a changed gripper endpoint command."""
        if self._gripper_position == position:
            return
        self._gripper_position = position
        self.joint_command.publish(JointState(name=[GRIPPER_JOINT_NAME], position=[position]))


def _twist_from_keys(
    keys: _ScancodeWrapper | set[int],
    *,
    linear_speed: float,
    angular_speed: float,
) -> tuple[TwistVector, TwistVector]:
    linear = [0.0, 0.0, 0.0]
    angular = [0.0, 0.0, 0.0]
    bindings = {
        pygame.K_w: (linear, 0, linear_speed),
        pygame.K_s: (linear, 0, -linear_speed),
        pygame.K_a: (linear, 1, linear_speed),
        pygame.K_d: (linear, 1, -linear_speed),
        pygame.K_q: (linear, 2, linear_speed),
        pygame.K_e: (linear, 2, -linear_speed),
        pygame.K_r: (angular, 0, angular_speed),
        pygame.K_f: (angular, 0, -angular_speed),
        pygame.K_t: (angular, 1, angular_speed),
        pygame.K_g: (angular, 1, -angular_speed),
        pygame.K_y: (angular, 2, angular_speed),
        pygame.K_h: (angular, 2, -angular_speed),
    }
    for key, (vector, axis, delta) in bindings.items():
        if key in keys if isinstance(keys, set) else keys[key]:
            vector[axis] += delta

    return (linear[0], linear[1], linear[2]), (angular[0], angular[1], angular[2])
