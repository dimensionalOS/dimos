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

"""Pygame window and explicit IPC entry point for keyboard teleoperation."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from multiprocessing.connection import Connection
import os
import sys
from typing import TYPE_CHECKING, Any

try:
    import pygame as pygame
except ImportError:
    pygame = None  # type: ignore[assignment]

if TYPE_CHECKING:
    from pygame.key import _ScancodeWrapper

TwistVector = tuple[float, float, float]


@dataclass(frozen=True)
class TwistCommand:
    linear: TwistVector = (0.0, 0.0, 0.0)
    angular: TwistVector = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class GripperCommand:
    opening: float


@dataclass(frozen=True)
class WindowReady:
    pass


@dataclass(frozen=True)
class WindowError:
    detail: str


@dataclass(frozen=True)
class Shutdown:
    pass


WindowMessage = TwistCommand | GripperCommand | WindowReady | WindowError


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


class KeyboardWindow:
    """Run the same input handling in a thread or on a child process's main thread."""

    def __init__(
        self,
        linear_speed: float,
        angular_speed: float,
        publish: Callable[[WindowMessage], None],
        stop_requested: Callable[[], bool],
    ) -> None:
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self._publish = publish
        self._stop_requested = stop_requested
        self._closed = False
        self._ready = False

    def run(self) -> None:
        if pygame is None:
            raise ImportError("pygame is required for keyboard teleop")
        if sys.platform.startswith("linux"):
            os.environ.setdefault("SDL_VIDEODRIVER", "x11")
        try:
            self._pygame_loop()
        finally:
            try:
                if self._ready:
                    self._publish_twist()
            finally:
                pygame.quit()

    def _pygame_loop(self) -> None:
        pygame.init()
        screen = pygame.display.set_mode((600, 400), pygame.SWSURFACE)
        pygame.display.set_caption("Keyboard Teleop")
        font = pygame.font.Font(None, 28)
        clock = pygame.time.Clock()
        self._ready = True
        self._publish(WindowReady())
        held_motion_keys: set[int] = set()
        was_moving = False

        while not (self._closed or self._stop_requested()):
            for event in pygame.event.get():
                if self._handle_pygame_event(event, held_motion_keys):
                    self._closed = True
                    break
            if self._closed:
                break

            linear, angular = _twist_from_keys(
                held_motion_keys,
                linear_speed=self.linear_speed,
                angular_speed=self.angular_speed,
            )
            linear_x, linear_y, linear_z = linear
            angular_x, angular_y, angular_z = angular

            is_moving = any(value != 0.0 for value in (*linear, *angular))
            if is_moving or was_moving:
                self._publish_twist(linear=linear, angular=angular)
                was_moving = is_moving

            # Draw UI
            screen.fill((30, 30, 30))
            y_pos = 20

            title = font.render("Keyboard Teleop", True, (255, 255, 255))
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

    def _handle_pygame_event(
        self,
        event: Any,
        held_motion_keys: set[int],
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
                self._publish_gripper_command(opening=1.0)
            elif event.key == right_bracket:
                self._publish_gripper_command(opening=0.0)
        elif event.type == pygame.KEYUP and event.key in _motion_key_codes():
            held_motion_keys.discard(event.key)
            linear, angular = _twist_from_keys(
                held_motion_keys,
                linear_speed=self.linear_speed,
                angular_speed=self.angular_speed,
            )
            self._publish_twist(linear=linear, angular=angular)
        return False

    def _publish_twist(
        self,
        *,
        linear: TwistVector = (0.0, 0.0, 0.0),
        angular: TwistVector = (0.0, 0.0, 0.0),
    ) -> None:
        self._publish(TwistCommand(linear, angular))

    def _publish_gripper_command(self, *, opening: float) -> None:
        self._publish(GripperCommand(opening))


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


def run_window(connection: Connection, linear_speed: float, angular_speed: float) -> None:
    """Own the GUI and close it on shutdown or parent EOF."""

    def stop_requested() -> bool:
        if not connection.poll():
            return False
        try:
            return isinstance(connection.recv(), Shutdown)
        except EOFError:
            return True

    try:
        KeyboardWindow(linear_speed, angular_speed, connection.send, stop_requested).run()
    except (EOFError, BrokenPipeError, ConnectionResetError):
        pass
    except Exception as exc:
        try:
            connection.send(WindowError(str(exc)))
        except (BrokenPipeError, ConnectionResetError):
            pass
        raise
    finally:
        connection.close()
