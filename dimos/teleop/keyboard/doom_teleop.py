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

from __future__ import annotations

import math
import os
import threading

import pygame

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Twist, Vector3
from dimos.msgs.std_msgs.Bool import Bool

os.environ.setdefault("SDL_VIDEODRIVER", "x11")


class DoomTeleop(Module):
    """Keyboard + mouse teleoperation in a DOOM/FPS style."""

    cmd_vel: Out[Twist]
    goal_pose: Out[PoseStamped]
    cancel_goal: Out[Bool]
    odom: In[PoseStamped]

    _stop_event: threading.Event
    _thread: threading.Thread | None = None
    _screen: pygame.Surface | None = None
    _clock: pygame.time.Clock | None = None
    _font: pygame.font.Font | None = None

    _keys_held: set[int] | None = None
    _mouse_buttons_held: set[int] | None = None
    _has_focus: bool = True
    _current_pose: PoseStamped | None = None

    _base_linear_speed: float = 0.6
    _base_angular_speed: float = 1.2
    _mouse_yaw_sensitivity: float = 0.003
    _goal_step_forward: float = 0.5
    _goal_step_degrees: float = 20.0

    def __init__(self) -> None:
        super().__init__()
        self._stop_event = threading.Event()

    @rpc
    def start(self) -> bool:
        super().start()

        self._stop_event.clear()
        self._keys_held = set()
        self._mouse_buttons_held = set()

        if self.odom:
            self.odom.subscribe(self._on_odom)

        self._thread = threading.Thread(target=self._pygame_loop, daemon=True)
        self._thread.start()
        return True

    @rpc
    def stop(self) -> None:
        self._publish_stop_twist()

        if self.cancel_goal:
            self.cancel_goal.publish(Bool(data=True))

        self._stop_event.set()

        if self._thread is None:
            raise RuntimeError("Cannot stop: thread was never started")
        self._thread.join(2)

        super().stop()

    def _on_odom(self, pose: PoseStamped) -> None:
        self._current_pose = pose

    def _publish_stop_twist(self) -> None:
        stop_twist = Twist()
        stop_twist.linear = Vector3(0.0, 0.0, 0.0)
        stop_twist.angular = Vector3(0.0, 0.0, 0.0)
        self.cmd_vel.publish(stop_twist)

    def _clear_motion(self) -> None:
        if self._keys_held is not None:
            self._keys_held.clear()
        if self._mouse_buttons_held is not None:
            self._mouse_buttons_held.clear()
        self._publish_stop_twist()

    def _pygame_loop(self) -> None:
        if self._keys_held is None or self._mouse_buttons_held is None:
            raise RuntimeError("Internal state not initialized")

        pygame.init()
        self._screen = pygame.display.set_mode((640, 320), pygame.SWSURFACE)
        pygame.display.set_caption("Doom Teleop (WSAD + Mouse)")
        self._clock = pygame.time.Clock()
        self._font = pygame.font.Font(None, 24)

        pygame.mouse.set_visible(True)
        pygame.mouse.get_rel()

        while not self._stop_event.is_set():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._stop_event.set()
                elif event.type == pygame.KEYDOWN:
                    self._handle_keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self._handle_keyup(event.key)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self._mouse_buttons_held.add(event.button)
                    self._handle_mouse_button_down(event.button)
                elif event.type == pygame.MOUSEBUTTONUP:
                    self._mouse_buttons_held.discard(event.button)
                elif event.type == pygame.ACTIVEEVENT:
                    if getattr(event, "gain", 0) == 0:
                        self._has_focus = False
                        self._clear_motion()
                    else:
                        self._has_focus = True

            twist = Twist()
            twist.linear = Vector3(0.0, 0.0, 0.0)
            twist.angular = Vector3(0.0, 0.0, 0.0)

            if self._has_focus:
                if pygame.K_w in self._keys_held:
                    twist.linear.x += self._base_linear_speed
                if pygame.K_s in self._keys_held:
                    twist.linear.x -= self._base_linear_speed
                if pygame.K_a in self._keys_held:
                    twist.angular.z += self._base_angular_speed
                if pygame.K_d in self._keys_held:
                    twist.angular.z -= self._base_angular_speed

                dx, _dy = pygame.mouse.get_rel()
                twist.angular.z += float(-dx) * self._mouse_yaw_sensitivity

                if 1 in self._mouse_buttons_held and twist.linear.x == 0.0:
                    twist.linear.x = 0.3

            self.cmd_vel.publish(twist)
            self._update_display(twist)

            if self._clock is None:
                raise RuntimeError("_clock not initialized")
            self._clock.tick(50)

        pygame.quit()

    def _handle_keydown(self, key: int) -> None:
        if self._keys_held is None:
            raise RuntimeError("_keys_held not initialized")

        self._keys_held.add(key)

        if key == pygame.K_SPACE:
            self._clear_motion()
            if self.cancel_goal:
                self.cancel_goal.publish(Bool(data=True))
            print("EMERGENCY STOP!")
        elif key == pygame.K_ESCAPE:
            self._stop_event.set()

    def _handle_keyup(self, key: int) -> None:
        if self._keys_held is None:
            raise RuntimeError("_keys_held not initialized")
        self._keys_held.discard(key)

    def _handle_mouse_button_down(self, button: int) -> None:
        if self._current_pose is None:
            return

        if button == 3 and self.goal_pose:
            goal = self._relative_goal(
                self._current_pose,
                forward=self._goal_step_forward,
                yaw_degrees=0.0,
            )
            self.goal_pose.publish(goal)
            print("Published forward step goal from right click.")
        elif button == 2 and self.goal_pose:
            goal = self._relative_goal(
                self._current_pose,
                forward=0.0,
                yaw_degrees=self._goal_step_degrees,
            )
            self.goal_pose.publish(goal)
            print("Published rotate-in-place goal from middle click.")

    @staticmethod
    def _relative_goal(
        current_pose: PoseStamped,
        forward: float,
        yaw_degrees: float,
    ) -> PoseStamped:
        local_offset = Vector3(forward, 0.0, 0.0)
        global_offset = current_pose.orientation.rotate_vector(local_offset)
        goal_position = current_pose.position + global_offset

        current_euler = current_pose.orientation.to_euler()
        goal_yaw = current_euler.yaw + math.radians(yaw_degrees)
        goal_euler = Vector3(current_euler.roll, current_euler.pitch, goal_yaw)
        goal_orientation = Quaternion.from_euler(goal_euler)

        return PoseStamped(
            position=goal_position,
            orientation=goal_orientation,
            frame_id=current_pose.frame_id,
        )

    def _update_display(self, twist: Twist) -> None:
        if self._screen is None or self._font is None or self._keys_held is None:
            raise RuntimeError("Display not initialized correctly")

        self._screen.fill((20, 20, 20))

        y_pos = 20
        focus_text = "FOCUSED" if self._has_focus else "OUT OF FOCUS (stopped)"
        lines = [
            f"Doom Teleop - {focus_text}",
            "",
            f"Linear X: {twist.linear.x:+.2f} m/s",
            f"Angular Z: {twist.angular.z:+.2f} rad/s",
            "",
            "Keyboard: W/S = forward/back, A/D = turn",
            "Mouse: move = look/turn, LMB = slow forward drive",
            "Mouse: RMB = step goal, MMB = rotate goal",
            "Space: E-stop (also cancels goal), ESC: quit",
        ]

        for text in lines:
            color = (0, 255, 255) if text.startswith("Doom Teleop") else (230, 230, 230)
            surf = self._font.render(text, True, color)
            self._screen.blit(surf, (20, y_pos))
            y_pos += 26

        moving = (
            abs(twist.linear.x) > 1e-3 or abs(twist.linear.y) > 1e-3 or abs(twist.angular.z) > 1e-3
        )
        color = (255, 0, 0) if moving else (0, 200, 0)
        pygame.draw.circle(self._screen, color, (600, 30), 12)

        pygame.display.flip()


doom_teleop = DoomTeleop.blueprint

__all__ = ["DoomTeleop", "doom_teleop"]
