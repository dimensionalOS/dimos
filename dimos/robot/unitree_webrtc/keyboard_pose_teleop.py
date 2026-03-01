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

from __future__ import annotations

import math
import os
import threading
from typing import Optional

import pygame

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.msgs.std_msgs.Bool import Bool

# Force X11 driver to avoid OpenGL threading issues (same as keyboard_teleop)
os.environ.setdefault("SDL_VIDEODRIVER", "x11")


STEP_FORWARD = 0.5  # meters per key press
STEP_LEFT = 0.5  # meters per key press
STEP_DEGREES = 15.0  # yaw change per key press


class KeyboardPoseTeleop(Module):
    """Pygame-based keyboard control module for pose goals.

    This module maps discrete key presses to relative pose offsets and publishes
    PoseStamped goals on the existing navigation goal topics.

    It is intended to complement the velocity-based KeyboardTeleop, by providing
    a way to send small relative navigation goals using the same navigation stack.
    """

    # Navigation goal interface (wired via existing transports, e.g. /goal_pose, /cancel_goal)
    goal_pose: Out[PoseStamped]
    cancel_goal: Out[Bool]

    # Current robot pose in a global frame, e.g. /odom
    odom: In[PoseStamped]

    _stop_event: threading.Event
    _thread: threading.Thread | None = None
    _screen: pygame.Surface | None = None
    _clock: pygame.time.Clock | None = None
    _font: pygame.font.Font | None = None
    _current_pose: Optional[PoseStamped] = None

    def __init__(self) -> None:
        super().__init__()
        self._stop_event = threading.Event()

    @rpc
    def start(self) -> bool:
        super().start()

        self._stop_event.clear()

        # Subscribe to odometry pose so we can generate relative goals
        if self.odom:
            self.odom.subscribe(self._on_odom)

        self._thread = threading.Thread(target=self._pygame_loop, daemon=True)
        self._thread.start()

        return True

    @rpc
    def stop(self) -> None:
        # Optionally cancel any active goal
        if self.cancel_goal:
            cancel_msg = Bool(data=True)
            self.cancel_goal.publish(cancel_msg)

        self._stop_event.set()

        if self._thread is None:
            raise RuntimeError("Cannot stop: thread was never started")
        self._thread.join(2)

        super().stop()

    def _on_odom(self, pose: PoseStamped) -> None:
        """Callback to update the current pose estimate."""
        self._current_pose = pose

    def _pygame_loop(self) -> None:
        pygame.init()
        self._screen = pygame.display.set_mode((520, 260), pygame.SWSURFACE)
        pygame.display.set_caption("Keyboard Pose Teleop")
        self._clock = pygame.time.Clock()
        self._font = pygame.font.Font(None, 24)

        while not self._stop_event.is_set():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._stop_event.set()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        # ESC quits
                        self._stop_event.set()
                    elif event.key == pygame.K_SPACE:
                        # Cancel current navigation goal
                        if self.cancel_goal:
                            cancel_msg = Bool(data=True)
                            self.cancel_goal.publish(cancel_msg)
                            print("CANCEL GOAL")
                    else:
                        self._handle_motion_key(event.key)

            self._update_display()

            if self._clock is None:
                raise RuntimeError("_clock not initialized")
            self._clock.tick(20)

        pygame.quit()

    def _handle_motion_key(self, key: int) -> None:
        """Map a single key press to a relative move and publish a goal."""
        if self._current_pose is None:
            # No pose estimate yet; cannot generate a sensible goal
            print("No odom pose received yet; ignoring key press.")
            return

        forward, left, degrees = 0.0, 0.0, 0.0

        # Arrow keys for relative translation in the robot's local frame
        if key == pygame.K_UP:
            forward = STEP_FORWARD
        elif key == pygame.K_DOWN:
            forward = -STEP_FORWARD
        elif key == pygame.K_LEFT:
            left = STEP_LEFT
        elif key == pygame.K_RIGHT:
            left = -STEP_LEFT

        # Q/E for yaw rotation in place
        elif key == pygame.K_q:
            degrees = STEP_DEGREES
        elif key == pygame.K_e:
            degrees = -STEP_DEGREES

        else:
            # Unmapped key
            return

        goal = self._generate_new_goal(self._current_pose, forward, left, degrees)

        if self.goal_pose:
            self.goal_pose.publish(goal)
            print(
                f"Published goal: forward={forward:.2f} m, left={left:.2f} m, "
                f"yaw_delta={degrees:.1f} deg"
            )

    @staticmethod
    def _generate_new_goal(
        current_pose: PoseStamped,
        forward: float,
        left: float,
        degrees: float,
    ) -> PoseStamped:
        """Generate a new PoseStamped goal using relative offsets.

        Logic mirrors the UnitreeSkillContainer.relative_move helper so that we
        reuse the same navigation interface and semantics:
        - (forward, left) are offsets in the robot's local frame.
        - degrees is the desired change in yaw at the goal.
        """
        local_offset = Vector3(forward, left, 0.0)
        # Rotate the local offset into the global frame using current orientation
        global_offset = current_pose.orientation.rotate_vector(local_offset)
        goal_position = current_pose.position + global_offset

        current_euler = current_pose.orientation.to_euler()
        goal_yaw = current_euler.yaw + math.radians(degrees)
        goal_euler = Vector3(current_euler.roll, current_euler.pitch, goal_yaw)
        goal_orientation = Quaternion.from_euler(goal_euler)

        return PoseStamped(
            position=goal_position,
            orientation=goal_orientation,
            frame_id=current_pose.frame_id,
        )

    def _update_display(self) -> None:
        if self._screen is None or self._font is None:
            raise RuntimeError("Not initialized correctly")

        self._screen.fill((30, 30, 30))

        y_pos = 20

        lines = [
            "Keyboard Pose Teleop",
            "",
            "Arrow keys: relative move (F/B/L/R)",
            "Q/E: rotate left/right",
            "Space: cancel goal",
            "ESC: quit",
            "",
        ]

        if self._current_pose is not None:
            pos = self._current_pose.position
            yaw = self._current_pose.orientation.to_euler().yaw
            lines.extend(
                [
                    f"Pose x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}",
                    f"Yaw={math.degrees(yaw):.1f} deg",
                ]
            )
        else:
            lines.append("Waiting for odom pose...")

        for text in lines:
            color = (0, 255, 255) if text.startswith("Keyboard Pose Teleop") else (255, 255, 255)
            surf = self._font.render(text, True, color)
            self._screen.blit(surf, (20, y_pos))
            y_pos += 28

        pygame.display.flip()


keyboard_pose_teleop = KeyboardPoseTeleop.blueprint

__all__ = ["KeyboardPoseTeleop", "keyboard_pose_teleop"]

