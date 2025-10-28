#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""Pygame Joystick Module for testing G1 humanoid control."""

import os
import threading

# Force X11 driver to avoid OpenGL threading issues
os.environ["SDL_VIDEODRIVER"] = "x11"

from dimos.core import Module, Out, rpc
from dimos.msgs.geometry_msgs import Twist, Vector3


class G1JoystickModule(Module):
    """Pygame-based joystick control module for G1 humanoid testing.

    Outputs standard Twist messages on /cmd_vel for velocity control.
    Simplified version without mode switching since G1 handles that differently.
    """

    twist_out: Out[Twist] = None  # Standard velocity commands

    def __init__(self, *args, **kwargs) -> None:
        Module.__init__(self, *args, **kwargs)
        self.pygame_ready = False
        self.running = False

    @rpc
    def start(self) -> bool:
        """Initialize pygame and start control loop."""
        super().start()

        try:
            import pygame
        except ImportError:
            print("ERROR: pygame not installed. Install with: pip install pygame")
            return False

        self.keys_held = set()
        self.pygame_ready = True
        self.running = True

        # Start pygame loop in background thread
        self._thread = threading.Thread(target=self._pygame_loop, daemon=True)
        self._thread.start()

        return True

    @rpc
    def stop(self) -> None:
        super().stop()

        self.running = False
        self.pygame_ready = False

        stop_twist = Twist()
        stop_twist.linear = Vector3(0, 0, 0)
        stop_twist.angular = Vector3(0, 0, 0)

        self._thread.join(2)

        self.twist_out.publish(stop_twist)

    def _pygame_loop(self) -> None:
        """Main pygame event loop - ALL pygame operations happen here."""
        import pygame

        pygame.init()
        self.screen = pygame.display.set_mode((500, 400), pygame.SWSURFACE)
        pygame.display.set_caption("G1 Humanoid Joystick Control")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)

        print("G1 JoystickModule started - Focus pygame window to control")
        print("Controls:")
        print("  WS = Forward/Back")
        print("  AD = Turn Left/Right")
        print("  Space = Emergency Stop")
        print("  ESC = Quit")

        while self.running and self.pygame_ready:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    self.keys_held.add(event.key)

                    if event.key == pygame.K_SPACE:
                        # Emergency stop - clear all keys and send zero twist
                        self.keys_held.clear()
                        stop_twist = Twist()
                        stop_twist.linear = Vector3(0, 0, 0)
                        stop_twist.angular = Vector3(0, 0, 0)
                        self.twist_out.publish(stop_twist)
                        print("EMERGENCY STOP!")
                    elif event.key == pygame.K_ESCAPE:
                        # ESC quits
                        self.running = False

                elif event.type == pygame.KEYUP:
                    self.keys_held.discard(event.key)

            # Generate Twist message from held keys
            twist = Twist()
            twist.linear = Vector3(0, 0, 0)
            twist.angular = Vector3(0, 0, 0)

            # Forward/backward (W/S)
            if pygame.K_w in self.keys_held:
                twist.linear.x = 0.5
            if pygame.K_s in self.keys_held:
                twist.linear.x = -0.5

            # Turning (A/D)
            if pygame.K_a in self.keys_held:
                twist.angular.z = 0.5
            if pygame.K_d in self.keys_held:
                twist.angular.z = -0.5

            # Always publish twist at 50Hz
            self.twist_out.publish(twist)

            self._update_display(twist)

            # Maintain 50Hz rate
            self.clock.tick(50)

        pygame.quit()
        print("G1 JoystickModule stopped")

    def _update_display(self, twist) -> None:
        """Update pygame window with current status."""
        import pygame

        self.screen.fill((30, 30, 30))

        y_pos = 20

        texts = [
            "G1 Humanoid Control",
            "",
            f"Linear X (Forward/Back): {twist.linear.x:+.2f} m/s",
            f"Angular Z (Turn L/R): {twist.angular.z:+.2f} rad/s",
            "",
            "Keys: " + ", ".join([pygame.key.name(k).upper() for k in self.keys_held if k < 256]),
        ]

        for text in texts:
            if text:
                color = (0, 255, 255) if text == "G1 Humanoid Control" else (255, 255, 255)
                surf = self.font.render(text, True, color)
                self.screen.blit(surf, (20, y_pos))
            y_pos += 30

        if twist.linear.x != 0 or twist.linear.y != 0 or twist.angular.z != 0:
            pygame.draw.circle(self.screen, (255, 0, 0), (450, 30), 15)  # Red = moving
        else:
            pygame.draw.circle(self.screen, (0, 255, 0), (450, 30), 15)  # Green = stopped

        y_pos = 300
        help_texts = ["WS: Move | AD: Turn", "Space: E-Stop | ESC: Quit"]
        for text in help_texts:
            surf = self.font.render(text, True, (150, 150, 150))
            self.screen.blit(surf, (20, y_pos))
            y_pos += 25

        pygame.display.flip()
