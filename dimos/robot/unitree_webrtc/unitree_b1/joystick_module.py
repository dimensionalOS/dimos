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

# Copyright 2025 Dimensional Inc.

"""Pygame Joystick Module for testing B1 control via LCM."""

import os
import threading

# Force X11 driver to avoid OpenGL threading issues
os.environ["SDL_VIDEODRIVER"] = "x11"

import time

from dimos.core import Module, Out, rpc
from dimos.msgs.geometry_msgs import Twist, TwistStamped, Vector3
from dimos.msgs.std_msgs import Int32


class JoystickModule(Module):
    """Pygame-based joystick control module for B1 testing.

    Outputs timestamped Twist messages on /cmd_vel and mode changes on /b1/mode.
    This allows testing the same interface that navigation will use.
    """

    twist_out: Out[TwistStamped]  # Timestamped velocity commands
    mode_out: Out[Int32]  # Mode changes

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        Module.__init__(self, *args, **kwargs)
        self.pygame_ready = False
        self.running = False
        self.current_mode = 0  # Start in IDLE mode for safety

    @rpc
    def start(self) -> bool:
        """Initialize pygame and start control loop."""

        super().start()

        try:
            import pygame
        except ImportError:
            print("ERROR: pygame not installed. Install with: pip install pygame")
            return False

        self.keys_held = set()  # type: ignore[var-annotated]
        self.pygame_ready = True
        self.running = True

        # Start pygame loop in background thread - ALL pygame ops will happen there
        self._thread = threading.Thread(target=self._pygame_loop, daemon=True)
        self._thread.start()

        return True

    @rpc
    def stop(self) -> None:
        """Stop the joystick module."""

        self.running = False
        self.pygame_ready = False

        # Send stop command
        stop_twist = Twist()
        stop_twist_stamped = TwistStamped(
            ts=time.time(),
            frame_id="base_link",
            linear=stop_twist.linear,
            angular=stop_twist.angular,
        )
        self.twist_out.publish(stop_twist_stamped)

        self._thread.join(2)

        super().stop()

    def _pygame_loop(self) -> None:
        """Main pygame event loop - ALL pygame operations happen here."""
        import pygame

        # Initialize pygame and create display IN THIS THREAD
        pygame.init()
        self.screen = pygame.display.set_mode((500, 400), pygame.SWSURFACE)
        pygame.display.set_caption("B1 Joystick Control (LCM)")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)

        print("JoystickModule started - Focus pygame window to control")
        print("Controls:")
        print("  Walk Mode: WASD = Move/Turn, JL = Strafe")
        print("  Stand Mode: WASD = Height/Yaw, JL = Roll, IK = Pitch")
        print("  1/2/0 = Stand/Walk/Idle modes")
        print("  Space/Q = Emergency Stop")
        print("  ESC = Quit (or use Ctrl+C)")

        while self.running and self.pygame_ready:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    self.keys_held.add(event.key)

                    # Mode changes - publish to mode_out for connection module
                    if event.key == pygame.K_0:
                        self.current_mode = 0
                        mode_msg = Int32()
                        mode_msg.data = 0
                        self.mode_out.publish(mode_msg)
                        print("Mode: IDLE")
                    elif event.key == pygame.K_1:
                        self.current_mode = 1
                        mode_msg = Int32()
                        mode_msg.data = 1
                        self.mode_out.publish(mode_msg)
                        print("Mode: STAND")
                    elif event.key == pygame.K_2:
                        self.current_mode = 2
                        mode_msg = Int32()
                        mode_msg.data = 2
                        self.mode_out.publish(mode_msg)
                        print("Mode: WALK")
                    elif event.key == pygame.K_SPACE or event.key == pygame.K_q:
                        self.keys_held.clear()
                        # Send IDLE mode for emergency stop
                        self.current_mode = 0
                        mode_msg = Int32()
                        mode_msg.data = 0
                        self.mode_out.publish(mode_msg)
                        # Also send zero twist
                        stop_twist = Twist()
                        stop_twist.linear = Vector3(0, 0, 0)
                        stop_twist.angular = Vector3(0, 0, 0)
                        stop_twist_stamped = TwistStamped(
                            ts=time.time(),
                            frame_id="base_link",
                            linear=stop_twist.linear,
                            angular=stop_twist.angular,
                        )
                        self.twist_out.publish(stop_twist_stamped)
                        print("EMERGENCY STOP!")
                    elif event.key == pygame.K_ESCAPE:
                        # ESC still quits for development convenience
                        self.running = False

                elif event.type == pygame.KEYUP:
                    self.keys_held.discard(event.key)

            # Generate Twist message from held keys
            twist = Twist()
            twist.linear = Vector3(0, 0, 0)
            twist.angular = Vector3(0, 0, 0)

            # Apply controls based on mode
            if self.current_mode == 2:  # WALK mode - movement control
                # Forward/backward (W/S)
                if pygame.K_w in self.keys_held:
                    twist.linear.x = 1.0  # Forward
                if pygame.K_s in self.keys_held:
                    twist.linear.x = -1.0  # Backward

                # Turning (A/D)
                if pygame.K_a in self.keys_held:
                    twist.angular.z = 1.0  # Turn left
                if pygame.K_d in self.keys_held:
                    twist.angular.z = -1.0  # Turn right

                # Strafing (J/L)
                if pygame.K_j in self.keys_held:
                    twist.linear.y = 1.0  # Strafe left
                if pygame.K_l in self.keys_held:
                    twist.linear.y = -1.0  # Strafe right

            elif self.current_mode == 1:  # STAND mode - body pose control
                # Height control (W/S) - use linear.z for body height
                if pygame.K_w in self.keys_held:
                    twist.linear.z = 1.0  # Raise body
                if pygame.K_s in self.keys_held:
                    twist.linear.z = -1.0  # Lower body

                # Yaw control (A/D) - use angular.z for body yaw
                if pygame.K_a in self.keys_held:
                    twist.angular.z = 1.0  # Rotate body left
                if pygame.K_d in self.keys_held:
                    twist.angular.z = -1.0  # Rotate body right

                # Roll control (J/L) - use angular.x for body roll
                if pygame.K_j in self.keys_held:
                    twist.angular.x = 1.0  # Roll left
                if pygame.K_l in self.keys_held:
                    twist.angular.x = -1.0  # Roll right

                # Pitch control (I/K) - use angular.y for body pitch
                if pygame.K_i in self.keys_held:
                    twist.angular.y = 1.0  # Pitch forward
                if pygame.K_k in self.keys_held:
                    twist.angular.y = -1.0  # Pitch backward

            twist_stamped = TwistStamped(
                ts=time.time(), frame_id="base_link", linear=twist.linear, angular=twist.angular
            )
            self.twist_out.publish(twist_stamped)

            # Update pygame display
            self._update_display(twist)

            # Maintain 50Hz rate
            self.clock.tick(50)

        pygame.quit()
        print("JoystickModule stopped")

    def _update_display(self, twist) -> None:  # type: ignore[no-untyped-def]
        """Update pygame window with current status."""
        import pygame

        self.screen.fill((30, 30, 30))

        # Mode display
        y_pos = 20
        mode_text = ["IDLE", "STAND", "WALK"][self.current_mode if self.current_mode < 3 else 0]
        mode_color = (
            (0, 255, 0)
            if self.current_mode == 2
            else (255, 255, 0)
            if self.current_mode == 1
            else (100, 100, 100)
        )

        texts = [
            f"Mode: {mode_text}",
            "",
            f"Linear X: {twist.linear.x:+.2f}",
            f"Linear Y: {twist.linear.y:+.2f}",
            f"Linear Z: {twist.linear.z:+.2f}",
            f"Angular X: {twist.angular.x:+.2f}",
            f"Angular Y: {twist.angular.y:+.2f}",
            f"Angular Z: {twist.angular.z:+.2f}",
            "Keys: " + ", ".join([pygame.key.name(k).upper() for k in self.keys_held if k < 256]),
        ]

        for i, text in enumerate(texts):
            if text:
                color = mode_color if i == 0 else (255, 255, 255)
                surf = self.font.render(text, True, color)
                self.screen.blit(surf, (20, y_pos))
            y_pos += 30

        if (
            twist.linear.x != 0
            or twist.linear.y != 0
            or twist.linear.z != 0
            or twist.angular.x != 0
            or twist.angular.y != 0
            or twist.angular.z != 0
        ):
            pygame.draw.circle(self.screen, (255, 0, 0), (450, 30), 15)  # Red = moving
        else:
            pygame.draw.circle(self.screen, (0, 255, 0), (450, 30), 15)  # Green = stopped

        y_pos = 300
        help_texts = ["WASD: Move | JL: Strafe | 1/2/0: Modes", "Space/Q: E-Stop | ESC: Quit"]
        for text in help_texts:
            surf = self.font.render(text, True, (150, 150, 150))
            self.screen.blit(surf, (20, y_pos))
            y_pos += 25

        pygame.display.flip()
