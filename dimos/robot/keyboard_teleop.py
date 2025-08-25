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

import threading
from typing import Optional, Set

from pynput import keyboard

from dimos.core import Module, Out, rpc
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


class KeyboardTeleop(Module):
    """
    Keyboard teleoperation module for robot control.

    Uses pynput to capture keyboard input and publishes Twist messages.

    Controls:
        - W/S: Forward/backward (linear.x)
        - A/D: Turn left/right (angular.z)
        - Left/Right arrows: Strafe left/right (linear.y)
        - Up/Down arrows: Pitch up/down (angular.y)
        - Space: Stop all movement
        - Shift: Speed boost (2x)
        - Ctrl: Slow mode (0.5x)

    Publishes:
        - movecmd: Twist messages for robot movement
    """

    # LCM outputs
    movecmd: Out[Twist] = None

    def __init__(
        self,
        linear_speed: float = 0.5,
        angular_speed: float = 0.8,
        publish_rate: float = 10.0,
        **kwargs,
    ):
        """
        Initialize Keyboard Teleop Module.

        Args:
            linear_speed: Base linear velocity (m/s)
            angular_speed: Base angular velocity (rad/s)
            publish_rate: Rate to publish commands (Hz)
        """
        super().__init__(**kwargs)

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.publish_rate = publish_rate

        # State tracking
        self._running = False
        self._keys_pressed: Set[keyboard.Key] = set()
        self._special_keys_pressed: Set[str] = set()

        # Current velocities
        self._linear_x = 0.0
        self._linear_y = 0.0
        self._angular_y = 0.0
        self._angular_z = 0.0

        # Threading
        self._listener: Optional[keyboard.Listener] = None
        self._publish_thread: Optional[threading.Thread] = None
        self._stop_publishing = threading.Event()

        logger.info(
            f"KeyboardTeleop initialized (linear_speed={linear_speed}, "
            f"angular_speed={angular_speed}, rate={publish_rate}Hz)"
        )

    @rpc
    def start(self):
        """Start the keyboard teleoperation module."""
        if self._running:
            logger.warning("Keyboard teleop already running")
            return

        self._running = True
        self._stop_publishing.clear()

        # Start keyboard listener
        self._listener = keyboard.Listener(
            on_press=self._on_key_press, on_release=self._on_key_release
        )
        self._listener.start()

        # Start publishing thread
        self._publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._publish_thread.start()

        logger.info("Keyboard teleop started")
        logger.info(
            "Controls: WASD (move/turn), Arrows (strafe/pitch), Space (stop), Shift (boost), Ctrl (slow)"
        )

    @rpc
    def stop(self):
        """Stop the keyboard teleoperation module."""
        if not self._running:
            return

        self._running = False
        self._stop_publishing.set()

        # Stop keyboard listener
        if self._listener:
            self._listener.stop()
            self._listener = None

        # Wait for publishing thread
        if self._publish_thread and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=2.0)

        # Send stop command
        self._send_stop()

        logger.info("Keyboard teleop stopped")

    def _on_key_press(self, key):
        """Handle key press events."""
        try:
            if hasattr(key, "char"):
                # Regular character key
                if key.char:
                    self._keys_pressed.add(key.char.lower())
            else:
                # Special key
                self._special_keys_pressed.add(str(key))

            self._update_velocities()

        except Exception as e:
            logger.error(f"Error handling key press: {e}")

    def _on_key_release(self, key):
        """Handle key release events."""
        try:
            if hasattr(key, "char"):
                # Regular character key
                if key.char:
                    self._keys_pressed.discard(key.char.lower())
            else:
                # Special key
                self._special_keys_pressed.discard(str(key))

            self._update_velocities()

        except Exception as e:
            logger.error(f"Error handling key release: {e}")

    def _update_velocities(self):
        """Update velocities based on pressed keys."""
        # Store previous values to check if we need to send stop
        prev_linear_x = self._linear_x
        prev_linear_y = self._linear_y
        prev_angular_y = self._angular_y
        prev_angular_z = self._angular_z

        # Get speed multiplier
        speed_multiplier = 1.0
        if (
            "Key.shift" in self._special_keys_pressed
            or "Key.shift_l" in self._special_keys_pressed
            or "Key.shift_r" in self._special_keys_pressed
        ):
            speed_multiplier = 2.0  # Boost mode
        elif (
            "Key.ctrl" in self._special_keys_pressed
            or "Key.ctrl_l" in self._special_keys_pressed
            or "Key.ctrl_r" in self._special_keys_pressed
        ):
            speed_multiplier = 0.5  # Slow mode

        # Reset velocities
        self._linear_x = 0.0
        self._linear_y = 0.0
        self._angular_y = 0.0
        self._angular_z = 0.0

        # Check for stop command (space)
        if "Key.space" in self._special_keys_pressed:
            # If we were moving before, send explicit stop
            if any([prev_linear_x, prev_linear_y, prev_angular_y, prev_angular_z]):
                self._send_stop()
            return  # Keep all velocities at 0

        # Linear X (forward/backward) - W/S
        if "w" in self._keys_pressed:
            self._linear_x = self.linear_speed * speed_multiplier
        elif "s" in self._keys_pressed:
            self._linear_x = -self.linear_speed * speed_multiplier

        # Angular Z (yaw/turn) - A/D
        if "a" in self._keys_pressed:
            self._angular_z = self.angular_speed * speed_multiplier
        elif "d" in self._keys_pressed:
            self._angular_z = -self.angular_speed * speed_multiplier

        # Linear Y (strafe) - Left/Right arrows
        if "Key.left" in self._special_keys_pressed:
            self._linear_y = self.linear_speed * speed_multiplier
        elif "Key.right" in self._special_keys_pressed:
            self._linear_y = -self.linear_speed * speed_multiplier

        # Angular Y (pitch) - Up/Down arrows
        if "Key.up" in self._special_keys_pressed:
            self._angular_y = self.angular_speed * speed_multiplier
        elif "Key.down" in self._special_keys_pressed:
            self._angular_y = -self.angular_speed * speed_multiplier

        # If we just stopped moving (all keys released), send explicit stop command
        if not any([self._linear_x, self._linear_y, self._angular_y, self._angular_z]) and any(
            [prev_linear_x, prev_linear_y, prev_angular_y, prev_angular_z]
        ):
            self._send_stop()

    def _publish_loop(self):
        """Main publishing loop that sends Twist messages at fixed rate."""
        import time

        period = 1.0 / self.publish_rate

        logger.info(f"Starting publish loop at {self.publish_rate}Hz")

        while not self._stop_publishing.is_set():
            try:
                # Only publish if there's actual movement
                if any([self._linear_x, self._linear_y, self._angular_y, self._angular_z]):
                    # Create and publish Twist message
                    twist = Twist(
                        linear=Vector3(self._linear_x, self._linear_y, 0.0),
                        angular=Vector3(0.0, self._angular_y, self._angular_z),
                    )

                    self.movecmd.publish(twist)

                    logger.debug(
                        f"Publishing: linear=({self._linear_x:.2f}, {self._linear_y:.2f}, 0), "
                        f"angular=(0, {self._angular_y:.2f}, {self._angular_z:.2f})"
                    )

            except Exception as e:
                logger.error(f"Error in publish loop: {e}")

            time.sleep(period)

        logger.info("Publish loop stopped")

    def _send_stop(self):
        """Send stop command (zero velocities)."""
        try:
            stop_twist = Twist()
            self.movecmd.publish(stop_twist)
            logger.debug("Sent stop command")
        except Exception as e:
            logger.error(f"Error sending stop command: {e}")

    def cleanup(self):
        """Clean up resources on module destruction."""
        self.stop()
