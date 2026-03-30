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

"""Velocity controller for M20 quadruped.

Accepts dimos Twist messages (m/s, rad/s), applies acceleration smoothing,
and sends velocity commands via one of two transports:

- **Navigation Mode** (/NAV_CMD via rclpy): Publishes absolute m/s values
  to the /NAV_CMD DDS topic. Requires rclpy and drdds on GOS. Enables
  the robot's built-in obstacle avoidance.
- **Regular Mode** (UDP): Normalizes to [-1,1] range and sends via UDP
  protocol. Teleop only — no obstacle avoidance.

Transport is selected by providing a `nav_cmd_publish` callback.
Reference: M20 Software Development Guide sections 1.2.5, 2.3.1
"""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable

from dimos.msgs.geometry_msgs.Twist import Twist
from ..protocol import M20Protocol

logger = logging.getLogger(__name__)

_ZERO_THRESHOLD = 0.001


@dataclass(frozen=True)
class M20SpeedLimits:
    """Physical speed limits for the M20 in standard walking gait.

    Used to convert between absolute velocities (m/s, rad/s)
    and the M20's normalized [-1,1] command range.
    """

    max_linear_x: float = 1.5  # m/s forward/backward
    max_linear_y: float = 0.5  # m/s lateral
    max_angular_yaw: float = 1.0  # rad/s rotation


@dataclass
class VelocityState:
    """Current velocity controller state."""

    # Target velocities (m/s and rad/s, from Twist)
    target_linear_x: float = 0.0
    target_linear_y: float = 0.0
    target_angular_yaw: float = 0.0

    # Smoothed velocities (m/s and rad/s)
    actual_linear_x: float = 0.0
    actual_linear_y: float = 0.0
    actual_angular_yaw: float = 0.0

    # Safety state
    emergency_stopped: bool = False
    paused: bool = False

    # Timestamps
    last_command_time: float = 0.0
    last_update_time: float = 0.0


class M20VelocityController:
    """Converts dimos Twist commands to M20 normalized velocity commands.

    Runs a 20Hz control loop that:
    1. Accepts Twist messages (m/s, rad/s) via set_twist()
    2. Applies acceleration smoothing
    3. Converts to M20 [-1,1] normalized range using speed_limits
    4. Sends via UDP protocol
    5. Ramps to zero on command timeout (safety)
    """

    def __init__(
        self,
        protocol: M20Protocol,
        speed_limits: M20SpeedLimits | None = None,
        linear_accel: float = 1.0,  # m/s^2
        angular_accel: float = 2.0,  # rad/s^2
        command_timeout: float = 0.5,  # seconds
        control_rate: int = 20,  # Hz (per M20 docs)
        nav_cmd_publish: Callable[[float, float, float], None] | None = None,
    ):
        if linear_accel <= 0 or angular_accel <= 0:
            raise ValueError("Acceleration values must be positive")
        if control_rate <= 0:
            raise ValueError("control_rate must be positive")
        if command_timeout <= 0:
            raise ValueError("command_timeout must be positive")

        self.protocol = protocol
        self.speed_limits = speed_limits or M20SpeedLimits()
        self._nav_cmd_publish = nav_cmd_publish

        if self.speed_limits.max_linear_x <= 0 or self.speed_limits.max_linear_y <= 0:
            raise ValueError("Speed limits must be positive")
        if self.speed_limits.max_angular_yaw <= 0:
            raise ValueError("Speed limits must be positive")

        self.state = VelocityState()
        self._lock = threading.Lock()

        self.linear_accel = linear_accel
        self.angular_accel = angular_accel
        self.command_timeout = command_timeout
        self.control_rate = control_rate

        self._running = False
        self._thread: threading.Thread | None = None

    def set_twist(self, twist: Twist) -> None:
        """Accept a dimos Twist as the velocity target.

        Twist.linear.x  = forward  (m/s)
        Twist.linear.y  = lateral  (m/s)
        Twist.angular.z = yaw      (rad/s)
        """
        with self._lock:
            if self.state.emergency_stopped:
                return

            lim = self.speed_limits
            self.state.target_linear_x = max(
                -lim.max_linear_x, min(lim.max_linear_x, twist.linear.x)
            )
            self.state.target_linear_y = max(
                -lim.max_linear_y, min(lim.max_linear_y, twist.linear.y)
            )
            self.state.target_angular_yaw = max(
                -lim.max_angular_yaw, min(lim.max_angular_yaw, twist.angular.z)
            )
            self.state.last_command_time = time.time()

    def emergency_stop(self, engage: bool) -> None:
        """Engage or release emergency stop."""
        with self._lock:
            if engage:
                self.state.emergency_stopped = True
                self.state.target_linear_x = 0.0
                self.state.target_linear_y = 0.0
                self.state.target_angular_yaw = 0.0
                self.state.actual_linear_x = 0.0
                self.state.actual_linear_y = 0.0
                self.state.actual_angular_yaw = 0.0
                logger.warning("EMERGENCY STOP ENGAGED")
            else:
                self.state.emergency_stopped = False
                logger.info("Emergency stop released")

        # Send zero velocity immediately on e-stop (outside lock to avoid
        # holding lock during I/O)
        if engage:
            self._send_zero()

    def start(self) -> None:
        """Start the 20Hz control loop thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
        )
        self._thread.start()
        logger.info("M20 velocity controller started")

    def stop(self) -> None:
        """Stop the control loop thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("M20 velocity controller stopped")

    def _control_loop(self) -> None:
        dt = 1.0 / self.control_rate

        while self._running:
            now = time.time()
            try:
                with self._lock:
                    # Ramp to zero if no recent commands
                    if now - self.state.last_command_time > self.command_timeout:
                        self.state.target_linear_x = 0.0
                        self.state.target_linear_y = 0.0
                        self.state.target_angular_yaw = 0.0

                    self._update_velocities(dt)

                self._publish_control()

                with self._lock:
                    self.state.last_update_time = now
            except Exception:
                logger.exception("Control loop error — sending zero velocity")
                try:
                    self._send_zero()
                except Exception:
                    pass

            elapsed = time.time() - now
            time.sleep(max(0.0, dt - elapsed))

    def _update_velocities(self, dt: float) -> None:
        """Apply acceleration limits for smooth velocity changes.

        Must be called with self._lock held.
        """
        max_lin = self.linear_accel * dt
        max_ang = self.angular_accel * dt

        def _smooth(target: float, actual: float, limit: float) -> float:
            delta = max(-limit, min(limit, target - actual))
            return actual + delta

        self.state.actual_linear_x = _smooth(
            self.state.target_linear_x, self.state.actual_linear_x, max_lin
        )
        self.state.actual_linear_y = _smooth(
            self.state.target_linear_y, self.state.actual_linear_y, max_lin
        )
        self.state.actual_angular_yaw = _smooth(
            self.state.target_angular_yaw, self.state.actual_angular_yaw, max_ang
        )

    def _publish_control(self) -> None:
        """Send velocity via /NAV_CMD (absolute m/s) or UDP (normalized [-1,1])."""
        with self._lock:
            if self.state.paused:
                return

            if self.state.emergency_stopped:
                self._send_zero()
                return

            if self._is_idle():
                return

            vx = self.state.actual_linear_x
            vy = self.state.actual_linear_y
            vyaw = self.state.actual_angular_yaw

        if self._nav_cmd_publish is not None:
            self._nav_cmd_publish(vx, vy, vyaw)
        else:
            lim = self.speed_limits
            self.protocol.send_velocity(
                x=vx / lim.max_linear_x,
                y=vy / lim.max_linear_y,
                yaw=vyaw / lim.max_angular_yaw,
            )

    def _send_zero(self) -> None:
        """Send zero velocity on all active transports."""
        if self._nav_cmd_publish is not None:
            self._nav_cmd_publish(0.0, 0.0, 0.0)
        self.protocol.send_velocity(0, 0, 0)

    def _is_idle(self) -> bool:
        """Must be called with self._lock held."""
        timed_out = time.time() - self.state.last_command_time > self.command_timeout
        near_zero = (
            abs(self.state.actual_linear_x) < _ZERO_THRESHOLD
            and abs(self.state.actual_linear_y) < _ZERO_THRESHOLD
            and abs(self.state.actual_angular_yaw) < _ZERO_THRESHOLD
        )
        return timed_out and near_zero

    @property
    def is_moving(self) -> bool:
        threshold = 0.01
        with self._lock:
            return (
                abs(self.state.actual_linear_x) > threshold
                or abs(self.state.actual_linear_y) > threshold
                or abs(self.state.actual_angular_yaw) > threshold
            )
