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

"""Velocity controller for the Deep Robotics M20 quadruped.

Sends velocity commands to /NAV_CMD via a TCP bridge on AOS (port 9740).
The bridge (`nav_cmd_rclpy_bridge.py`) publishes to /NAV_CMD via rclpy,
which is the only proven path that basic_server accepts.

Publishes at 10 Hz (per M20 SDK recommendation). Includes a safety
watchdog that zeros velocities when no command arrives within the timeout.

Prerequisites:
    - Robot must be in navigation mode with Agile gait (0x3002)
    - basic_server service must be running on AOS
    - nav_cmd_rclpy_bridge.py must be running on AOS (port 9740)
    - Built-in planner (handler) should be stopped to avoid conflicts
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import dataclass

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
NAV_CMD_BRIDGE_PORT = 9740  # TCP port on AOS for nav_cmd_rclpy_bridge
MAX_LINEAR_VEL = 1.0  # m/s
MAX_YAW_RATE = 1.5  # rad/s
SEND_HZ = 10  # M20 SDK recommends 10 Hz for /NAV_CMD
WATCHDOG_TIMEOUT_S = 0.3  # Zero velocities if no cmd for 300 ms


@dataclass
class NavCmd:
    """Local representation of a /NAV_CMD velocity command.

    Field names match the drdds/msg/NavCmdValue structure from the M20 SDK.
    """

    x_vel: float = 0.0  # forward/backward (m/s, +forward)
    y_vel: float = 0.0  # left/right      (m/s, +left)
    yaw_vel: float = 0.0  # yaw rate         (rad/s, +ccw)

    @classmethod
    def from_twist(cls, twist: Twist) -> NavCmd:
        """Convert a dimos Twist to a clamped NavCmd."""
        return cls(
            x_vel=max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, twist.linear.x)),
            y_vel=max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, twist.linear.y)),
            yaw_vel=max(-MAX_YAW_RATE, min(MAX_YAW_RATE, twist.angular.z)),
        )

    @classmethod
    def zero(cls) -> NavCmd:
        return cls()

    @property
    def is_zero(self) -> bool:
        return self.x_vel == 0.0 and self.y_vel == 0.0 and self.yaw_vel == 0.0


class M20VelocityController:
    """Publishes velocity commands to /NAV_CMD at 10 Hz via TCP bridge.

    Connects to nav_cmd_rclpy_bridge on AOS which publishes to /NAV_CMD
    via rclpy. Falls back to log-only mode if the bridge is unreachable.

    Parameters
    ----------
    aos_host : str
        AOS IP address for the TCP bridge connection.
    test_mode : bool
        If True, log commands instead of publishing.
    """

    def __init__(self, *, aos_host: str = "10.21.31.103", test_mode: bool = False) -> None:
        self._test_mode = test_mode
        self._aos_host = aos_host
        self._lock = threading.Lock()
        self._current_cmd = NavCmd.zero()
        self._last_cmd_time = time.monotonic()
        self._running = False
        self._send_thread: threading.Thread | None = None
        self._watchdog_thread: threading.Thread | None = None
        self._packet_count = 0
        self._timeout_active = False

        # TCP socket to AOS bridge — set up on start()
        self._socket: socket.socket | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        if self._running:
            return

        self._init_publisher()

        self._running = True
        self._last_cmd_time = time.monotonic()

        self._send_thread = threading.Thread(
            target=self._send_loop, daemon=True, name="M20VelSendLoop"
        )
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True, name="M20VelWatchdog"
        )
        self._send_thread.start()
        self._watchdog_thread.start()

        logger.info(
            "M20VelocityController started (bridge=%s, test=%s)",
            self._socket is not None,
            self._test_mode,
        )

    def stop(self) -> None:
        if not self._running:
            return

        # Send zero velocities before shutting down
        with self._lock:
            self._current_cmd = NavCmd.zero()
        self._publish_once(NavCmd.zero())

        self._running = False

        if self._send_thread:
            self._send_thread.join(timeout=0.5)
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=0.5)

        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
        self._socket = None
        logger.info("M20VelocityController stopped")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def send(self, twist: Twist) -> None:
        """Accept a new velocity command (called from _on_cmd_vel)."""
        cmd = NavCmd.from_twist(twist)
        with self._lock:
            self._current_cmd = cmd
            self._last_cmd_time = time.monotonic()
            self._timeout_active = False

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _init_publisher(self) -> None:
        """Connect to the TCP bridge on AOS for /NAV_CMD publishing."""
        if self._test_mode:
            logger.info("[TEST] M20VelocityController in test mode — no publishing")
            return

        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(5.0)
            self._socket.connect((self._aos_host, NAV_CMD_BRIDGE_PORT))
            self._socket.settimeout(None)
            logger.info(
                "Connected to nav_cmd_bridge at %s:%d for /NAV_CMD",
                self._aos_host, NAV_CMD_BRIDGE_PORT,
            )
        except Exception as exc:
            logger.warning(
                "Failed to connect to nav_cmd_bridge at %s:%d: %s — "
                "velocity commands will be logged only.",
                self._aos_host, NAV_CMD_BRIDGE_PORT, exc,
            )
            self._socket = None

    def _publish_once(self, cmd: NavCmd) -> None:
        """Send a velocity command to the AOS bridge."""
        if self._test_mode:
            if self._packet_count % SEND_HZ == 0:
                logger.info(
                    "[TEST] NavCmd x_vel=%.3f y_vel=%.3f yaw_vel=%.3f | count=%d",
                    cmd.x_vel,
                    cmd.y_vel,
                    cmd.yaw_vel,
                    self._packet_count,
                )
            return

        if self._socket is None:
            return

        try:
            self._socket.sendall(struct.pack("fff", cmd.x_vel, cmd.y_vel, cmd.yaw_vel))
        except Exception as exc:
            if self._packet_count % SEND_HZ == 0:
                logger.error("Failed to send /NAV_CMD via bridge: %s", exc)
            # Try to reconnect
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
            try:
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._socket.settimeout(5.0)
                self._socket.connect((self._aos_host, NAV_CMD_BRIDGE_PORT))
                self._socket.settimeout(None)
                logger.info("Reconnected to nav_cmd_bridge")
            except Exception:
                self._socket = None

    def _send_loop(self) -> None:
        """Continuously publish the current command at 10 Hz."""
        period = 1.0 / SEND_HZ
        while self._running:
            try:
                with self._lock:
                    cmd = self._current_cmd

                self._publish_once(cmd)
                self._packet_count += 1

                if self._packet_count % (SEND_HZ * 5) == 0:
                    logger.debug(
                        "M20 vel send: x_vel=%.3f y_vel=%.3f yaw_vel=%.3f count=%d",
                        cmd.x_vel,
                        cmd.y_vel,
                        cmd.yaw_vel,
                        self._packet_count,
                    )

                time.sleep(period)
            except Exception as exc:
                if self._running:
                    logger.error("M20 vel send error: %s", exc)

    def _watchdog_loop(self) -> None:
        """Zero velocities if no command arrives within the timeout."""
        while self._running:
            try:
                elapsed = time.monotonic() - self._last_cmd_time
                if elapsed > WATCHDOG_TIMEOUT_S:
                    if not self._timeout_active:
                        logger.warning(
                            "M20 velocity watchdog: no cmd for %.1fs — zeroing", elapsed
                        )
                        with self._lock:
                            self._current_cmd = NavCmd.zero()
                        self._timeout_active = True
                else:
                    if self._timeout_active:
                        logger.info("M20 velocity watchdog: commands resumed")
                        self._timeout_active = False

                time.sleep(0.05)
            except Exception as exc:
                if self._running:
                    logger.error("M20 vel watchdog error: %s", exc)
