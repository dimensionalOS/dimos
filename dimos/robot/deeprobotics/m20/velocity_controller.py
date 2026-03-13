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

Converts dimos Twist messages to /NAV_CMD DDS messages via the drdds library
and publishes at 50 Hz. Includes a safety watchdog that zeros velocities
when no command arrives within the timeout window.

The drdds package is only available on the NOS host (installed by deploy.sh
setup). When running off-robot the controller operates in log-only mode.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from dimos.msgs.geometry_msgs import Twist
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Conditional import of drdds — only available on the M20 NOS host.
try:
    from drdds.msg import NavCmd as DDSNavCmd  # type: ignore[import-untyped]

    DRDDS_AVAILABLE = True
except ImportError:
    DRDDS_AVAILABLE = False

# Conditional rclpy import — fallback DDS publishing path via ROS 2.
try:
    import rclpy  # type: ignore[import-untyped]
    from rclpy.node import Node  # type: ignore[import-untyped]

    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False

# --- Velocity limits (m/s, rad/s) -----------------------------------------
MAX_LINEAR_VEL = 1.0
MAX_YAW_RATE = 1.5
SEND_HZ = 50
WATCHDOG_TIMEOUT_S = 0.2


@dataclass
class NavCmd:
    """Local representation of a /NAV_CMD velocity command.

    Fields mirror the drdds/msg/NavCmd DDS type used by the M20 locomotion
    controller on the AOS board.
    """

    vx: float = 0.0  # forward  (m/s, +forward)
    vy: float = 0.0  # lateral  (m/s, +left)
    vyaw: float = 0.0  # yaw rate (rad/s, +ccw)

    @classmethod
    def from_twist(cls, twist: Twist) -> NavCmd:
        """Convert a dimos Twist to a clamped NavCmd."""
        return cls(
            vx=max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, twist.linear.x)),
            vy=max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, twist.linear.y)),
            vyaw=max(-MAX_YAW_RATE, min(MAX_YAW_RATE, twist.angular.z)),
        )

    @classmethod
    def zero(cls) -> NavCmd:
        return cls()

    @property
    def is_zero(self) -> bool:
        return self.vx == 0.0 and self.vy == 0.0 and self.vyaw == 0.0


class M20VelocityController:
    """Publishes velocity commands to /NAV_CMD at 50 Hz.

    Parameters
    ----------
    test_mode : bool
        If True, log commands instead of publishing over DDS.
    """

    def __init__(self, *, test_mode: bool = False) -> None:
        self._test_mode = test_mode
        self._lock = threading.Lock()
        self._current_cmd = NavCmd.zero()
        self._last_cmd_time = time.monotonic()
        self._running = False
        self._send_thread: threading.Thread | None = None
        self._watchdog_thread: threading.Thread | None = None
        self._packet_count = 0
        self._timeout_active = False

        # DDS publisher handle — set up on start()
        self._dds_publisher: object | None = None
        self._rclpy_node: object | None = None

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
            "M20VelocityController started (drdds=%s, rclpy=%s, test=%s)",
            DRDDS_AVAILABLE,
            RCLPY_AVAILABLE,
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

        if self._rclpy_node is not None:
            self._rclpy_node.destroy_node()  # type: ignore[union-attr]
            self._rclpy_node = None

        self._dds_publisher = None
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
        """Set up the DDS publisher for /NAV_CMD."""
        if self._test_mode:
            logger.info("[TEST] M20VelocityController in test mode — no DDS publishing")
            return

        if DRDDS_AVAILABLE:
            try:
                # drdds provides a direct DDS publisher for NavCmd
                import drdds  # type: ignore[import-untyped]

                self._dds_publisher = drdds.Publisher("/NAV_CMD", DDSNavCmd)
                logger.info("DDS publisher created via drdds for /NAV_CMD")
                return
            except Exception as exc:
                logger.warning("Failed to create drdds publisher: %s", exc)

        if RCLPY_AVAILABLE:
            try:
                if not rclpy.ok():  # type: ignore[attr-defined]
                    rclpy.init()

                self._rclpy_node = Node("m20_velocity_controller")

                # Try importing the ROS 2 NavCmd wrapper from drdds-ros2-msgs
                try:
                    from drdds.msg import NavCmd as ROSNavCmd  # type: ignore[import-untyped,no-redef]

                    self._dds_publisher = self._rclpy_node.create_publisher(  # type: ignore[union-attr]
                        ROSNavCmd, "/NAV_CMD", 10
                    )
                    logger.info("DDS publisher created via rclpy + drdds-ros2-msgs for /NAV_CMD")
                    return
                except ImportError:
                    logger.warning(
                        "drdds-ros2-msgs not available — cannot publish NavCmd via rclpy"
                    )
            except Exception as exc:
                logger.warning("Failed to create rclpy publisher: %s", exc)

        logger.warning(
            "No DDS backend available for /NAV_CMD — velocity commands will be logged only. "
            "Install drdds (deploy.sh setup) to enable robot control."
        )

    def _publish_once(self, cmd: NavCmd) -> None:
        """Publish a single NavCmd to /NAV_CMD."""
        if self._test_mode:
            if self._packet_count % SEND_HZ == 0:
                logger.info(
                    "[TEST] NavCmd vx=%.3f vy=%.3f vyaw=%.3f | count=%d",
                    cmd.vx,
                    cmd.vy,
                    cmd.vyaw,
                    self._packet_count,
                )
            return

        if self._dds_publisher is None:
            return

        try:
            if DRDDS_AVAILABLE and hasattr(self._dds_publisher, "publish"):
                msg = DDSNavCmd()
                msg.vx = cmd.vx  # type: ignore[attr-defined]
                msg.vy = cmd.vy  # type: ignore[attr-defined]
                msg.vyaw = cmd.vyaw  # type: ignore[attr-defined]
                self._dds_publisher.publish(msg)  # type: ignore[union-attr]
            elif RCLPY_AVAILABLE and hasattr(self._dds_publisher, "publish"):
                msg = DDSNavCmd()
                msg.vx = cmd.vx  # type: ignore[attr-defined]
                msg.vy = cmd.vy  # type: ignore[attr-defined]
                msg.vyaw = cmd.vyaw  # type: ignore[attr-defined]
                self._dds_publisher.publish(msg)  # type: ignore[union-attr]
        except Exception as exc:
            if self._packet_count % SEND_HZ == 0:
                logger.error("Failed to publish /NAV_CMD: %s", exc)

    def _send_loop(self) -> None:
        """Continuously publish the current command at 50 Hz."""
        period = 1.0 / SEND_HZ
        while self._running:
            try:
                with self._lock:
                    cmd = self._current_cmd

                self._publish_once(cmd)
                self._packet_count += 1

                if self._packet_count % SEND_HZ == 0:
                    logger.debug(
                        "M20 vel send loop: vx=%.3f vy=%.3f vyaw=%.3f count=%d",
                        cmd.vx,
                        cmd.vy,
                        cmd.vyaw,
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
