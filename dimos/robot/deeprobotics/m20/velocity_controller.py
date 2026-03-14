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
and publishes at 10 Hz (per M20 SDK recommendation). Includes a safety
watchdog that zeros velocities when no command arrives within the timeout.

/NAV_CMD message structure (drdds/msg/NavCmd):
    MetaType header|meta     (Foxy uses 'header', Humble uses 'meta')
        uint64 frame_id
        Timestamp timestamp
            int32 sec
            uint32 nsec
    NavCmdValue data
        float32 x_vel        (forward/backward, m/s, +forward)
        float32 y_vel        (left/right, m/s, +left)
        float32 yaw_vel      (yaw rate, rad/s, +ccw)

Prerequisites:
    - Robot must be in navigation mode with Agile gait (0x3002)
    - basic_server service must be running on AOS
    - Built-in planner (handler) should be stopped to avoid conflicts

The drdds package is installed into the NOS host venv by deploy.sh setup
(extracted from the nav container). When running off-robot the controller
operates in log-only mode.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from dimos.msgs.geometry_msgs import Twist
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# ---------------------------------------------------------------------------
# Conditional import of drdds — only available on the M20 NOS host.
# deploy.sh setup extracts drdds from the Humble nav container into the host
# Python 3.10 venv.  Foxy uses 'header', Humble uses 'meta'.
# ---------------------------------------------------------------------------
DRDDS_AVAILABLE = False
_DRDDS_HEADER_FIELD: str = "header"  # default to Foxy naming

try:
    from drdds.msg import NavCmd as DDSNavCmd  # type: ignore[import-untyped]

    DRDDS_AVAILABLE = True

    # Detect Foxy ('header') vs Humble ('meta') field naming
    _probe = DDSNavCmd()
    if hasattr(_probe, "meta"):
        _DRDDS_HEADER_FIELD = "meta"
    elif hasattr(_probe, "header"):
        _DRDDS_HEADER_FIELD = "header"
    del _probe
except ImportError:
    pass

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
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
    """Publishes velocity commands to /NAV_CMD at 10 Hz.

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
            "M20VelocityController started (drdds=%s, header_field=%s, test=%s)",
            DRDDS_AVAILABLE,
            _DRDDS_HEADER_FIELD,
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
        """Set up the DDS publisher for /NAV_CMD.

        Uses rclpy with the drdds NavCmd message type.  The ROSNav module
        already initialises rclpy, so we reuse its context.  On the NOS host
        the drdds package is installed from the Humble nav container by
        deploy.sh setup.
        """
        if self._test_mode:
            logger.info("[TEST] M20VelocityController in test mode — no DDS publishing")
            return

        if not DRDDS_AVAILABLE:
            logger.warning(
                "drdds not available — /NAV_CMD will be logged only. "
                "Run 'deploy.sh setup' to install drdds bindings."
            )
            return

        try:
            import rclpy  # type: ignore[import-untyped]
            from rclpy.node import Node  # type: ignore[import-untyped]

            if not rclpy.ok():  # type: ignore[attr-defined]
                rclpy.init()

            node = Node("m20_velocity_controller")
            self._dds_publisher = node.create_publisher(DDSNavCmd, "/NAV_CMD", 10)
            logger.info(
                "DDS publisher created via rclpy for /NAV_CMD (header_field=%s)",
                _DRDDS_HEADER_FIELD,
            )
        except Exception as exc:
            logger.warning(
                "Failed to create rclpy publisher for /NAV_CMD: %s — "
                "velocity commands will be logged only.",
                exc,
            )

    def _populate_dds_msg(self, cmd: NavCmd) -> object:
        """Create and populate a DDSNavCmd message from a local NavCmd."""
        msg = DDSNavCmd()

        # Populate the header/meta with current timestamp
        hdr = getattr(msg, _DRDDS_HEADER_FIELD)
        hdr.frame_id = 0
        now = time.time()
        hdr.timestamp.sec = int(now)
        hdr.timestamp.nsec = int((now % 1) * 1e9)

        # Populate velocity fields
        msg.data.x_vel = cmd.x_vel  # type: ignore[attr-defined]
        msg.data.y_vel = cmd.y_vel  # type: ignore[attr-defined]
        msg.data.yaw_vel = cmd.yaw_vel  # type: ignore[attr-defined]

        return msg

    def _publish_once(self, cmd: NavCmd) -> None:
        """Publish a single NavCmd to /NAV_CMD."""
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

        if self._dds_publisher is None:
            return

        try:
            msg = self._populate_dds_msg(cmd)
            self._dds_publisher.publish(msg)  # type: ignore[union-attr]
        except Exception as exc:
            if self._packet_count % SEND_HZ == 0:
                logger.error("Failed to publish /NAV_CMD: %s", exc)

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
