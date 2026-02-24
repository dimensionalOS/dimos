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

"""Unitree Go2 WebRTC adapter — wraps UnitreeWebRTCConnection for quadruped base control.

The Go2 is a quadruped robot with 3 DOF velocity control: [vx, vy, wz].
This adapter uses WebRTC to communicate wirelessly with the robot,
unlike the DDS-based UnitreeGo2TwistAdapter which requires hardwired ethernet.

The adapter wraps the existing UnitreeWebRTCConnection which handles all
WebRTC protocol details (asyncio event loop, datachannel, RTC topics).
"""

from __future__ import annotations

import math
import threading
import time
from typing import TYPE_CHECKING

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from reactivex.disposable import Disposable

    from dimos.hardware.drive_trains.registry import TwistBaseAdapterRegistry
    from dimos.msgs.geometry_msgs import Pose
    from dimos.robot.unitree.connection import UnitreeWebRTCConnection

logger = setup_logger()


class UnitreeGo2WebRTCAdapter:
    """TwistBaseAdapter implementation for Unitree Go2 over WebRTC.

    Communicates with Go2 via WebRTC using the UnitreeWebRTCConnection driver.
    Expects 3 DOF: [vx, vy, wz] where:
      - vx: forward/backward velocity (m/s)
      - vy: left/right lateral velocity (m/s)
      - wz: yaw rotation velocity (rad/s)

    Args:
        dof: Number of velocity DOFs (must be 3 for Go2)
        ip: Robot IP address (default: "192.168.12.1")
        address: Alias for ip (used by ControlCoordinator's HardwareComponent)
    """

    def __init__(
        self,
        dof: int = 3,
        ip: str | None = None,
        address: str | None = None,
        **_: object,
    ) -> None:
        if dof != 3:
            raise ValueError(f"Go2 only supports 3 DOF (vx, vy, wz), got {dof}")

        # Accept either ip= or address= (coordinator passes address=component.address)
        self._ip = ip or address or "192.168.12.1"
        self._conn: UnitreeWebRTCConnection | None = None
        self._lock = threading.Lock()
        self._enabled = False

        # Last commanded velocities (WebRTC doesn't provide velocity feedback)
        self._last_velocities = [0.0, 0.0, 0.0]

        # Latest odometry from odom_stream subscription
        self._latest_odom: list[float] | None = None
        self._odom_disposable: Disposable | None = None

    # =========================================================================
    # Connection
    # =========================================================================

    def connect(self) -> bool:
        """Connect to Go2 over WebRTC."""
        try:
            from dimos.robot.unitree.connection import UnitreeWebRTCConnection

            logger.info(f"Connecting to Go2 via WebRTC at {self._ip}...")
            conn = UnitreeWebRTCConnection(self._ip)

            # Subscribe to odometry for read_odometry()
            self._odom_disposable = conn.odom_stream().subscribe(
                on_next=self._on_odom,
                on_error=lambda e: logger.warning(f"Odom stream error: {e}"),
            )

            self._conn = conn
            logger.info("Connected to Go2 via WebRTC")
            return True

        except Exception as e:
            logger.error(f"Failed to connect to Go2 via WebRTC: {e}")
            self._conn = None
            return False

    def disconnect(self) -> None:
        """Disconnect and safely shut down."""
        if self._odom_disposable is not None:
            self._odom_disposable.dispose()
            self._odom_disposable = None

        conn = self._conn
        if conn is not None:
            try:
                self._send_zero_velocity(conn)
                time.sleep(0.3)
                logger.info("Lying down Go2 via WebRTC...")
                conn.liedown()
                time.sleep(2)
            except Exception as e:
                logger.error(f"Error during shutdown sequence: {e}")

            try:
                conn.disconnect()
            except Exception as e:
                logger.error(f"Error during WebRTC disconnect: {e}")

        self._conn = None
        self._enabled = False
        with self._lock:
            self._last_velocities = [0.0, 0.0, 0.0]
            self._latest_odom = None

    def is_connected(self) -> bool:
        """Check if connected to Go2."""
        return self._conn is not None

    # =========================================================================
    # Info
    # =========================================================================

    def get_dof(self) -> int:
        """Go2 is always 3 DOF (vx, vy, wz)."""
        return 3

    # =========================================================================
    # State Reading
    # =========================================================================

    def read_velocities(self) -> list[float]:
        """Return last commanded velocities (WebRTC doesn't provide velocity feedback)."""
        with self._lock:
            return self._last_velocities.copy()

    def read_odometry(self) -> list[float] | None:
        """Read odometry as [x, y, theta] from WebRTC odom stream."""
        with self._lock:
            if self._latest_odom is None:
                return None
            return self._latest_odom.copy()

    # =========================================================================
    # Control
    # =========================================================================

    def write_velocities(self, velocities: list[float]) -> bool:
        """Send velocity command to Go2 over WebRTC.

        Args:
            velocities: [vx, vy, wz] in standard frame (m/s, m/s, rad/s)
        """
        if len(velocities) != 3:
            return False

        conn = self._conn
        if conn is None:
            return False

        if not self._enabled:
            logger.warning("Go2 WebRTC not enabled, ignoring velocity command")
            return False

        vx, vy, wz = velocities

        with self._lock:
            self._last_velocities = list(velocities)

        return self._send_velocity(conn, vx, vy, wz)

    def write_stop(self) -> bool:
        """Stop all motion by sending zero velocity."""
        with self._lock:
            self._last_velocities = [0.0, 0.0, 0.0]

        conn = self._conn
        if conn is None:
            return False

        return self._send_zero_velocity(conn)

    # =========================================================================
    # Enable/Disable
    # =========================================================================

    def write_enable(self, enable: bool) -> bool:
        """Enable/disable the platform.

        When enabling: StandUp → BalanceStand → ready for WIRELESS_CONTROLLER commands.
        When disabling, stops motion.
        """
        conn = self._conn
        if conn is None:
            return False

        if enable:
            try:
                logger.info("Standing up Go2 via WebRTC...")
                conn.standup()
                time.sleep(3)  # Wait for standup to complete

                logger.info("Activating BalanceStand locomotion mode...")
                conn.balance_stand()
                time.sleep(2)  # Wait for locomotion mode to activate

                self._enabled = True
                logger.info("Go2 WebRTC enabled (BalanceStand active)")
                return True
            except Exception as e:
                logger.error(f"Failed to enable Go2 via WebRTC: {e}")
                return False
        else:
            self.write_stop()
            self._enabled = False
            logger.info("Go2 WebRTC disabled")
            return True

    def read_enabled(self) -> bool:
        """Check if platform is enabled."""
        return self._enabled

    # =========================================================================
    # Internal
    # =========================================================================

    def _on_odom(self, pose: Pose) -> None:
        """Callback for odom_stream subscription. Extracts [x, y, yaw] from Pose."""
        try:
            x = float(pose.position.x)
            y = float(pose.position.y)

            # Convert quaternion to yaw
            qx = float(pose.orientation.x)
            qy = float(pose.orientation.y)
            qz = float(pose.orientation.z)
            qw = float(pose.orientation.w)
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

            with self._lock:
                self._latest_odom = [x, y, yaw]
        except Exception as e:
            logger.warning(f"Error processing odom message: {e}")

    def _send_velocity(
        self, conn: UnitreeWebRTCConnection, vx: float, vy: float, wz: float
    ) -> bool:
        """Send velocity to Go2 via WebRTC move() command."""
        from dimos.msgs.geometry_msgs import Twist, Vector3

        try:
            twist = Twist(
                linear=Vector3(x=vx, y=vy, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=wz),
            )
            return conn.move(twist)
        except Exception as e:
            logger.error(f"Error sending Go2 WebRTC velocity: {e}")
            return False

    def _send_zero_velocity(self, conn: UnitreeWebRTCConnection) -> bool:
        """Send zero velocity to stop the robot."""
        return self._send_velocity(conn, 0.0, 0.0, 0.0)


def register(registry: TwistBaseAdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("unitree_go2_webrtc", UnitreeGo2WebRTCAdapter)


__all__ = ["UnitreeGo2WebRTCAdapter"]
