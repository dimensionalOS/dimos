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

"""Unitree G1 WebRTC adapter — wraps UnitreeWebRTCConnection for humanoid base control.

The G1 is a humanoid robot with 3 DOF velocity control: [vx, vy, wz].
This adapter uses WebRTC to communicate wirelessly with the robot,
wrapping the existing UnitreeWebRTCConnection which handles all
WebRTC protocol details (asyncio event loop, datachannel, RTC topics).

Locomotion initialization is configurable via the ``locomotion_mode``
parameter:
  - ``"walk"`` (default): standup → G1 WalkMode (api_id 7101, data 500)
  - ``"balance_stand"``: standup → BalanceStand (SPORT_CMD 1002)
"""

from __future__ import annotations

import math
import threading
import time
from typing import TYPE_CHECKING

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from reactivex.abc import DisposableBase

    from dimos.hardware.drive_trains.registry import TwistBaseAdapterRegistry
    from dimos.msgs.geometry_msgs import Pose
    from dimos.robot.unitree.connection import UnitreeWebRTCConnection

logger = setup_logger()

# G1 LocoClient API IDs and FSM states
_G1_SET_FSM_ID = 7101  # SetFsmId — also used for walk mode with data=500
_G1_SPORT_TOPIC = "rt/api/sport/request"
_FSM_LOCK_STAND = 4
_FSM_LOCOMOTION = 200


class UnitreeG1WebRTCTwistAdapter:
    """TwistBaseAdapter implementation for Unitree G1 humanoid over WebRTC.

    Communicates with G1 via WebRTC using the UnitreeWebRTCConnection driver.
    Expects 3 DOF: [vx, vy, wz] where:
      - vx: forward/backward velocity (m/s)
      - vy: left/right lateral velocity (m/s)
      - wz: yaw rotation velocity (rad/s)

    Args:
        dof: Number of velocity DOFs (must be 3 for G1)
        ip: Robot IP address (default: "192.168.123.164")
        address: Alias for ip (used by ControlCoordinator's HardwareComponent)
        locomotion_mode: Locomotion activation mode — ``"walk"`` (default)
            uses G1-specific WalkMode, ``"balance_stand"`` uses the
            BalanceStand command.
    """

    def __init__(
        self,
        dof: int = 3,
        ip: str | None = None,
        address: str | None = None,
        locomotion_mode: str = "walk",
        **_: object,
    ) -> None:
        if dof != 3:
            raise ValueError(f"G1 only supports 3 DOF (vx, vy, wz), got {dof}")

        # Accept either ip= or address= (coordinator passes address=component.address)
        self._ip = ip or address or "192.168.123.164"
        self._locomotion_mode = locomotion_mode
        self._conn: UnitreeWebRTCConnection | None = None
        self._lock = threading.Lock()
        self._enabled = False

        # Last commanded velocities (WebRTC doesn't provide velocity feedback)
        self._last_velocities = [0.0, 0.0, 0.0]

        # Latest odometry from odom_stream subscription
        self._latest_odom: list[float] | None = None
        self._odom_disposable: DisposableBase | None = None

    # =========================================================================
    # Connection
    # =========================================================================

    def connect(self) -> bool:
        """Connect to G1 over WebRTC."""
        try:
            from dimos.robot.unitree.connection import UnitreeWebRTCConnection

            logger.info(f"Connecting to G1 via WebRTC at {self._ip}...")
            conn = UnitreeWebRTCConnection(self._ip)

            # Subscribe to odometry for read_odometry()
            self._odom_disposable = conn.odom_stream().subscribe(
                on_next=self._on_odom,
                on_error=lambda e: logger.warning(f"Odom stream error: {e}"),
            )

            self._conn = conn
            logger.info("Connected to G1 via WebRTC")

            # Enter lock stand (FSM 4) so the robot is standing and ready
            logger.info("G1 WebRTC: entering lock stand (FSM 4)...")
            conn.publish_request(
                _G1_SPORT_TOPIC,
                {"api_id": _G1_SET_FSM_ID, "parameter": {"data": _FSM_LOCK_STAND}},
            )
            time.sleep(6)  # Wait for robot to finish standing
            logger.info("G1 WebRTC: lock stand ready")

            return True

        except Exception as e:
            logger.error(f"Failed to connect to G1 via WebRTC: {e}")
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
                logger.info("Disconnecting G1 via WebRTC...")
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
        """Check if connected to G1."""
        return self._conn is not None

    # =========================================================================
    # Info
    # =========================================================================

    def get_dof(self) -> int:
        """G1 base is always 3 DOF (vx, vy, wz)."""
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
        """Send velocity command to G1 over WebRTC.

        Args:
            velocities: [vx, vy, wz] in standard frame (m/s, m/s, rad/s)
        """
        if len(velocities) != 3:
            return False

        conn = self._conn
        if conn is None:
            return False

        if not self._enabled:
            logger.warning("G1 WebRTC not enabled, ignoring velocity command")
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

        When enabling, stands up and activates the configured locomotion mode.
        When disabling, stops motion.
        """
        conn = self._conn
        if conn is None:
            return False

        if enable:
            try:
                logger.info("G1 WebRTC: starting locomotion (FSM 200)...")
                conn.publish_request(
                    _G1_SPORT_TOPIC,
                    {"api_id": _G1_SET_FSM_ID, "parameter": {"data": _FSM_LOCOMOTION}},
                )
                time.sleep(3)  # Wait for locomotion to activate

                self._enabled = True
                logger.info("G1 WebRTC: locomotion ready")
                return True
            except Exception as e:
                logger.error(f"Failed to enable G1 via WebRTC: {e}")
                return False
        else:
            self.write_stop()
            self._enabled = False
            logger.info("G1 WebRTC disabled")
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
        """Send velocity to G1 via WebRTC move() command."""
        from dimos.msgs.geometry_msgs import Twist, Vector3

        try:
            twist = Twist(
                linear=Vector3(x=vx, y=vy, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=wz),
            )
            return conn.move(twist)
        except Exception as e:
            logger.error(f"Error sending G1 WebRTC velocity: {e}")
            return False

    def _send_zero_velocity(self, conn: UnitreeWebRTCConnection) -> bool:
        """Send zero velocity to stop the robot."""
        return self._send_velocity(conn, 0.0, 0.0, 0.0)


def register(registry: TwistBaseAdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("unitree_g1_webrtc", UnitreeG1WebRTCTwistAdapter)


__all__ = ["UnitreeG1WebRTCTwistAdapter"]
