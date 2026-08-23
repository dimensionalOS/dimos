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

"""FlowBase adapter — wraps Portal RPC client for holonomic base control.

Frame convention: FlowBase uses inverted Y-axis compared to standard convention.
We negate vy and wz when sending to the hardware.

  Standard (ROS):     FlowBase:
      +Y                -Y
      ↑                  ↑
   ───┼──→ +X         ───┼──→ +X
      |                  |
"""

from __future__ import annotations

from collections.abc import Callable
import math
import threading
import time
from typing import Any

import numpy as np

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_ADDRESS = "172.6.2.20:11323"
DEFAULT_ODOMETRY_TOPIC = "/wheel_odometry"
DEFAULT_ODOMETRY_FRAME = "wheel_odom"
DEFAULT_BASE_FRAME = "base_link"


class FlowBaseAdapter:
    """TwistBaseAdapter implementation for FlowBase holonomic platform.

    Communicates with FlowBase controller via Portal RPC over TCP.
    Expects 3 DOF: [vx, vy, wz] (holonomic base).

    Args:
        dof: Number of velocity DOFs (must be 3 for FlowBase)
        address: Portal RPC address as "host:port" (default: ``DEFAULT_ADDRESS``)
    """

    def __init__(
        self,
        dof: int = 3,
        address: str | None = None,
        odometry_topic: str = DEFAULT_ODOMETRY_TOPIC,
        odometry_frame: str = DEFAULT_ODOMETRY_FRAME,
        base_frame: str = DEFAULT_BASE_FRAME,
        transport_cls: type = LCMTransport,
        clock: Callable[[], float] = time.time,
        monotonic_clock: Callable[[], float] = time.monotonic,
        **_: object,
    ) -> None:
        if dof != 3:
            raise ValueError(f"FlowBase only supports 3 DOF (holonomic), got {dof}")

        self._address = address or DEFAULT_ADDRESS
        self._client = None
        self._connected = False
        self._enabled = False
        self._lock = threading.RLock()
        self._odometry_topic = odometry_topic
        self._odometry_frame = odometry_frame
        self._base_frame = base_frame
        self._transport_cls = transport_cls
        self._clock = clock
        self._monotonic_clock = monotonic_clock
        self._odometry_transport: Any = None
        self._previous_odometry: tuple[float, float, float, float] | None = None

        # Last commanded velocities (in standard frame, before negation)
        self._last_velocities = [0.0, 0.0, 0.0]

    def connect(self) -> bool:
        """Connect to FlowBase controller via Portal RPC."""
        client = None
        odometry_transport = None
        try:
            import portal

            client = portal.Client(self._address)
            odometry_transport = self._transport_cls(self._odometry_topic, Odometry)
            with self._lock:
                self._client = client
                self._odometry_transport = odometry_transport
                self._previous_odometry = None
                self._last_velocities = [0.0, 0.0, 0.0]
                self._enabled = False
                self._connected = True
            logger.info(f"Connected to FlowBase at {self._address}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to FlowBase at {self._address}: {e}")
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass
            if odometry_transport is not None:
                try:
                    odometry_transport.stop()
                except Exception:
                    pass
            with self._lock:
                self._client = None
                self._odometry_transport = None
                self._connected = False
            return False

    def disconnect(self) -> None:
        """Disconnect and send zero velocity."""
        with self._lock:
            if self._connected and self._client:
                self._send_velocity(0.0, 0.0, 0.0)
            client = self._client
            odometry_transport = self._odometry_transport
            self._connected = False
            self._client = None
            self._odometry_transport = None
            self._previous_odometry = None
            self._last_velocities = [0.0, 0.0, 0.0]
            self._enabled = False

        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        if odometry_transport is not None:
            try:
                odometry_transport.stop()
            except Exception:
                pass

    def is_connected(self) -> bool:
        """Check if connected to FlowBase."""
        return self._connected

    def get_dof(self) -> int:
        """FlowBase is always 3 DOF (vx, vy, wz)."""
        return 3

    def read_velocities(self) -> list[float]:
        """Return last commanded velocities (FlowBase doesn't report actual)."""
        with self._lock:
            return self._last_velocities.copy()

    def read_odometry(self) -> list[float] | None:
        """Publish wheel odometry and return coordinator virtual-joint values.

        ``ConnectedTwistBase`` treats this list as the position of its virtual
        ``vx/vy/wz`` joints. Keep those values as velocities; the integrated
        pose is published separately as ``nav_msgs.Odometry``.
        """
        if not self._connected or not self._client:
            return None

        try:
            with self._lock:
                odom = self._client.get_odometry({}).result()
                if odom is None:
                    return self._last_velocities.copy()

                translation = odom["translation"]
                x = float(translation[0])
                y = -float(translation[1])
                yaw = -float(odom["rotation"])
                message = self._make_odometry_message(
                    self._clock(), self._monotonic_clock(), x, y, yaw
                )
                transport = self._odometry_transport
                if transport is not None:
                    try:
                        transport.publish(message)
                    except Exception as e:
                        logger.error(f"Error publishing FlowBase wheel odometry: {e}")
                return self._last_velocities.copy()
        except Exception as e:
            logger.error(f"Error reading FlowBase odometry: {e}")
            return self.read_velocities()

    def write_velocities(self, velocities: list[float]) -> bool:
        """Send velocity command to FlowBase.

        Args:
            velocities: [vx, vy, wz] in standard frame (m/s, rad/s)
        """
        if len(velocities) != 3:
            return False

        if not self._connected or not self._client:
            return False

        vx, vy, wz = velocities
        with self._lock:
            self._last_velocities = list(velocities)

        # Negate vy and wz for FlowBase's inverted Y-axis frame
        return self._send_velocity(vx, -vy, -wz)

    def write_stop(self) -> bool:
        """Stop all motion."""
        with self._lock:
            self._last_velocities = [0.0, 0.0, 0.0]
        if not self._connected or not self._client:
            return False
        return self._send_velocity(0.0, 0.0, 0.0)

    def write_enable(self, enable: bool) -> bool:
        """Enable/disable the platform (FlowBase is always enabled when connected)."""
        self._enabled = enable
        return True

    def read_enabled(self) -> bool:
        """Check if platform is enabled."""
        return self._enabled

    def _send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        """Send raw velocity to FlowBase via Portal RPC."""
        try:
            command = {
                "target_velocity": np.array([vx, vy, wz]),
                "frame": "local",
            }
            with self._lock:
                assert self._client is not None
                self._client.set_target_velocity(command).result()
            return True
        except Exception as e:
            logger.error(f"Error sending FlowBase velocity: {e}")
            return False

    def _make_odometry_message(
        self,
        ts: float,
        monotonic_ts: float,
        x: float,
        y: float,
        yaw: float,
    ) -> Odometry:
        twist = Twist()
        if self._previous_odometry is not None:
            last_ts, last_x, last_y, last_yaw = self._previous_odometry
            dt = monotonic_ts - last_ts
            if dt > 0.0:
                dx = x - last_x
                dy = y - last_y
                forward = math.cos(yaw) * dx + math.sin(yaw) * dy
                left = -math.sin(yaw) * dx + math.cos(yaw) * dy
                turn = math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
                twist = Twist(
                    linear=Vector3(forward / dt, left / dt, 0.0),
                    angular=Vector3(0.0, 0.0, turn / dt),
                )
        self._previous_odometry = (monotonic_ts, x, y, yaw)

        return Odometry(
            ts=ts,
            frame_id=self._odometry_frame,
            child_frame_id=self._base_frame,
            pose=Pose(
                position=Vector3(x, y, 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
            ),
            twist=twist,
        )
