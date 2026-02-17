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

import logging
import threading
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.registry import TwistBaseAdapterRegistry

logger = logging.getLogger(__name__)


class FlowBaseAdapter:
    """TwistBaseAdapter implementation for FlowBase holonomic platform.

    Communicates with FlowBase controller via Portal RPC over TCP.
    Expects 3 DOF: [vx, vy, wz] (holonomic base).

    Args:
        dof: Number of velocity DOFs (must be 3 for FlowBase)
        address: Portal RPC address as "host:port" (default: "172.6.2.20:11323")
    """

    def __init__(self, dof: int = 3, address: str | None = None, **_: object) -> None:
        if dof != 3:
            raise ValueError(f"FlowBase only supports 3 DOF (holonomic), got {dof}")

        self._address = address or "172.6.2.20:11323"
        self._client = None
        self._connected = False
        self._enabled = False
        self._lock = threading.Lock()

        # Last commanded velocities (in standard frame, before negation)
        self._last_velocities = [0.0, 0.0, 0.0]

    # =========================================================================
    # Connection
    # =========================================================================

    def connect(self) -> bool:
        """Connect to FlowBase controller via Portal RPC."""
        try:
            import portal

            self._client = portal.Client(self._address)
            self._connected = True
            logger.info(f"Connected to FlowBase at {self._address}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to FlowBase at {self._address}: {e}")
            self._connected = False
            return False

    def disconnect(self) -> None:
        """Disconnect and send zero velocity."""
        if self._connected and self._client:
            try:
                self._send_velocity(0.0, 0.0, 0.0)
            except Exception:
                pass
            try:
                self._client.close()
            except Exception:
                pass
        self._connected = False
        self._client = None

    def is_connected(self) -> bool:
        """Check if connected to FlowBase."""
        return self._connected

    # =========================================================================
    # Info
    # =========================================================================

    def get_dof(self) -> int:
        """FlowBase is always 3 DOF (vx, vy, wz)."""
        return 3

    # =========================================================================
    # State Reading
    # =========================================================================

    def read_velocities(self) -> list[float]:
        """Return last commanded velocities (FlowBase doesn't report actual)."""
        return self._last_velocities.copy()

    def read_odometry(self) -> list[float] | None:
        """Read odometry from FlowBase as [x, y, theta]."""
        if not self._connected or not self._client:
            return None

        try:
            with self._lock:
                odom = self._client.get_odometry({}).result()

            if odom is None:
                return None

            translation = odom["translation"]  # [x, y]
            rotation = odom["rotation"]  # theta in radians
            return [float(translation[0]), float(translation[1]), float(rotation)]
        except Exception as e:
            logger.error(f"Error reading FlowBase odometry: {e}")
            return None

    # =========================================================================
    # Control
    # =========================================================================

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
        self._last_velocities = list(velocities)

        # Negate vy and wz for FlowBase's inverted Y-axis frame
        return self._send_velocity(vx, -vy, -wz)

    def write_stop(self) -> bool:
        """Stop all motion."""
        self._last_velocities = [0.0, 0.0, 0.0]
        if not self._connected or not self._client:
            return False
        return self._send_velocity(0.0, 0.0, 0.0)

    # =========================================================================
    # Enable/Disable
    # =========================================================================

    def write_enable(self, enable: bool) -> bool:
        """Enable/disable the platform (FlowBase is always enabled when connected)."""
        self._enabled = enable
        return True

    def read_enabled(self) -> bool:
        """Check if platform is enabled."""
        return self._enabled

    # =========================================================================
    # Internal
    # =========================================================================

    def _send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        """Send raw velocity to FlowBase via Portal RPC."""
        try:
            command = {
                "target_velocity": np.array([vx, vy, wz]),
                "frame": "local",
            }
            with self._lock:
                self._client.set_target_velocity(command).result()
            return True
        except Exception as e:
            logger.error(f"Error sending FlowBase velocity: {e}")
            return False


def register(registry: TwistBaseAdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("flowbase", FlowBaseAdapter)


__all__ = ["FlowBaseAdapter"]
