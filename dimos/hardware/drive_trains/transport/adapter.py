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

"""Transport-based twist adapter — bridges coordinator to a driver module via pub/sub.

Used when a robot's driver module (e.g., GO2Connection) owns the hardware connection
and publishes sensor data / accepts commands via transports. The coordinator
communicates with the driver through pub/sub topics instead of calling hardware directly.

This adapter is generic and robot-agnostic. The transport type (LCM, ROS, etc.)
is selected by registering different adapter_type names in the blueprint:

    adapter_type="transport_lcm"   # LCM (default, lightweight, same machine)
    adapter_type="transport_ros"   # ROS (interop with ROS ecosystem)

Topic naming convention:
    cmd_vel: /{hardware_id}/cmd_vel
    odom:    /{hardware_id}/odom
"""

from __future__ import annotations

import math
import threading
from functools import partial
from typing import TYPE_CHECKING, Any

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.registry import TwistBaseAdapterRegistry

logger = setup_logger()


class TransportTwistAdapter:
    """TwistBaseAdapter that reads/writes via pub/sub transports.

    The coordinator's tick loop calls write_velocities() which publishes a Twist
    to the cmd_vel topic. A driver module (e.g., GO2Connection) subscribes to
    that topic and forwards it to the robot hardware.

    Odometry is read by subscribing to the driver module's odom topic.

    Args:
        dof: Number of velocity DOFs (typically 3: vx, vy, wz).
        hardware_id: Used as topic prefix — /{hardware_id}/cmd_vel, /{hardware_id}/odom.
        transport_cls: Transport class to use (LCMTransport, ROSTransport, etc.).
    """

    def __init__(
        self,
        dof: int = 3,
        hardware_id: str = "base",
        transport_cls: type = LCMTransport,
        **_: object,
    ) -> None:
        self._dof = dof
        self._prefix = hardware_id
        self._transport_cls = transport_cls
        self._lock = threading.Lock()
        self._last_velocities = [0.0] * dof
        self._latest_odom: list[float] | None = None
        self._cmd_vel_transport: Any = None
        self._odom_transport: Any = None
        self._odom_unsub: Any = None
        self._connected = False
        self._enabled = False

    def connect(self) -> bool:
        """Set up pub/sub transports for cmd_vel and odom."""
        try:
            cmd_vel_topic = f"/{self._prefix}/cmd_vel"
            odom_topic = f"/{self._prefix}/odom"

            self._cmd_vel_transport = self._transport_cls(cmd_vel_topic, Twist)
            self._odom_transport = self._transport_cls(odom_topic, PoseStamped)

            self._odom_unsub = self._odom_transport.subscribe(self._on_odom)

            self._connected = True
            logger.info(
                f"TransportTwistAdapter connected: cmd_vel={cmd_vel_topic}, odom={odom_topic}"
            )
            return True
        except Exception as e:
            logger.error(f"TransportTwistAdapter failed to connect: {e}")
            return False

    def disconnect(self) -> None:
        """Tear down transports."""
        self.write_stop()

        if self._odom_unsub is not None:
            try:
                self._odom_unsub()
            except Exception:
                pass
            self._odom_unsub = None

        if self._cmd_vel_transport is not None:
            try:
                self._cmd_vel_transport.stop()
            except Exception:
                pass
            self._cmd_vel_transport = None

        if self._odom_transport is not None:
            try:
                self._odom_transport.stop()
            except Exception:
                pass
            self._odom_transport = None

        self._connected = False
        self._enabled = False
        with self._lock:
            self._last_velocities = [0.0] * self._dof
            self._latest_odom = None

    def is_connected(self) -> bool:
        return self._connected

def get_dof(self) -> int:
        return self._dof

def read_velocities(self) -> list[float]:
        with self._lock:
            return self._last_velocities.copy()

    def read_odometry(self) -> list[float] | None:
        with self._lock:
            if self._latest_odom is None:
                return None
            return self._latest_odom.copy()

def write_velocities(self, velocities: list[float]) -> bool:
        if len(velocities) != self._dof:
            return False

        if self._cmd_vel_transport is None:
            return False

        if not self._enabled:
            return False

        with self._lock:
            self._last_velocities = list(velocities)

        twist = Twist(
            linear=Vector3(x=velocities[0], y=velocities[1] if self._dof > 1 else 0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=velocities[2] if self._dof > 2 else 0.0),
        )
        try:
            self._cmd_vel_transport.broadcast(None, twist)
            return True
        except Exception as e:
            logger.error(f"TransportTwistAdapter write error: {e}")
            return False

    def write_stop(self) -> bool:
        with self._lock:
            self._last_velocities = [0.0] * self._dof

        if self._cmd_vel_transport is None:
            return False

        twist = Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0),
        )
        try:
            self._cmd_vel_transport.broadcast(None, twist)
            return True
        except Exception as e:
            logger.error(f"TransportTwistAdapter stop error: {e}")
            return False

def write_enable(self, enable: bool) -> bool:
        self._enabled = enable
        if not enable:
            self.write_stop()
        return True

    def read_enabled(self) -> bool:
        return self._enabled

def _on_odom(self, msg: PoseStamped) -> None:
        try:
            x = float(msg.position.x)
            y = float(msg.position.y)

            qx = float(msg.orientation.x)
            qy = float(msg.orientation.y)
            qz = float(msg.orientation.z)
            qw = float(msg.orientation.w)
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

            with self._lock:
                self._latest_odom = [x, y, yaw]
        except Exception as e:
            logger.warning(f"TransportTwistAdapter odom error: {e}")


def register(registry: TwistBaseAdapterRegistry) -> None:
    """Register transport-based adapters for different transport types."""
    registry.register("transport_lcm", partial(TransportTwistAdapter, transport_cls=LCMTransport))

    try:
        from dimos.core.transport import ROSTransport

        registry.register("transport_ros", partial(TransportTwistAdapter, transport_cls=ROSTransport))
    except ImportError:
        pass


__all__ = ["TransportTwistAdapter"]
