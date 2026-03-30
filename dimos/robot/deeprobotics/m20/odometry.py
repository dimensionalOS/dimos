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

"""Basic dead-reckoning odometry for M20 quadruped.

Integrates velocity commands over time to estimate robot pose.
This is a crude but functional approach for initial PoC testing.
Should be replaced with M20's onboard /odom DDS topic or IMU fusion
for production use (see di-snz59.6).
"""

import logging
import math
import threading
import time
from typing import Callable

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

logger = logging.getLogger(__name__)


class M20DeadReckonOdometry:
    """Dead-reckoning odometry from velocity commands.

    Tracks the smoothed velocities from the velocity controller and
    integrates them to produce a PoseStamped estimate. Publishes at
    a configurable rate (default 10Hz to match the nav stack).

    This is intentionally simple — no IMU fusion, no drift correction.
    For the PoC, it provides the odom stream the SLAM stack needs.
    """

    def __init__(
        self,
        publish_callback: Callable[[PoseStamped], None],
        rate: float = 10.0,
        frame_id: str = "world",
    ):
        if rate <= 0:
            raise ValueError("rate must be positive")
        self._publish = publish_callback
        self._rate = rate
        self._frame_id = frame_id

        # Pose state
        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0

        # Current velocities (set externally by velocity controller)
        self._vx: float = 0.0
        self._vy: float = 0.0
        self._vyaw: float = 0.0

        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def update_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        """Update current velocity estimate (called from velocity controller)."""
        with self._lock:
            self._vx = vx
            self._vy = vy
            self._vyaw = vyaw

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """Reset odometry to a known pose."""
        with self._lock:
            self._x = x
            self._y = y
            self._yaw = yaw
        logger.info(f"Odometry reset to x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}deg")

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._integration_loop, daemon=True)
        self._thread.start()
        logger.info(f"Dead-reckoning odometry started at {self._rate}Hz")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _integration_loop(self) -> None:
        dt = 1.0 / self._rate
        last_time = time.monotonic()

        while self._running:
            now = time.monotonic()
            elapsed = now - last_time
            last_time = now

            with self._lock:
                vx, vy, vyaw = self._vx, self._vy, self._vyaw

                # Integrate velocities in robot frame → world frame
                cos_yaw = math.cos(self._yaw)
                sin_yaw = math.sin(self._yaw)

                self._x += (vx * cos_yaw - vy * sin_yaw) * elapsed
                self._y += (vx * sin_yaw + vy * cos_yaw) * elapsed
                self._yaw += vyaw * elapsed

                # Normalize yaw to [-pi, pi]
                self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

                pose = PoseStamped(
                    position=Vector3(self._x, self._y, 0.0),
                    orientation=Quaternion.from_euler(
                        Vector3(0.0, 0.0, self._yaw)
                    ),
                    frame_id=self._frame_id,
                    ts=time.time(),
                )

            self._publish(pose)

            sleep_time = max(0.0, dt - (time.monotonic() - now))
            time.sleep(sleep_time)

    @property
    def pose(self) -> tuple[float, float, float]:
        """Current pose estimate (x, y, yaw)."""
        with self._lock:
            return (self._x, self._y, self._yaw)
