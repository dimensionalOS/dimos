# Copyright 2026 Dimensional Inc.
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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License")
# ...SPDX short form
"""Test module with streams for Python API stream access testing.

Publishes synthetic sensor data (odometry, images, point clouds) on a background thread.
No hardware required.
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np

from dimos.agents.annotation import skill
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class StreamTestModule(Module):
    """Module that publishes synthetic odometry, camera, and lidar at ~10Hz."""

    odom: Out[PoseStamped]
    color_image: Out[Image]
    lidar: Out[PointCloud2]
    cmd_vel: In[Twist]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._thread = None
        self._publish_count = 0
        # Simulated robot state
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0  # radians
        self._vx = 0.0
        self._wz = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        # Subscribe to cmd_vel for motion commands (may not have transport
        # if deployed without a blueprint)
        try:
            if self.cmd_vel.transport is not None:
                self.cmd_vel.subscribe(self._on_cmd_vel)
        except (AttributeError, TypeError):
            pass  # No transport wired — running standalone
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().stop()

    def _on_cmd_vel(self, twist):
        """Handle velocity commands."""
        self._vx = twist.linear.x if hasattr(twist, "linear") else 0.0
        self._wz = twist.angular.z if hasattr(twist, "angular") else 0.0

    def _publish_loop(self) -> None:
        """Publish synthetic sensor data at ~10Hz."""
        dt = 0.1
        while not self._stop_event.is_set():
            self._publish_count += 1

            # Update simulated position based on velocity
            self._yaw += self._wz * dt
            self._x += self._vx * math.cos(self._yaw) * dt
            self._y += self._vx * math.sin(self._yaw) * dt

            # Publish odometry
            qw = math.cos(self._yaw / 2)
            qz = math.sin(self._yaw / 2)
            pose = PoseStamped(
                position=Vector3(self._x, self._y, 0.0),
                orientation=Quaternion(0.0, 0.0, qz, qw),
            )
            self.odom.publish(pose)

            # Publish synthetic camera image (8x8 RGB)
            pixels = np.full((8, 8, 3), self._publish_count % 256, dtype=np.uint8)
            img = Image(
                data=pixels,
                format=ImageFormat.RGB,
            )
            self.color_image.publish(img)

            # Publish synthetic point cloud (obstacle at 2m ahead)
            obstacle_dist = max(0.5, 2.0 - self._x * 0.1)  # gets closer as robot moves
            points = np.array(
                [
                    [obstacle_dist, -0.2, 0.0],
                    [obstacle_dist, 0.0, 0.0],
                    [obstacle_dist, 0.2, 0.0],
                    [obstacle_dist + 0.1, -0.1, 0.1],
                    [obstacle_dist + 0.1, 0.1, 0.1],
                ],
                dtype=np.float32,
            )
            cloud = PointCloud2.from_numpy(points, timestamp=time.time())
            try:
                self.lidar.publish(cloud)
            except Exception:
                pass  # Don't let lidar encoding failure kill odom/camera

            time.sleep(dt)

    @rpc
    def get_publish_count(self) -> int:
        return self._publish_count

    @rpc
    def get_position(self) -> dict:
        return {"x": self._x, "y": self._y, "yaw": self._yaw}

    @skill
    def relative_move(self, forward: float = 0.0, yaw: float = 0.0) -> str:
        """Move the robot forward (meters) and/or rotate (degrees)."""
        # Simulate instant move
        yaw_rad = math.radians(yaw)
        self._yaw += yaw_rad
        self._x += forward * math.cos(self._yaw)
        self._y += forward * math.sin(self._yaw)
        return f"Moved forward={forward}m yaw={yaw}deg -> ({self._x:.2f}, {self._y:.2f})"

    @skill
    def get_position_str(self) -> str:
        """Get the current position as a string."""
        return f"x={self._x:.2f}, y={self._y:.2f}, yaw={math.degrees(self._yaw):.1f}deg"

    @skill
    def stand_up(self) -> str:
        """Stand up (simulated)."""
        return "Standing"

    @skill
    def stand_down(self) -> str:
        """Sit down (simulated)."""
        return "Sitting"

    @skill
    def get_skills_list(self) -> list:
        """List available skills."""
        return ["relative_move", "get_position_str", "stand_up", "stand_down"]


# Blueprint for testing
demo_stream_test = autoconnect(
    StreamTestModule.blueprint(),
)

__all__ = ["StreamTestModule", "demo_stream_test"]
