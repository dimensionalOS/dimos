# Copyright 2025 Dimensional Inc.
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

from __future__ import annotations

from dimos.robot.capabilities import Connection, Move, Lidar, Stop, Video, Odometry
from dimos.types.vector import Vector
from typing import Any
import os


class Robot:
    """Helper class for robots."""

    def __init__(self, conn: Connection):
        self.conn = conn
        self.output_dir = os.path.join(os.getcwd(), "assets", "output")
        self._modules: dict[str, Any] = {}

    def move(self, velocity: Vector, duration: float = 0.0) -> bool:
        if not isinstance(self.conn, Move):
            raise RuntimeError("Connection object does not support Move capability")
        return self.conn.move(velocity, duration)

    def stop(self) -> bool:
        if not isinstance(self.conn, Stop):
            raise RuntimeError("Connection object does not support Stop capability")
        return self.conn.stop()

    def lidar_stream(self):
        if not isinstance(self.conn, Lidar):
            raise RuntimeError("Lidar capability unavailable")
        return self.conn.lidar_stream()

    def video_stream(self):
        if not isinstance(self.conn, Video):
            raise RuntimeError("Video capability unavailable")
        return self.conn.video_stream()

    def odom_stream(self):
        if not isinstance(self.conn, Odometry):
            raise RuntimeError("Odometry capability unavailable")
        return self.conn.odom_stream()

    def __getattr__(self, item: str) -> Any:
        """Fallback – forward any unknown attribute to the connection object."""
        return getattr(self.conn, item)

    def get_module(self, cls):
        return next((m for m in self._modules.values() if isinstance(m, cls)), None)

    def get_skills(self):
        if hasattr(self, "skill_library"):
            return self.skill_library
        else:
            raise AttributeError("Skill library does not exist")
