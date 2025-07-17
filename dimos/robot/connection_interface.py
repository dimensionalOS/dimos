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

from abc import ABC, abstractmethod
from typing import Optional
from reactivex.observable import Observable
from dimos.types.vector import Vector

__all__ = ["ConnectionInterface"]


class ConnectionInterface(ABC):
    """Abstract base class for robot connection interfaces.

    This class defines the minimal interface that all connection types (ROS, WebRTC, etc.)
    must implement to provide robot control and data streaming capabilities.
    """

    @abstractmethod
    def move(self, velocity: Vector, duration: float = 0.0) -> bool:
        """Send movement command to the robot using velocity commands.

        Args:
            velocity: Velocity vector [x, y, yaw] where:
                     x: Forward/backward velocity (m/s)
                     y: Left/right velocity (m/s)
                     yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds). If 0, command is continuous

        Returns:
            bool: True if command was sent successfully
        """
        pass

    @abstractmethod
    def get_video_stream(self, fps: int = 30) -> Optional[Observable]:
        """Get the video stream from the robot's camera.

        Args:
            fps: Frames per second for the video stream

        Returns:
            Observable: An observable stream of video frames or None if not available
        """
        pass

    @abstractmethod
    def stop(self) -> bool:
        """Stop the robot's movement.

        Returns:
            bool: True if stop command was sent successfully
        """
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the robot and clean up resources."""
        pass
