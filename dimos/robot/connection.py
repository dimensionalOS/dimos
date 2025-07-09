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

"""Base connection class for DIMOS robots."""

from abc import ABC, abstractmethod
from typing import Optional


class BaseConnection(ABC):
    """Base class that all robot connections must inherit from.

    This is the required foundation for any robot connection, providing
    the minimal interface needed to establish and manage a connection.

    Concrete connections should inherit from this class and implement
    whichever capability protocols (Move, Video, Lidar, etc.) they support.
    """

    def __init__(self, ip: str):
        """Initialize connection with network identifier.

        Args:
            ip: Network identifier (IP address, hostname, serial port, etc.)
        """
        self.ip = ip

    @abstractmethod
    def connect(self) -> None:
        """Establish connection to the robot.

        This method should handle all necessary setup to establish
        communication with the robot hardware.
        """
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Close connection and clean up resources.

        This method should properly close the connection and release
        any allocated resources (threads, sockets, etc.).
        """
        pass
