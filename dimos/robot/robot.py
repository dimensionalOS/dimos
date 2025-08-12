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

"""Minimal robot interface for DIMOS robots."""

from abc import ABC
from typing import List

from dimos.types.robot_capabilities import RobotCapability


class Robot(ABC):
    """Minimal abstract base class for all DIMOS robots.

    This class provides the essential interface that all robot implementations
    can share, with no required methods - just common properties and helpers.
    """

    def __init__(self):
        """Initialize the robot with basic properties."""
        self.capabilities: List[RobotCapability] = []
        self.skill_library = None

    def has_capability(self, capability: RobotCapability) -> bool:
        """Check if the robot has a specific capability.

        Args:
            capability: The capability to check for

        Returns:
            bool: True if the robot has the capability
        """
        return capability in self.capabilities

    def get_skills(self):
        """Get the robot's skill library.

        Returns:
            The robot's skill library for managing skills
        """
        return self.skill_library

    def cleanup(self):
        """Clean up robot resources.

        Override this method to provide cleanup logic.
        """
        pass
