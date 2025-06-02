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

"""Abstract base class for manipulation skills."""

from typing import Optional

from dimos.skills.skills import AbstractRobotSkill, Colors
from dimos.robot.robot import Robot
from dimos.manipulation.manipulation_interface import ManipulationInterface
from dimos.types.robot_capabilities import RobotCapability


class AbstractManipulationSkill(AbstractRobotSkill):
    """Base class for all manipulation-related skills.

    This abstract class provides access to the robot's manipulation memory system.
    """

    def __init__(self, *args, robot: Optional[Robot] = None, **kwargs):
        """Initialize the manipulation skill.

        Args:
            robot: The robot instance to associate with this skill
        """
        super().__init__(*args, robot=robot, **kwargs)

        if self._robot and not self._robot.manipulation_interface:
            raise NotImplementedError(
                "This robot does not have a manipulation interface implemented"
            )

    @property
    def manipulation_interface(self) -> Optional[ManipulationInterface]:
        """Get the robot's manipulation interface.

        Returns:
            ManipulationInterface: The robot's manipulation interface or None if not available

        Raises:
            RuntimeError: If the robot doesn't have the MANIPULATION capability
        """
        if self._robot is None:
            return None

        if not self._robot.has_capability(RobotCapability.MANIPULATION):
            raise RuntimeError("This robot does not have manipulation capabilities")

        return self._robot.manipulation_interface
