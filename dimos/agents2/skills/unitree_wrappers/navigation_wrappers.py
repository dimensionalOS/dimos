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

"""
Navigation skill wrappers for agents2 framework.
These wrap the existing navigation skills using the new @skill decorator.
"""

from typing import Optional
from dimos.protocol.skill.skill import SkillContainer, skill
from dimos.skills.navigation import NavigateWithText as OldNavigateWithText
from dimos.skills.navigation import GetPose as OldGetPose
from dimos.skills.navigation import NavigateToGoal as OldNavigateToGoal
from dimos.skills.navigation import Explore as OldExplore
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.agents2.skills.unitree_wrappers.navigation")


class NavigationSkillContainer(SkillContainer):
    """Container for navigation skills using the new framework."""

    def __init__(self, robot=None):
        """Initialize the navigation skill container.

        Args:
            robot: The robot instance to use for navigation
        """
        super().__init__()
        self._robot = robot

    @skill()
    def navigate_with_text(
        self,
        query: str = "",
        limit: int = 1,
        distance: float = 0.3,
        skip_visual_search: bool = False,
        timeout: float = 40.0,
    ) -> str:
        """Query an existing semantic map using natural language or navigate to an object in view.

        First attempts to locate an object in the robot's camera view using vision.
        If the object is found, navigates to it. If not, falls back to querying the
        semantic map for a location matching the description.

        Args:
            query: Text query to search for in the semantic map
            limit: Maximum number of results to return
            distance: Desired distance to maintain from object in meters
            skip_visual_search: Skip visual search for object in view
            timeout: Maximum time to spend navigating in seconds
        """
        try:
            old_skill = OldNavigateWithText(
                robot=self._robot,
                query=query,
                limit=limit,
                distance=distance,
                skip_visual_search=skip_visual_search,
                timeout=timeout,
            )
            result = old_skill()
            return str(result)
        except Exception as e:
            logger.error(f"Error in navigate_with_text: {e}")
            return f"Error navigating with text: {str(e)}"

    @skill()
    def get_pose(self) -> str:
        """Get the robot's current pose (position and orientation).

        Returns the robot's current location including x, y, z coordinates
        and orientation as roll, pitch, yaw.
        """
        try:
            old_skill = OldGetPose(robot=self._robot)
            result = old_skill()
            return str(result)
        except Exception as e:
            logger.error(f"Error in get_pose: {e}")
            return f"Error getting pose: {str(e)}"

    @skill()
    def navigate_to_goal(
        self, x: float, y: float, theta: Optional[float] = None, timeout: float = 40.0
    ) -> str:
        """Navigate to a specific goal position.

        Args:
            x: X coordinate of the goal in meters
            y: Y coordinate of the goal in meters
            theta: Orientation at goal in radians (optional)
            timeout: Maximum time to spend navigating in seconds
        """
        try:
            old_skill = OldNavigateToGoal(robot=self._robot, x=x, y=y, theta=theta, timeout=timeout)
            result = old_skill()
            return str(result)
        except Exception as e:
            logger.error(f"Error in navigate_to_goal: {e}")
            return f"Error navigating to goal: {str(e)}"

    @skill()
    def explore(self, strategy: str = "random", duration: float = 30.0) -> str:
        """Explore the environment using a specified strategy.

        Args:
            strategy: Exploration strategy ('random', 'frontier', 'systematic')
            duration: How long to explore in seconds
        """
        try:
            old_skill = OldExplore(robot=self._robot, strategy=strategy, duration=duration)
            result = old_skill()
            return str(result)
        except Exception as e:
            logger.error(f"Error in explore: {e}")
            return f"Error exploring: {str(e)}"
