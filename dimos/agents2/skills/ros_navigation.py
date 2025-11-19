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

import time
from typing import TYPE_CHECKING, Any

from dimos.core.skill_module import SkillModule
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.protocol.skill.skill import skill
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

if TYPE_CHECKING:
    from dimos.robot.unitree_webrtc.unitree_g1 import UnitreeG1

logger = setup_logger(__file__)


# TODO: Remove, deprecated
class RosNavigation(SkillModule):
    _robot: "UnitreeG1"
    _started: bool

    def __init__(self, robot: "UnitreeG1") -> None:
        self._robot = robot
        self._similarity_threshold = 0.23
        self._started = False

    def start(self) -> None:
        self._started = True

    def stop(self) -> None:
        super().stop()

    @skill()
    def navigate_with_text(self, query: str) -> str:
        """Navigate to a location by querying the existing semantic map using natural language.

        CALL THIS SKILL FOR ONE SUBJECT AT A TIME. For example: "Go to the person wearing a blue shirt in the living room",
        you should call this skill twice, once for the person wearing a blue shirt and once for the living room.

        Args:
            query: Text query to search for in the semantic map
        """

        if not self._started:
            raise ValueError(f"{self} has not been started.")

        success_msg = self._navigate_using_semantic_map(query)
        if success_msg:
            return success_msg

        return "Failed to navigate."

    def _navigate_using_semantic_map(self, query: str) -> str:
        results = self._robot.spatial_memory.query_by_text(query)

        if not results:
            return f"No matching location found in semantic map for '{query}'"

        best_match = results[0]

        goal_pose = self._get_goal_pose_from_result(best_match)

        if not goal_pose:
            return f"Found a result for '{query}' but it didn't have a valid position."

        result = self._robot.nav.go_to(goal_pose)

        if not result:
            return f"Failed to navigate for '{query}'"

        return f"Successfuly arrived at '{query}'"

    @skill()
    def stop_movement(self) -> str:
        """Immediatly stop moving."""

        if not self._started:
            raise ValueError(f"{self} has not been started.")

        self._robot.cancel_navigation()

        return "Stopped"

    def _get_goal_pose_from_result(self, result: dict[str, Any]) -> PoseStamped | None:
        similarity = 1.0 - (result.get("distance") or 1)
        if similarity < self._similarity_threshold:
            logger.warning(
                f"Match found but similarity score ({similarity:.4f}) is below threshold ({self._similarity_threshold})"
            )
            return None

        metadata = result.get("metadata")
        if not metadata:
            return None

        first = metadata[0]
        pos_x = first.get("pos_x", 0)
        pos_y = first.get("pos_y", 0)
        theta = first.get("rot_z", 0)

        return PoseStamped(
            ts=time.time(),
            position=make_vector3(pos_x, pos_y, 0),
            orientation=euler_to_quaternion(make_vector3(0, 0, theta)),
            frame_id="map",
        )


ros_navigation_skill = RosNavigation.blueprint

__all__ = ["RosNavigation", "ros_navigation_skill"]
