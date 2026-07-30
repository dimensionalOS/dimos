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

"""DimSim-specific semantic-viewpoint navigation semantics."""

import math

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.dimsim.spatial_memory import DimSimSpatialMemorySpec

_CURRENT_VIEWPOINT_THRESHOLD_M = 0.75


class DimSimNavigationSkillContainer(NavigationSkillContainer):
    """Describe semantic-map hits as viewpoints rather than object poses."""

    _spatial_memory: DimSimSpatialMemorySpec

    def _navigate_using_semantic_map(self, query: str) -> str:
        detection_viewpoint = self._spatial_memory.query_detection_viewpoint(query)
        if detection_viewpoint is not None:
            return self._navigate_to_viewpoint(
                detection_viewpoint,
                current_message=(
                    f"The camera viewpoint where the lookout detected '{query}' "
                    "is the current camera viewpoint"
                ),
                found_message=(f"Found the camera viewpoint where the lookout detected '{query}'"),
            )

        results = self._spatial_memory.query_by_text(query)
        if not results:
            return f"No matching location found in semantic map for '{query}'"

        goal_pose = self._get_goal_pose_from_result(results[0])
        if goal_pose is None:
            return f"Found a result for '{query}' but it didn't have a reliable position."

        return self._navigate_to_viewpoint(
            goal_pose,
            current_message=(
                f"The semantic-map match for '{query}' is the current camera viewpoint"
            ),
            found_message=f"Found a prior camera viewpoint matching '{query}'",
        )

    def _navigate_to_viewpoint(
        self,
        goal_pose: PoseStamped,
        *,
        current_message: str,
        found_message: str,
    ) -> str:
        if self._latest_odom is not None:
            viewpoint_distance = math.hypot(
                goal_pose.position.x - self._latest_odom.position.x,
                goal_pose.position.y - self._latest_odom.position.y,
            )
            if viewpoint_distance <= _CURRENT_VIEWPOINT_THRESHOLD_M:
                return (
                    f"{current_message}, not the object's position. No navigation "
                    "was started. Observe the current frame and approach the visible "
                    "object with bounded local movement."
                )

        return self._navigate_to(
            goal_pose,
            f"{found_message}, not a confirmed object position",
        )
