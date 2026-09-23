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

"""Show a named location becoming a generated navigation goal without moving hardware."""

import math
from unittest.mock import Mock, patch

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped
import numpy as np

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.msgs.geometry import quaternion_euler, quaternion_from_euler


def main() -> None:
    skill = NavigationSkillContainer()
    memory, navigator = Mock(), Mock()
    try:
        with (
            patch.object(skill, "_spatial_memory", memory, create=True),
            patch.object(skill, "_navigation", navigator, create=True),
        ):
            skill._skill_started = True
            original = PoseStamped(
                pose=Pose(
                    position=Point(x=1, y=2, z=3),
                    orientation=quaternion_from_euler(0.2, -0.3, math.pi / 2),
                )
            )
            skill._on_odom(PoseStamped.decode(original.encode()))
            print(skill.tag_location("desk"))
            location = memory.tag_location.call_args.args[0]
            memory.query_tagged_location.return_value = location
            print(skill._navigate_by_tagged_location("desk"))
            goal = PoseStamped.decode(navigator.set_goal.call_args.args[0].encode())
            np.testing.assert_allclose(
                quaternion_euler(goal.pose.orientation), (0.2, -0.3, math.pi / 2)
            )
            assert goal.pose.position == original.pose.position
            print(f"Generated CDR goal: frame={goal.header.frame_id}, position=(1, 2, 3)")
            print(f"Preserved roll, pitch, yaw: {quaternion_euler(goal.pose.orientation)}")
            print("Memory and navigation RPCs stubbed; no inference or hardware motion.")
    finally:
        skill.stop()


if __name__ == "__main__":
    main()
