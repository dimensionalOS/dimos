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

"""Generated navigation goals and named-location orientation round trips."""

import math

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped
import pytest

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.msgs.geometry import quaternion_euler, quaternion_from_euler


@pytest.fixture
def skill(mocker):
    value = NavigationSkillContainer()
    memory = mocker.Mock()
    navigator = mocker.Mock()
    mocker.patch.object(value, "_spatial_memory", memory, create=True)
    mocker.patch.object(value, "_navigation", navigator, create=True)
    value._skill_started = True
    try:
        yield value, memory, navigator
    finally:
        value.stop()


def test_tagged_location_preserves_euler_orientation_through_cdr_goal(skill):
    value, memory, navigator = skill
    angles = (0.2, -0.3, math.pi / 2)
    pose = PoseStamped(
        pose=Pose(position=Point(x=1, y=2, z=3), orientation=quaternion_from_euler(*angles))
    )
    value._on_odom(PoseStamped.decode(pose.encode()))
    assert "Tagged 'desk'" in value.tag_location("desk")
    tagged = memory.tag_location.call_args.args[0]
    assert tagged.position == (1, 2, 3)
    assert tagged.rotation == pytest.approx(angles)
    memory.query_tagged_location.return_value = tagged
    assert "Found a tagged location" in value._navigate_by_tagged_location("desk")
    goal = navigator.set_goal.call_args.args[0]
    decoded = PoseStamped.decode(goal.encode())
    assert decoded.header.frame_id == "map"
    assert decoded.pose.position == pose.pose.position
    assert quaternion_euler(decoded.pose.orientation) == pytest.approx(angles)


def test_semantic_result_builds_generated_map_goal(skill):
    value, _, navigator = skill
    result = {"distance": 0.1, "metadata": [{"pos_x": 4, "pos_y": -2, "rot_z": math.pi / 2}]}
    goal = value._get_goal_pose_from_result(result)
    assert goal is not None
    value._navigate_to(goal, "Found desk")
    decoded = PoseStamped.decode(navigator.set_goal.call_args.args[0].encode())
    assert decoded.header.frame_id == "map"
    assert decoded.pose.position == Point(x=4, y=-2)
    assert quaternion_euler(decoded.pose.orientation) == pytest.approx((0, 0, math.pi / 2))
    assert value._get_goal_pose_from_result({"distance": 0.9}) is None
