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

from dimos.navigation.smart_nav.main import smart_nav
from dimos.navigation.smart_nav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smart_nav.modules.far_planner.far_planner import FarPlanner
from dimos.navigation.smart_nav.modules.simple_planner.simple_planner import SimplePlanner


def test_default_smart_nav_disconnects_click_waypoint() -> None:
    blueprint = smart_nav(use_simple_planner=True)

    assert blueprint.remapping_map[(ClickToGoal, "way_point")] == "_click_way_point_unused"
    assert (SimplePlanner, "goal") not in blueprint.remapping_map
    assert (SimplePlanner, "clicked_point") not in blueprint.remapping_map


def test_direct_click_waypoint_bypasses_simple_planner_goal_inputs() -> None:
    blueprint = smart_nav(use_simple_planner=True, direct_click_waypoint=True)

    assert blueprint.remapping_map[(ClickToGoal, "way_point")] == "way_point"
    assert blueprint.remapping_map[(SimplePlanner, "goal")] == "_global_planner_goal_unused"
    assert blueprint.remapping_map[(SimplePlanner, "clicked_point")] == "_simple_clicked_point_unused"


def test_direct_click_waypoint_bypasses_far_planner_goal_input() -> None:
    blueprint = smart_nav(use_simple_planner=False, direct_click_waypoint=True)

    assert blueprint.remapping_map[(ClickToGoal, "way_point")] == "way_point"
    assert blueprint.remapping_map[(FarPlanner, "goal")] == "_global_planner_goal_unused"
