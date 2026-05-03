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

import pytest

from dimos.navigation.smart_nav.main import smart_nav
from dimos.navigation.smart_nav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smart_nav.modules.far_planner.far_planner import FarPlanner
from dimos.navigation.smart_nav.modules.pgo.pgo import PGO
from dimos.navigation.smart_nav.modules.simple_planner.simple_planner import SimplePlanner
from dimos.navigation.smart_nav.modules.terrain_analysis.terrain_analysis import (
    TerrainAnalysis,
)


def test_default_smart_nav_disconnects_click_waypoint() -> None:
    blueprint = smart_nav(use_simple_planner=True)

    assert blueprint.remapping_map[(ClickToGoal, "way_point")] == "_click_way_point_unused"
    assert (SimplePlanner, "goal") not in blueprint.remapping_map
    assert (SimplePlanner, "clicked_point") not in blueprint.remapping_map
    assert blueprint.remapping_map[(SimplePlanner, "odometry")] == "corrected_odometry"
    assert blueprint.remapping_map[(ClickToGoal, "odometry")] == "corrected_odometry"
    assert blueprint.remapping_map[(TerrainAnalysis, "odometry")] == "corrected_odometry"
    assert blueprint.remapping_map[(PGO, "global_map")] == "global_map_pgo"


def test_direct_click_waypoint_bypasses_simple_planner_goal_inputs() -> None:
    blueprint = smart_nav(use_simple_planner=True, direct_click_waypoint=True)

    assert blueprint.remapping_map[(ClickToGoal, "way_point")] == "way_point"
    assert blueprint.remapping_map[(SimplePlanner, "goal")] == "_global_planner_goal_unused"
    assert (
        blueprint.remapping_map[(SimplePlanner, "clicked_point")] == "_simple_clicked_point_unused"
    )


def test_direct_click_waypoint_bypasses_far_planner_goal_input() -> None:
    blueprint = smart_nav(use_simple_planner=False, direct_click_waypoint=True)

    assert blueprint.remapping_map[(ClickToGoal, "way_point")] == "way_point"
    assert blueprint.remapping_map[(FarPlanner, "goal")] == "_global_planner_goal_unused"


def test_can_disable_pgo_and_keep_planners_on_raw_odometry() -> None:
    blueprint = smart_nav(use_simple_planner=True, use_pgo=False)

    assert (PGO, "global_map") not in blueprint.remapping_map
    assert (SimplePlanner, "odometry") not in blueprint.remapping_map
    assert (ClickToGoal, "odometry") not in blueprint.remapping_map
    assert (TerrainAnalysis, "odometry") not in blueprint.remapping_map


def test_can_publish_pgo_without_feeding_corrected_odometry_to_planners() -> None:
    blueprint = smart_nav(
        use_simple_planner=True,
        use_pgo=True,
        use_pgo_corrected_odometry=False,
    )

    assert blueprint.remapping_map[(PGO, "global_map")] == "global_map_pgo"
    assert (SimplePlanner, "odometry") not in blueprint.remapping_map
    assert (ClickToGoal, "odometry") not in blueprint.remapping_map
    assert (TerrainAnalysis, "odometry") not in blueprint.remapping_map


def test_corrected_odometry_requires_pgo() -> None:
    with pytest.raises(ValueError, match="use_pgo_corrected_odometry"):
        smart_nav(use_pgo=False, use_pgo_corrected_odometry=True)
