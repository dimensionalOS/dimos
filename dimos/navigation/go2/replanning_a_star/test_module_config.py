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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import GlobalConfig
from dimos.navigation.experimental.patrolling.module import PatrollingConfig, PatrollingModule
from dimos.navigation.go2.replanning_a_star.module import (
    ReplanningAStarPlanner,
    ReplanningAStarPlannerConfig,
)


def test_robot_geometry_comes_from_the_shared_global_unless_overridden() -> None:
    g = GlobalConfig(robot_width=0.5, robot_rotation_diameter=0.9)

    assert ReplanningAStarPlannerConfig(g=g).robot_width == 0.5
    assert PatrollingConfig(g=g).robot_rotation_diameter == 0.9
    assert ReplanningAStarPlannerConfig(g=g, robot_width=0.4).robot_width == 0.4


def test_one_global_flag_reaches_the_planner_and_the_patrol_module() -> None:
    parser = BlueprintConfigParser(
        autoconnect(ReplanningAStarPlanner.blueprint(), PatrollingModule.blueprint())
    )
    parsed = parser.parse(["--robot-width", "0.5"], environ={})
    g = GlobalConfig(**parsed.global_config_values())

    assert g.robot_width == 0.5
    assert (
        ReplanningAStarPlannerConfig(
            g=g, **parsed.module_kwargs("replanningastarplanner")
        ).robot_width
        == 0.5
    )
    assert PatrollingConfig(g=g, **parsed.module_kwargs("patrollingmodule")).robot_width == 0.5
