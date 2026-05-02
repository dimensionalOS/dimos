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

"""E2E integration test: cross-wall planning using SimplePlanner.

Mirrors ``test_cross_wall_planning_far.py`` but swaps FarPlanner for
SimplePlanner (grid A*). Same blueprint, same waypoint sequence, same
success thresholds — apples-to-apples comparison plus a z-ceiling guard
to catch the robot climbing geometry.
"""

from __future__ import annotations

import pytest

# create_nav_stack pulls in PGO which requires gtsam — skip the whole module
# if it isn't installed.
pytest.importorskip("gtsam")

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack
from dimos.navigation.nav_stack.tests.conftest import (
    CROSS_WALL_LOCAL_PLANNER,
    CROSS_WALL_PATH_FOLLOWER,
    CROSS_WALL_TERRAIN_ANALYSIS,
    run_cross_wall_test,
)
from dimos.simulation.unity.module import UnityBridgeModule

pytestmark = [pytest.mark.slow, pytest.mark.skipif_in_ci]

# If the robot's z ever exceeds this, it has gone through the ceiling /
# climbed on top of geometry — navigation is broken. The sim's terrain-z
# estimate drifts ~0.3 m near walls (wall points within the 0.5 m terrain
# sampling radius pull the ground estimate upward), so this must tolerate
# vehicle_height (1.24 m) + terrain drift while still catching
# through-the-roof failures (roof is at ~3 m+).
MAX_ALLOWED_Z = 2.1


class TestCrossWallPlanningSimple:
    """E2E: cross-wall routing with SimplePlanner (A* on 2D costmap)."""

    def test_cross_wall_sequence_simple(self):
        blueprint = (
            autoconnect(
                UnityBridgeModule.blueprint(
                    unity_binary="",
                    unity_scene="home_building_1",
                    vehicle_height=1.24,
                ),
                create_nav_stack(
                    use_simple_planner=True,
                    terrain_analysis=CROSS_WALL_TERRAIN_ANALYSIS,
                    local_planner=CROSS_WALL_LOCAL_PLANNER,
                    path_follower=CROSS_WALL_PATH_FOLLOWER,
                    simple_planner={
                        "cell_size": 0.3,
                        "obstacle_height_threshold": 0.15,
                        "inflation_radius": 0.7,
                        "lookahead_distance": 2.0,
                        "replan_rate": 5.0,
                        # Tighten stuck-detection for the test so doorways
                        # that the wider inflation blocks get opened up
                        # within a few seconds of non-progress.
                        "stuck_seconds": 4.0,
                        "stuck_shrink_factor": 0.5,
                    },
                ),
                MovementManager.blueprint(),
            )
            .remappings(
                [
                    (UnityBridgeModule, "terrain_map", "terrain_map_ext"),
                ]
            )
            .global_config(n_workers=8, robot_model="unitree_g1", simulation=True)
        )

        run_cross_wall_test(blueprint, label="simple", max_z=MAX_ALLOWED_Z)
