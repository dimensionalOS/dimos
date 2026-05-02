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

"""E2E integration test: cross-wall planning through Unity sim (FAR planner).

Verifies that the FAR planner routes through doorways instead of through walls.
Uses the full navigation stack (same blueprint as unitree_g1_nav_sim).
"""

from __future__ import annotations

import pytest

# create_nav_stack pulls in PGO which requires gtsam — skip the whole module
# if it isn't installed.
pytest.importorskip("gtsam")

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.navigation.nav_stack.tests.conftest import (
    CROSS_WALL_LOCAL_PLANNER,
    CROSS_WALL_PATH_FOLLOWER,
    CROSS_WALL_TERRAIN_ANALYSIS,
    run_cross_wall_test,
)
from dimos.robot.unitree.g1.g1_rerun import g1_static_robot
from dimos.simulation.unity.module import UnityBridgeModule
from dimos.visualization.vis_module import vis_module

pytestmark = [pytest.mark.slow, pytest.mark.skipif_in_ci]

# Z-ceiling guard: if the robot's z exceeds this, it went through the
# ceiling/roof — the planner is "cheating" by driving over walls.
# Same threshold as the SimplePlanner test.
MAX_ALLOWED_Z = 2.1


class TestCrossWallPlanning:
    def test_cross_wall_sequence(self):
        blueprint = (
            autoconnect(
                UnityBridgeModule.blueprint(
                    unity_binary="",
                    unity_scene="home_building_1",
                    vehicle_height=1.24,
                ),
                create_nav_stack(
                    terrain_analysis=CROSS_WALL_TERRAIN_ANALYSIS,
                    local_planner=CROSS_WALL_LOCAL_PLANNER,
                    path_follower=CROSS_WALL_PATH_FOLLOWER,
                    far_planner={
                        "sensor_range": 15.0,
                        "is_static_env": True,
                        "converge_dist": 1.5,
                    },
                    record=True,
                ),
                MovementManager.blueprint(),
                vis_module(
                    viewer_backend=global_config.viewer,
                    rerun_config=nav_stack_rerun_config(
                        {
                            "blueprint": UnityBridgeModule.rerun_blueprint,
                            "visual_override": {
                                "world/camera_info": UnityBridgeModule.rerun_suppress_camera_info,
                            },
                            "static": {
                                "world/color_image": UnityBridgeModule.rerun_static_pinhole,
                                "world/tf/robot": g1_static_robot,
                            },
                        }
                    ),
                ),
            )
            .remappings(
                [
                    (UnityBridgeModule, "terrain_map", "terrain_map_ext"),
                ]
            )
            .global_config(n_workers=8, robot_model="unitree_g1", simulation=True)
        )

        run_cross_wall_test(blueprint, label="far", max_z=MAX_ALLOWED_Z)
