#!/usr/bin/env python3
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

"""Demo runner: simple cross-wall sim with agentic_debug rerun config.

Builds the same blueprint as unitree_g1_nav_sim but passes
``agentic_debug=True`` to ``nav_stack_rerun_config`` so nav markers +
the PGO pose graph render lifted above terrain (``_AGENTIC_DEBUG_LIFT``,
3m). Used to validate the PGO pose-graph publication added on
``jeff/feat/better_pgo`` by visual inspection (screenshot dimos-viewer
mid-run).

Run manually:
    source .venv/bin/activate
    uv run python -m dimos.navigation.nav_stack.demo_better_pgo_viz
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.navigation.nav_stack.tests.conftest import run_cross_wall_test
from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_nav_sim import nav_config
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.g1_rerun import g1_static_robot
from dimos.simulation.unity.module import UnityBridgeModule
from dimos.visualization.vis_module import vis_module


def build_blueprint() -> Blueprint:
    return (
        autoconnect(
            UnityBridgeModule.blueprint(
                vehicle_height=G1.height_clearance,
                lock_z=True,
                publish_images=False,
            ),
            create_nav_stack(**{**nav_config, "planner": "simple"}),
            MovementManager.blueprint(),
            vis_module(
                viewer_backend=global_config.viewer,
                rerun_config=nav_stack_rerun_config(
                    {
                        "visual_override": {
                            "world/camera_info": UnityBridgeModule.rerun_suppress_camera_info,
                        },
                        "static": {
                            "world/color_image": UnityBridgeModule.rerun_static_pinhole,
                            "world/tf/robot": g1_static_robot,
                        },
                    },
                    agentic_debug=True,
                    vis_throttle=0.1,
                ),
            ),
        )
        .remappings(
            [
                (UnityBridgeModule, "terrain_map", "terrain_map_ext"),
                (MovementManager, "way_point", "_mgr_way_point_unused"),
            ]
        )
        .global_config(dtop=True, n_workers=8, robot_model="unitree_g1", simulation=True)
    )


if __name__ == "__main__":
    run_cross_wall_test(build_blueprint(), label="simple-better-pgo")
