#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""G1 nav sim — SimplePlanner + PGO loop closure + local obstacle avoidance.

Full navigation stack with:
- SimplePlanner grid-based A* global route planner
- PGO pose graph optimization with loop closure detection (GTSAM iSAM2)
- Local planner for reactive obstacle avoidance
- Path follower for velocity control

Odometry routing (per CMU ICRA 2022 Fig. 11):
- Local path modules (LocalPlanner, PathFollower, SensorScanGen):
  use raw odometry — they follow paths in the local odometry frame.
- Global/terrain modules (SimplePlanner, MovementManager, TerrainAnalysis):
  use PGO corrected_odometry — they need globally consistent positions
  for terrain classification, costmap building, and goal coordinates.

Data flow:
    Click → MovementManager (corrected_odom) → goal → SimplePlanner (corrected_odom)
    → way_point → LocalPlanner (raw odom) → path → PathFollower (raw odom)
    → nav_cmd_vel → MovementManager → cmd_vel → UnityBridgeModule

    registered_scan + odometry → PGO → corrected_odometry + global_map
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.robot.unitree.g1.config import G1_LOCAL_PLANNER_PRECOMPUTED_PATHS
from dimos.robot.unitree.g1.g1_rerun import g1_static_robot
from dimos.simulation.unity.module import UnityBridgeModule
from dimos.visualization.vis_module import vis_module


def _rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                # start in a top-down view and full-screened
                eye_controls=rrb.EyeControls3D(
                    position=(0.0, 0.0, 20.0),
                    look_target=(0.0, 0.0, 0.0),
                    eye_up=(0.0, 0.0, 1.0),
                ),
            ),
        ),
        collapse_panels=True,
    )


vehicle_height = 1.24
unitree_g1_nav_sim = (
    autoconnect(
        UnityBridgeModule.blueprint(
            unity_binary="",
            unity_scene="home_building_1",
            vehicle_height=vehicle_height,
        ),
        create_nav_stack(
            use_simple_planner=False,
            vehicle_height=vehicle_height,
            terrain_analysis={
                "ground_height_threshold": 0.05,
                "min_relative_z": -1.5,
            },
            local_planner={
                "paths_dir": str(G1_LOCAL_PLANNER_PRECOMPUTED_PATHS),
                # Sim uses higher speeds than the real robot defaults
                "max_speed": 2.0,
                "autonomy_speed": 2.0,
                "min_relative_z": -1.5,
                "freeze_ang": 180.0,
            },
            path_follower={
                # Sim uses higher speeds than the real robot defaults
                "max_speed": 2.0,
                "autonomy_speed": 2.0,
                "max_acceleration": 4.0,
                "max_yaw_rate": 80.0,
            },
        ),
        MovementManager.blueprint(),
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=nav_stack_rerun_config(
                {
                    "blueprint": _rerun_blueprint,
                    "visual_override": {
                        "world/camera_info": UnityBridgeModule.rerun_suppress_camera_info,
                    },
                    "static": {
                        "world/color_image": UnityBridgeModule.rerun_static_pinhole,
                        "world/tf/robot": g1_static_robot,
                    },
                    # Rate-limit heavy point cloud topics to prevent
                    # rerun viewer backpressure crashes.
                    "max_hz": {
                        "world/registered_scan": 2.0,
                        "world/terrain_map": 2.0,
                        "world/terrain_map_ext": 1.0,
                        "world/global_map_pgo": 1.0,
                        "world/costmap_cloud": 2.0,
                        "world/obstacle_cloud": 2.0,
                        "world/free_paths": 2.0,
                    },
                }
            ),
        ),
    )
    .remappings(
        [
            # Unity needs the extended (persistent) terrain map for Z-height, not the local one
            (UnityBridgeModule, "terrain_map", "terrain_map_ext"),
            # Planner owns way_point — disconnect MovementManager's click relay
            (MovementManager, "way_point", "_mgr_way_point_unused"),
        ]
    )
    .global_config(n_workers=8, robot_model="unitree_g1", simulation=True)
)
