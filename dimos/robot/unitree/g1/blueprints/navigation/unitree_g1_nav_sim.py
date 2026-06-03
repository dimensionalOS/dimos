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

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.robot.unitree.g1.config import G1, G1_LOCAL_PLANNER_PRECOMPUTED_PATHS
from dimos.robot.unitree.g1.g1_rerun import g1_static_robot
from dimos.simulation.unity.module import UnityBridgeModule
from dimos.visualization.vis_module import vis_module

nav_config: dict[str, Any] = dict(
    planner="simple",
    vehicle_height=G1.height_clearance,
    max_speed=2.0,  # m/s, higher than real robot defaults
    terrain_analysis={
        "ground_height_threshold": 0.05,
        "min_relative_z": -1.5,
    },
    terrain_map_ext={
        "decay_time": 120,
    },
    local_planner={
        "paths_dir": str(G1_LOCAL_PLANNER_PRECOMPUTED_PATHS),
        "min_relative_z": -1.5,
        "freeze_ang": 180.0,
        "obstacle_height_threshold": 0.02,
        "publish_free_paths": True,  # turn off visual for better runtime performance
    },
    path_follower={
        # these effect smoothness quite a bit
        "max_acceleration": 2.0,
        "max_yaw_rate": 60.0,
    },
)

unitree_g1_nav_sim = (
    autoconnect(
        UnityBridgeModule.blueprint(
            vehicle_height=G1.height_clearance,
            lock_z=True,
            publish_images=False,
        ),
        create_nav_stack(**nav_config),
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
                # Rate-limit heavy point cloud topics to prevent rerun crashing
                vis_throttle=0.1,
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
