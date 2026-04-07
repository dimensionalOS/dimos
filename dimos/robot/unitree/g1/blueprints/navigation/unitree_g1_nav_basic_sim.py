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

"""G1 basic nav sim — reactive local planner only (no global route planning).

Click-to-navigate sends waypoints directly to the local planner, which
reactively avoids obstacles. Good for short-range navigation but can get
stuck in dead ends. For global route planning, use unitree-g1-nav-sim.
"""

from __future__ import annotations

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.smart_nav.main import smart_nav, smart_nav_rerun_config
from dimos.robot.unitree.g1.blueprints.navigation.g1_rerun import g1_static_robot
from dimos.simulation.unity.module import UnityBridgeModule
from dimos.visualization.vis_module import vis_module

unitree_g1_nav_basic_sim = (
    autoconnect(
        UnityBridgeModule.blueprint(
            unity_binary="",
            unity_scene="home_building_1",
            vehicle_height=1.24,
        ),
        smart_nav(
            terrain_analysis={
                "obstacle_height_threshold": 0.1,
                "ground_height_threshold": 0.05,
                "max_relative_z": 0.3,
                "min_relative_z": -1.5,
            },
            local_planner={
                "max_speed": 2.0,
                "autonomy_speed": 2.0,
                "obstacle_height_threshold": 0.1,
                "max_relative_z": 0.3,
                "min_relative_z": -1.5,
                "freeze_ang": 180.0,
                "two_way_drive": False,
            },
            path_follower={
                "max_speed": 2.0,
                "autonomy_speed": 2.0,
                "max_acceleration": 4.0,
                "slow_down_distance_threshold": 0.5,
                "omni_dir_goal_threshold": 0.5,
                "two_way_drive": False,
            },
            far_planner={
                "sensor_range": 15.0,
                "is_static_env": True,
                "converge_dist": 1.5,
            },
        ),
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=smart_nav_rerun_config(
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
            # Unity needs the extended (persistent) terrain map for Z-height, not the local one
            (UnityBridgeModule, "terrain_map", "terrain_map_ext"),
        ]
    )
    .global_config(n_workers=8, robot_model="unitree_g1", simulation=True)
)


def main() -> None:
    unitree_g1_nav_basic_sim.build().loop()


__all__ = ["unitree_g1_nav_basic_sim"]

if __name__ == "__main__":
    main()
