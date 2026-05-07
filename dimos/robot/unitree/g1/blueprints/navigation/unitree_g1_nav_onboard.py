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

"""G1 nav onboard blueprint: FAR planner + PGO + local obstacle avoidance."""

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.robot.unitree.g1.config import G1, G1_LOCAL_PLANNER_PRECOMPUTED_PATHS
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk
from dimos.robot.unitree.g1.g1_rerun import (
    g1_odometry_tf_override,
    g1_static_robot,
)
from dimos.visualization.vis_module import vis_module

unitree_g1_nav_onboard = (
    autoconnect(
        FastLio2.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
            mount=G1.internal_odom_offsets["mid360_link"],
            map_freq=1.0,
            config="default.yaml",
        ),
        create_nav_stack(
            use_simple_planner=True,
            vehicle_height=G1.height_clearance,
            max_speed=0.5,
            terrain_analysis={
                "no_decay_distance": 0,
                "obstacle_height_threshold": 0.01,
                "ground_height_threshold": 0.01,
                "sensor_range": 40, # meters
            },
            local_planner={
                "paths_dir": str(G1_LOCAL_PLANNER_PRECOMPUTED_PATHS),
                "publish_free_paths": False,
            },
            simple_planner={
                "cell_size": 0.2,
                "obstacle_height_threshold": 0.10,
                "inflation_radius": 0.5,
                "lookahead_distance": 2.0,
                "replan_rate": 5.0,
                "replan_cooldown": 2.0,
            },
        ),
        MovementManager.blueprint(),
        G1HighLevelDdsSdk.blueprint(),
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=nav_stack_rerun_config(
                {
                    "visual_override": {"world/odometry": g1_odometry_tf_override},
                    "static": {"world/tf/robot": g1_static_robot},
                    "memory_limit": "1GB",
                }
            ),
        ),
    )
    .remappings(
        [
            # FastLio2 outputs "lidar"; SmartNav modules expect "registered_scan"
            (FastLio2, "lidar", "registered_scan"),
            (FastLio2, "global_map", "global_map_fastlio"),
            # Planner owns way_point — disconnect MovementManager's click relay
            (MovementManager, "way_point", "_mgr_way_point_unused"),
        ]
    )
    .global_config(n_workers=12, robot_model="unitree_g1")
)


__all__ = ["unitree_g1_nav_onboard"]
