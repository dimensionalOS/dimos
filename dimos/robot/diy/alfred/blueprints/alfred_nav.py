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

import os
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack, nav_stack_rerun_config
from dimos.robot.diy.alfred.config import ALFRED, LOCAL_PLANNER_PRECOMPUTED_PATHS
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.visualization.vis_module import vis_module

nav_config: dict[str, Any] = dict(
    planner="simple",
    vehicle_height=0.5,
    max_speed=0.8,
    terrain_analysis={
        "obstacle_height_threshold": 0.15,
        "ground_height_threshold": 0.10,
        "sensor_range": 20,
    },
    local_planner={
        "paths_dir": str(LOCAL_PLANNER_PRECOMPUTED_PATHS),
        "publish_free_paths": False,
    },
    simple_planner={
        "cell_size": 0.2,
        "obstacle_height_threshold": 0.15,
        "inflation_radius": 0.3,
        "lookahead_distance": 2.0,
        "replan_rate": 5.0,
        "replan_cooldown": 2.0,
    },
)

alfred_nav = (
    autoconnect(
        FastLio2.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.1.5"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.1.189"),
            mount=ALFRED.internal_odom_offsets["mid360_link"],
            map_freq=1.0,
            config="default.yaml",
        ),
        create_nav_stack(**nav_config),
        MovementManager.blueprint(),
        AlfredHighLevel.blueprint(),
        vis_module(
            global_config.viewer,
            rerun_config={
                **nav_stack_rerun_config({"memory_limit": "1GB"}, vis_throttle=0.5),
                "rerun_open": "native",
            },
        ),
    )
    .remappings(
        [
            # nav stack needs "registered_scan"
            (FastLio2, "lidar", "registered_scan"),
            (FastLio2, "global_map", "global_map_fastlio"),
            # SimplePlanner / FarPlanner owns way_point — disconnect MovementManager's
            (MovementManager, "way_point", "_mgr_way_point_unused"),
        ]
    )
    .global_config(n_workers=8)
)
