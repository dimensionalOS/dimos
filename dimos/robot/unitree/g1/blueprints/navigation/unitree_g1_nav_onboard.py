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

"""G1 nav onboard — FAR planner + PGO loop closure + local obstacle avoidance.

Full navigation stack on real hardware with:
- FAR visibility-graph global route planner
- PGO pose graph optimization with loop closure detection (GTSAM iSAM2)
- Local planner for reactive obstacle avoidance
- Path follower for velocity control
- FastLio2 SLAM from Livox Mid-360 lidar
- G1HighLevelDdsSdk for robot velocity commands

Odometry routing (per CMU ICRA 2022 Fig. 11):
- Local path modules (LocalPlanner, PathFollower, SensorScanGen):
  use raw odometry — they follow paths in the local odometry frame.
- Global/terrain modules (FarPlanner, ClickToGoal, TerrainAnalysis):
  use PGO corrected_odometry — they need globally consistent positions
  for terrain classification, visibility graphs, and goal coordinates.

Data flow:
    Click → ClickToGoal (corrected_odom) → goal → FarPlanner (corrected_odom)
    → way_point → LocalPlanner (raw odom) → path → PathFollower (raw odom)
    → nav_cmd_vel → CmdVelMux → cmd_vel → G1HighLevelDdsSdk

    registered_scan + odometry → PGO → corrected_odometry + global_map
"""

from __future__ import annotations

import os

from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.navigation.smart_nav.main import smart_nav, smart_nav_rerun_config
from dimos.navigation.smart_nav.modules.pgo.pgo import PGO
from dimos.robot.unitree.g1.blueprints.navigation.g1_rerun import (
    g1_odometry_tf_override,
    g1_static_robot,
)
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

unitree_g1_nav_onboard = (
    autoconnect(
        FastLio2.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
            mount=G1.internal_odom_offsets["mid360_link"],
            map_freq=1.0,
            config="lio_autonomy.yaml",
        ),
        smart_nav(
            use_simple_planner=True,
            vehicle_height=G1.height_clearance,
            # path_follower={"omni_dir_goal_threshold": 0.0},
            terrain_analysis={
                "obstacle_height_threshold": 0.01,
                "ground_height_threshold": 0.01,
                "max_relative_z": 0.3,
                "min_relative_z": -1.5,
            },
            local_planner={
                "max_speed": 2.0,
                "autonomy_speed": 2.0,
                "obstacle_height_threshold": 0.05,
                "max_relative_z": 0.3,
                "min_relative_z": -1.5,
                "freeze_ang": 180.0,
                "two_way_drive": False,
            },
            path_follower={
                "max_speed": 1.6,
                "autonomy_speed": 1.6,
                "max_acceleration": 3.5,
                "slow_down_distance_threshold": 0.5,
                "omni_dir_goal_threshold": 0.5,
                "two_way_drive": False,
            },
            simple_planner={
                "cell_size": 0.2,
                "obstacle_height_threshold": 0.16,
                "inflation_radius": 0.2,
                "lookahead_distance": 2.0,
                "replan_rate": 5.0,
                "replan_cooldown": 2.0,
            },
            far_planner={
                "sensor_range": 15.0,
                "is_static_env": False,
                "converge_dist": 1.5,
            },
        ),
        G1HighLevelDdsSdk.blueprint(),
        RerunBridgeModule.blueprint(
            **smart_nav_rerun_config(
                {
                    "visual_override": {"world/odometry": g1_odometry_tf_override},
                    "static": {"world/tf/robot": g1_static_robot},
                    "memory_limit": "1GB",
                }
            )
        ),
        RerunWebSocketServer.blueprint(),
    )
    .remappings(
        [
            # FastLio2 outputs "lidar"; SmartNav modules expect "registered_scan"
            (FastLio2, "lidar", "registered_scan"),
            (FastLio2, "global_map", "global_map_fastlio"),
        ]
    )
    .global_config(n_workers=12, robot_model="unitree_g1")
)


def main() -> None:
    unitree_g1_nav_onboard.build().loop()


__all__ = ["unitree_g1_nav_onboard"]

if __name__ == "__main__":
    main()
