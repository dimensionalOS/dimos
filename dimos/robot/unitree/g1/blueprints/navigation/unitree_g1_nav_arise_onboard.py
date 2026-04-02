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

"""G1 with AriseSLAM on real hardware.

Uses the C++ AriseSLAM module (feature-based LiDAR-IMU SLAM) instead of
FastLio2. The raw Mid-360 driver provides body-frame point clouds and IMU
data; AriseSLAM produces world-frame registered scans and odometry that feed
the rest of the SmartNav stack.

Data flow:
    Mid360 → raw lidar (body frame) + imu
    → AriseSLAM → registered_scan (world frame) + odometry
    → SensorScanGeneration → TerrainAnalysis → LocalPlanner → PathFollower
    → G1HighLevelDdsSdk
"""

from __future__ import annotations

import os
from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.navigation.smartnav.blueprints._rerun_helpers import (
    global_map_override,
    goal_path_override,
    path_override,
    sensor_scan_override,
    static_floor,
    static_robot,
    terrain_map_ext_override,
    terrain_map_override,
    waypoint_override,
)
from dimos.navigation.smartnav.modules.arise_slam.arise_slam import AriseSLAM
from dimos.navigation.smartnav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smartnav.modules.global_map.global_map import GlobalMap
from dimos.navigation.smartnav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smartnav.modules.path_follower.path_follower import PathFollower
from dimos.navigation.smartnav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.navigation.smartnav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.smartnav.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk
from dimos.visualization.rerun.bridge import RerunBridgeModule, _resolve_viewer_mode


def _rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(origin="world", name="3D"),
    )


_rerun_config = {
    "blueprint": _rerun_blueprint,
    "pubsubs": [LCM()],
    "min_interval_sec": 0.25,
    "visual_override": {
        "world/sensor_scan": sensor_scan_override,
        "world/terrain_map": terrain_map_override,
        "world/terrain_map_ext": terrain_map_ext_override,
        "world/global_map": global_map_override,
        "world/path": path_override,
        "world/way_point": waypoint_override,
        "world/goal_path": goal_path_override,
    },
    "static": {
        "world/floor": static_floor,
        "world/tf/robot": static_robot,
    },
}

unitree_g1_nav_arise_onboard = (
    autoconnect(
        Mid360.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
            enable_imu=True,
        ),
        AriseSLAM.blueprint(
            extra_args=[
                "--scanVoxelSize",
                "0.1",
                "--maxRange",
                "50.0",
            ]
        ),
        SensorScanGeneration.blueprint(),
        TerrainAnalysis.blueprint(
            obstacle_height_thre=0.2,
            max_rel_z=1.5,
            vehicle_height=1.2,
        ),
        TerrainMapExt.blueprint(),
        LocalPlanner.blueprint(
            autonomy_mode=True,
            max_speed=1.0,
            autonomy_speed=1.0,
            obstacle_height_thre=0.2,
            max_rel_z=1.5,
            min_rel_z=-1.5,
        ),
        PathFollower.blueprint(
            autonomy_mode=True,
            max_speed=1.0,
            autonomy_speed=1.0,
            max_accel=2.0,
            slow_dwn_dis_thre=0.2,
        ),
        ClickToGoal.blueprint(),
        GlobalMap.blueprint(),
        G1HighLevelDdsSdk.blueprint(),
        RerunBridgeModule.blueprint(viewer_mode=_resolve_viewer_mode(), **_rerun_config),
    )
    .remappings(
        [
            # Mid360 outputs "lidar" (body frame); AriseSLAM expects "raw_points"
            (Mid360, "lidar", "raw_points"),
        ]
    )
    .global_config(n_workers=8, robot_model="unitree_g1")
)


def main() -> None:
    unitree_g1_nav_arise_onboard.build().loop()


__all__ = ["unitree_g1_nav_arise_onboard"]

if __name__ == "__main__":
    main()
