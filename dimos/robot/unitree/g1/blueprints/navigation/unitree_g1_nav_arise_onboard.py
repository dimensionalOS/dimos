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

WARNING: This is how AriseSLAM should be used, but it is untested.

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

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.navigation.smart_nav.modules.arise_slam.arise_slam import AriseSLAM
from dimos.navigation.smart_nav.modules.global_map.global_map import GlobalMap
from dimos.navigation.smart_nav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.robot.unitree.g1.blueprints.navigation._smart_nav import _rerun_config, _smart_nav
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk
from dimos.visualization.rerun.bridge import RerunBridgeModule

unitree_g1_nav_arise_onboard = (
    autoconnect(
        Mid360.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
            enable_imu=True,
        ),
        AriseSLAM.blueprint(
            mount=G1.internal_odom_offsets["mid360_link"],
            scan_voxel_size=0.1,
            max_range=50.0,
        ),
        SensorScanGeneration.blueprint(),
        _smart_nav,
        GlobalMap.blueprint(),
        G1HighLevelDdsSdk.blueprint(),
        RerunBridgeModule.blueprint(**_rerun_config),
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
