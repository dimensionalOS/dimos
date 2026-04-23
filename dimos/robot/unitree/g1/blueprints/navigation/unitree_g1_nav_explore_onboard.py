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

"""G1 with TARE autonomous exploration on real hardware.

Zero-ROS navigation stack: TARE frontier-based exploration drives the robot
autonomously through the environment without a user-specified goal. ClickToGoal
is present for visualization but its waypoint output is disconnected so TARE
has exclusive control of LocalPlanner's waypoint input.

Data flow:
    FastLio2 → registered_scan + odometry
    TarePlanner → way_point → LocalPlanner → PathFollower → G1HighLevelDdsSdk
"""

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.navigation.smart_nav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smart_nav.modules.global_map.global_map import GlobalMap
from dimos.navigation.smart_nav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.navigation.smart_nav.modules.tare_planner.tare_planner import TarePlanner
from dimos.robot.unitree.g1.blueprints.navigation._smart_nav import _rerun_config, _smart_nav
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk
from dimos.visualization.rerun.bridge import RerunBridgeModule

unitree_g1_nav_explore_onboard = (
    autoconnect(
        FastLio2.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
            mount=G1.internal_odom_offsets["mid360_link"],
            map_freq=0.0,  # GlobalMap handles accumulation
        ),
        SensorScanGeneration.blueprint(),
        _smart_nav,
        TarePlanner.blueprint(),
        GlobalMap.blueprint(),
        G1HighLevelDdsSdk.blueprint(),
        RerunBridgeModule.blueprint(**_rerun_config),
    )
    .remappings(
        [
            # FastLio2 outputs "lidar"; SmartNav modules expect "registered_scan"
            (FastLio2, "lidar", "registered_scan"),
            # TarePlanner drives way_point to LocalPlanner.
            # Disconnect ClickToGoal's way_point so it doesn't conflict.
            (ClickToGoal, "way_point", "_click_way_point_unused"),
        ]
    )
    .global_config(n_workers=8, robot_model="unitree_g1")
)


def main() -> None:
    unitree_g1_nav_explore_onboard.build().loop()


__all__ = ["unitree_g1_nav_explore_onboard"]

if __name__ == "__main__":
    main()
