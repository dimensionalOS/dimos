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

"""G1 basic nav onboard — local planner + path follower only (no FAR/PGO).

Lightweight navigation stack for real hardware: uses SmartNav C++ native
modules for terrain analysis, local planning, and path following.
FastLio2 provides SLAM from a Livox Mid-360 lidar. No global route
planner (FAR) or loop closure (PGO). For the full stack, use
unitree_g1_nav_onboard.
"""

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.navigation.smart_nav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.robot.unitree.g1.blueprints.navigation._smart_nav import _rerun_config, _smart_nav
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

unitree_g1_nav_basic_onboard = (
    autoconnect(
        FastLio2.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
            mount=G1.internal_odom_offsets["mid360_link"],
            map_freq=1.0,  # Publish global map at 1 Hz
        ),
        SensorScanGeneration.blueprint(),
        _smart_nav,
        G1HighLevelDdsSdk.blueprint(),
        RerunBridgeModule.blueprint(**_rerun_config),
        RerunWebSocketServer.blueprint(),
    )
    .remappings(
        [
            # FastLio2 outputs "lidar"; SmartNav modules expect "registered_scan"
            (FastLio2, "lidar", "registered_scan"),
        ]
    )
    .global_config(n_workers=8, robot_model="unitree_g1")
)


def main() -> None:
    unitree_g1_nav_basic_onboard.build().loop()


__all__ = ["unitree_g1_nav_basic_onboard"]

if __name__ == "__main__":
    main()
