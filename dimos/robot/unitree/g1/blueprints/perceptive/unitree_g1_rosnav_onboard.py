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

"""G1 with ROSNav in hardware mode + replanning A* local planner."""

import os

from dimos.core.blueprints import autoconnect
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.navigation.rosnav.rosnav_module import ROSNav
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_onboard import unitree_g1_onboard
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

unitree_g1_rosnav_onboard = (
    autoconnect(
        unitree_g1_onboard,
        replanning_a_star_planner(),
        ROSNav.blueprint(
            mode="hardware",
            vehicle_height=1.24,
            unitree_ip=os.getenv("ROBOT_IP", "192.168.12.1"),
            unitree_conn=os.getenv("ROSNAV_UNITREE_CONN", "LocalAP"),
            lidar_interface=os.getenv("ROSNAV_LIDAR_INTERFACE", "eth0"),
            lidar_computer_ip=os.getenv("ROSNAV_LIDAR_COMPUTER_IP", "192.168.123.5"),
            lidar_gateway=os.getenv("ROSNAV_LIDAR_GATEWAY", "192.168.123.1"),
            lidar_ip=os.getenv("ROSNAV_LIDAR_IP", "192.168.123.120"),
        ),
    )
    .remappings(
        [
            (WebsocketVisModule, "cmd_vel", "teleop_cmd_vel"),
        ]
    )
    .global_config(n_workers=8, robot_model="unitree_g1")
)

__all__ = ["unitree_g1_rosnav_onboard"]
