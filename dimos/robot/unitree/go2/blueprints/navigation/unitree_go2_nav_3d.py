#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
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

"""Go2 3D nav stack: fastlio odometry, raytraced voxel map, MLS planning, path following.

Expects a mid360 + fastlio running against a lidar mounted on the robot.
The go2's built-in lidar and odometry are remapped aside and unused for
navigation. The map, paths, and robot pose all live in fastlio's world frame.
"""

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection

voxel_size = 0.05
# Height of the head-mounted lidar above the ground while standing.
go2_lidar_height = 0.5

unitree_go2_nav_3d = autoconnect(
    unitree_go2_basic.remappings(
        [
            (GO2Connection, "lidar", "lidar_l1"),
            (GO2Connection, "odom", "odom_go2"),
        ]
    ),
    FastLio2.blueprint(
        host_ip=os.getenv("LIDAR_HOST_IP", "192.168.1.5"),
        lidar_ip=os.getenv("LIDAR_IP", "192.168.1.155"),
        map_freq=-1.0,
    ).remappings([(FastLio2, "global_map", "global_map_fastlio")]),
    RayTracingVoxelMap.blueprint(voxel_size=voxel_size),
    MLSPlannerNative.blueprint(
        world_frame="odom",
        voxel_size=voxel_size,
        robot_height=go2_lidar_height,
    ),
    GoalRelay.blueprint(),
    BasicPathFollower.blueprint(),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2")

__all__ = ["unitree_go2_nav_3d"]
