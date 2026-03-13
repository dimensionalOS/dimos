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

"""Blueprint for M20 with ROSNav Docker navigation.

Composes M20Connection (UDP-only) + VoxelGridMapper + CostMapper + ROSNav.
The CMU navigation stack (FASTLIO2 + FAR planner + base_autonomy) runs inside
a Docker container on the NOS; ROSNav bridges DDS topics to dimos transports.
"""

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.nav_msgs import Path
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.std_msgs import Bool
from dimos.navigation.rosnav import ros_nav
from dimos.robot.deeprobotics.m20.connection import m20_connection

m20_rosnav = (
    autoconnect(
        m20_connection(enable_ros=False, enable_lidar=False),
        voxel_mapper(voxel_size=0.1),
        cost_mapper(),
        ros_nav(),
    )
    .global_config(n_dask_workers=4, robot_model="deeprobotics_m20")
    .transports(
        {
            # ROSNav cmd_vel -> M20Connection cmd_vel
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            # ROSNav pointcloud -> VoxelGridMapper lidar (shared channel)
            ("pointcloud", PointCloud2): LCMTransport("/lidar", PointCloud2),
            ("global_pointcloud", PointCloud2): LCMTransport("/map", PointCloud2),
            # Navigation goal and path topics
            ("goal_req", PoseStamped): LCMTransport("/goal_req", PoseStamped),
            ("goal_active", PoseStamped): LCMTransport("/goal_active", PoseStamped),
            ("path_active", Path): LCMTransport("/path_active", Path),
            # Goal lifecycle
            ("goal_reached", Bool): LCMTransport("/goal_reached", Bool),
            ("cancel_goal", Bool): LCMTransport("/cancel_goal", Bool),
        }
    )
)
