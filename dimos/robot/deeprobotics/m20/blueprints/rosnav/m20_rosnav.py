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
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.voxels import voxel_mapper
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.nav_msgs import Path
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.std_msgs import Bool
from dimos.navigation.rosnav import ros_nav
from dimos.robot.deeprobotics.m20.connection import m20_connection
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis

# NOS-tuned parameters from domain expert (Lesh):
# - voxel_size=0.05: fine enough for indoor nav (doors ~0.8m wide)
# - publish_interval=1.0: 1Hz voxel grid publish (manageable on RK3588)
# - max_height=0.7: ignore points above 70cm
# - HeightCostConfig: gradient-based terrain slope analysis with continuous costs
#   ignore_noise=0.05 matched to voxel resolution, can_climb=0.25 (~M20 step limit)
m20_rosnav = (
    autoconnect(
        m20_connection(enable_ros=False, enable_lidar=False, lidar_height=0.47),
        voxel_mapper(voxel_size=0.05, publish_interval=1.0, max_height=0.7),
        cost_mapper(config=HeightCostConfig(
            max_height=0.7,
            resolution=0.05,
            ignore_noise=0.05,
            can_climb=0.25,
            smoothing=5.0,
        )),
        ros_nav(),
        websocket_vis(),
    )
    .global_config(
        n_dask_workers=2,
        robot_model="deeprobotics_m20",
        robot_ip="10.21.31.103",
        robot_width=0.45,
        robot_rotation_diameter=0.6,
    )
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
