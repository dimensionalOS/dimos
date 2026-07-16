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

"""Local Go2 stack with a dedicated LCM control plane.

Camera, lidar, and mapping payloads use latest-frame shared-memory channels so
they cannot saturate LCM and starve browser keyboard commands. This blueprint
is intended for a simulator or robot stack running on the same machine as its
mapping and visualization modules.
"""

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.transport import pSHMTransport
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

# Point clouds grow as the robot explores. These capacities cover the raw lidar
# stream and up to roughly four million xyz float32 points in the global map.
_LIDAR_CAPACITY = 16 * 1024 * 1024
_GLOBAL_MAP_CAPACITY = 64 * 1024 * 1024
_COSTMAP_CAPACITY = 16 * 1024 * 1024


unitree_go2_shm = unitree_go2.transports(
    {
        ("color_image", Image): pSHMTransport(
            "/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
        ),
        ("lidar", PointCloud2): pSHMTransport("/lidar", default_capacity=_LIDAR_CAPACITY),
        ("global_map", PointCloud2): pSHMTransport(
            "/global_map", default_capacity=_GLOBAL_MAP_CAPACITY
        ),
        ("global_costmap", OccupancyGrid): pSHMTransport(
            "/global_costmap", default_capacity=_COSTMAP_CAPACITY
        ),
    }
).global_config(transport="lcm")
