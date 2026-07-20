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

from dimos.core.transport import pSHMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_shm import unitree_go2_shm


def test_unitree_go2_shm_isolates_sensor_traffic_from_lcm_controls() -> None:
    shared_memory_streams = {
        ("color_image", Image),
        ("lidar", PointCloud2),
        ("global_map", PointCloud2),
        ("global_costmap", OccupancyGrid),
    }

    assert set(unitree_go2_shm.transport_map) == shared_memory_streams
    assert all(
        isinstance(unitree_go2_shm.transport_map[key], pSHMTransport)
        for key in shared_memory_streams
    )
    assert ("tele_cmd_vel", Twist) not in unitree_go2_shm.transport_map
    assert ("cmd_vel", Twist) not in unitree_go2_shm.transport_map
    assert unitree_go2_shm.global_config_overrides["transport"] == "lcm"
