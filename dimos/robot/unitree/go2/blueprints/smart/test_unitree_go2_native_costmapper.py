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

from dimos.core.coordination.blueprints import TransportSpec
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.native_costmapper.module import NativeCostMapper
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_native_costmapper import (
    unitree_go2_native_costmapper,
)
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_shm import unitree_go2_shm


def test_unitree_go2_native_costmapper_preserves_shm_stack_behavior() -> None:
    expected_modules = {blueprint.module for blueprint in unitree_go2_shm.active_blueprints}
    expected_modules.remove(CostMapper)
    expected_modules.add(NativeCostMapper)

    actual_modules = {
        blueprint.module for blueprint in unitree_go2_native_costmapper.active_blueprints
    }

    assert actual_modules == expected_modules
    assert (
        unitree_go2_native_costmapper.global_config_overrides
        == unitree_go2_shm.global_config_overrides
    )
    assert unitree_go2_native_costmapper.remapping_map == unitree_go2_shm.remapping_map


def test_unitree_go2_native_costmapper_only_bridges_native_streams_to_lcm() -> None:
    transport_map = unitree_go2_native_costmapper.transport_map

    assert isinstance(transport_map[("color_image", Image)], pSHMTransport)
    assert isinstance(transport_map[("lidar", PointCloud2)], pSHMTransport)

    global_map = transport_map[("global_map", PointCloud2)]
    global_costmap = transport_map[("global_costmap", OccupancyGrid)]
    assert isinstance(global_map, TransportSpec)
    assert global_map.cls is LCMTransport
    assert isinstance(global_costmap, TransportSpec)
    assert global_costmap.cls is LCMTransport
