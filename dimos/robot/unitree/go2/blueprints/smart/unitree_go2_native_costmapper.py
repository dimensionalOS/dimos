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

"""Keyboard-safe Go2 SHM stack using the native Rust costmapper."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.native_costmapper.module import NativeCostMapper
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_shm import unitree_go2_shm

unitree_go2_native_costmapper = autoconnect(
    unitree_go2_shm.disabled_modules(CostMapper),
    NativeCostMapper.blueprint(),
).transports(
    {
        # NativeModule subprocesses use the global LCM/Zenoh backend and cannot
        # attach to Python pSHM channels. Keep every other SHM override intact.
        ("global_map", PointCloud2): LCMTransport.spec("/global_map", PointCloud2),
        ("global_costmap", OccupancyGrid): LCMTransport.spec("/global_costmap", OccupancyGrid),
    }
)
