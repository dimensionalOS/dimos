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

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_vis import unitree_g1_vis
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk

# Underscore-prefixed: a shared sub-blueprint, not a runnable blueprint of its own.
_unitree_g1_onboard = autoconnect(
    PointLio.blueprint(host_ip=G1.lidar_host_ip, lidar_ip=G1.lidar_ip),
    G1HighLevelDdsSdk.blueprint(),
    unitree_g1_vis,
).global_config(n_workers=12, robot_model="unitree_g1")
