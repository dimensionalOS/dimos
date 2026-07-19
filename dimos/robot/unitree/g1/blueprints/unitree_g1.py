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

"""Default G1 blueprint: GR00T whole-body control + raytracing lidar navigation.

Real hardware runs the GR00T WBC policy (from ``unitree-g1-groot-wbc``) fed by
the shared raytracing nav middle (``_unitree_g1_nav_simple``) via Point-LIO
(MID-360), executed through the coordinator's twist_command. In simulation the
groot blueprint already runs its own voxel-grid nav, so this is just that
blueprint.
"""

from __future__ import annotations

import os
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import unitree_g1_groot_wbc
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_nav_simple import _unitree_g1_nav_simple

# groot-wbc already runs its own voxel-grid nav stack in simulation; on real
# hardware add Point-LIO plus the shared raytracing nav middle.
if global_config.simulation:
    _nav_modules: tuple[Any, ...] = ()
else:
    _nav_modules = (
        PointLio.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
        ),
        _unitree_g1_nav_simple,
    )

unitree_g1 = autoconnect(unitree_g1_groot_wbc, *_nav_modules).global_config(n_workers=12)
