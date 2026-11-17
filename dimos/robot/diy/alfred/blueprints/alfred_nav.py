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

"""Alfred navigating off the Mid-360 alone, with Point-LIO for odometry and premap relocalization.

dimos run alfred-nav
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.mapping.relocalization.lidar.relocalize import MID360_NAV
from dimos.robot.diy.alfred.blueprints._nav import _alfred_nav, _alfred_rerun_config
from dimos.robot.diy.alfred.blueprints.alfred_hardware import _alfred_lidar_hardware
from dimos.robot.diy.alfred.config import ALFRED_HARDWARE, LIO_BLIND_M
from dimos.visualization.vis_module import vis_module

MID360_MAX_RANGE_METERS = 15.0
"""The Mid-360 reaches far past this, but the voxel map raytraces every point from the
sensor origin, so the far returns cost the most and carve the least reliable space."""


alfred_nav = autoconnect(
    _alfred_lidar_hardware,
    mid360_for_pointlio(lidar_ip=ALFRED_HARDWARE.mid360_ip),
    PointLio.blueprint(blind=LIO_BLIND_M),
    _alfred_nav(map_cloud_topic="lidar", map_max_range=MID360_MAX_RANGE_METERS),
    LocalMapRelocalization.blueprint(world_frame="odom", relocalize=MID360_NAV),
    vis_module(global_config.viewer, rerun_config=_alfred_rerun_config()),
).global_config(n_workers=12, robot_model="alfred")
