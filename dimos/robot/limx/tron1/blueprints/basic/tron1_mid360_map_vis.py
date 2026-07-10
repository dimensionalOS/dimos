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

"""TRON1 map-visualization stack using a Livox Mid360 + Point-LIO."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.robot.limx.tron1.connection import TRON1Connection
from dimos.visualization.vis_module import vis_module

_VOXEL_SIZE = 0.05
_DEFAULT_CLEARANCE_METERS = 1.2
_DEFAULT_STEP_HEIGHT_METERS = 0.08

tron1_mid360_map_vis = autoconnect(
    TRON1Connection.blueprint(),
    PointLio.blueprint(frame_id="world"),
    RayTracingVoxelMap.blueprint(voxel_size=_VOXEL_SIZE),
    CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=_VOXEL_SIZE,
            can_pass_under=_DEFAULT_CLEARANCE_METERS,
            can_climb=_DEFAULT_STEP_HEIGHT_METERS,
        ),
        initial_safe_radius_meters=0.3,
    ),
    vis_module(viewer_backend=global_config.viewer),
).global_config(
    n_workers=8,
    robot_model="tron1_mid360",
)
