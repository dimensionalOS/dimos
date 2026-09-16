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

"""The ray tracer and MLS planner settings every Go2 nav_3d blueprint shares."""

from dimos.mapping.ray_tracing.module import RayTracingVoxelMapConfig
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNativeConfig
from dimos.robot.unitree.go2.constants import BASE_LINK_HEIGHT, ROBOT_HEIGHT

voxel_size = 0.08
wall_clearance_m = 0.1

ray_tracing_config = RayTracingVoxelMapConfig(voxel_size=voxel_size, global_emit_every=50)

# global_map is remapped off by every user, so the planner runs purely on the
# incremental local_map + region_bounds pair. viz_publish_hz is per blueprint.
mls_planner_config = MLSPlannerNativeConfig(
    voxel_size=voxel_size,
    robot_height=ROBOT_HEIGHT,
    start_z_offset_m=BASE_LINK_HEIGHT,
    wall_clearance_m=wall_clearance_m,
)
