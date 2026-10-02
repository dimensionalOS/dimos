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

from dimos.core.coordination.blueprints import Blueprint
from dimos.mapping.ray_tracing.module import RayTracingVoxelMapConfig
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.mapping.relocalization.lidar.relocalize import GO2_NAV
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNativeConfig
from dimos.robot.unitree.go2.constants import BASE_LINK_HEIGHT, ROBOT_HEIGHT

voxel_size = 0.08
wall_clearance_m = 0.1

ray_tracing_config = RayTracingVoxelMapConfig(
    voxel_size=voxel_size, global_emit_every=50, viz_emit_every=5
)

# Every user remaps global_map off. viz_publish_hz is set per blueprint.
mls_planner_config = MLSPlannerNativeConfig(
    voxel_size=voxel_size,
    robot_height=ROBOT_HEIGHT,
    start_z_offset_m=BASE_LINK_HEIGHT,
    wall_clearance_m=wall_clearance_m,
)


def relocalization(republish_loaded_map: float) -> Blueprint:
    """The premap relocalizer every Go2 nav_3d stack shares.

    The republish covers a ray tracer that missed the one-shot loaded_map publish.
    Zero under zenoh, where loaded_map is a never-drop channel.
    """
    return LocalMapRelocalization.blueprint(
        world_frame="odom",
        republish_loaded_map=republish_loaded_map,
        relocalize=GO2_NAV,
    )
