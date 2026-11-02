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

"""The ray tracer that warps its map through each loop closure in place."""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.stream import In
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap, RayTracingVoxelMapConfig
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class ReformRayTracingVoxelMapConfig(RayTracingVoxelMapConfig):
    executable: str = "result/bin/reform_ray_tracing"

    # Spatial width of the blend between keyframe corrections, m. 0 picks
    # twice the mean keyframe spacing.
    reform_blend_sigma_m: float = 0.0
    # Temporal width, s, which keeps the passes of a revisited place apart. A
    # voxel is stamped when it lands, a little after it was seen, so this is
    # wider than the keyframe interval. 0 picks twice the mean interval.
    reform_blend_time_sigma_s: float = 5.0
    # A keyframe whose correction moves its surroundings less than this waits
    # for a later closure. Half a voxel moves no voxel.
    reform_min_shift_m: float = 0.04
    # A warp blends one keyframe per cell of this size (and of the time width in
    # time), m: the one nearest the cell's center. 0 picks the blend width,
    # negative blends every keyframe.
    reform_thin_cell_m: float = 0.0
    # Where to write the module's map state every 10 s for offline checks, empty for never.
    dump_path: str = ""


class ReformRayTracingVoxelMap(RayTracingVoxelMap):
    """RayTracingVoxelMap that follows a pose-graph optimizer's loop closures.

    On each loop_closure_event it warps every voxel into the corrected frame
    with fast-reform, at the time the voxel landed, instead of waiting to
    re-trace a corrected map. pose_graph supplies the keyframes that did not
    move, which hold their surroundings in place. world_frame should be the
    optimizer's corrected frame, so new clouds land where the warp moved the map.
    """

    config: ReformRayTracingVoxelMapConfig

    # One record per keyframe: x, y, z, node_id, ts, qx, qy, qz, qw.
    pose_graph: In[PointCloud2]
    # One record per keyframe a closure moved: its pre pose as in pose_graph,
    # then dx, dy, dz, dqx, dqy, dqz, dqw with post = delta * pre.
    loop_closure_event: In[PointCloud2]


if TYPE_CHECKING:
    ReformRayTracingVoxelMap()
