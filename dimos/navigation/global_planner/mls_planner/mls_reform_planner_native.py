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

"""The MLS planner that warps itself through each loop closure in place."""

from __future__ import annotations

from dimos.core.stream import In
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.global_planner.mls_planner.mls_planner_native import (
    MLSPlannerNative,
    MLSPlannerNativeConfig,
)


class MLSReformPlannerNativeConfig(MLSPlannerNativeConfig):
    executable: str = "result/bin/mls_reform_planner"

    # Spatial width of the blend between keyframe corrections, m. 0 picks twice
    # the mean keyframe spacing.
    reform_blend_sigma_m: float = 0.0
    # Temporal width, s, which keeps the passes of a revisited place apart. A
    # voxel is stamped when it lands, a little after it was seen, so this is
    # wider than the keyframe interval. 0 picks twice the mean interval.
    reform_blend_time_sigma_s: float = 5.0
    # A keyframe whose correction moves its surroundings less than this waits
    # for a later closure. Half a voxel moves no voxel.
    reform_min_shift_m: float = 0.04
    # Past this fraction of the voxels moving in one closure, the graph is
    # rebuilt from the warped voxels instead of repaired around them: a loop
    # closure moves most of the map, and repairing nearly all of it costs ~3x a
    # rebuild (huge_loop_pointlio, reform_bench.py).
    reform_rebuild_fraction: float = 0.25


class MLSReformPlannerNative(MLSPlannerNative):
    """MLSPlannerNative that follows a pose-graph optimizer's loop closures.

    On each loop_closure_event it warps every voxel, node, the cached path and
    the active goal into the corrected frame with fast-reform, and repairs the
    graph only around what moved, instead of rebuilding from a corrected map.
    pose_graph supplies the keyframes that did not move, which hold their
    surroundings in place.
    """

    config: MLSReformPlannerNativeConfig

    # One record per keyframe: x, y, z, node_id, ts, qx, qy, qz, qw.
    pose_graph: In[PointCloud2]
    # One record per keyframe a closure moved: its pre pose as in pose_graph,
    # then dx, dy, dz, dqx, dqy, dqz, dqw with post = delta * pre.
    loop_closure_event: In[PointCloud2]
