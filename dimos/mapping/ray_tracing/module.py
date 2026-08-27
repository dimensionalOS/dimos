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


from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec import mapping

# Max stamp gap between a cloud and the transform used to register it (s).
# One LIO scan period, so any cloud stamped within a pose sample's period
# can register against it.
TF_MATCH_TOLERANCE_S = 0.1


class RayTracingVoxelMapConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "result/bin/voxel_ray_tracing"
    # Writing flake.lock dirties the tree, so the locked dimos-repo NAR hash never matches
    # the next build.
    build_command: str | None = "nix build -L --no-write-lock-file path:."
    stdin_config: bool = True

    voxel_size: float = 0.1
    # Fine cells per voxel edge for the local_map_fine output: fine cell size is
    # voxel_size / fine_divisor. Zero disables the fine layer.
    fine_divisor: int = 0
    # Maximum range for ray tracing
    max_range: float = 30.0
    # Proportion of points that are ray traced
    # Higher subsample means less tracing
    ray_subsample: int = 1
    # Extend rays past the end point to clear shadows
    shadow_depth: float = 0.1
    # Ignore voxels within this range of points for ray tracing clearing
    grace_depth: float = 0.2
    # Bounds for the health of voxels. Positive health means voxel is occupied.
    min_health: int = -1
    max_health: int = 5
    # Don't clear a miss when abs of ray dot normal is below this, clear it when above.
    # Higher clears only on direct hits, lower clears on slight grazes too.
    graze_cos: float = 0.7
    # Occupied neighbors a surface voxel needs to appear in the local map. Zero
    # emits all. Higher drops isolated returns. The global map is unfiltered.
    support_min: int = 4
    # Publish the accumulated local maps and region bounds every Nth frame.
    # Zero disables them.
    emit_every: int = 1
    # Publish the global map every Nth frame. Zero disables it.
    global_emit_every: int = 1
    # Size the local region to this percentile of batch point distances.
    region_percentile: float = 95.0
    # Fixed frame clouds are registered and published in. Each cloud is placed
    # by the tf lookup world_frame -> cloud frame_id at the cloud stamp.
    world_frame: str = "odom"
    # Max stamp gap between a cloud and the transform used to register it (s).
    tf_match_tolerance_s: float = TF_MATCH_TOLERANCE_S
    # How long to wait for a late transform before dropping a cloud (s).
    tf_wait_timeout_s: float = 0.1
    # Worker threads for parallel map work.
    worker_threads: int = 4


class RayTracingVoxelMap(NativeModule, mapping.GlobalPointcloud):
    """Rust voxel-map module with raycast clearing of dynamic objects."""

    config: RayTracingVoxelMapConfig

    lidar: In[PointCloud2]
    # World-frame points a sensor knows to be empty. Their voxels are deleted
    # outright, reaching space ray tracing cannot clear: a wrist camera's own
    # arm occludes the volume behind it, so no ray ever misses through it.
    voxel_clear_mask: In[PointCloud2]
    tf: In[TFMessage]
    global_map: Out[PointCloud2]
    local_map: Out[PointCloud2]
    local_map_fine: Out[PointCloud2]
    region_bounds: Out[PoseStamped]


if TYPE_CHECKING:
    RayTracingVoxelMap()
