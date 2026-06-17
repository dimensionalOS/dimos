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

"""Native Rust voxel-map module with raycast clearing.

Subscribes to a world-frame ``PointCloud2`` (e.g. from FastLio2's
``lidar`` output) and matching ``Odometry``, maintains a global
voxel hash set, and publishes the accumulated map on ``global_map``
as a :class:`DynamicCloud` (per-voxel health + slow-clock sequence
stamp).

Algorithm (v1):
    * Insert the voxel of every point into the global hash set.
    * For every point, walk the 3-D DDA ray from the latest
      odometry position to the point and remove every intermediate
      voxel from the map.  The endpoint voxel is preserved.
    * A "slow clock" sequence counter increments every
      ``sequence_period_secs`` (default 1.0s).  Any voxel touched
      while still uncertain (health <= 0) is stamped with the
      current sequence value; once health > 0 the stamp freezes,
      capturing "when did this voxel become confirmed."

Map override:
    Publishing to ``map_override`` with a :class:`DynamicCloud`
    fully replaces the internal voxel state with the override's
    contents.  The slow-clock counter snaps to
    ``max(override.sequence)``, even if that's less than the
    current value — the override is authoritative.

The Rust binary at ``rust/`` does the heavy lifting.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.DynamicCloud import DynamicCloud
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class RayTracingVoxelMapConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "result/bin/voxel_ray_tracing"
    build_command: str | None = "nix build path:."
    stdin_config: bool = True

    voxel_size: float = 0.1
    # Skip rays longer than this (meters); 0 disables the limit.
    max_range: float = 30.0
    # Controls what portion of rays we perform ray tracing on.
    # Honestly we probably should always have this at 1 unless you don't care about a clean map.
    # Higher num means less ray tracing.
    ray_subsample: int = 1
    # Extend rays past the end point to clear shadows
    shadow_depth: float = 0.2
    # Bounds for the health of voxels. Positive health means voxel is occupied.
    min_health: int = -1
    max_health: int = 1
    # Seconds between sequence-counter increments ("slow clock").
    sequence_period_secs: float = 1.0
    # Spare a clearing miss when |ray dot surface normal| is below this.
    graze_cos: float = 0.7
    # Only spare a voxel whose neighborhood was hit within this many frames.
    recency_window: int = 15


class RayTracingVoxelMap(NativeModule):
    """Rust voxel-map module with raycast clearing of dynamic objects."""

    config: RayTracingVoxelMapConfig

    lidar: In[PointCloud2]
    odometry: In[Odometry]

    map_override: In[DynamicCloud]
    global_map: Out[DynamicCloud]
    local_map: Out[PointCloud2]


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    RayTracingVoxelMap()
