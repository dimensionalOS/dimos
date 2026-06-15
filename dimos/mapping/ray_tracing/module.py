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
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import mapping


class RayTracingVoxelMapConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/voxel_ray_tracing"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True

    voxel_size: float = 0.1
    # Maximum range for ray tracing
    max_range: float = 30.0
    # Proportion of points that are ray traced
    # Higher subsample means less tracing
    ray_subsample: int = 1
    # Extend rays past the end point to clear shadows
    shadow_depth: float = 0.2
    # Ignore voxels within this range of points for ray tracing clearing
    grace_depth: float = 0.2
    # Bounds for the health of voxels. Positive health means voxel is occupied.
    min_health: int = -2
    max_health: int = 1
    # Spare a clearing miss when |ray dot surface normal| is below this.
    graze_cos: float = 0.7
    # Only spare a voxel whose neighborhood was hit within this many frames.
    recency_window: int = 15


class RayTracingVoxelMap(NativeModule, mapping.GlobalPointcloud):
    """Rust voxel-map module with raycast clearing of dynamic objects."""

    config: RayTracingVoxelMapConfig

    lidar: In[PointCloud2]
    odometry: In[Odometry]
    global_map: Out[PointCloud2]
    local_map: Out[PointCloud2]


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    RayTracingVoxelMap()
