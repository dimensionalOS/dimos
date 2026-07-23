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

"""Python NativeModule wrappers for the Rust go2 mapping stack (issue #2820).

Drop-in replacements for :class:`dimos.mapping.voxels.VoxelGridMapper` and
:class:`dimos.mapping.costmapper.CostMapper`: same port names, message types,
and tuning knobs, so ``autoconnect`` wires them identically.

Usage::

    from dimos.mapping.rust_mappers import RustCostMapper, RustVoxelGridMapper

    autoconnect(
        ...,
        RustVoxelGridMapper.blueprint(emit_every=5),
        RustCostMapper.blueprint(),
    )

Build (no nix required)::

    cd dimos/mapping/rust && cargo build --release
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec import mapping


class RustVoxelGridMapperConfig(NativeModuleConfig):
    # cargo (not nix) so the module builds on machines without nix; the flake
    # convention used by other native modules can be layered on later.
    cwd: str | None = "rust"
    executable: str = "target/release/voxel_mapper"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True

    # Mirrors VoxelGridMapperConfig (dimos/mapping/voxels.py). block_count and
    # device are Open3D/CUDA-specific and intentionally dropped.
    voxel_size: float = 0.05
    carve_columns: bool = True
    # "frame_id" is reserved on NativeModuleConfig (stripped from the stdin
    # payload), so the Rust config calls it map_frame_id.
    map_frame_id: str = "world"
    emit_every: int = 1


class RustVoxelGridMapper(NativeModule, mapping.GlobalPointcloud):
    """Rust port of VoxelGridMapper: lidar clouds -> accumulated voxel map."""

    config: RustVoxelGridMapperConfig

    lidar: In[PointCloud2]
    global_map: Out[PointCloud2]


class RustCostMapperConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/cost_mapper"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True

    # Mirrors costmapper.Config + HeightCostConfig, flattened: the Python
    # module nests the algo tuning in `config.config`, which doesn't survive
    # the flat stdin JSON mapping.
    algo: str = "height_cost"
    resolution: float = 0.05
    can_pass_under: float = 0.6
    can_climb: float = 0.15
    ignore_noise: float = 0.05
    smoothing: float = 1.0
    initial_safe_radius_meters: float = 0.0


class RustCostMapper(NativeModule):
    """Rust port of CostMapper: global map cloud -> terrain-slope costmap."""

    config: RustCostMapperConfig

    global_map: In[PointCloud2]
    merged_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]


# Verify the modules construct (mirrors other native-module wrappers).
if TYPE_CHECKING:
    RustVoxelGridMapper()
    RustCostMapper()
