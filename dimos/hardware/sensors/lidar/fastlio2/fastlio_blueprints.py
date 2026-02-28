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

from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.mapping.voxels import VoxelGridMapper
from dimos.visualization.rerun.bridge import rerun_bridge

voxel_size = 0.05

mid360_fastlio = autoconnect(
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=-1),
    rerun_bridge(
        visual_override={
            "world/lidar": lambda grid: grid.to_rerun(voxel_size=voxel_size, mode="boxes"),
        }
    ),
).global_config(n_dask_workers=2, robot_model="mid360_fastlio2")

mid360_fastlio_voxels = autoconnect(
    FastLio2.blueprint(),
    VoxelGridMapper.blueprint(publish_interval=1.0, voxel_size=voxel_size, carve_columns=False),
    rerun_bridge(
        visual_override={
            "world/global_map": lambda grid: grid.to_rerun(voxel_size=voxel_size, mode="boxes"),
            "world/lidar": None,
        }
    ),
).global_config(n_dask_workers=3, robot_model="mid360_fastlio2_voxels")

mid360_fastlio_voxels_native = autoconnect(
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=3.0),
    rerun_bridge(
        visual_override={
            "world/lidar": None,
            "world/global_map": lambda grid: grid.to_rerun(voxel_size=voxel_size, mode="boxes"),
        }
    ),
).global_config(n_dask_workers=2, robot_model="mid360_fastlio2")
