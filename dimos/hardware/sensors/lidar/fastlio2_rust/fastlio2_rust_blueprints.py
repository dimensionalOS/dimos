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

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2_rust.fastlio2_rust_replay import LivoxDbReplay
from dimos.hardware.sensors.lidar.fastlio2_rust.module import FastLio2Rust
from dimos.hardware.sensors.lidar.fastlio2_rust.recorder import FastLio2Recorder
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.voxels import VoxelGridMapper
from dimos.utils.data import LfsPath
from dimos.visualization.vis_module import vis_module

voxel_size = 0.05

fastlio2_rust = autoconnect(
    Mid360.blueprint(),
    FastLio2Rust.blueprint(),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=3)

fastlio2_rust_record = autoconnect(
    Mid360.blueprint(),
    FastLio2Rust.blueprint(),
    FastLio2Recorder.blueprint(),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=4)

fastlio2_rust_replay = autoconnect(
    LivoxDbReplay.blueprint(
        dataset=LfsPath("fastlio_stairwell_odom_divergence.db"),
    ),
    FastLio2Rust.blueprint(),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=4)

fastlio2_rust_voxels = autoconnect(
    Mid360.blueprint(),
    FastLio2Rust.blueprint(),
    VoxelGridMapper.blueprint(voxel_size=voxel_size, carve_columns=False),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=4)

fastlio2_rust_ray_trace = autoconnect(
    Mid360.blueprint(),
    FastLio2Rust.blueprint(),
    RayTracingVoxelMap.blueprint(voxel_size=voxel_size),
    vis_module(
        "rerun",
        rerun_config={
            "visual_override": {
                "world/lidar": None,
            },
        },
    ),
).global_config(n_workers=6)
