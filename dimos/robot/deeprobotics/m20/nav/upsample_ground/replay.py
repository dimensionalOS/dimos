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

"""Replay blueprint for inspecting M20 ground-upsample synchronization."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.robot.deeprobotics.m20.blueprints.basic import rerun
from dimos.robot.deeprobotics.m20.nav.upsample_ground.module import UpsampleGround
from dimos.robot.deeprobotics.m20.rrd_replay import M20RrdReplay
from dimos.robot.deeprobotics.m20.tf import M20TF

_VOXEL_SIZE_M = 0.05

_replay_ray_tracer = RayTracingVoxelMap.blueprint(
    executable="target/release/voxel_ray_tracing",
    build_command="cargo build --release --bin voxel_ray_tracing",
    voxel_size=_VOXEL_SIZE_M,
    max_range=8.0,
    shadow_depth=0.1,
    min_health=-1,
    max_health=10,
    emit_every=2,
    ray_subsample=1,
    global_emit_every=1,
    auto_build=True,
    support_min=0,
    registered_clouds=True,
).remappings(
    [
        (RayTracingVoxelMap, "lidar", "slam_aligned_points"),
        (RayTracingVoxelMap, "odometry", "slam_odom"),
    ]
)

_upsample_ground = UpsampleGround.blueprint(
    auto_build=True,
    debug=True,
).remappings(
    [
        (UpsampleGround, "current_frame", "slam_aligned_points"),
        (UpsampleGround, "odometry", "slam_odom"),
    ]
)

m20_upsample_ground_replay = autoconnect(
    rerun,
    M20RrdReplay.blueprint(),
    M20TF.blueprint().remappings([(M20TF, "odometry", "slam_odom")]),
    _replay_ray_tracer,
    _upsample_ground,
).global_config(
    replay=True,
    n_workers=8,
    robot_model="m20",
)
