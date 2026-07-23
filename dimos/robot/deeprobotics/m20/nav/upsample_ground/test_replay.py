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

from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.robot.deeprobotics.m20.nav.upsample_ground.module import UpsampleGround
from dimos.robot.deeprobotics.m20.nav.upsample_ground.replay import (
    m20_upsample_ground_replay,
)
from dimos.robot.deeprobotics.m20.rrd_replay import M20RrdReplay
from dimos.robot.deeprobotics.m20.tf import M20TF
from dimos.visualization.rerun.bridge import RerunBridgeModule


def test_replay_blueprint_contains_replay_tf_visualization_and_mapping() -> None:
    modules = {atom.module for atom in m20_upsample_ground_replay.blueprints}

    assert {
        M20RrdReplay,
        M20TF,
        RerunBridgeModule,
        RayTracingVoxelMap,
        UpsampleGround,
    }.issubset(modules)


def test_replay_blueprint_wires_synchronization_inputs() -> None:
    remappings = m20_upsample_ground_replay.remapping_map

    assert remappings[(RayTracingVoxelMap, "lidar")] == "slam_aligned_points"
    assert remappings[(RayTracingVoxelMap, "odometry")] == "slam_odom"
    assert remappings[(M20TF, "odometry")] == "slam_odom"
    assert remappings[(UpsampleGround, "current_frame")] == "slam_aligned_points"
    assert remappings[(UpsampleGround, "odometry")] == "slam_odom"


def test_replay_blueprint_enables_native_sync_diagnostics() -> None:
    upsample_atom = next(
        atom for atom in m20_upsample_ground_replay.blueprints if atom.module is UpsampleGround
    )

    assert upsample_atom.kwargs["debug"] is True
    assert upsample_atom.kwargs["auto_build"] is True
    assert m20_upsample_ground_replay.global_config_overrides["replay"] is True
