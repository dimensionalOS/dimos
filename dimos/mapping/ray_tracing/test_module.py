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

import pytest

from dimos.mapping.ray_tracing.module import RayTracingVoxelMapConfig


def test_pose_match_tolerance_defaults_to_point_one_seconds() -> None:
    assert RayTracingVoxelMapConfig().pose_match_tolerance_s == 0.1


def test_pose_match_tolerance_accepts_non_default_value() -> None:
    config = RayTracingVoxelMapConfig(pose_match_tolerance_s=0.025)
    assert config.pose_match_tolerance_s == 0.025


def test_map_frame_accepts_navigation_and_manipulation_frames() -> None:
    assert RayTracingVoxelMapConfig(map_frame="odom").map_frame == "odom"
    assert RayTracingVoxelMapConfig(map_frame="world").map_frame == "world"


def test_map_frame_rejects_empty_value() -> None:
    with pytest.raises(ValueError):
        RayTracingVoxelMapConfig(map_frame="")


@pytest.mark.parametrize("value", [0.0, -0.001])
def test_pose_match_tolerance_rejects_non_positive_values(value: float) -> None:
    with pytest.raises(ValueError):
        RayTracingVoxelMapConfig(pose_match_tolerance_s=value)
