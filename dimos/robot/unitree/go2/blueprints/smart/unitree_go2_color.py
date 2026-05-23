#!/usr/bin/env python3
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

"""unitree-go2 with the global voxel map fed by LidarColorModule's colored output.

Same as ``unitree-go2`` except ``VoxelGridMapper.lidar`` is remapped from the
raw ``lidar`` topic to ``colored_lidar`` (published by ``LidarColorModule``),
so the accumulated global map preserves per-voxel RGB.

Usage:
    dimos run unitree-go2-color
"""

from dimos.mapping.voxels import VoxelGridMapper
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

unitree_go2_color = unitree_go2.remappings([(VoxelGridMapper, "lidar", "colored_lidar")])

__all__ = ["unitree_go2_color"]
