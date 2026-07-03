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

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class PointCloudMapSaveConfig(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/m20_pointcloud_map_save"
    build_command: str | None = (
        "cmake -S . -B build -DPython3_EXECUTABLE=\"$(uv run python -c 'import sys; print(sys.executable)')\" "
        "&& cmake --build build -j$(nproc) "
        "&& cmake --install build"
    )

    translation_threshold_m: float = 0.5
    rotation_threshold_rad: float = math.radians(15.0)
    voxel_size: float = 0.0
    world_frame_id: str = "world"
    save_path: str | None = None


class PointCloudMapSave(NativeModule):
    """Native C++ module that keyframe-accumulates point clouds in odom frame."""

    config: PointCloudMapSaveConfig

    lidar: In[PointCloud2]
    odometry: In[Odometry]
    global_map: Out[PointCloud2]


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    PointCloudMapSave()
