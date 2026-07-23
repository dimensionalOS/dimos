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

"""Native C++ ground-upsampling module for the DeepRobotics M20.

The current implementation provides the timestamp-synchronization stage. Ground
estimation and synthetic-point fusion are added in later implementation stages.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class UpsampleGroundConfig(NativeModuleConfig):
    """Configuration for the native M20 ground-upsampling process."""

    cwd: str | None = "cpp"
    executable: str = "build/upsample_ground_native"
    build_command: str | None = (
        "cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --parallel"
    )

    max_future_sync_delta_s: float = Field(default=0.05, ge=0.0)
    current_frame_queue_size: int = Field(default=20, gt=0)
    global_map_queue_size: int = Field(default=20, gt=0)
    odometry_queue_size: int = Field(default=200, gt=0)
    debug: bool = False


class UpsampleGround(NativeModule):
    """Synchronize current cloud, global map, and odometry before ground estimation."""

    config: UpsampleGroundConfig

    current_frame: In[PointCloud2]
    global_map: In[PointCloud2]
    odometry: In[Odometry]
    global_map_upsample_ground: Out[PointCloud2]


if TYPE_CHECKING:
    UpsampleGround()
