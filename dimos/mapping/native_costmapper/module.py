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

from dataclasses import asdict
from typing import Any

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.occupancy import HeightCostConfig, OccupancyConfig
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class Config(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/native_costmapper"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True

    algo: str = "height_cost"
    config: OccupancyConfig = Field(default_factory=HeightCostConfig)
    initial_safe_radius_meters: float = 0.0

    def to_config_dict(self) -> dict[str, Any]:
        values = super().to_config_dict()
        values["config"] = asdict(self.config)
        return values


class NativeCostMapper(NativeModule):
    """Native Rust drop-in replacement for the Python cost mapper."""

    config: Config
    global_map: In[PointCloud2]
    merged_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]


native_cost_mapper = NativeCostMapper.blueprint()
