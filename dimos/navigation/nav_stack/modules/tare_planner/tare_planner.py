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

"""TarePlanner NativeModule: C++ frontier-based autonomous exploration planner.

Ported from tare_planner. Uses sensor coverage planning and frontier detection
to autonomously explore unknown environments.
"""

from __future__ import annotations

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class TarePlannerConfig(NativeModuleConfig):
    """Config for the TARE planner native module."""

    cwd: str | None = "."
    executable: str = "result/bin/tare_planner"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-tare-planner/v0.1.0 --no-write-lock-file"
    )

    # Exploration parameters
    exploration_range: float = 20.0
    update_rate: float = 1.0
    sensor_range: float = 20.0


class TarePlanner(NativeModule):
    """TARE planner: frontier-based autonomous exploration.

    Maintains a coverage map and detects frontiers (boundaries between
    explored and unexplored space). Plans exploration paths that maximize
    information gain. Outputs waypoints for the local planner.

    Ports:
        registered_scan (In[PointCloud2]): World-frame point cloud for coverage updates.
        odometry (In[Odometry]): Vehicle state.
        way_point (Out[PointStamped]): Exploration waypoint for local planner.
    """

    config: TarePlannerConfig

    @rpc
    def start(self) -> None:
        super().start()

    def stop(self) -> None:
        super().stop()

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    way_point: Out[PointStamped]
