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

"""FarPlanner NativeModule: C++ visibility-graph route planner.

Ported from far_planner + boundary_handler + graph_decoder. Builds a
visibility graph from registered scans, finds routes to goals, and
outputs intermediate waypoints for the local planner.
"""

from __future__ import annotations

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.change_detect import Glob, PathEntry


class FarPlannerConfig(NativeModuleConfig):
    """Config for the FAR planner native module."""

    cwd: str | None = "."
    executable: str = "result/bin/far_planner"
    build_command: str | None = "nix build github:dimensionalOS/dimos-module-far-planner/v0.1.0 --no-write-lock-file"

    # Planner parameters
    visibility_range: float = 15.0
    update_rate: float = 2.0
    robot_dim: float = 0.5
    sensor_range: float = 20.0


class FarPlanner(NativeModule):
    """FAR planner: visibility-graph global route planner.

    Builds and maintains a visibility graph from registered point clouds,
    then finds shortest paths through the graph to navigation goals.
    Outputs intermediate waypoints for the local planner.

    Ports:
        registered_scan (In[PointCloud2]): World-frame point cloud for graph updates.
        odometry (In[Odometry]): Vehicle state.
        goal (In[PointStamped]): User-specified navigation goal.
        way_point (Out[PointStamped]): Intermediate waypoint for local planner.
    """

    default_config: type[FarPlannerConfig] = FarPlannerConfig  # type: ignore[assignment]

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    goal: In[PointStamped]
    way_point: Out[PointStamped]
