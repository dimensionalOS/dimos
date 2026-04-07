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
visibility graph from the classified terrain map, finds routes to goals,
and outputs intermediate waypoints for the local planner.
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class FarPlannerConfig(NativeModuleConfig):
    """Config for the FAR planner native module."""

    cwd: str | None = str(Path(__file__).resolve().parent)
    executable: str = "result/bin/far_planner"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-far-planner/v0.2.0 --no-write-lock-file"
    )

    # C++ binary uses snake_case CLI args.
    cli_name_override: dict[str, str] = {
        "robot_dimension": "robot_dim",
    }

    # Planner parameters
    visibility_range: float = 15.0
    update_rate: float = 5.0
    robot_dimension: float = 0.5
    sensor_range: float = 15.0
    is_static_env: bool = False
    converge_dist: float = 0.8


class FarPlanner(NativeModule):
    """FAR planner: visibility-graph global route planner.

    Builds and maintains a visibility graph from classified terrain maps,
    then finds shortest paths through the graph to navigation goals.
    Outputs intermediate waypoints for the local planner.

    Ports:
        terrain_map_ext (In[PointCloud2]): Extended terrain map (classified obstacles).
        registered_scan (In[PointCloud2]): Raw lidar scan (for future dynamic obs).
        odometry (In[Odometry]): Vehicle state (corrected by PGO).
        goal (In[PointStamped]): User-specified navigation goal.
        way_point (Out[PointStamped]): Intermediate waypoint for local planner.
    """

    default_config: type[FarPlannerConfig] = FarPlannerConfig  # type: ignore[assignment]

    terrain_map_ext: In[PointCloud2]
    terrain_map: In[PointCloud2]
    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    goal: In[PointStamped]
    way_point: Out[PointStamped]
    goal_path: Out[NavPath]
