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

"""LocalPlanner NativeModule: C++ local path planner with obstacle avoidance.

Ported from localPlanner.cpp. Uses pre-computed path sets and DWA-like
evaluation to select collision-free paths toward goals.
"""

from __future__ import annotations

from typing import Any

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data


def _default_paths_dir() -> str:
    """Resolve path data from LFS."""
    return str(get_data("smartnav_paths"))


class LocalPlannerConfig(NativeModuleConfig):
    """Config for the local planner native module.

    Field names map to C++ CLI args via snake_case → camelCase conversion
    (e.g. ``autonomy_mode`` → ``--autonomyMode``).
    Fields with ``None`` default are omitted from the CLI.
    """

    cwd: str | None = "."
    executable: str = "result/bin/local_planner"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-local-planner/v0.1.1 --no-write-lock-file"
    )

    # Path data directory (auto-resolved from LFS)
    paths_dir: str = ""

    def model_post_init(self, __context: Any) -> None:
        super().model_post_init(__context)
        if not self.paths_dir:
            self.paths_dir = _default_paths_dir()

    # Vehicle config: "omniDir" for mecanum, "standard" for ackermann.
    vehicle_config: str = "omniDir"

    # --- Speed limits ---

    # Maximum velocity the planner will command (m/s).
    max_speed: float = 2.0
    # Velocity cap during autonomous navigation (m/s).
    autonomy_speed: float = 1.0

    # --- Mode flags ---

    # Enable fully autonomous waypoint-following mode.
    autonomy_mode: bool | None = None
    # Use terrain analysis cost map for obstacle avoidance.
    use_terrain_analysis: bool | None = None

    # --- Obstacle detection ---

    # Points higher than this above ground are classified as obstacles (m).
    obstacle_height_thre: float = 0.15
    # Height-band filter: maximum z relative to robot (m).
    max_rel_z: float | None = None
    # Height-band filter: minimum z relative to robot (m).
    min_rel_z: float | None = None

    # --- Goal parameters ---

    # Minimum clearance around goal position for path planning (m).
    goal_clearance: float = 0.5
    # Goal x-coordinate in local frame (m).
    goal_x: float = 0.0
    # Goal y-coordinate in local frame (m).
    goal_y: float = 0.0


class LocalPlanner(NativeModule):
    """Local path planner with obstacle avoidance.

    Evaluates pre-computed path sets against current obstacle map to select
    the best collision-free path toward the goal. Supports smart joystick,
    waypoint, and manual control modes.

    Ports:
        registered_scan (In[PointCloud2]): Obstacle point cloud.
        odometry (In[Odometry]): Vehicle state estimation.
        terrain_map (In[PointCloud2]): Terrain cost map from TerrainAnalysis
            (intensity = obstacle height). Used when useTerrainAnalysis is enabled.
        joy_cmd (In[Twist]): Joystick/teleop velocity commands.
        way_point (In[PointStamped]): Navigation goal waypoint.
        path (Out[Path]): Selected local path for path follower.
    """

    default_config: type[LocalPlannerConfig] = LocalPlannerConfig  # type: ignore[assignment]

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map: In[PointCloud2]
    joy_cmd: In[Twist]
    way_point: In[PointStamped]
    path: Out[Path]
