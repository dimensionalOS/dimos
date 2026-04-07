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

from pathlib import Path
from typing import Any

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data


def _default_paths_dir() -> str:
    """Resolve path data from LFS."""
    return str(get_data("smart_nav_paths"))


class LocalPlannerConfig(NativeModuleConfig):
    """Config for the local planner native module.

    Fields with ``None`` default are omitted from the CLI.
    """

    cwd: str | None = str(Path(__file__).resolve().parent)
    executable: str = "result/bin/local_planner"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-local-planner/v0.1.1 --no-write-lock-file"
    )

    # C++ binary uses camelCase CLI args (except paths_dir).
    cli_name_override: dict[str, str] = {
        "vehicle_config": "vehicleConfig",
        "max_speed": "maxSpeed",
        "autonomy_speed": "autonomySpeed",
        "autonomy_mode": "autonomyMode",
        "use_terrain_analysis": "useTerrainAnalysis",
        "obstacle_height_threshold": "obstacleHeightThre",
        "max_relative_z": "maxRelZ",
        "min_relative_z": "minRelZ",
        "goal_clearance": "goalClearance",
        "goal_reached_threshold": "goalReachedThreshold",
        "goal_behind_range": "goalBehindRange",
        "freeze_ang": "freezeAng",
        "two_way_drive": "twoWayDrive",
        "goal_x": "goalX",
        "goal_y": "goalY",
    }

    # Path data directory (auto-resolved from LFS)
    paths_dir: str = ""

    def model_post_init(self, __context: Any) -> None:
        super().model_post_init(__context)
        if not self.paths_dir:
            self.paths_dir = _default_paths_dir()

    # Vehicle config: "omniDir" for mecanum, "standard" for ackermann.
    vehicle_config: str = "omniDir"

    # Maximum velocity the planner will command (m/s).
    max_speed: float = 2.0
    # Velocity cap during autonomous navigation (m/s).
    autonomy_speed: float = 1.0

    # Enable fully autonomous waypoint-following mode.
    autonomy_mode: bool | None = None
    # Use terrain analysis cost map for obstacle avoidance.
    use_terrain_analysis: bool | None = None

    # Points higher than this above ground are classified as obstacles (m).
    obstacle_height_threshold: float = 0.15
    # Height-band filter: maximum z relative to robot (m).
    max_relative_z: float | None = None
    # Height-band filter: minimum z relative to robot (m).
    min_relative_z: float | None = None

    # Minimum clearance around goal position for path planning (m).
    goal_clearance: float = 0.5
    # Distance from goal at which the local planner considers it reached (m).
    goal_reached_threshold: float | None = None
    # When goal is behind the robot and within this range, robot stops (m).
    goal_behind_range: float | None = None
    # Freeze angle (deg): if goal direction exceeds this, robot freezes for
    # freezeTime.  Set to 180 for omni-dir robots to disable freeze.
    freeze_ang: float | None = None
    # Allow driving in reverse.  False = robot must turn to face goal first.
    two_way_drive: bool | None = None
    # Goal x-coordinate in local frame (m). None = omit from CLI (binary default).
    goal_x: float | None = None
    # Goal y-coordinate in local frame (m). None = omit from CLI (binary default).
    goal_y: float | None = None


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
        path (Out[NavPath]): Selected local path for path follower.
        obstacle_cloud (Out[PointCloud2]): Vehicle-frame crop of the planner's
            obstacle cloud (intensity = height above ground). Published at
            ~5 Hz for debugging/visualization of what the planner is actually
            treating as obstacles.
    """

    default_config: type[LocalPlannerConfig] = LocalPlannerConfig  # type: ignore[assignment]

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map: In[PointCloud2]
    joy_cmd: In[Twist]
    way_point: In[PointStamped]
    path: Out[NavPath]
    obstacle_cloud: Out[PointCloud2]
