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

from dimos_lcm.geometry_msgs import PolygonStamped
from dimos_lcm.std_msgs import Float32

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int8 import Int8


class LocalPlannerConfig(NativeModuleConfig):
    cwd: str | None = str(Path(__file__).resolve().parent)
    executable: str = "result/bin/local_planner"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-local-planner/v0.6.0 --no-write-lock-file"
    )

    # C++ binary uses camelCase CLI args.
    cli_name_override: dict[str, str] = {
        "max_speed": "maxSpeed",
        "autonomy_speed": "autonomySpeed",
        "autonomy_mode": "autonomyMode",
        "use_terrain_analysis": "useTerrainAnalysis",
        "obstacle_height_threshold": "obstacleHeightThre",
        "ground_height_threshold": "groundHeightThre",
        "cost_height_thre1": "costHeightThre1",
        "cost_height_thre2": "costHeightThre2",
        "max_relative_z": "maxRelZ",
        "min_relative_z": "minRelZ",
        "goal_clearance": "goalClearance",
        "goal_reached_threshold": "goalReachedThreshold",
        "goal_behind_range": "goalBehindRange",
        "goal_yaw_threshold": "goalYawThreshold",
        "freeze_ang": "freezeAng",
        "freeze_time": "freezeTime",
        "two_way_drive": "twoWayDrive",
        "goal_x": "goalX",
        "goal_y": "goalY",
        "vehicle_length": "vehicleLength",
        "vehicle_width": "vehicleWidth",
        "sensor_offset_x": "sensorOffsetX",
        "sensor_offset_y": "sensorOffsetY",
        "laser_voxel_size": "laserVoxelSize",
        "terrain_voxel_size": "terrainVoxelSize",
        "check_obstacle": "checkObstacle",
        "check_rot_obstacle": "checkRotObstacle",
        "adjacent_range": "adjacentRange",
        "use_cost": "useCost",
        "slow_path_num_thre": "slowPathNumThre",
        "slow_group_num_thre": "slowGroupNumThre",
        "point_per_path_thre": "pointPerPathThre",
        "dir_weight": "dirWeight",
        "dir_thre": "dirThre",
        "dir_to_vehicle": "dirToVehicle",
        "path_scale": "pathScale",
        "min_path_scale": "minPathScale",
        "path_scale_step": "pathScaleStep",
        "path_scale_by_speed": "pathScaleBySpeed",
        "min_path_range": "minPathRange",
        "path_range_step": "pathRangeStep",
        "path_range_by_speed": "pathRangeBySpeed",
        "path_crop_by_goal": "pathCropByGoal",
        "joy_to_speed_delay": "joyToSpeedDelay",
        "joy_to_check_obstacle_delay": "joyToCheckObstacleDelay",
        "omni_dir_goal_thre": "omniDirGoalThre",
        "publish_free_paths": "publishFreePaths",
        "max_momentum_penalty": "maxMomentumPenalty",
    }

    # Path data directory. When empty, the C++ binary falls back to its
    # bundled `share/local_planner/paths` library (shipped in v0.4.0+).
    paths_dir: str = ""

    # Vehicle length for collision checking (m).
    vehicle_length: float = 0.5
    # Vehicle width for collision checking (m).
    vehicle_width: float = 0.5
    # Sensor X offset from vehicle center (m).
    sensor_offset_x: float | None = None
    # Sensor Y offset from vehicle center (m).
    sensor_offset_y: float | None = None

    # Maximum velocity the planner will command (m/s).
    max_speed: float = 0.75
    # Velocity cap during autonomous navigation (m/s).
    autonomy_speed: float = 0.75

    # Enable fully autonomous waypoint-following mode.
    autonomy_mode: bool | None = None
    # Use terrain analysis cost map for obstacle avoidance.
    use_terrain_analysis: bool = True
    # Check obstacles along paths.
    check_obstacle: bool = True
    # Check rotation obstacles near the vehicle.
    check_rot_obstacle: bool | None = None
    # Use terrain cost for path penalty scoring.
    use_cost: bool | None = None

    # Points higher than this above ground are classified as obstacles (m).
    obstacle_height_threshold: float = 0.1
    # Ground height threshold for cost computation (m).
    ground_height_threshold: float = 0.1
    # Upper cost height threshold (m).
    cost_height_thre1: float | None = None
    # Lower cost height threshold (m).
    cost_height_thre2: float | None = None
    # Height-band filter: maximum z relative to robot (m).
    max_relative_z: float = 0.3
    # Height-band filter: minimum z relative to robot (m).
    min_relative_z: float = -0.4
    # Maximum range for obstacle consideration (m).
    adjacent_range: float = 3.5
    # Voxel size for laser cloud downsampling (m).
    laser_voxel_size: float | None = None
    # Voxel size for terrain cloud downsampling (m).
    terrain_voxel_size: float | None = None

    # Direction weight for path scoring.
    dir_weight: float = 0.02
    # Direction threshold for candidate filtering (deg).
    dir_thre: float = 90.0
    # Use direction relative to vehicle instead of goal.
    dir_to_vehicle: bool | None = None
    # Path scale factor (shrinks candidate paths).
    path_scale: float = 0.875
    # Minimum path scale before giving up.
    min_path_scale: float = 0.675
    # Path scale decrement step.
    path_scale_step: float = 0.1
    # Scale path range by joystick speed.
    path_scale_by_speed: bool | None = None
    # Minimum path range before giving up (m).
    min_path_range: float = 0.8
    # Path range decrement step (m).
    path_range_step: float = 0.6
    # Scale path range by joystick speed.
    path_range_by_speed: bool | None = None
    # Crop paths by goal distance.
    path_crop_by_goal: bool | None = None
    # Min blocked points to mark a path as obstructed.
    point_per_path_thre: int = 2
    # Threshold for slow-down by path count.
    slow_path_num_thre: int = 5
    # Threshold for slow-down by group count.
    slow_group_num_thre: int = 1
    # Omni-directional goal distance threshold (m).
    omni_dir_goal_thre: float = 0.5

    # Minimum clearance around goal position for path planning (m).
    goal_clearance: float = 0.6
    # Distance from goal at which the local planner considers it reached (m).
    goal_reached_threshold: float = 0.3
    # When goal is behind the robot and within this range, robot stops (m).
    goal_behind_range: float = 0.8
    # Goal yaw tolerance (rad).
    goal_yaw_threshold: float = 0.15
    # Freeze angle (deg): if goal direction exceeds this, robot freezes for
    # freezeTime.  Set to 180 for omni-dir robots to disable freeze.
    freeze_ang: float | None = None
    # Freeze duration (s).
    freeze_time: float | None = None
    # Allow driving in reverse.  False = robot must turn to face goal first.
    two_way_drive: bool | None = None
    # Goal x-coordinate in local frame (m). None = omit from CLI (binary default).
    goal_x: float | None = None
    # Goal y-coordinate in local frame (m). None = omit from CLI (binary default).
    goal_y: float | None = None

    # Delay before speed command overrides joystick (s).
    joy_to_speed_delay: float | None = None
    # Delay before obstacle check override from autonomy (s).
    joy_to_check_obstacle_delay: float | None = None

    # Publish free_paths visualization cloud. Disable to save CPU.
    publish_free_paths: bool | None = None

    # Momentum penalty: biases path selection toward continuing current motion.
    # 0.0 = disabled (default). Higher values penalize direction changes at speed.
    # Penalty = (angleDiff/180)² × (speed/maxSpeed) × max_momentum_penalty.
    max_momentum_penalty: float | None = None


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
    """

    config: LocalPlannerConfig

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map: In[PointCloud2]
    joy_cmd: In[Twist]
    way_point: In[PointStamped]
    goal_pose: In[PoseStamped]
    speed: In[Float32]
    navigation_boundary: In[PolygonStamped]
    added_obstacles: In[PointCloud2]
    check_obstacle: In[Bool]
    cancel_goal: In[Bool]

    path: Out[NavPath]
    effective_cmd_vel: Out[Twist]
    obstacle_cloud: Out[PointCloud2]
    free_paths: Out[PointCloud2]
    slow_down: Out[Int8]
    goal_reached: Out[Bool]
