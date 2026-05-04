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

"""LocalPlanner NativeModule: C++ local path planner with obstacle avoidance."""

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

    paths_dir: str = ""

    vehicle_length: float = 0.5  # m
    vehicle_width: float = 0.5  # m
    sensor_offset_x: float | None = None  # m
    sensor_offset_y: float | None = None  # m

    max_speed: float = 0.75  # m/s
    autonomy_speed: float = 0.75  # m/s

    autonomy_mode: bool | None = None
    use_terrain_analysis: bool = True
    check_obstacle: bool = True
    check_rot_obstacle: bool | None = None
    use_cost: bool | None = None

    obstacle_height_threshold: float = 0.1  # m
    ground_height_threshold: float = 0.1  # m
    cost_height_thre1: float | None = None  # m
    cost_height_thre2: float | None = None  # m
    max_relative_z: float = 0.3  # m
    min_relative_z: float = -0.4  # m
    adjacent_range: float = 3.5  # m
    laser_voxel_size: float | None = None  # m
    terrain_voxel_size: float | None = None  # m

    dir_weight: float = 0.02
    dir_thre: float = 90.0  # deg
    dir_to_vehicle: bool | None = None
    path_scale: float = 0.875
    min_path_scale: float = 0.675
    path_scale_step: float = 0.1
    path_scale_by_speed: bool | None = None
    min_path_range: float = 0.8  # m
    path_range_step: float = 0.6  # m
    path_range_by_speed: bool | None = None
    path_crop_by_goal: bool | None = None
    point_per_path_thre: int = 2
    slow_path_num_thre: int = 5
    slow_group_num_thre: int = 1
    omni_dir_goal_thre: float = 0.5  # m

    goal_clearance: float = 0.6  # m
    goal_reached_threshold: float = 0.3  # m
    goal_behind_range: float = 0.8  # m
    goal_yaw_threshold: float = 0.15  # rad
    # Set freeze_ang to 180 for omni-dir robots to disable freeze.
    freeze_ang: float | None = None  # deg
    freeze_time: float | None = None  # s
    two_way_drive: bool | None = None
    goal_x: float | None = None  # m
    goal_y: float | None = None  # m

    joy_to_speed_delay: float | None = None  # s
    joy_to_check_obstacle_delay: float | None = None  # s

    publish_free_paths: bool | None = None

    # Penalty = (angleDiff/180)^2 * (speed/maxSpeed) * max_momentum_penalty
    max_momentum_penalty: float | None = None


class LocalPlanner(NativeModule):
    """Local path planner with obstacle avoidance."""

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
