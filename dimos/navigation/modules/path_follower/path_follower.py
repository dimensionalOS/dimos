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

"""PathFollower NativeModule: C++ pure pursuit path tracking controller.

Ported from pathFollower.cpp. Follows a given path using pure pursuit
with PID yaw control, outputting velocity commands.
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.std_msgs.Int8 import Int8


class PathFollowerConfig(NativeModuleConfig):
    """Config for the path follower native module.

    Fields with ``None`` default are omitted from the CLI.
    """

    cwd: str | None = str(Path(__file__).resolve().parent)
    executable: str = "result/bin/path_follower"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-path-follower/v0.2.0 --no-write-lock-file"
    )

    # C++ binary uses camelCase CLI args.
    cli_name_override: dict[str, str] = {
        "look_ahead_distance": "lookAheadDis",
        "max_speed": "maxSpeed",
        "max_yaw_rate": "maxYawRate",
        "goal_tolerance": "goalTolerance",
        "vehicle_config": "vehicleConfig",
        "autonomy_mode": "autonomyMode",
        "autonomy_speed": "autonomySpeed",
        "max_acceleration": "maxAccel",
        "slow_down_distance_threshold": "slowDwnDisThre",
        "omni_dir_goal_threshold": "omniDirGoalThre",
        "omni_dir_diff_threshold": "omniDirDiffThre",
        "two_way_drive": "twoWayDrive",
    }

    # Look-ahead distance for the pure pursuit controller (m).
    look_ahead_distance: float = 0.5
    # Maximum velocity the follower will command (m/s).
    max_speed: float = 0.75
    # Maximum yaw rate for turning (deg/s).  The C++ binary converts to
    # rad/s internally (``maxYawRate * PI / 180``).
    max_yaw_rate: float = 40.0

    # Distance from goal at which the follower considers it reached (m).
    goal_tolerance: float = 0.3

    # Vehicle kinematics model: "omniDir" for mecanum, "standard" for ackermann.
    vehicle_config: str = "omniDir"
    # Omni-directional mode: distance threshold (m) below which the robot strafes
    # instead of turning.  Set to 0 to disable omni mode (robot turns to face heading).
    omni_dir_goal_threshold: float = 0.5
    # Omni-directional heading tolerance (rad).
    omni_dir_diff_threshold: float = 1.5

    # Enable fully autonomous path-following mode.
    autonomy_mode: bool | None = None
    # Velocity cap during autonomous navigation (m/s).
    autonomy_speed: float = 0.75

    # Allow driving in reverse (two-way drive).  Set to False to force the
    # robot to turn and face the goal before driving forward.
    two_way_drive: bool = False

    # Maximum linear acceleration (m/s²).
    max_acceleration: float = 1.5
    # Distance threshold below which the follower begins slowing down (m).
    slow_down_distance_threshold: float = 0.875


class PathFollower(NativeModule):
    """Pure pursuit path follower with PID yaw control.

    Takes a path from the local planner and the current vehicle state,
    then computes velocity commands to follow the path.

    Ports:
        path (In[NavPath]): Local path to follow.
        odometry (In[Odometry]): Vehicle state estimation.
        cmd_vel (Out[Twist]): Velocity commands for the vehicle.
    """

    config: PathFollowerConfig

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    path: In[NavPath]
    odometry: In[Odometry]
    slow_down: In[Int8]
    safety_stop: In[Int8]
    cmd_vel: Out[Twist]
