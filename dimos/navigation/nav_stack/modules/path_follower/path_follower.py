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

"""PathFollower NativeModule: C++ pure pursuit path tracking controller."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.std_msgs.Int8 import Int8


class PathFollowerConfig(NativeModuleConfig):
    cwd: str | None = str(Path(__file__).resolve().parent)
    executable: str = "result/bin/path_follower"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-path-follower/v0.2.0 --no-write-lock-file"
    )

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

    look_ahead_distance: float = 0.5  # m
    max_speed: float = 0.75  # m/s
    max_yaw_rate: float = 40.0  # deg/s (C++ converts to rad/s internally)

    goal_tolerance: float = 0.3  # m

    vehicle_config: Literal["omniDir", "standard"] = "omniDir"
    omni_dir_goal_threshold: float = 0.5  # m, set to 0 to disable omni mode
    omni_dir_diff_threshold: float = 1.5  # rad

    autonomy_mode: bool | None = None
    autonomy_speed: float = 0.75  # m/s

    two_way_drive: bool = False
    max_acceleration: float = 1.5  # m/s^2
    slow_down_distance_threshold: float = 0.875  # m


class PathFollower(NativeModule):
    """Pure pursuit path follower with PID yaw control."""

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
