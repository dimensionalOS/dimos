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

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.change_detect import Glob, PathEntry


class PathFollowerConfig(NativeModuleConfig):
    """Config for the path follower native module."""

    cwd: str | None = "."
    executable: str = "result/bin/path_follower"
    build_command: str | None = "nix build github:dimensionalOS/dimos-module-path-follower/v0.1.0 --no-write-lock-file"

    # Pure pursuit parameters
    look_ahead_distance: float = 0.5
    max_speed: float = 2.0
    max_yaw_rate: float = 1.5

    # Goal tolerance
    goal_tolerance: float = 0.3

    # Vehicle config
    vehicle_config: str = "omniDir"


class PathFollower(NativeModule):
    """Pure pursuit path follower with PID yaw control.

    Takes a path from the local planner and the current vehicle state,
    then computes velocity commands to follow the path.

    Ports:
        path (In[Path]): Local path to follow.
        odometry (In[Odometry]): Vehicle state estimation.
        cmd_vel (Out[Twist]): Velocity commands for the vehicle.
    """

    default_config: type[PathFollowerConfig] = PathFollowerConfig  # type: ignore[assignment]

    path: In[Path]
    odometry: In[Odometry]
    cmd_vel: Out[Twist]
