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


class PathFollowerConfig(NativeModuleConfig):
    """Config for the path follower native module.

    Field names map to C++ CLI args via snake_case → camelCase conversion
    (e.g. ``autonomy_mode`` → ``--autonomyMode``).
    Fields with ``None`` default are omitted from the CLI.
    """

    cwd: str | None = "."
    executable: str = "result/bin/path_follower"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-path-follower/v0.1.0 --no-write-lock-file"
    )

    # --- Pure pursuit parameters ---

    # Look-ahead distance for the pure pursuit controller (m).
    look_ahead_distance: float = 0.5
    # Maximum velocity the follower will command (m/s).
    max_speed: float = 2.0
    # Maximum yaw rate for turning (rad/s).
    max_yaw_rate: float = 1.5

    # --- Goal ---

    # Distance from goal at which the follower considers it reached (m).
    goal_tolerance: float = 0.3

    # --- Vehicle ---

    # Vehicle kinematics model: "omniDir" for mecanum, "standard" for ackermann.
    vehicle_config: str = "omniDir"

    # --- Mode flags ---

    # Enable fully autonomous path-following mode.
    autonomy_mode: bool | None = None
    # Velocity cap during autonomous navigation (m/s).
    autonomy_speed: float | None = None

    # --- Acceleration / slowdown ---

    # Maximum linear acceleration (m/s²).
    max_accel: float | None = None
    # Distance threshold below which the follower begins slowing down (m).
    slow_dwn_dis_thre: float | None = None


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
