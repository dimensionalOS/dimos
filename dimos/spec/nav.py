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

from typing import Protocol

from dimos.core.stream import Out
from dimos.msgs.nav_msgs.Odometry import Odometry as OdometryMsg
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class TrajectoryController(Protocol):
    odometry: In[OdometryMsg]
    path: In[Path]

    cmd_vel: Out[Twist]
    goal_reached: Out[bool]

    # equivalent to emitting an empty path?
    stop_movement: In[bool]


# this can take a radius of a robot
# or even better some more precise shape of the robot (e.g. a polygon) to avoid collisions
# probably force based repelling (like native go2 obstacle avoidance is better than just stopping the robot)
class ObstacleAvoidance(Protocol):
    # LoGlo/terrainmap perception within some radius
    lidar: In[PointCloud2]
    odometry: In[Odometry]
    path: In[Path]
    safe_path: Out[Path]
