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

"""G1 physical description and sensor odometry offsets."""

from __future__ import annotations

import math
from pathlib import Path

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.config import RobotConfig
from dimos.utils.data import LfsPath

# this is robot-specific, but only needed for the local_planner module
# generated via CMU' `pathGenerator` (autonomous_exploration_development_environment/local_planner)
# probably only needs to be regenerated on robots that are notably different than the g1 (the go2 in rage mode probably needs different local planning paths)
G1_LOCAL_PLANNER_PRECOMPUTED_PATHS = LfsPath("unitree_g1_local_planner_precomputed_paths")

G1 = RobotConfig(
    name="unitree_g1",
    model_path=Path(__file__).parent / "g1.urdf",
    height_clearance=1.2,
    width_clearance=0.6,
    internal_odom_offsets={
        # Mid-360 lidar: 1.2 m above ground, mounted upside-down (180° around X).
        "mid360_link": Pose(0.0, 0.0, 1.2, *Quaternion.from_euler(Vector3(math.pi, -0.1, 0.0))),
    },
)
