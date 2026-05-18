# Copyright 2025-2026 Dimensional Inc.
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

"""Go2 physical description and sensor odometry offsets."""

from __future__ import annotations

from pathlib import Path

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.config import RobotConfig
from dimos.robot.unitree.g1.config import G1_LOCAL_PLANNER_PRECOMPUTED_PATHS

# Reuse G1's precomputed local-planner paths until a Go2-specific set is
# generated. The Go2's turning radius is tighter than the G1's, so a future
# regen via CMU's pathGenerator would yield smoother trajectories (especially
# in rage mode), but G1's set is workable as a starting point.
GO2_LOCAL_PLANNER_PRECOMPUTED_PATHS = G1_LOCAL_PLANNER_PRECOMPUTED_PATHS

GO2 = RobotConfig(
    name="unitree_go2",
    model_path=Path(__file__).parent / "go2.urdf",
    # base_link box from go2.urdf is 0.70 (length) x 0.31 (width) x 0.40 (height).
    height_clearance=0.5,
    width_clearance=0.4,
    internal_odom_offsets={
        # Mid-360 lidar mounted on top of the Go2's back, centered laterally,
        # ~0.4 m above the floor.
        "mid360_link": Pose(0.0, 0.0, 0.4, *Quaternion.from_euler(Vector3(0, 0, 0))),
    },
)
