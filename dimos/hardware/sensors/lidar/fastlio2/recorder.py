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

"""Record FAST-LIO odometry + lidar into a memory2 SQLite db.

A ``Recorder`` that records its In ports under their own names
(``fastlio_odometry`` / ``fastlio_lidar``) — wire them to FastLio2's
``odometry`` / ``lidar`` outputs with ``.remappings()``. Poses come straight
from the odometry stream (``@pose_setter_for``): each lidar frame is stamped with
the latest odometry pose so ``fastlio_lidar`` carries the trajectory and ``dimos
map global`` can register it.
"""

from __future__ import annotations

from dimos.core.stream import In
from dimos.memory2.module import OnExisting, Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class FastLio2RecorderConfig(RecorderConfig):
    # Append into a populated db (keep other streams); replace only our own.
    on_existing: OnExisting = OnExisting.APPEND


class FastLio2Recorder(Recorder):
    config: FastLio2RecorderConfig

    fastlio_odometry: In[Odometry]
    fastlio_lidar: In[PointCloud2]

    _last_odom_pose: Pose | None = None

    @pose_setter_for("fastlio_odometry")
    async def _odom_pose(self, msg: Odometry) -> Pose | None:
        pose = getattr(msg, "pose", None)
        self._last_odom_pose = getattr(pose, "pose", None) if pose is not None else None
        return self._last_odom_pose

    @pose_setter_for("fastlio_lidar")
    async def _lidar_pose(self, msg: PointCloud2) -> Pose | None:
        # Most-recent odometry pose, stamped directly (no tf). None before the
        # first odometry -> frame stored unposed, map-skipped.
        return self._last_odom_pose
