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

"""Record Point-LIO odometry + lidar into a memory SQLite db.

A ``Recorder`` that records its In ports under their own names
(``pointlio_odometry`` / ``pointlio_lidar``) — wire them to PointLio's
``odometry`` / ``lidar`` outputs with ``.remappings()``. Poses come straight
from the odometry stream: each lidar frame is stamped with
the latest odometry pose so ``pointlio_lidar`` carries the trajectory and ``dimos
map global`` can register the body-frame cloud directly (no ``pose-fill`` pass).
"""

from __future__ import annotations

from dimos.core.stream import In
from dimos.memory.module import OnExisting, PoseSetter, Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class PointlioRecorderConfig(RecorderConfig):
    # Append into a populated db (keep other streams); replace only our own.
    on_existing: OnExisting = OnExisting.APPEND


class PointlioRecorder(Recorder):
    config: PointlioRecorderConfig

    pointlio_odometry: In[Odometry]
    pointlio_lidar: In[PointCloud2]

    _last_odom_pose: Pose | None = None

    async def _odom_pose(self, msg: Odometry) -> Pose | None:
        self._last_odom_pose = msg.pose.pose
        return self._last_odom_pose

    async def _lidar_pose(self, msg: PointCloud2) -> Pose | None:
        return self._last_odom_pose

    def pose_setters(self) -> dict[str, PoseSetter]:
        return {
            "pointlio_odometry": self._odom_pose,
            "pointlio_lidar": self._lidar_pose,
        }
