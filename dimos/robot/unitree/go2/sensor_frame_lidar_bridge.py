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

"""Republish the Go2's onboard lidar in the sensor (base_link) frame.

The onboard Go2 accumulates its lidar in the odom/world frame and reports its
pose as a PoseStamped. The jnav PGO instead wants a raw sensor-frame scan plus a
nav_msgs Odometry. This bridge subtracts the latest odom pose from every point
(world -> base_link), re-emits the scan with its frame_id set to base_link, and
publishes the pose as an Odometry.
"""

from __future__ import annotations

import numpy as np

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class SensorFrameLidarBridgeConfig(ModuleConfig):
    # Sensor frame the scan is re-expressed in (child of the odom frame).
    base_frame_id: str = "base_link"
    # Fixed parent frame the odom pose lives in.
    odom_frame_id: str = "world"


class SensorFrameLidarBridge(Module):
    world_lidar: In[PointCloud2]
    odom: In[PoseStamped]
    lidar: Out[PointCloud2]
    odometry: Out[Odometry]

    config: SensorFrameLidarBridgeConfig

    _latest_odom: PoseStamped | None = None

    async def handle_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom
        self.odometry.publish(self._to_odometry(odom))

    async def handle_world_lidar(self, cloud: PointCloud2) -> None:
        odom = self._latest_odom
        if odom is None:
            return
        self.lidar.publish(self._to_sensor_frame(cloud, odom))

    def _to_odometry(self, odom: PoseStamped) -> Odometry:
        return Odometry(
            ts=odom.ts,
            frame_id=self.config.odom_frame_id,
            child_frame_id=self.config.base_frame_id,
            pose=Pose(position=odom.position, orientation=odom.orientation),
        )

    def _to_sensor_frame(self, cloud: PointCloud2, odom: PoseStamped) -> PointCloud2:
        world_to_base = Transform.from_pose(self.config.base_frame_id, odom).inverse()
        points = cloud.points_f32()
        if len(points) == 0:
            return PointCloud2.from_numpy(
                points, frame_id=self.config.base_frame_id, timestamp=cloud.ts
            )
        homogeneous = np.hstack([points, np.ones((len(points), 1), dtype=np.float32)])
        sensor_points = (world_to_base.to_matrix() @ homogeneous.T).T[:, :3]
        return PointCloud2.from_numpy(
            sensor_points,
            frame_id=self.config.base_frame_id,
            timestamp=cloud.ts,
            intensities=cloud.intensities_f32(),
        )
