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

"""Re-express Point-LIO's Mid-360 scan and odometry in the Go2's base_link.

Point-LIO reports odom -> mid360_link and stamps its scan in mid360_link, which on
the Go2 rig is pitched 60 deg down. GSC PGO wants a level body frame (its scan
context bins by height), so this re-emits the scan in base_link and the odometry as
odom -> base_link, using the rig's mount geometry.
"""

from __future__ import annotations

import numpy as np

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.static_tf_publisher import frames_to_edge_transforms
from dimos.robot.unitree.go2.go2_mid360_static_transforms import FRAMES


def mid360_to_base_link() -> Transform:
    """mid360_link -> base_link from the rig's mount tree."""
    edges = {t.child_frame_id: t for t in frames_to_edge_transforms(FRAMES)}
    return (edges["front_camera"] + edges["mid360_link"]).inverse()


class Mid360ToBaseLinkConfig(ModuleConfig):
    base_frame_id: str = "base_link"


class Mid360ToBaseLink(Module):
    sensor_lidar: In[PointCloud2]
    sensor_odometry: In[Odometry]
    cloud: Out[PointCloud2]
    odometry: Out[Odometry]

    config: Mid360ToBaseLinkConfig

    # mid360_link -> base_link, and its inverse for moving points from mid360_link into base_link.
    _sensor_to_base: np.ndarray = mid360_to_base_link().to_matrix()
    _points_to_base: np.ndarray = np.linalg.inv(_sensor_to_base)

    async def handle_sensor_odometry(self, odom: Odometry) -> None:
        odom_to_sensor = Transform.from_pose(odom.child_frame_id, odom.to_pose_stamped())
        odom_to_base = Transform.from_matrix(
            odom_to_sensor.to_matrix() @ self._sensor_to_base,
            ts=odom.ts,
            frame_id=odom.frame_id,
            child_frame_id=self.config.base_frame_id,
        )
        self.odometry.publish(
            Odometry(
                ts=odom.ts,
                frame_id=odom.frame_id,
                child_frame_id=self.config.base_frame_id,
                pose=Pose(position=odom_to_base.translation, orientation=odom_to_base.rotation),
            )
        )

    async def handle_sensor_lidar(self, scan: PointCloud2) -> None:
        points = scan.points_f32()
        if len(points) > 0:
            to_base = self._points_to_base
            points = (points @ to_base[:3, :3].T + to_base[:3, 3]).astype(np.float32)
        self.cloud.publish(
            PointCloud2.from_numpy(
                points,
                frame_id=self.config.base_frame_id,
                timestamp=scan.ts,
                intensities=scan.intensities_f32(),
            )
        )
