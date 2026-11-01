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

"""Hang the R1's ``base_link`` off Point-LIO's lidar pose instead of wheel odometry."""

from __future__ import annotations

from functools import cache

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, StaticTfPublisherConfig
from dimos.robot.assets.model import LoadedRobotModel
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL


class R1ProLioConfig(StaticTfPublisherConfig):
    base_frame: str = "base_link"
    # Point-LIO's moving sensor frame (its sensor_frame_id).
    lidar_frame: str = "lidar_pointlio_link"
    # The URDF's name for the same lidar.
    chassis_lidar_frame: str = "lidar_chassis_left_link"


@cache
def lio_model(lidar_frame: str, chassis_lidar_frame: str) -> LoadedRobotModel:
    """The R1 URDF with Point-LIO's frame added at the chassis lidar, loaded on first use so importing a blueprint needs no robot assets."""
    return R1PRO_MODEL.with_fixed_frame(lidar_frame, chassis_lidar_frame).load()


def joint_transform(model: LoadedRobotModel, child_link: str) -> Transform:
    """The fixed joint that places ``child_link`` under its parent."""
    joint = next(j for j in model.joints if j.child_link == child_link)
    return Transform(
        translation=Vector3(*joint.origin_xyz),
        rotation=Quaternion.from_euler(Vector3(*joint.origin_rpy)),
        frame_id=joint.parent_link,
        child_frame_id=joint.child_link,
    )


def mount_transforms(config: R1ProLioConfig) -> list[Transform]:
    """Rooted at Point-LIO's frame, plus the URDF's chassis-lidar edge."""
    model = lio_model(config.lidar_frame, config.chassis_lidar_frame)
    base_to_chassis_lidar = joint_transform(model, config.chassis_lidar_frame)
    assert base_to_chassis_lidar.frame_id == config.base_frame
    base_to_lidar = base_to_chassis_lidar + joint_transform(model, config.lidar_frame)
    return [-base_to_lidar, base_to_chassis_lidar]


def base_pose(odometry: Odometry, lidar_to_base: Transform) -> PoseStamped:
    """Point-LIO's lidar pose carried through the mount to base_link."""
    odom_to_lidar = Transform(
        translation=odometry.pose.position,
        rotation=odometry.pose.orientation,
        frame_id=odometry.frame_id,
        ts=odometry.ts,
    )
    odom_to_base = odom_to_lidar + lidar_to_base
    return PoseStamped(
        ts=odometry.ts,
        frame_id=odometry.frame_id,
        position=odom_to_base.translation,
        orientation=odom_to_base.rotation,
    )


class R1ProLio(StaticTfPublisher):
    """Hangs base_link under Point-LIO's lidar frame, and publishes base_link's pose from Point-LIO's odometry."""

    config: R1ProLioConfig

    odometry: In[Odometry]
    pose: Out[PoseStamped]

    def transforms(self) -> list[Transform]:
        return mount_transforms(self.config)

    @rpc
    def start(self) -> None:
        super().start()
        self._lidar_to_base = mount_transforms(self.config)[0]
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))

    def _on_odometry(self, msg: Odometry) -> None:
        self.pose.publish(base_pose(msg, self._lidar_to_base))
