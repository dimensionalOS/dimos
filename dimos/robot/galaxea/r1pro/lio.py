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

from dimos_generated.geometry_msgs.msg import (
    PoseStamped,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.std_msgs.msg import Header
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry import (
    compose_transforms,
    inverse_transform,
    pose_from_transform,
    quaternion_from_euler,
    transform_from_odometry,
)
from dimos.msgs.time import time_from_seconds
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


def joint_transform(model: LoadedRobotModel, child_link: str) -> TransformStamped:
    """The fixed joint that places ``child_link`` under its parent."""
    joint = next(j for j in model.joints if j.child_link == child_link)
    return TransformStamped(
        header=Header(stamp=time_from_seconds(0.0), frame_id=joint.parent_link),
        child_frame_id=joint.child_link,
        transform=Transform(
            translation=Vector3(*joint.origin_xyz),
            rotation=quaternion_from_euler(*joint.origin_rpy),
        ),
    )


def mount_transforms(config: R1ProLioConfig) -> list[TransformStamped]:
    """Rooted at Point-LIO's frame, plus the URDF's chassis-lidar edge."""
    model = lio_model(config.lidar_frame, config.chassis_lidar_frame)
    base_to_chassis_lidar = joint_transform(model, config.chassis_lidar_frame)
    assert base_to_chassis_lidar.header.frame_id == config.base_frame
    base_to_lidar = compose_transforms(
        base_to_chassis_lidar, joint_transform(model, config.lidar_frame)
    )
    return [inverse_transform(base_to_lidar), base_to_chassis_lidar]


def base_pose(odometry: Odometry, lidar_to_base: TransformStamped) -> PoseStamped:
    """Point-LIO's lidar pose carried through the mount to base_link."""
    odom_to_lidar = transform_from_odometry(odometry)
    return pose_from_transform(compose_transforms(odom_to_lidar, lidar_to_base))


class R1ProLio(StaticTfPublisher):
    """Hangs base_link under Point-LIO's lidar frame, and publishes base_link's pose from Point-LIO's odometry."""

    config: R1ProLioConfig

    odometry: In[Odometry]
    pose: Out[PoseStamped]

    def transforms(self) -> list[TransformStamped]:
        return mount_transforms(self.config)

    @rpc
    def start(self) -> None:
        super().start()
        self._lidar_to_base = mount_transforms(self.config)[0]
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))

    def _on_odometry(self, msg: Odometry) -> None:
        self.pose.publish(base_pose(msg, self._lidar_to_base))
