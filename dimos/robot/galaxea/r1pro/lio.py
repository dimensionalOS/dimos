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

"""Hang the R1's body off Point-LIO's estimate instead of its wheels.

Point-LIO publishes ``odom -> lidar_pointlio_link``: where the *lidar* is. Every
other frame on the robot is described relative to ``base_link``, so on its own
that edge is a branch nothing else is attached to, and the rest of the robot
still rides whatever publishes ``odom -> base_link``. On the R1 that is wheel
odometry, integrated from the chassis speed the controller was *commanded* --
not measured -- which is fine for a few metres and hopeless around a spin.

:class:`R1ProLioMountTf` supplies the edge that joins them,
``lidar_pointlio_link -> base_link``, the inverse of the vendor URDF's fixed
lidar mount. With the connection's own odometry turned off
(``publish_odom=False``) ``base_link`` then has exactly one parent and the whole
robot -- chassis lidar, head camera, and so every cloud either of them produces --
is placed by lidar-inertial odometry. Leaving the connection's odometry on as
well would give ``base_link`` two parents, and which one a consumer resolves
through depends on arrival order.

``publish_odom=False`` also takes away the ``base_link ->
lidar_chassis_left_link`` edge, which the connection publishes on the same
stamp as its odometry, so that edge is republished here too. Without it the
chassis lidar's own cloud has no path to the world and the map never sees it.

:class:`R1ProLioOdomPose` is the other half: the planners take a
``PoseStamped`` and Point-LIO emits an ``Odometry``, which is the same pose
with a twist beside it.

The estimator itself is dimos's Rust Point-LIO fed by dimos's own Mid-360
driver; the blueprint composes them, with the lidar's address from
:mod:`dimos.robot.galaxea.r1pro.config`.
"""

from __future__ import annotations

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.protocol.tf.static_tf_publisher import (
    FrameSpec,
    StaticTfPublisher,
    frames_to_edge_transforms,
)
from dimos.robot.galaxea.r1pro.connection import LIDAR_MOUNT_XYZ
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ODOM_FRAME = "odom"
LIDAR_FRAME = "lidar_pointlio_link"
BASE_FRAME = "base_link"
CHASSIS_LIDAR_FRAME = "lidar_chassis_left_link"

FRAMES: list[FrameSpec] = [
    (BASE_FRAME, None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    # Point-LIO's frame and the URDF's sit at the same place on the chassis;
    # they differ only in who is allowed to be their parent.
    (LIDAR_FRAME, BASE_FRAME, LIDAR_MOUNT_XYZ, (0.0, 0.0, 0.0)),
    (CHASSIS_LIDAR_FRAME, BASE_FRAME, LIDAR_MOUNT_XYZ, (0.0, 0.0, 0.0)),
]


def mount_transforms() -> list[Transform]:
    """The mount tree as published: rooted at Point-LIO's frame."""
    edges = {t.child_frame_id: t for t in frames_to_edge_transforms(FRAMES)}
    return [-edges[LIDAR_FRAME], edges[CHASSIS_LIDAR_FRAME]]


class R1ProLioMountTf(StaticTfPublisher):
    """Publishes the mount tree under Point-LIO on a fixed interval."""

    def transforms(self) -> list[Transform]:
        return mount_transforms()


class R1ProLioOdomPose(Module):
    """Re-publish Point-LIO's odometry as the bare pose the planners read.

    The local planners and the movement manager take a ``PoseStamped``; the
    connection used to give them one off its wheel odometry. Point-LIO publishes
    an ``Odometry``, which carries the same pose with a twist beside it, so this
    is a projection rather than a conversion -- but the ports are typed and one
    will not connect to the other.
    """

    odometry: In[Odometry]
    pose: Out[PoseStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))

    def _on_odometry(self, msg: Odometry) -> None:
        self.pose.publish(
            PoseStamped(
                ts=msg.ts,
                frame_id=msg.frame_id,
                position=msg.pose.position,
                orientation=msg.pose.orientation,
            )
        )
