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

:class:`R1ProMid360` is dimos's own Mid-360 driver, which feeds the Rust
Point-LIO, with one R1-specific addition: when nobody told it where the lidar
is, it reads the vendor's own ``MID360_config.json`` rather than failing. See
:mod:`dimos.robot.galaxea.r1pro.vendor_lidar`.
"""

from __future__ import annotations

import os

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.hardware.sensors.lidar.livox.module import Mid360, Mid360Config
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.protocol.tf.static_tf_publisher import (
    FrameSpec,
    StaticTfPublisher,
    frames_to_edge_transforms,
)
from dimos.robot.galaxea.r1pro.connection import LIDAR_MOUNT_XYZ
from dimos.robot.galaxea.r1pro.vendor_lidar import (
    VendorLidarConfigError,
    read_vendor_lidar_network,
)
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


class R1ProMid360Config(Mid360Config):
    # No factory-default address: on an R1 the vendor's file answers instead.
    lidar_ip: str | None = Field(default_factory=lambda: os.environ.get("DIMOS_MID360_LIDAR_IP"))


class R1ProMid360(Mid360):
    """dimos's Mid-360 driver on the R1's chassis lidar.

    Identical to :class:`Mid360` except for how it learns the lidar's address.
    ``Mid360`` assumes the factory IP when ``lidar_ip`` (config or
    ``DIMOS_MID360_LIDAR_IP``) is unset. On an R1 the real address is already
    written down in the vendor's ``MID360_config.json``, so when nothing sets it
    this reads it from there, and only fails -- naming the file and the
    variables -- when that file cannot answer either.

    The sensor itself: a Livox streams to the host that last asked it to, so
    while this runs the vendor's ``livox_ros_driver2`` receives nothing and its
    ``/hdas/lidar_chassis_left`` topic goes quiet. The vendor driver does not
    recover on its own; the R1 README says how to give it the sensor back.
    """

    config: R1ProMid360Config

    @rpc
    def start(self) -> None:
        self._resolve_vendor_network()
        super().start()

    def _resolve_vendor_network(self) -> None:
        if self.config.lidar_ip:
            return
        try:
            network = read_vendor_lidar_network()
        except VendorLidarConfigError as error:
            raise RuntimeError(
                f"R1ProMid360: the chassis lidar's address is unknown. {error}"
            ) from error
        self.config.lidar_ip = network.lidar_ip
        if not self.config.host_ip:
            self.config.host_ip = network.host_ip
        logger.info(
            "R1ProMid360: lidar %s, host %s, from the vendor's MID360_config.json",
            network.lidar_ip,
            self.config.host_ip,
        )
