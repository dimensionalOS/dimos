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

"""The Go2 as it appears on the graph when it runs the go2web zenoh bridge.

The bridge (go2web ``src/dimos_zenoh.rs``) publishes Point-LIO odom, the clouds and H.264
video and consumes ``cmd_vel``/``command``; nothing here produces them, declaring the ports
is what puts them on the graph. On top of :class:`Go2Base` this adds what the bridge does
not send: the ``odom -> mid360_link`` tf edge, the mount tree and the camera intrinsics.
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Literal

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher
from dimos.robot.unitree.go2.base import Go2Base, Go2BaseConfig
from dimos.robot.unitree.go2.go2_mid360_static_transforms import OPTICAL_RPY
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GO2ZenohConfig(Go2BaseConfig):
    # Point-LIO owns mid360_link.
    tf_root: Literal["base_link", "mid360_link"] = "mid360_link"


class GO2Zenoh(Go2Base, StaticTfPublisher):
    """The go2's zenoh-side streams, plus the static data the robot doesn't send."""

    config: GO2ZenohConfig

    lidar: Out[PointCloud2]  # per-scan, in the LIO's own sensor frame
    pointlio_map: Out[PointCloud2]  # accumulated world map, frame `odom`

    @rpc
    def start(self) -> None:
        super().start()
        self.spawn(self._publish_camera_info())
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._publish_tf, self.odometry))
        )

    def _startup_pose(self) -> None:
        """Stand, and drop the head L1: Point-LIO runs off the MID-360, not that one."""
        self.set_lidar(False)
        super()._startup_pose()

    @rpc
    def stop(self) -> None:
        self.liedown()
        super().stop()

    # The bridge knows sport verbs, api ids and the L1 switch only.
    def _unsupported(self, name: str) -> None:
        logger.warning("%s is not a go2web bridge verb; use GO2DDS", name)

    @rpc
    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        self._unsupported("set_obstacle_avoidance")

    @rpc
    def set_rage_mode(self, enable: bool) -> None:
        self._unsupported("set_rage_mode")

    @rpc
    def switch_joystick(self, enable: bool = True) -> None:
        self._unsupported("switch_joystick")

    @rpc
    def set_light(self, level: int) -> None:
        self._unsupported("set_light")

    @rpc
    def set_led(self, color: str) -> None:
        self._unsupported("set_led")

    @rpc
    def set_volume(self, level: int) -> None:
        self._unsupported("set_volume")

    def _publish_tf(self, odom: Odometry) -> None:
        """The one moving edge, odom -> mid360_link; the bridge publishes no tf."""
        self.tf.publish(TFMessage(Transform.from_pose(odom.child_frame_id, odom.to_pose_stamped())))

    def mount_edges(self) -> dict[str, Transform]:
        """The mount tree by child frame, measured outward from base_link."""
        base_to_camera = Transform(
            translation=Vector3(*self.config.camera_xyz),
            frame_id="base_link",
            child_frame_id="front_camera",
        )
        camera_to_mid360 = Transform(
            translation=Vector3(*self.config.mid360_xyz),
            rotation=Quaternion.from_euler(
                Vector3(*(math.radians(float(d)) for d in self.config.mid360_mount))
            ),
            frame_id="front_camera",
            child_frame_id="mid360_link",
        )
        camera_to_optical = Transform(
            rotation=Quaternion.from_euler(Vector3(*OPTICAL_RPY)),
            frame_id="front_camera",
            child_frame_id="camera_optical",
        )
        return {t.child_frame_id: t for t in (base_to_camera, camera_to_mid360, camera_to_optical)}

    def transforms(self) -> list[Transform]:
        edges = self.mount_edges()
        if self.config.tf_root == "mid360_link":
            return [-edges["mid360_link"], -edges["front_camera"], edges["camera_optical"]]
        return list(edges.values())

    async def _publish_camera_info(self) -> None:
        period = 1.0 / self.config.camera_info_hz
        while self._running:
            self.camera_info.publish(self.config.camera_info.with_ts(time.time()))
            await asyncio.sleep(period)
