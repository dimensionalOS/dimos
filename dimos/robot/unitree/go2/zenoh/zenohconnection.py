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

The bridge (go2web ``src/dimos_zenoh.rs``) publishes odom, the clouds and H.264 video and
consumes ``cmd_vel``/``command``; nothing here produces them, declaring the ports is what
puts them on the graph. Port names are the wire contract — keys are
``dimos/<port>/<msg.NAME>`` — so remap in the blueprint rather than renaming.

This module adds what the bridge does not send: the ``odom -> mid360_link`` tf edge, the
static mount tree, and the camera intrinsics.
"""

from __future__ import annotations

import asyncio
import math
import threading
import time

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, StaticTfPublisherConfig
from dimos.robot.unitree.go2.connection import _camera_info_static

# Mount geometry measured on this rig (metres). Not go2_mid360_static_transforms — that
# is the recording rig: different lidar angle, tree hung off base_link.
CAMERA_XYZ = Vector3(0.32715, -0.00003, 0.04297)  # base_link -> front_camera
MID360_XYZ = Vector3(-0.032, 0.0, 0.12)  # front_camera -> mid360_link: 3.2cm back, 12cm up
# rpy mapping a sensor frame to its optical frame (x-right, y-down, z-forward)
OPTICAL_RPY = Vector3(-math.pi / 2, 0.0, -math.pi / 2)


class GO2ZenohConfig(StaticTfPublisherConfig):
    # front_camera -> mid360_link, fixed-axis rpy in degrees. The 60 deg tilt lands on
    # roll because the lidar sits yawed 90 deg on its bracket. Both yaw signs level the
    # body but differ by 180 deg of heading — flip it if the camera looks backwards.
    mid360_mount_rpy_deg: tuple[float, float, float] = (-60.0, 0.0, -90.0)
    camera_info_hz: float = Field(default=1.0, gt=0.0)


class GO2Zenoh(StaticTfPublisher):
    """The go2's zenoh-side streams, plus the static data the robot doesn't send."""

    config: GO2ZenohConfig

    # Consumed by the on-robot bridge, never published here.
    cmd_vel: In[Twist]
    # Action verbs ("sit", "hello", ...) or a bare sport api id. The rpcs below publish
    # onto it; nothing else in the graph does.
    command: In[String]
    odometry: Out[Odometry]
    lidar: Out[PointCloud2]  # per-scan, in the LIO's own sensor frame
    pointlio_map: Out[PointCloud2]  # accumulated world map, frame `odom`
    video: Out[CompressedVideo]  # front camera, H.264 annex-B

    # Ours: nothing on the robot emits intrinsics.
    camera_info: Out[CameraInfo]

    _camera_info: CameraInfo = _camera_info_static()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._publish_tf, self.odometry))
        )
        self.spawn(self._publish_camera_info())

        # Off the calling thread so start() returns; verbs sent before zenoh matches our
        # publisher against the bridge are dropped.
        timer = threading.Timer(5.0, self._startup_pose)
        timer.daemon = True
        timer.start()
        self.register_disposable(Disposable(timer.cancel))

    def _startup_pose(self) -> None:
        """Stand, and drop the head L1 — Point-LIO runs off the MID-360, not that one."""
        self.standup()
        self.set_lidar(False)

    @rpc
    def stop(self) -> None:
        self.liedown()
        super().stop()

    @rpc
    def send_command(self, verb: str) -> None:
        """Fire an action verb at the bridge (see go2web ``topics::sport_id``)."""
        self.command.transport.publish(String(verb))

    @rpc
    def sport_command(self, api_id: int) -> None:
        """Same, by raw sport api id — the bridge parses a numeric verb."""
        self.send_command(str(api_id))

    @rpc
    def standup(self) -> None:
        self.send_command("stand-up")

    @rpc
    def liedown(self) -> None:
        self.send_command("stand-down")

    @rpc
    def balance_stand(self) -> None:
        self.send_command("balance")

    @rpc
    def sit(self) -> None:
        self.send_command("sit")

    @rpc
    def hello(self) -> None:
        self.send_command("hello")

    @rpc
    def jump(self) -> None:
        self.send_command("jump")

    @rpc
    def set_lidar(self, enabled: bool) -> None:
        self.send_command("lidar on" if enabled else "lidar off")

    def transforms(self) -> list[Transform]:
        """The mount tree, rooted at mid360_link because Point-LIO owns that frame.

        Measured outward from the body, but odom -> mid360_link is the only live edge, so
        the two edges above the lidar are inverted — otherwise mid360_link has two parents
        and the body snaps between them at 35 Hz.
        """
        base_to_camera = Transform(
            translation=CAMERA_XYZ,
            frame_id="base_link",
            child_frame_id="front_camera",
        )
        camera_to_mid360 = Transform(
            translation=MID360_XYZ,
            rotation=Quaternion.from_euler(
                Vector3(*(math.radians(d) for d in self.config.mid360_mount_rpy_deg))
            ),
            frame_id="front_camera",
            child_frame_id="mid360_link",
        )
        camera_to_optical = Transform(
            rotation=Quaternion.from_euler(OPTICAL_RPY),
            frame_id="front_camera",
            child_frame_id="camera_optical",
        )
        return [-camera_to_mid360, -base_to_camera, camera_to_optical]

    def _publish_tf(self, odom: Odometry) -> None:
        """The one moving edge, odom -> mid360_link; the bridge publishes no tf."""
        self.tf.publish(Transform.from_pose(odom.child_frame_id, odom.to_pose_stamped()))

    async def _publish_camera_info(self) -> None:
        period = 1.0 / self.config.camera_info_hz
        while self._running:
            self._camera_info.ts = time.time()
            self.camera_info.publish(self._camera_info)
            await asyncio.sleep(period)
