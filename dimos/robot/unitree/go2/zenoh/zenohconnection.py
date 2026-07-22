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

The bridge (go2web ``src/dimos_zenoh.rs``, on by default via the ``dimos_zenoh`` cargo
feature) publishes Point-LIO odom, the per-scan and world clouds and H.264 video
straight onto dimos's own zenoh keys, and consumes ``cmd_vel``. Nothing
here produces any of it — declaring the ports is what puts the robot's topics on the
graph, the same arrangement as a NativeModule whose binary publishes out of process (see
:class:`PointLio`). The port names are the wire contract: the bridge's keys are
``dimos/<port>/<msg.NAME>``, so renaming a port here silently disconnects it. Remap in
the blueprint instead.

What the bridge does not send, and this module supplies: the ``odom -> mid360_link`` tf
edge, the static mount tree, and the camera intrinsics.
"""

from __future__ import annotations

import asyncio
import math
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
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, StaticTfPublisherConfig
from dimos.robot.unitree.go2.connection import _camera_info_static

# Mount geometry, measured on this rig (metres). Kept here rather than shared with
# go2_mid360_static_transforms: that describes the recording rig, which carries the
# lidar at a different angle and hangs its tree off base_link.
CAMERA_XYZ = Vector3(0.32715, -0.00003, 0.04297)  # base_link -> front_camera
MID360_XYZ = Vector3(-0.032, 0.0, 0.12)  # front_camera -> mid360_link: 3.2cm back, 12cm up
# rpy mapping a sensor frame to its optical frame (x-right, y-down, z-forward)
OPTICAL_RPY = Vector3(-math.pi / 2, 0.0, -math.pi / 2)


class GO2ZenohConfig(StaticTfPublisherConfig):
    # Mid-360 mount rotation on the front_camera -> mid360_link edge: fixed-axis rpy,
    # degrees. The lidar is yawed 90 deg on its bracket, so its 60 deg downward tilt
    # lands on roll about the lidar's own x-axis rather than pitch — Point-LIO reports
    # (-60.7, -1.4, +0.8) for this rig standing still, the 1.4 being the dog's stance.
    # Both yaw signs level the body; they differ by 180 deg of heading, so flip the sign
    # if the camera ends up looking backwards.
    mid360_mount_rpy_deg: tuple[float, float, float] = (-60.0, 0.0, -90.0)
    camera_info_hz: float = Field(default=1.0, gt=0.0)


class GO2Zenoh(StaticTfPublisher):
    """The go2's zenoh-side streams, plus the static data the robot doesn't send."""

    config: GO2ZenohConfig

    # Owned by the on-robot bridge, never published here.
    cmd_vel: In[Twist]
    odometry: Out[Odometry]
    lidar: Out[PointCloud2]  # per-scan, in the LIO's own sensor frame
    pointlio_map: Out[PointCloud2]  # accumulated world map, frame `odom`
    video: Out[CompressedVideo]  # front camera, H.264 annex-B

    # Ours: nothing on the robot emits intrinsics.
    camera_info: Out[CameraInfo]

    _camera_info: CameraInfo = _camera_info_static()

    @rpc
    def start(self) -> None:
        super().start()  # static mount tree onto tf
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._publish_tf, self.odometry))
        )
        self.spawn(self._publish_camera_info())

    def transforms(self) -> list[Transform]:
        """The mount tree, rooted at mid360_link because Point-LIO owns that frame.

        The rig is measured outward from the body, but the only live edge here is
        odom -> mid360_link. Publishing the measured direction as-is would leave
        mid360_link with two parents (odom and front_camera) and the body would snap
        between them at 35 Hz, so the two edges above the lidar are inverted and the
        body hangs off the sensor instead.
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
