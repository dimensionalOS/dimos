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

"""M20 TF + CameraInfo publisher.

The drdds->zenoh bridge forwards SLAM odometry (``slam_odom``, frame_id=``map``)
and the cloud topics (``grid_map_3d`` / ``slam_map`` in ``base_link``), but it
publishes no transforms. Without a ``map -> base_link`` edge the rerun viewer
can't place the ``base_link`` clouds relative to the map.

This module turns ``slam_odom`` into that transform and tacks on the static
camera mount chain, and emits the front-camera ``CameraInfo`` (pinhole model)
so 2D/3D views line up.
"""

from threading import Event, Thread

from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

# Static camera mount chain: base_link -> camera_link -> camera_optical.
# camera_optical applies the ROS optical convention (x-right, y-down, z-forward).
# TODO: measure the real M20 front-camera mount offset/orientation.
_CAMERA_LINK_XYZ = (0.3, 0.0, 0.0)
_OPTICAL_ROT = Quaternion(-0.5, 0.5, -0.5, 0.5)

# TODO: replace with the real M20 front-camera calibration intrinsics.
_CAMERA_INFO = CameraInfo.from_intrinsics(
    fx=607.0,
    fy=607.0,
    cx=640.0,
    cy=360.0,
    width=1280,
    height=720,
    frame_id="camera_optical",
)


class M20TF(Module):
    """Publish the M20 TF tree and front-camera CameraInfo."""

    odometry: In[Odometry]
    camera_info: Out[CameraInfo]

    def start(self) -> None:
        super().start()
        self.register_disposable(self.odometry.subscribe(self._publish_tf))
        self._stop = Event()
        self._camera_info_thread = Thread(
            target=self._publish_camera_info, name="m20-camera-info", daemon=True
        )
        self._camera_info_thread.start()

    def stop(self) -> None:
        stop = getattr(self, "_stop", None)
        if stop is not None:
            stop.set()
        super().stop()

    @classmethod
    def _odom_to_tf(cls, odom: Odometry) -> list[Transform]:
        # slam_odom IS the map -> base_link pose (its frame_id is the parent).
        base_link = Transform(
            translation=odom.position,
            rotation=odom.orientation,
            frame_id=odom.frame_id,
            child_frame_id="base_link",
            ts=odom.ts,
        )
        camera_link = Transform(
            translation=Vector3(*_CAMERA_LINK_XYZ),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=odom.ts,
        )
        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=_OPTICAL_ROT,
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=odom.ts,
        )
        return [base_link, camera_link, camera_optical]

    def _publish_tf(self, odom: Odometry) -> None:
        self.tf.publish(*self._odom_to_tf(odom))

    def _publish_camera_info(self) -> None:
        while not self._stop.is_set():
            self.camera_info.publish(_CAMERA_INFO)
            self._stop.wait(1.0)
