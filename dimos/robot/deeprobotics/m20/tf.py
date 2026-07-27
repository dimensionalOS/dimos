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

This module turns ``slam_odom`` into that transform, adds the static front/rear
camera mount chains, and emits their ``CameraInfo`` pinhole models so 2D/3D
views line up.
"""

from threading import Event, Thread

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

FRONT_CAMERA_OPTICAL_FRAME = "camera_optical"
REAR_CAMERA_OPTICAL_FRAME = "rear_camera_optical"

# Static camera mount chains. Front optical +Z points along base_link +X;
# rear optical +Z points along base_link -X from an origin 0.3 m behind it.
# TODO: measure the real M20 camera mount offsets/orientations.
_CAMERA_LINK_XYZ = (0.3, 0.0, 0.0)
_REAR_CAMERA_LINK_XYZ = (-0.3, 0.0, 0.0)
_OPTICAL_ROT = Quaternion(-0.5, 0.5, -0.5, 0.5)

# The M20 RTSP streams and recorded images are 800x600. Focal length remains
# provisional until the real front/rear calibration is available.
_CAMERA_WIDTH = 800
_CAMERA_HEIGHT = 600
_CAMERA_FOCAL_LENGTH = 607.0
_CAMERA_INFO = CameraInfo.from_intrinsics(
    fx=_CAMERA_FOCAL_LENGTH,
    fy=_CAMERA_FOCAL_LENGTH,
    cx=_CAMERA_WIDTH / 2.0,
    cy=_CAMERA_HEIGHT / 2.0,
    width=_CAMERA_WIDTH,
    height=_CAMERA_HEIGHT,
    frame_id=FRONT_CAMERA_OPTICAL_FRAME,
)
_REAR_CAMERA_INFO = CameraInfo.from_intrinsics(
    fx=_CAMERA_FOCAL_LENGTH,
    fy=_CAMERA_FOCAL_LENGTH,
    cx=_CAMERA_WIDTH / 2.0,
    cy=_CAMERA_HEIGHT / 2.0,
    width=_CAMERA_WIDTH,
    height=_CAMERA_HEIGHT,
    frame_id=REAR_CAMERA_OPTICAL_FRAME,
)


def camera_mount_transforms(ts: float = 0.0) -> list[Transform]:
    """Return the fixed base-to-camera transform chain."""
    return [
        Transform(
            translation=Vector3(*_CAMERA_LINK_XYZ),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=ts,
        ),
        Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=_OPTICAL_ROT,
            frame_id="camera_link",
            child_frame_id=FRONT_CAMERA_OPTICAL_FRAME,
            ts=ts,
        ),
        Transform(
            translation=Vector3(*_REAR_CAMERA_LINK_XYZ),
            rotation=Quaternion(0.0, 0.0, 1.0, 0.0),
            frame_id="base_link",
            child_frame_id="rear_camera_link",
            ts=ts,
        ),
        Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=_OPTICAL_ROT,
            frame_id="rear_camera_link",
            child_frame_id=REAR_CAMERA_OPTICAL_FRAME,
            ts=ts,
        ),
    ]


def front_camera_info() -> CameraInfo:
    """Return the fixed front-camera calibration."""
    return _CAMERA_INFO


def rear_camera_info() -> CameraInfo:
    """Return the fixed rear-camera calibration."""
    return _REAR_CAMERA_INFO


class M20TF(Module):
    """Publish the M20 TF tree and front/rear CameraInfo."""

    odometry: In[Odometry]
    camera_info: Out[CameraInfo]
    camera_info_rear: Out[CameraInfo]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._publish_tf)))
        self._stop = Event()
        self._camera_info_thread = Thread(
            target=self._publish_camera_info, name="m20-camera-info", daemon=True
        )
        self._camera_info_thread.start()

    @rpc
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
        # Keep the fixed edges on /tf for consumers that cannot receive a
        # latched /tf_static stream. The Rerun blueprint logs these edges as
        # timeless data and filters these repeated copies from its dynamic view.
        return [base_link, *camera_mount_transforms(odom.ts)]

    def _publish_tf(self, odom: Odometry) -> None:
        self.tf.publish(*self._odom_to_tf(odom))

    def _publish_camera_info(self) -> None:
        while not self._stop.is_set():
            self.camera_info.publish(_CAMERA_INFO)
            self.camera_info_rear.publish(_REAR_CAMERA_INFO)
            self._stop.wait(1.0)
