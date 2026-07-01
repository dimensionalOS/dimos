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

"""The RealSense D435i + Mid-360 rig: static mount frames, recorder, record blueprints.

A single physical sensor assembly described in one place: the mount geometry published
onto tf (:class:`Mid360RealsenseStaticTf`), the memory2 recorder
(:class:`Mid360RealsenseRecorder`), and the record blueprints that wire them to the
live sensors.

Point-LIO odom+lidar and the RealSense color/depth/pointcloud streams are recorded into
a memory2 db, with the rig's mount frames published continuously onto tf. Two variants:
``mid360_realsense_record`` (db only) and ``mid360_realsense_record_with_pcap`` (also
captures a raw .pcap of the Mid-360 UDP stream).

The lidar IPs come from each module's own config (``DIMOS_MID360_LIDAR_IP`` for the
Mid-360 / pcap capture and Point-LIO both read ``DIMOS_MID360_LIDAR_IP``)::

    export DIMOS_MID360_LIDAR_IP=192.168.1.155
    dimos run mid360-realsense-record            # db only
    dimos run mid360-realsense-record-with-pcap  # db + raw pcap

Frame sources
-------------
RealSense D435i frame transforms are transcribed from the official
realsense2_description xacro (urdf/_d435.urdf.xacro + urdf/_d435i_imu_modules.urdf.xacro,
use_nominal_extrinsics=true).

Mid-360 geometry (manual): body is 65 x 65 x 60 mm; the point-cloud origin O lies on the
central vertical axis, ~47 mm above the base. The IMU chip is *not* on that axis. The
lidar-to-IMU extrinsic comes from the official Mid-360 config (extrinsic_T flipped gives
the IMU position in lidar coords).
"""

from __future__ import annotations

import math

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder
from dimos.hardware.sensors.lidar.virtual_mid360.recorder import Mid360PcapRecorder
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.cmu_nav.frames import FRAME_ODOM
from dimos.protocol.tf.static_tf_publisher import (
    FrameSpec,
    StaticTfPublisher,
    frames_to_edge_transforms,
)

CAMERA_ANGLE_UP = math.radians(10)

# Mid-360 box: pitched down from bottom_screw_frame, then offset back/up in that frame
BOX_PITCH_DOWN = math.radians(26) + CAMERA_ANGLE_UP
BOX_BACK = 0.085
BOX_UP = 0.037  # ~4cm up

# Physical constants from _d435.urdf.xacro (meters)
CAM_HEIGHT = 0.025
DEPTH_PY = 0.0175
DEPTH_PZ = CAM_HEIGHT / 2
MOUNT_FROM_CENTER_OFFSET = 0.0149
GLASS_TO_FRONT = 0.1e-3
ZERO_DEPTH_TO_GLASS = 4.2e-3
MESH_X_OFFSET = MOUNT_FROM_CENTER_OFFSET - GLASS_TO_FRONT - ZERO_DEPTH_TO_GLASS

DEPTH_TO_INFRA1_OFFSET = 0.0
DEPTH_TO_INFRA2_OFFSET = -0.050
DEPTH_TO_COLOR_OFFSET = 0.015
IMU_XYZ = (-0.01174, -0.00552, 0.0051)

# rpy that maps a sensor frame to its optical frame (z-forward, x-right, y-down)
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)

# Mid-360 internal frames (manual: point-cloud origin O ~47mm above base, on central axis).
# Box center is 30mm above base, so O sits +17mm along box +z.
LIDAR_ABOVE_BOX_CENTER = 0.017
# IMU position in point-cloud (lidar) coordinates, from Livox Mid-360 extrinsics.
IMU_IN_LIDAR = (0.011, 0.02329, -0.04412)

# The physical mount tree (parent -> child). The gravity-flat "world" helper frame from
# the offline tooling is omitted here — during recording, world comes from odometry.
FRAMES: list[FrameSpec] = [
    ("bottom_screw_frame", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("link", "bottom_screw_frame", (MESH_X_OFFSET, DEPTH_PY, DEPTH_PZ), (0.0, 0.0, 0.0)),
    ("depth_frame", "link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("depth_optical_frame", "depth_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("infra1_frame", "link", (0.0, DEPTH_TO_INFRA1_OFFSET, 0.0), (0.0, 0.0, 0.0)),
    ("infra1_optical_frame", "infra1_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("infra2_frame", "link", (0.0, DEPTH_TO_INFRA2_OFFSET, 0.0), (0.0, 0.0, 0.0)),
    ("infra2_optical_frame", "infra2_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("color_frame", "link", (0.0, DEPTH_TO_COLOR_OFFSET, 0.0), (0.0, 0.0, 0.0)),
    ("color_optical_frame", "color_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("accel_frame", "link", IMU_XYZ, (0.0, 0.0, 0.0)),
    ("accel_optical_frame", "accel_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("gyro_frame", "link", IMU_XYZ, (0.0, 0.0, 0.0)),
    ("gyro_optical_frame", "gyro_frame", (0.0, 0.0, 0.0), OPTICAL_RPY),
    ("box_pitch_frame", "bottom_screw_frame", (0.0, 0.0, 0.0), (0.0, BOX_PITCH_DOWN, 0.0)),
    ("box_center", "box_pitch_frame", (-BOX_BACK, 0.0, BOX_UP), (0.0, 0.0, 0.0)),
    ("lidar_frame", "box_center", (0.0, 0.0, LIDAR_ABOVE_BOX_CENTER), (0.0, 0.0, 0.0)),
    ("imu_frame", "lidar_frame", IMU_IN_LIDAR, (0.0, 0.0, 0.0)),
]


class Mid360RealsenseStaticTf(StaticTfPublisher):
    """Publishes the RealSense/Mid-360 mount tree onto tf on a fixed interval."""

    def transforms(self) -> list[Transform]:
        return frames_to_edge_transforms(FRAMES)


class Mid360RealsenseRecorder(PointlioRecorder):
    """Records Point-LIO odom+lidar plus the RealSense streams into a memory2 db.

    Trajectory is baked into ``pointlio_lidar`` via the inherited ``@pose_setter_for``.
    The raw Livox stream is NOT recorded here — enable the pcap recorder in the record
    blueprint to capture it. Companion streams are recorded as-is and anchored via the
    static mount frames published on tf.
    """

    # pointlio_odometry / pointlio_lidar are inherited from PointlioRecorder.
    color_image: In[Image]
    realsense_depth_image: In[Image]
    realsense_pointcloud: In[PointCloud2]
    realsense_camera_info: In[CameraInfo]
    realsense_depth_camera_info: In[CameraInfo]


mid360_realsense_record = autoconnect(
    RealSenseCamera.blueprint().remappings(
        [
            (RealSenseCamera, "depth_image", "realsense_depth_image"),
            (RealSenseCamera, "pointcloud", "realsense_pointcloud"),
            (RealSenseCamera, "camera_info", "realsense_camera_info"),
            (RealSenseCamera, "depth_camera_info", "realsense_depth_camera_info"),
        ]
    ),
    Mid360.blueprint().remappings(
        [
            (Mid360, "lidar", "livox_lidar"),
            (Mid360, "imu", "livox_imu"),
        ]
    ),
    PointLio.blueprint(frame_mapping={FRAME_ODOM: "world"}).remappings(
        [
            (PointLio, "lidar", "pointlio_lidar"),
            (PointLio, "odometry", "pointlio_odometry"),
        ]
    ),
    Mid360RealsenseRecorder.blueprint(),
    # Continuously republishes the rig's mount frames onto tf (no latched static tf).
    Mid360RealsenseStaticTf.blueprint(),
).global_config(n_workers=8)

# Same rig, also capturing a raw .pcap of the Mid-360 UDP stream.
mid360_realsense_record_with_pcap = autoconnect(
    mid360_realsense_record,
    Mid360PcapRecorder.blueprint(),
)
