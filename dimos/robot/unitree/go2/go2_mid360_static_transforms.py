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

"""Static mount frames for the Go2 + Mid-360 + front-camera rig.

Published continuously onto tf while recording (see :class:`Go2Mid360StaticTf`) so the
mount geometry lands in the recording's tf stream.

The tree is rooted at ``world``, with identity ``world -> map -> odom`` edges.
Point-LIO runs with identity extrinsics: its odometry child frame IS the sensor
frame, so odometry supplies the moving ``odom -> mid360_link`` edge and the rest
of the robot hangs off it:

    world -> map -> odom -> mid360_link -> base_link -> front_camera -> camera_optical

Mount geometry (measured on the physical rig)
---------------------------------------------
- base_link -> front_camera: 32.7cm forward, ~4.3cm up (URDF front_camera mount).
- front_camera -> mid360_link: lidar is 3.2cm back, 12cm up, pitched 44 deg down.
- front_camera -> camera_optical: the standard ROS optical rotation (x-right, y-down,
  z-forward).

``mid360_link -> base_link`` is derived as the inverse of the composed mount
(base_link -> front_camera -> mid360_link) rather than measured directly.
"""

from __future__ import annotations

import math

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.tf.static_tf_publisher import (
    FrameSpec,
    StaticTfPublisher,
    frames_to_edge_transforms,
)

MID360_PITCH_DOWN = math.radians(44.0)

# rpy that maps a sensor frame to its optical frame (z-forward, x-right, y-down)
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)

BASE_TO_CAMERA_XYZ = (0.32715, -0.00003, 0.04297)
CAMERA_TO_MID360_XYZ = (-0.032, 0.0, 0.12)
CAMERA_TO_MID360_RPY = (0.0, MID360_PITCH_DOWN, 0.0)


def _mid360_to_base() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    base_to_camera = Transform(
        translation=Vector3(*BASE_TO_CAMERA_XYZ),
        rotation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
    )
    camera_to_mid360 = Transform(
        translation=Vector3(*CAMERA_TO_MID360_XYZ),
        rotation=Quaternion.from_euler(Vector3(*CAMERA_TO_MID360_RPY)),
    )
    mid360_to_base = (base_to_camera + camera_to_mid360).inverse()
    translation = mid360_to_base.translation
    rpy = mid360_to_base.rotation.to_euler()
    return (translation.x, translation.y, translation.z), (rpy.x, rpy.y, rpy.z)


MID360_TO_BASE_XYZ, MID360_TO_BASE_RPY = _mid360_to_base()

FRAMES: list[FrameSpec] = [
    ("world", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("map", "world", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("odom", "map", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    # odom -> mid360_link is the moving edge, published by Point-LIO.
    ("base_link", "mid360_link", MID360_TO_BASE_XYZ, MID360_TO_BASE_RPY),
    ("front_camera", "base_link", BASE_TO_CAMERA_XYZ, (0.0, 0.0, 0.0)),
    ("camera_optical", "front_camera", (0.0, 0.0, 0.0), OPTICAL_RPY),
]


class Go2Mid360StaticTf(StaticTfPublisher):
    """Publishes the Go2/Mid-360 mount tree onto tf on a fixed interval."""

    def transforms(self) -> list[Transform]:
        return frames_to_edge_transforms(FRAMES)
