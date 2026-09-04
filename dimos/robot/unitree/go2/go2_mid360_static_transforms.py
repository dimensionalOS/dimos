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

Published continuously onto tf (see :class:`Go2Mid360StaticTf`) so the mount geometry
lands in the tf stream and companion streams (camera, go2 lidar) can be anchored to
``base_link``.

Mount geometry (measured on the physical rig)
---------------------------------------------
- base_link -> front_camera: 32.7cm forward, ~4.3cm up (URDF front_camera mount).
- front_camera -> mid360_link: lidar is 3.2cm back, 12cm up, pitched 60 deg down.
- front_camera -> camera_optical: the standard ROS optical rotation (x-right, y-down,
  z-forward).

The published tree is rooted at mid360_link so the static edges stay off the entities
the live odom -> mid360_link edge writes. The tf buffer composes either direction.
"""

from __future__ import annotations

import math

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.tf.static_tf_publisher import (
    FrameSpec,
    StaticTfPublisher,
    frames_to_edge_transforms,
)

MID360_PITCH_DOWN = math.radians(60.0)

# Mount geometry shared by every go2 + mid360 rig (metres).
CAMERA_XYZ = (0.32715, -0.00003, 0.04297)  # base_link -> front_camera
MID360_XYZ = (-0.032, 0.0, 0.12)  # front_camera -> mid360_link: 3.2cm back, 12cm up

# rpy that maps a sensor frame to its optical frame (z-forward, x-right, y-down)
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)

# front_camera -> mid360_link, fixed-axis rpy in degrees, by rig.
MID360_MOUNT_PRESETS: dict[str, tuple[float, float, float]] = {
    # Pointing straight ahead, pitched 60 deg down.
    "SF": (0.0, 60.0, 0.0),
    # The 60 deg tilt lands on roll because this lidar sits yawed 90 deg on its bracket.
    "ATHENS": (-60.0, 0.0, -90.0),
}

FRAMES: list[FrameSpec] = [
    ("base_link", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("front_camera", "base_link", CAMERA_XYZ, (0.0, 0.0, 0.0)),
    ("mid360_link", "front_camera", MID360_XYZ, (0.0, MID360_PITCH_DOWN, 0.0)),
    ("camera_optical", "front_camera", (0.0, 0.0, 0.0), OPTICAL_RPY),
]


def mount_transforms() -> list[Transform]:
    """The mount tree as published: rooted at mid360_link."""
    edges = {t.child_frame_id: t for t in frames_to_edge_transforms(FRAMES)}
    return [-edges["mid360_link"], -edges["front_camera"], edges["camera_optical"]]


def base_link_from_camera_optical() -> Transform:
    """Composed base_link -> camera_optical transform from the static mount tree.

    The known-geometry fallback used to place AprilTags for recordings from this rig
    that carry a ``camera_info`` stream but no tf tree (so the extrinsic can't be read
    from the recording itself)."""
    edges = {t.child_frame_id: t for t in frames_to_edge_transforms(FRAMES)}
    return edges["front_camera"] + edges["camera_optical"]


class Go2Mid360StaticTf(StaticTfPublisher):
    """Publishes the Go2/Mid-360 mount tree onto tf on a fixed interval."""

    def transforms(self) -> list[Transform]:
        return mount_transforms()
