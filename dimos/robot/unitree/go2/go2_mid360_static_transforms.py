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
mount geometry lands in the recording's tf stream and companion streams (camera, go2
lidar) can be anchored to ``base_link``.

Mount geometry (measured on the physical rig)
---------------------------------------------
- base_link -> front_camera: 32.7cm forward, ~4.3cm up (URDF front_camera mount).
- front_camera -> mid360_link: lidar is 3.2cm back, 12cm up, pitched 44 deg down.
- front_camera -> camera_optical: the standard ROS optical rotation (x-right, y-down,
  z-forward).
"""

from __future__ import annotations

import math

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.tf.static_tf_publisher import (
    FrameSpec,
    StaticTfPublisher,
    frames_to_edge_transforms,
)

MID360_PITCH_DOWN = math.radians(44.0)

# rpy that maps a sensor frame to its optical frame (z-forward, x-right, y-down)
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)

FRAMES: list[FrameSpec] = [
    ("base_link", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("front_camera", "base_link", (0.32715, -0.00003, 0.04297), (0.0, 0.0, 0.0)),
    ("mid360_link", "front_camera", (-0.032, 0.0, 0.12), (0.0, MID360_PITCH_DOWN, 0.0)),
    ("camera_optical", "front_camera", (0.0, 0.0, 0.0), OPTICAL_RPY),
]


def base_link_from_mid360() -> Transform:
    """Composed base_link -> mid360_link transform from the static mount tree."""
    edges = {t.child_frame_id: t for t in frames_to_edge_transforms(FRAMES)}
    return edges["front_camera"] + edges["mid360_link"]


class Go2Mid360StaticTf(StaticTfPublisher):
    """Publishes the Go2/Mid-360 mount tree onto tf on a fixed interval."""

    def transforms(self) -> list[Transform]:
        return frames_to_edge_transforms(FRAMES)
