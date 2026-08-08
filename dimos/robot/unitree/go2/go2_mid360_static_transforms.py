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
from functools import lru_cache

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.protocol.tf.static_tf_publisher import (
    FrameSpec,
    StaticTfPublisher,
    frames_to_edge_transforms,
)
from dimos.protocol.tf.tf import MultiTBuffer

MID360_PITCH_DOWN = math.radians(60.0)
# Point-LIO worlds use the start sensor pose as the origin (z≈0), not the floor.
# A raw mid360→base_link compose drops the body below z=0, so the Go2 box looks
# half-buried under bottom_cutoff=0 / the z=0.5 grid. Lift by the usual Go2
# standing height so viz matches go2_odom (~0.32 m).
POINTLIO_BASE_VIZ_Z_OFFSET = 0.32

# rpy that maps a sensor frame to its optical frame (z-forward, x-right, y-down)
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)

FRAMES: list[FrameSpec] = [
    ("base_link", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("front_camera", "base_link", (0.32715, -0.00003, 0.04297), (0.0, 0.0, 0.0)),
    ("mid360_link", "front_camera", (-0.032, 0.0, 0.12), (0.0, MID360_PITCH_DOWN, 0.0)),
    ("camera_optical", "front_camera", (0.0, 0.0, 0.0), OPTICAL_RPY),
]


def mount_transforms() -> list[Transform]:
    """The mount tree as published: rooted at mid360_link."""
    edges = {t.child_frame_id: t for t in frames_to_edge_transforms(FRAMES)}
    return [-edges["mid360_link"], -edges["front_camera"], edges["camera_optical"]]


@lru_cache(maxsize=1)
def mid360_link_to_base_link() -> Transform:
    """Static ``mid360_link`` → ``base_link`` from the measured mount tree."""
    buffer = MultiTBuffer()
    buffer.receive_transform(*mount_transforms())
    leg = buffer.get("mid360_link", "base_link")
    if leg is None:
        raise RuntimeError("failed to compose mid360_link → base_link from mount_transforms()")
    return leg


def pointlio_odom_to_base_link_pose(odom: Odometry) -> PoseStamped:
    """Convert Point-LIO ``world``→``mid360_link`` odom to a ``base_link`` pose.

    ``GO2Connection`` publishes replay odom as ``world``→``base_link`` for the
    Go2 box. Point-LIO's child frame is the pitched lidar (~60° down); composing
    through the mount puts the visualized robot along the body / motion axis.

    Z is taken from the sensor height plus :data:`POINTLIO_BASE_VIZ_Z_OFFSET`
    so the box is not half-cut by ``bottom_cutoff=0`` in Point-LIO worlds.
    """
    sensor_pose = odom.to_pose_stamped()
    world_to_sensor = Transform.from_pose("mid360_link", sensor_pose)
    world_to_base = world_to_sensor + mid360_link_to_base_link()
    lifted = Transform(
        translation=Vector3(
            world_to_base.translation.x,
            world_to_base.translation.y,
            world_to_sensor.translation.z + POINTLIO_BASE_VIZ_Z_OFFSET,
        ),
        rotation=world_to_base.rotation,
        frame_id=world_to_base.frame_id,
        child_frame_id=world_to_base.child_frame_id,
        ts=world_to_base.ts,
    )
    return lifted.to_pose(ts=odom.ts)


class Go2Mid360StaticTf(StaticTfPublisher):
    """Publishes the Go2/Mid-360 mount tree onto tf on a fixed interval."""

    def transforms(self) -> list[Transform]:
        return mount_transforms()
