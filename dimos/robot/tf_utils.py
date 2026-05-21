# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from collections.abc import Iterable

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

# REP-103/REP-105 optical-frame rotation: camera_link -> camera_optical.
_CAMERA_OPTICAL_ROTATION = Quaternion(-0.5, 0.5, -0.5, 0.5)


def odom_to_tf(
    odom: PoseStamped,
    *,
    camera_link_offset: Vector3 = Vector3(0.3, 0.0, 0.0),
    with_optical: bool = True,
    extras: Iterable[Transform] = (),
) -> list[Transform]:
    """Build the standard TF chain from an odometry pose.

    Produces, in order:
      odom.frame_id -> base_link        (from the pose)
      base_link     -> camera_link      (translation = camera_link_offset)
      camera_link   -> camera_optical   (only if with_optical)
      ...extras
    """
    transforms: list[Transform] = [Transform.from_pose("base_link", odom)]
    transforms.append(
        Transform(
            translation=camera_link_offset,
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=odom.ts,
        )
    )
    if with_optical:
        transforms.append(
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=_CAMERA_OPTICAL_ROTATION,
                frame_id="camera_link",
                child_frame_id="camera_optical",
                ts=odom.ts,
            )
        )
    transforms.extend(extras)
    return transforms
