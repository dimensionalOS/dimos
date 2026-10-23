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

"""SDK-independent CDR frame conversions used by the ZED producer."""

from __future__ import annotations

from collections.abc import Sequence

from dimos.hardware.sensors.camera.spec import OPTICAL_ROTATION
from dimos.message_utils.geometry import compose_transforms, inverse_transform
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage


def transform_from_arrays(
    translation: Sequence[float],
    rotation_xyzw: Sequence[float],
    *,
    parent: str,
    child: str,
    stamp: Time,
) -> TransformStamped:
    if len(translation) != 3 or len(rotation_xyzw) != 4:
        raise ValueError("ZED pose requires three translation and four quaternion components")
    return TransformStamped(
        header=Header(frame_id=parent, stamp=stamp),
        child_frame_id=child,
        transform=Transform(
            translation=Vector3(
                x=float(translation[0]), y=float(translation[1]), z=float(translation[2])
            ),
            rotation=Quaternion(
                x=float(rotation_xyzw[0]),
                y=float(rotation_xyzw[1]),
                z=float(rotation_xyzw[2]),
                w=float(rotation_xyzw[3]),
            ),
        ),
    )


def tracking_to_base(
    world_to_camera: TransformStamped,
    base_transform: Transform | None,
    base_frame: str,
) -> TransformStamped:
    if base_transform is None:
        return world_to_camera
    mount = TransformStamped(
        header=Header(frame_id=base_frame, stamp=world_to_camera.header.stamp),
        child_frame_id=world_to_camera.child_frame_id,
        transform=base_transform,
    )
    return compose_transforms(world_to_camera, inverse_transform(mount))


def frame_message(
    *,
    camera_name: str,
    base_frame: str,
    base_transform: Transform | None,
    sdk_extrinsics: Transform,
    stamp: Time,
    tracking: TransformStamped | None = None,
) -> TFMessage:
    camera_link = camera_name + "_link"
    transforms = []
    if base_transform is not None:
        transforms.append(
            TransformStamped(
                header=Header(frame_id=base_frame, stamp=stamp),
                child_frame_id=camera_link,
                transform=base_transform,
            )
        )
    for kind in ["depth", "color"]:
        frame = camera_name + "_" + kind + "_frame"
        optical = camera_name + "_" + kind + "_optical_frame"
        # Preserve the producer's existing inverse SDK extrinsics convention.
        edge = inverse_transform(
            TransformStamped(
                header=Header(frame_id=camera_link, stamp=stamp),
                child_frame_id=frame,
                transform=sdk_extrinsics,
            )
        )
        edge.header.frame_id, edge.child_frame_id = camera_link, frame
        transforms.append(edge)
        transforms.append(
            TransformStamped(
                header=Header(frame_id=frame, stamp=stamp),
                child_frame_id=optical,
                transform=Transform(rotation=OPTICAL_ROTATION, translation=Vector3(x=0, y=0, z=0)),
            )
        )
    if tracking is not None:
        transforms.append(tracking)
    return TFMessage(transforms=transforms)
