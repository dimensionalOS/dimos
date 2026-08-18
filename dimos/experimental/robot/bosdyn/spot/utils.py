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

"""Pure helpers for Spot: URDF mount extrinsics and bosdyn image decoding."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.model_parser import JointDescription, parse_model
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def decode_image(response: Any, frame_id: str, time_converter: Any) -> Image | None:
    """Turn a bosdyn ImageResponse into a dimos Image, or None if unsupported.

    Stamps each image with its true capture time: the robot-clock `acquisition_time`
    converted to local time by `time_converter`, bosdyn's live clock-skew estimate
    (`RobotTimeConverter`). Polling faster than the sensor returns the same frame, so
    keeping the sensor timestamp lets downstream drop the repeat instead of seeing a
    fresh wall-clock stamp.
    """
    from bosdyn.api import image_pb2  # type: ignore[import-not-found]

    shot = response.shot.image
    pixel_format = shot.pixel_format
    ts = time_converter.local_seconds_from_robot_timestamp(response.shot.acquisition_time)

    if shot.format == image_pb2.Image.FORMAT_JPEG:
        import cv2

        buffer = np.frombuffer(shot.data, dtype=np.uint8)
        decoded = cv2.imdecode(buffer, cv2.IMREAD_UNCHANGED)
        if decoded is None:
            logger.error(f"Failed to decode JPEG image from {frame_id}")
            return None
        image_format = ImageFormat.GRAY if decoded.ndim == 2 else ImageFormat.BGR
        return Image.from_numpy(decoded, format=image_format, frame_id=frame_id, ts=ts)

    if shot.format != image_pb2.Image.FORMAT_RAW:
        logger.error(f"Unsupported Spot image encoding {shot.format} from {frame_id}")
        return None

    dtype, channels, image_format = raw_layout(pixel_format)
    if dtype is None:
        logger.error(f"Unsupported Spot pixel format {pixel_format} from {frame_id}")
        return None

    array = np.frombuffer(shot.data, dtype=dtype)
    array = (
        array.reshape(shot.rows, shot.cols)
        if channels == 1
        else array.reshape(shot.rows, shot.cols, channels)
    )
    return Image.from_numpy(array, format=image_format, frame_id=frame_id, ts=ts)


def joint_to_transform(joint: JointDescription) -> Transform:
    return Transform(
        translation=Vector3(*joint.origin_xyz),
        rotation=Quaternion.from_euler(Vector3(*joint.origin_rpy)),
        frame_id=joint.parent_link,
        child_frame_id=joint.child_link,
    )


def camera_mount_transforms(
    urdf_path: str | Path, base_frame_id: str, optical_frames: list[str]
) -> list[Transform]:
    """Compose each base_frame_id -> optical_frame extrinsic from the URDF's fixed joints.

    Walks the fixed-joint chain (base_link -> body -> {pos}_camera -> optical) up
    from each optical frame and folds the per-joint origins into one transform, so
    the recorded images resolve a pose against the live odom->base_link edge.
    """
    model = parse_model(urdf_path)
    urdf_root = model.root_link
    joint_by_child = {joint.child_link: joint for joint in model.joints}
    transforms: list[Transform] = []
    for optical_frame in optical_frames:
        chain: list[JointDescription] = []
        current = optical_frame
        while current != urdf_root and current in joint_by_child:
            joint = joint_by_child[current]
            chain.append(joint)
            current = joint.parent_link
        if current != urdf_root:
            logger.warning(f"URDF has no fixed chain from {urdf_root} to {optical_frame}")
            continue
        edges = [joint_to_transform(joint) for joint in reversed(chain)]
        composed = edges[0]
        for edge in edges[1:]:
            composed = composed + edge
        composed.frame_id = base_frame_id
        transforms.append(composed)
    return transforms


def roll_optical_frame(transform: Transform, quarter_turns: int) -> Transform:
    """Roll a camera's optical frame `quarter_turns` * 90° about its viewing (z) axis.

    Pairs with `rotate_image_quarter_turns`: rotating the image alone leaves the 3D
    frame at its raw mount orientation, so frustums and depth back-projection land
    rotated. Rolling the frame by the same amount realigns 3D with the upright image.
    """
    if not quarter_turns:
        return transform
    roll = Quaternion.from_euler(Vector3(0.0, 0.0, quarter_turns * math.pi / 2))
    return Transform(
        translation=transform.translation,
        rotation=transform.rotation * roll,
        frame_id=transform.frame_id,
        child_frame_id=transform.child_frame_id,
        ts=transform.ts,
    )


def rotate_image_quarter_turns(image: Image, quarter_turns: int) -> Image:
    """Rotate an Image by `quarter_turns` * 90° CCW (negative for CW)."""
    rotated = np.rot90(image.data, k=quarter_turns)
    return Image.from_numpy(rotated, format=image.format, frame_id=image.frame_id, ts=image.ts)


def rotate_camera_info_quarter_turns(info: CameraInfo, quarter_turns: int) -> CameraInfo:
    """Rotate a pinhole CameraInfo to match `rotate_image_quarter_turns`.

    Each CCW quarter turn swaps the focal lengths and remaps the principal point so
    the intrinsics stay consistent with the rotated pixel grid (width/height swap).
    """
    fx, fy, cx, cy = info.K[0], info.K[4], info.K[2], info.K[5]
    width, height = info.width, info.height
    for _ in range(quarter_turns % 4):
        fx, fy = fy, fx
        cx, cy = cy, (width - 1) - cx
        width, height = height, width
    return CameraInfo.from_intrinsics(
        fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height, frame_id=info.frame_id
    ).with_ts(info.ts)


def camera_info_from_response(response: Any, source_name: str, ts: float) -> CameraInfo | None:
    """Build a CameraInfo from a bosdyn image response's pinhole intrinsics."""
    source = response.source
    if not source.HasField("pinhole"):
        return None
    intrinsics = source.pinhole.intrinsics
    info = CameraInfo.from_intrinsics(
        fx=intrinsics.focal_length.x,
        fy=intrinsics.focal_length.y,
        cx=intrinsics.principal_point.x,
        cy=intrinsics.principal_point.y,
        width=source.cols,
        height=source.rows,
        frame_id=source_name,
    )
    return info.with_ts(ts)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def raw_layout(pixel_format: int) -> tuple[Any, int, ImageFormat]:
    from bosdyn.api import image_pb2  # type: ignore[import-not-found]

    layouts: dict[int, tuple[Any, int, ImageFormat]] = {
        image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8: (np.uint8, 1, ImageFormat.GRAY),
        image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16: (np.uint16, 1, ImageFormat.GRAY16),
        image_pb2.Image.PIXEL_FORMAT_DEPTH_U16: (np.uint16, 1, ImageFormat.DEPTH16),
        image_pb2.Image.PIXEL_FORMAT_RGB_U8: (np.uint8, 3, ImageFormat.RGB),
        image_pb2.Image.PIXEL_FORMAT_RGBA_U8: (np.uint8, 4, ImageFormat.RGBA),
    }
    return layouts.get(pixel_format, (None, 0, ImageFormat.GRAY))
