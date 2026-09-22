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

"""Shared helpers for fiducial unit tests."""

from __future__ import annotations

import cv2
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import CameraInfo, Image
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.msgs.image import image_from_array
from dimos.msgs.time import time_from_seconds


def camera_info(ts: float = 10.0) -> CameraInfo:
    info = CameraInfo(
        width=640,
        height=480,
        k=[600, 0, 320, 0, 600, 240, 0, 0, 1],
        header=Header(frame_id="camera_optical"),
    )
    info.header.stamp = time_from_seconds(ts)
    return info


def blank_image(ts: float = 10.0) -> Image:
    return image_from_array(
        np.full((480, 640, 3), 255, dtype=np.uint8),
        encoding="bgr8",
        header=Header(frame_id="camera_optical", stamp=time_from_seconds(ts)),
    )


def synthetic_marker_image(marker_id: int = 7, ts: float = 10.0, inverted: bool = False) -> Image:
    """Render a marker on a white field; `inverted` swaps tag and quiet zone to negative.

    The inverted form is what a multicolor 3D print looks like with its two filaments
    swapped: a dark plate margin, a light tag border, and dark data cells.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    side_px = 220
    tile = np.zeros((side_px, side_px), dtype=np.uint8)
    cv2.aruco.generateImageMarker(dictionary, marker_id, side_px, tile)
    if inverted:
        # The quiet zone inverts with the tag, exactly as the printed plate does.
        margin_px = 55
        block = np.full((side_px + 2 * margin_px,) * 2, 255, dtype=np.uint8)
        block[margin_px : margin_px + side_px, margin_px : margin_px + side_px] = tile
        tile = 255 - block
        side_px = tile.shape[0]
    canvas = np.full((480, 640), 255, dtype=np.uint8)
    y0 = (canvas.shape[0] - side_px) // 2
    x0 = (canvas.shape[1] - side_px) // 2
    canvas[y0 : y0 + side_px, x0 : x0 + side_px] = tile
    return image_from_array(
        cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR),
        encoding="bgr8",
        header=Header(frame_id="camera_optical", stamp=time_from_seconds(ts)),
    )


def world_T_optical(ts: float = 10.0) -> TransformStamped:
    return TransformStamped(
        header=Header(frame_id="world", stamp=time_from_seconds(ts)),
        child_frame_id="camera_optical",
        transform=Transform(
            translation=Vector3(x=1.0, y=2.0, z=3.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
