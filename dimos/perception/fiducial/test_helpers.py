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
import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def camera_info(ts: float = 10.0) -> CameraInfo:
    info = CameraInfo.from_intrinsics(
        fx=600.0,
        fy=600.0,
        cx=320.0,
        cy=240.0,
        width=640,
        height=480,
        frame_id="camera_optical",
    )
    info.ts = ts
    return info


def blank_image(ts: float = 10.0) -> Image:
    return Image(
        data=np.full((480, 640, 3), 255, dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=ts,
    )


def synthetic_marker_image(marker_id: int = 7, ts: float = 10.0) -> Image:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    side_px = 220
    tile = np.zeros((side_px, side_px), dtype=np.uint8)
    cv2.aruco.generateImageMarker(dictionary, marker_id, side_px, tile)
    canvas = np.full((480, 640), 255, dtype=np.uint8)
    y0 = (canvas.shape[0] - side_px) // 2
    x0 = (canvas.shape[1] - side_px) // 2
    canvas[y0 : y0 + side_px, x0 : x0 + side_px] = tile
    return Image(
        data=cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=ts,
    )


def world_T_optical(ts: float = 10.0) -> Transform:
    return Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
        child_frame_id="camera_optical",
        ts=ts,
    )
