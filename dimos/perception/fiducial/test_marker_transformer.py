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

import cv2
import numpy as np
import pytest

pytest.importorskip("cv2.aruco")

from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.fiducial.marker_transformer import (
    DetectMarkers,
    detect_markers_in_image,
)


def _camera_info(ts: float = 10.0) -> CameraInfo:
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


def _synthetic_marker_image(marker_id: int = 7, ts: float = 10.0) -> Image:
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


def _world_T_optical(ts: float = 10.0) -> Transform:
    return Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
        child_frame_id="camera_optical",
        ts=ts,
    )


def test_detect_markers_in_image_builds_rich_marker_detection() -> None:
    marker_id = 7
    marker_length_m = 0.18
    image = _synthetic_marker_image(marker_id)
    info = _camera_info(image.ts)

    detections = detect_markers_in_image(
        image,
        camera_info=info,
        world_T_optical=_world_T_optical(image.ts),
        marker_length_m=marker_length_m,
        aruco_dictionary="DICT_APRILTAG_36h11",
    )

    assert len(detections) == 1
    det = detections[0]
    assert det.marker_id == marker_id
    assert det.name == "DICT_APRILTAG_36h11:7"
    assert det.image is image
    assert det.frame_id == "world"
    assert det.size.x == pytest.approx(marker_length_m)
    assert det.size.y == pytest.approx(marker_length_m)
    assert det.size.z == pytest.approx(0.0)
    assert det.confidence == pytest.approx(1.0)
    assert det.reprojection_error < 0.1
    assert det.bbox == pytest.approx((210.0, 130.0, 429.0, 349.0), abs=2.0)
    assert det.center.x == pytest.approx(1.0, abs=0.02)
    assert det.center.y == pytest.approx(2.0, abs=0.02)
    assert det.center.z > 3.3

    msg = det.to_detection3d_msg()
    assert msg.id == str(marker_id)
    assert msg.results[0].hypothesis.class_id == "DICT_APRILTAG_36h11:7"


def test_detect_markers_in_image_returns_empty_for_no_marker_frame() -> None:
    ts = 11.0
    image = Image(
        data=np.full((480, 640, 3), 255, dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=ts,
    )

    detections = detect_markers_in_image(
        image,
        camera_info=_camera_info(ts),
        world_T_optical=_world_T_optical(ts),
        marker_length_m=0.18,
        aruco_dictionary="DICT_APRILTAG_36h11",
    )

    assert detections == []


def test_detect_markers_transformer_preserves_observation_context_and_tags() -> None:
    marker_id = 7
    image = _synthetic_marker_image(marker_id, ts=12.0)
    obs = Observation[Image](
        id=42,
        ts=image.ts,
        data_type=Image,
        pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        _data=image,
    )
    transformer = DetectMarkers(
        camera_info=_camera_info(image.ts),
        marker_length_m=0.18,
        aruco_dictionary="DICT_APRILTAG_36h11",
    )

    results = list(transformer(iter([obs])))

    assert len(results) == 1
    out = results[0]
    assert out.id == obs.id
    assert out.ts == obs.ts
    assert out.data.marker_id == marker_id
    assert out.data.image is image
    assert out.pose is not None
    assert out.tags["marker_id"] == marker_id
    assert out.tags["track_id"] == marker_id
