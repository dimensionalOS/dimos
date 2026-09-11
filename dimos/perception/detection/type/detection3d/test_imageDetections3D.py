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

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.bbox import Detection3DBBox
from dimos.perception.detection.type.detection3d.imageDetections3D import ImageDetections3D
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker


def _image(ts: float = 123.456) -> Image:
    return Image(
        data=np.zeros((80, 100, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=ts,
    )


def _marker(
    image: Image,
    *,
    marker_id: int,
    confidence: float = 1.0,
    frame_id: str = "world",
) -> Detection3DMarker:
    x1 = 10.0 + marker_id
    y1 = 12.0 + marker_id
    x2 = x1 + 20.0
    y2 = y1 + 18.0
    return Detection3DMarker(
        bbox=(x1, y1, x2, y2),
        track_id=-1,
        class_id=marker_id,
        confidence=confidence,
        name="",
        ts=image.ts,
        image=image,
        center=Vector3(float(marker_id), 2.0, 3.0),
        size=Vector3(0.16, 0.16, 0.0),
        frame_id=frame_id,
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        marker_id=marker_id,
        corners_px=np.array(
            [[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
            dtype=np.float32,
        ),
        dictionary="DICT_APRILTAG_36h11",
        reprojection_error=0.05,
    )


def test_to_ros_detection3d_array_serializes_plain_bbox_results() -> None:
    image = _image()
    det = Detection3DBBox(
        bbox=(4.0, 5.0, 20.0, 25.0),
        track_id=-1,
        class_id=9,
        confidence=0.75,
        name="box",
        ts=image.ts,
        image=image,
        center=Vector3(1.0, 2.0, 3.0),
        size=Vector3(0.4, 0.5, 0.6),
        frame_id="world",
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    msg = ImageDetections3D(image, [det]).to_ros_detection3d_array()

    assert msg.detections_length == 1
    assert msg.detections[0].results_length == 1

    decoded = Detection3DArray.lcm_decode(msg.lcm_encode())
    decoded_det = decoded.detections[0]
    assert decoded_det.results_length == 1
    assert decoded_det.results[0].hypothesis.class_id == "9"
    assert decoded_det.results[0].hypothesis.score == pytest.approx(0.75)
    assert decoded_det.bbox.center.position.x == pytest.approx(1.0)
    assert decoded_det.bbox.size.z == pytest.approx(0.6)


def test_to_ros_detection3d_array_preserves_marker_wire_identity() -> None:
    image = _image()
    detections = ImageDetections3D(
        image,
        [
            _marker(image, marker_id=7),
            _marker(image, marker_id=42),
        ],
    )

    msg = detections.to_ros_detection3d_array()

    assert msg.header.frame_id == "world"
    assert msg.ts == pytest.approx(image.ts)
    assert msg.detections_length == 2
    assert len(msg.detections) == 2

    first = msg.detections[0]
    assert first.header.frame_id == "world"
    assert first.id == "7"
    assert first.results_length == 1
    assert first.results[0].hypothesis.class_id == "DICT_APRILTAG_36h11:7"
    assert first.results[0].hypothesis.score == pytest.approx(1.0)
    assert first.bbox.center.position.x == pytest.approx(7.0)
    assert first.bbox.size.x == pytest.approx(0.16)
    assert first.bbox.size.z == pytest.approx(0.0)

    decoded = Detection3DArray.lcm_decode(msg.lcm_encode())
    assert decoded.header.frame_id == "world"
    assert decoded.detections_length == 2
    assert decoded.detections[1].id == "42"
    assert decoded.detections[1].results[0].hypothesis.class_id == "DICT_APRILTAG_36h11:42"


def test_to_ros_detection3d_array_uses_override_and_handles_empty_frames() -> None:
    image = _image()

    msg = ImageDetections3D(image, [_marker(image, marker_id=3)]).to_ros_detection3d_array(
        frame_id="map"
    )

    assert msg.header.frame_id == "map"
    assert msg.ts == pytest.approx(image.ts)
    assert msg.detections_length == 1

    empty = ImageDetections3D(image, []).to_ros_detection3d_array(frame_id="world")

    assert empty.header.frame_id == "world"
    assert empty.ts == pytest.approx(image.ts)
    assert empty.detections_length == 0
    assert empty.detections == []


def test_filter_and_annotated_image_work_for_3d_marker_detections() -> None:
    image = _image()
    keep = _marker(image, marker_id=5, confidence=0.95)
    drop = _marker(image, marker_id=6, confidence=0.25)
    detections = ImageDetections3D(image, [keep, drop])

    filtered = detections.filter(lambda det: det.confidence > 0.5)

    assert isinstance(filtered, ImageDetections3D)
    assert filtered.detections == [keep]
    ros_msg = filtered.to_ros_detection3d_array()
    assert ros_msg.detections_length == 1
    assert ros_msg.detections[0].id == "5"

    annotated = filtered.annotated_image()
    assert annotated.ts == pytest.approx(image.ts)
    assert np.count_nonzero(annotated.data) > 0
    assert np.count_nonzero(image.data) == 0
