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

from dimos_generated.geometry_msgs.msg import Quaternion, Vector3
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import Detection3DArray
import numpy as np
import pytest

from dimos.msgs.image import image_from_array, image_view
from dimos.msgs.time import time_from_seconds, to_seconds
from dimos.perception.detection.type.detection3d.bbox import Detection3DBBox
from dimos.perception.detection.type.detection3d.imageDetections3D import ImageDetections3D
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker


def _image(ts: float = 123.456) -> Image:
    return image_from_array(
        np.zeros((80, 100, 3), dtype=np.uint8),
        encoding="bgr8",
        header=Header(frame_id="camera_optical", stamp=time_from_seconds(ts)),
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
        ts=to_seconds(image.header.stamp),
        image=image,
        center=Vector3(x=float(marker_id), y=2.0, z=3.0),
        size=Vector3(x=0.16, y=0.16, z=0.0),
        frame_id=frame_id,
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
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
        ts=to_seconds(image.header.stamp),
        image=image,
        center=Vector3(x=1.0, y=2.0, z=3.0),
        size=Vector3(x=0.4, y=0.5, z=0.6),
        frame_id="world",
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )

    msg = ImageDetections3D(image, [det]).to_ros_detection3d_array()

    assert len(msg.detections) == 1
    assert len(msg.detections[0].results) == 1

    decoded = Detection3DArray.decode(msg.encode())
    decoded_det = decoded.detections[0]
    assert len(decoded_det.results) == 1
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
    assert to_seconds(msg.header.stamp) == pytest.approx(to_seconds(image.header.stamp))
    assert len(msg.detections) == 2
    assert len(msg.detections) == 2

    first = msg.detections[0]
    assert first.header.frame_id == "world"
    assert first.id == "7"
    assert len(first.results) == 1
    assert first.results[0].hypothesis.class_id == "DICT_APRILTAG_36h11:7"
    assert first.results[0].hypothesis.score == pytest.approx(1.0)
    assert first.bbox.center.position.x == pytest.approx(7.0)
    assert first.bbox.size.x == pytest.approx(0.16)
    assert first.bbox.size.z == pytest.approx(0.0)

    decoded = Detection3DArray.decode(msg.encode())
    assert decoded.header.frame_id == "world"
    assert len(decoded.detections) == 2
    assert decoded.detections[1].id == "42"
    assert decoded.detections[1].results[0].hypothesis.class_id == "DICT_APRILTAG_36h11:42"


def test_to_ros_detection3d_array_uses_override_and_handles_empty_frames() -> None:
    image = _image()

    msg = ImageDetections3D(image, [_marker(image, marker_id=3)]).to_ros_detection3d_array(
        frame_id="map"
    )

    assert msg.header.frame_id == "map"
    assert to_seconds(msg.header.stamp) == pytest.approx(to_seconds(image.header.stamp))
    assert len(msg.detections) == 1

    empty = ImageDetections3D(image, []).to_ros_detection3d_array(frame_id="world")

    assert empty.header.frame_id == "world"
    assert to_seconds(empty.header.stamp) == pytest.approx(to_seconds(image.header.stamp))
    assert len(empty.detections) == 0
    assert len(empty.detections) == 0


def test_filter_and_annotated_image_work_for_3d_marker_detections() -> None:
    image = _image()
    keep = _marker(image, marker_id=5, confidence=0.95)
    drop = _marker(image, marker_id=6, confidence=0.25)
    detections = ImageDetections3D(image, [keep, drop])

    filtered = detections.filter(lambda det: det.confidence > 0.5)

    assert isinstance(filtered, ImageDetections3D)
    assert filtered.detections == [keep]
    ros_msg = filtered.to_ros_detection3d_array()
    assert len(ros_msg.detections) == 1
    assert ros_msg.detections[0].id == "5"

    annotated = filtered.annotated_image()
    assert to_seconds(annotated.header.stamp) == pytest.approx(to_seconds(image.header.stamp))
    assert np.count_nonzero(image_view(annotated)) > 0
    assert np.count_nonzero(image_view(image)) == 0


def test_marker_pose_and_array_keep_exact_source_stamp() -> None:
    image = _image()
    image.header.stamp.sec = 1700000000
    image.header.stamp.nanosec = 123456789
    marker = _marker(image, marker_id=42)
    pose = marker.pose
    message = marker.to_detection3d_msg()
    array = ImageDetections3D(image, [marker]).to_ros_detection3d_array()
    decoded = Detection3DArray.decode(array.encode())
    assert pose.header.stamp == image.header.stamp
    assert message.header.stamp == image.header.stamp
    assert decoded.header.stamp == image.header.stamp
    assert decoded.detections[0].header.stamp == image.header.stamp
    marker.center.x = 999
    assert pose.pose.position.x == 42
    assert decoded.detections[0].bbox.center.position.x == 42
