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

from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
import pytest
import rerun as rr

from dimos.msgs.time import time_from_seconds
from dimos.visualization.rerun.message_helpers import detection_boxes


def _detection3d(
    *,
    ts: float = 12.5,
    frame_id: str = "world",
    marker_id: str = "7",
    class_id: str = "DICT_APRILTAG_36h11:7",
) -> Detection3D:
    det = Detection3D()
    det.header = Header(stamp=time_from_seconds(ts), frame_id=frame_id)
    det.id = marker_id
    det.results = [
        ObjectHypothesisWithPose(
            hypothesis=ObjectHypothesis(
                class_id=class_id,
                score=1.0,
            )
        )
    ]
    det.bbox = BoundingBox3D(
        center=Pose(
            position=Point(x=1.0, y=2.0, z=3.0),
            orientation=Quaternion(z=0.70710678, w=0.70710678),
        ),
        size=Vector3(x=0.2, y=0.4),
    )
    return det


def test_detection3d_frame_id_comes_from_header() -> None:
    det = _detection3d(frame_id="map")

    assert det.header.frame_id == "map"


def test_detection3darray_to_rerun_preserves_wire_pose_size_and_identity() -> None:
    msg = Detection3DArray(
        header=Header(stamp=time_from_seconds(12.5), frame_id="world"),
        detections=[_detection3d()],
    )

    boxes = detection_boxes(msg)

    assert msg.header.frame_id == "world"
    assert isinstance(boxes, rr.Boxes3D)
    assert boxes.centers.as_arrow_array().to_pylist() == [[1.0, 2.0, 3.0]]
    assert boxes.half_sizes.as_arrow_array().to_pylist()[0] == pytest.approx([0.1, 0.2, 0.0])
    assert boxes.quaternions.as_arrow_array().to_pylist()[0] == pytest.approx(
        [0.0, 0.0, 0.70710678, 0.70710678]
    )
    assert boxes.labels.as_arrow_array().to_pylist() == ["DICT_APRILTAG_36h11:7 id=7"]


def test_detection3darray_to_rerun_empty_array_is_safe() -> None:
    msg = Detection3DArray(
        header=Header(stamp=time_from_seconds(12.5), frame_id="world"),
        detections=[],
    )

    boxes = detection_boxes(msg)

    assert isinstance(boxes, rr.Boxes3D)
    assert boxes.centers.as_arrow_array().to_pylist() == []
    assert boxes.labels.as_arrow_array().to_pylist() == []
