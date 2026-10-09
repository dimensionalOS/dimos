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

from dataclasses import dataclass
from unittest.mock import patch

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import rerun as rr

from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.message_helpers import detection_boxes


@dataclass
class Topic:
    name: str


def _detection_array() -> Detection3DArray:
    det = Detection3D(
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        results=[],
        bbox=BoundingBox3D(
            center=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            size=Vector3(x=0.0, y=0.0, z=0.0),
        ),
        id="",
    )
    det.header = Header(stamp=Time(sec=10, nanosec=0), frame_id="world")
    det.id = "4"
    det.results = [
        ObjectHypothesisWithPose(
            hypothesis=ObjectHypothesis(
                class_id="DICT_APRILTAG_36h11:4",
                score=1.0,
            ),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                covariance=np.zeros(36, dtype=np.float64),
            ),
        )
    ]
    det.bbox = BoundingBox3D(
        center=Pose(
            position=Point(x=1.0, y=2.0, z=3.0),
            orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
        ),
        size=Vector3(x=0.1, y=0.1, z=0.0),
    )
    return Detection3DArray(
        header=Header(stamp=Time(sec=10, nanosec=0), frame_id="world"),
        detections=[det],
    )


def test_detection3darray_bridge_attaches_topic_entity_to_message_frame() -> None:
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}

    try:
        with patch("rerun.log") as mock_log:
            bridge._on_message(
                cdr_decode(cdr_encode(_detection_array()), Detection3DArray),
                Topic("/marker_detection/detections"),
            )
    finally:
        bridge.stop()

    assert mock_log.call_count == 2
    assert mock_log.call_args_list[0].args[0] == "world/marker_detection/detections"
    assert isinstance(mock_log.call_args_list[0].args[1], rr.Boxes3D)
    assert mock_log.call_args_list[1].args[0] == "world/marker_detection/detections"

    transform = mock_log.call_args_list[1].args[1]
    assert isinstance(transform, rr.Transform3D)
    assert transform.parent_frame.as_arrow_array().to_pylist() == ["tf#/world"]


def test_generated_detection_boxes_preserve_geometry_and_labels() -> None:
    boxes = detection_boxes(cdr_decode(cdr_encode(_detection_array()), Detection3DArray))
    assert boxes.centers.as_arrow_array().to_pylist() == [[1, 2, 3]]
    assert boxes.labels.as_arrow_array().to_pylist() == ["DICT_APRILTAG_36h11:4 id=4"]
    assert boxes.quaternions.as_arrow_array().to_pylist() == [[0, 0, 0, 1]]
    empty = detection_boxes(
        Detection3DArray(header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""), detections=[])
    )
    assert empty.centers.as_arrow_array().to_pylist() == []
