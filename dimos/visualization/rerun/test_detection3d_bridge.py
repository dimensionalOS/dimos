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

import numpy as np
import rerun as rr

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.messages.vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule


@dataclass
class Topic:
    name: str


def _detection_array() -> Detection3DArray:
    pose = Pose(Point(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    header = Header(Time(10, 0), "world")
    det = Detection3D(
        header=header,
        id="4",
        results=[
            ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesis(class_id="DICT_APRILTAG_36h11:4", score=1.0),
                pose=PoseWithCovariance(pose=pose, covariance=np.zeros(36, dtype=np.float64)),
            )
        ],
        bbox=BoundingBox3D(center=pose, size=Vector3(0.1, 0.1, 0.0)),
    )
    return Detection3DArray(header=header, detections=[det])


def test_detection3darray_bridge_attaches_topic_entity_to_message_frame() -> None:
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}

    try:
        with patch("rerun.log") as mock_log:
            bridge._on_message(_detection_array(), Topic("/marker_detection/detections"))
    finally:
        bridge.stop()

    assert mock_log.call_count == 3
    assert mock_log.call_args_list[0].args[0] == "world/marker_detection/detections"
    assert isinstance(mock_log.call_args_list[0].args[1], rr.Boxes3D)
    assert mock_log.call_args_list[1].args[0] == "world/marker_detection/detections/message"
    assert mock_log.call_args_list[2].args[0] == "world/marker_detection/detections"

    transform = mock_log.call_args_list[2].args[1]
    assert isinstance(transform, rr.Transform3D)
    assert transform.parent_frame.as_arrow_array().to_pylist() == ["tf#/world"]
