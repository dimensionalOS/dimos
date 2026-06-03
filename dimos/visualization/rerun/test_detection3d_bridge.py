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

from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import rerun as rr

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.visualization.rerun.bridge import RerunBridgeModule


@dataclass
class Topic:
    name: str


def _detection_array() -> Detection3DArray:
    det = Detection3D()
    det.header = Header(10.0, "world")
    det.id = "4"
    det.results = [
        ObjectHypothesisWithPose(
            hypothesis=ObjectHypothesis(
                class_id="DICT_APRILTAG_36h11:4",
                score=1.0,
            )
        )
    ]
    det.results_length = len(det.results)
    det.bbox = BoundingBox3D(
        center=Pose(
            position=Vector3(1.0, 2.0, 3.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        size=Vector3(0.1, 0.1, 0.0),
    )
    return Detection3DArray(
        header=Header(10.0, "world"),
        detections=[det],
        detections_length=1,
    )


def test_detection3darray_bridge_attaches_topic_entity_to_message_frame() -> None:
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}

    try:
        with patch("dimos.visualization.rerun.bridge.rr.log") as mock_log:
            bridge._on_message(_detection_array(), Topic("/marker_detection/detections_3d"))
    finally:
        bridge.stop()

    assert mock_log.call_count == 2
    assert mock_log.call_args_list[0].args[0] == "world/marker_detection/detections_3d"
    assert isinstance(mock_log.call_args_list[0].args[1], rr.Boxes3D)
    assert mock_log.call_args_list[1].args[0] == "world/marker_detection/detections_3d"

    transform = mock_log.call_args_list[1].args[1]
    assert isinstance(transform, rr.Transform3D)
    assert transform.parent_frame.as_arrow_array().to_pylist() == ["tf#/world"]
