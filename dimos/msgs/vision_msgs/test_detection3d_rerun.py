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

from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import pytest
import rerun as rr

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray


def _detection3d(
    *,
    ts: float = 12.5,
    frame_id: str = "world",
    marker_id: str = "7",
    class_id: str = "DICT_APRILTAG_36h11:7",
) -> Detection3D:
    det = Detection3D()
    det.header = Header(ts, frame_id)
    det.id = marker_id
    det.results = [
        ObjectHypothesisWithPose(
            hypothesis=ObjectHypothesis(
                class_id=class_id,
                score=1.0,
            )
        )
    ]
    det.results_length = len(det.results)
    det.bbox = BoundingBox3D(
        center=Pose(
            position=Vector3(1.0, 2.0, 3.0),
            orientation=Quaternion(0.0, 0.0, 0.70710678, 0.70710678),
        ),
        size=Vector3(0.2, 0.4, 0.0),
    )
    return det


def test_detection3d_frame_id_comes_from_header() -> None:
    det = _detection3d(frame_id="map")

    assert det.frame_id == "map"


def test_detection3darray_to_rerun_preserves_wire_pose_size_and_identity() -> None:
    msg = Detection3DArray(
        header=Header(12.5, "world"),
        detections=[_detection3d()],
        detections_length=1,
    )

    boxes = msg.to_rerun()

    assert msg.frame_id == "world"
    assert isinstance(boxes, rr.Boxes3D)
    assert boxes.centers.as_arrow_array().to_pylist() == [[1.0, 2.0, 3.0]]
    assert boxes.half_sizes.as_arrow_array().to_pylist()[0] == pytest.approx([0.1, 0.2, 0.0])
    assert boxes.quaternions.as_arrow_array().to_pylist()[0] == pytest.approx(
        [0.0, 0.0, 0.70710678, 0.70710678]
    )
    assert boxes.labels.as_arrow_array().to_pylist() == ["DICT_APRILTAG_36h11:7 id=7"]


def test_detection3darray_to_rerun_empty_array_is_safe() -> None:
    msg = Detection3DArray(
        header=Header(12.5, "world"),
        detections=[],
        detections_length=0,
    )

    boxes = msg.to_rerun()

    assert isinstance(boxes, rr.Boxes3D)
    assert boxes.centers.as_arrow_array().to_pylist() == []
    assert boxes.labels.as_arrow_array().to_pylist() == []
