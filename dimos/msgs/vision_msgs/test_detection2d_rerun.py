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

from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
import pytest
import rerun as rr

from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2D import Detection2D
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray


def _detection2d(
    *,
    ts: float = 12.5,
    frame_id: str = "camera_optical",
    marker_id: str = "7",
    class_id: str = "DICT_APRILTAG_36h11:7",
    center_x: float = 100.0,
    center_y: float = 200.0,
    width: float = 30.0,
    height: float = 40.0,
) -> Detection2D:
    det = Detection2D()
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
    det.bbox = BoundingBox2D(
        center=Pose2D(position=Point2D(x=center_x, y=center_y), theta=0.0),
        size_x=width,
        size_y=height,
    )
    return det


def test_detection2d_frame_id_comes_from_header() -> None:
    msg = Detection2DArray(
        header=Header(12.5, "camera_optical"),
        detections=[_detection2d()],
        detections_length=1,
    )

    assert msg.frame_id == "camera_optical"


def test_detection2darray_to_rerun_preserves_pixel_bbox_and_identity() -> None:
    msg = Detection2DArray(
        header=Header(12.5, "camera_optical"),
        detections=[_detection2d()],
        detections_length=1,
    )

    boxes = msg.to_rerun()

    assert isinstance(boxes, rr.Boxes2D)
    # XYWH top-left (100-15, 200-20) + size -> rerun stores center + half size.
    assert boxes.centers.as_arrow_array().to_pylist() == [[100.0, 200.0]]
    assert boxes.half_sizes.as_arrow_array().to_pylist()[0] == pytest.approx([15.0, 20.0])
    # Same label contract as 3D: dictionary-qualified class plus marker id.
    assert boxes.labels.as_arrow_array().to_pylist() == ["DICT_APRILTAG_36h11:7 id=7"]


def test_detection2darray_to_rerun_empty_array_clears_overlay() -> None:
    msg = Detection2DArray(
        header=Header(12.5, "camera_optical"),
        detections=[],
        detections_length=0,
    )

    boxes = msg.to_rerun()

    assert isinstance(boxes, rr.Boxes2D)
    assert boxes.centers.as_arrow_array().to_pylist() == []
    assert boxes.labels.as_arrow_array().to_pylist() == []
