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
import rerun as rr

from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2D import Detection2D
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray


def _detection2d(cx: float, cy: float, w: float, h: float, label: str, marker: str) -> Detection2D:
    det = Detection2D()
    det.header = Header(1.0, "camera_color_optical_frame")
    det.id = marker
    det.results = [ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id=label, score=0.9))]
    det.results_length = len(det.results)
    # Fresh objects per detection: the generated defaults are shared instances.
    det.bbox = BoundingBox2D(center=Pose2D(position=Point2D(x=cx, y=cy)), size_x=w, size_y=h)
    return det


def test_detection2d_array_draws_labelled_pixel_boxes() -> None:
    array = Detection2DArray(
        header=Header(1.0, "camera_color_optical_frame"),
        detections=[
            _detection2d(320, 240, 100, 50, "banana", "b1"),
            _detection2d(100, 100, 20, 20, "", ""),
        ],
        detections_length=2,
    )

    boxes = array.to_rerun()

    assert isinstance(boxes, rr.Boxes2D)
    assert boxes.half_sizes is not None and boxes.labels is not None
    first = array.to_json()[0]
    assert first["label"] == "banana"
    assert first["bbox"] == {"cx": 320.0, "cy": 240.0, "w": 100.0, "h": 50.0}
