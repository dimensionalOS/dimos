# Copyright 2026 Dimensional Inc.
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

from dimos.benchmark.vqa.evaluation.scoring import evaluate_examples
from dimos.benchmark.vqa.generation.pipeline import generate_ground_truth
from dimos.benchmark.vqa.models import CalibratedFrame
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


class _Answerer:
    def __init__(self) -> None:
        self.calls: list[tuple[Image, str]] = []

    def answer(self, image: Image, question: str) -> str:
        self.calls.append((image, question))
        return "Yes."


class _Detector:
    def __init__(self, image: Image, detection: Detection2DSeg) -> None:
        self._image = image
        self._detection = detection

    def detect(self, image: Image, query: str) -> ImageDetections2D:
        assert image is self._image
        return ImageDetections2D(image, [self._detection] if query == "chair" else [])


class _Segmenter:
    def segment(self, detections: ImageDetections2D) -> ImageDetections2D:
        return detections


def test_single_frame_ground_truth_and_image_only_evaluation() -> None:
    image = Image.from_numpy(np.zeros((6, 6, 3), dtype=np.uint8))
    frame = CalibratedFrame(
        id="frame-1",
        image=image,
        pointcloud=PointCloud2.from_numpy(
            np.array([[0.0, 0.0, 1.0], [0.4, 0.0, 1.0], [-0.4, 0.0, 1.0]], dtype=np.float32)
        ),
        camera_info=CameraInfo.from_intrinsics(3.0, 3.0, 3.0, 3.0, 6, 6),
        pointcloud_to_camera=Transform.identity(),
        image_is_rectified=True,
    )
    mask = np.full((6, 6), 255, dtype=np.uint8)
    detection = Detection2DSeg((0.0, 0.0, 5.0, 5.0), 0, -1, 1.0, "chair", 0.0, image, mask)

    examples = generate_ground_truth(
        frame, ["chair", "table"], _Detector(image, detection), _Segmenter()
    )
    answerer = _Answerer()
    evaluations = evaluate_examples(frame.image, examples, answerer)

    assert {example.expected_answer for example in examples} == {"yes", "no", "center"}
    assert all(image_arg is image for image_arg, _ in answerer.calls)
    assert evaluations[0].passed
    assert not evaluations[1].passed
