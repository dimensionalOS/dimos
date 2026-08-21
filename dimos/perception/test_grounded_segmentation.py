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

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.point import Detection2DPoint
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.grounded_segmentation import GroundedSegmentationModule


class FakeGrounder:
    def __init__(self) -> None:
        self.queries: list[tuple[Image, str]] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def query_detections(
        self, image: Image, query: str, **_kwargs: object
    ) -> ImageDetections2D[Detection2DBBox]:
        self.queries.append((image, query))
        return ImageDetections2D(
            image,
            [Detection2DBBox((1.0, 1.0, 3.0, 3.0), 99, -1, 1.0, query, image.ts, image)],
        )

    def query_points(
        self, image: Image, query: str, **_kwargs: object
    ) -> ImageDetections2D[Detection2DPoint]:
        self.queries.append((image, f"point:{query}"))
        return ImageDetections2D(image, [Detection2DPoint(2.0, 2.0, query, image.ts, image)])


class FakeSegmenter:
    def segment(self, detections: ImageDetections2D) -> ImageDetections2D[Detection2DSeg]:
        masks = []
        for detection in detections:
            mask = np.zeros((4, 4), dtype=np.uint8)
            mask[1:3, 1:3] = 255
            masks.append(
                Detection2DSeg.from_sam2_result(
                    mask,
                    detection.track_id,
                    detections.image,
                    name=detection.name,
                )
            )
        return ImageDetections2D(detections.image, masks)

    def segment_points(
        self, detections: ImageDetections2D[Detection2DPoint]
    ) -> ImageDetections2D[Detection2DSeg]:
        masks = []
        for detection in detections:
            mask = np.zeros((4, 4), dtype=np.uint8)
            mask[1:3, 1:3] = 255
            masks.append(
                Detection2DSeg.from_sam2_result(
                    mask,
                    detection.track_id,
                    detections.image,
                    name=detection.name,
                )
            )
        return ImageDetections2D(detections.image, masks)


def test_segment_uses_explicit_image_and_preserves_prompt_labels() -> None:
    image = Image(
        np.zeros((4, 4, 3), dtype=np.uint8),
        ImageFormat.RGB,
        "camera_optical",
        42.0,
    )
    grounder = FakeGrounder()
    module = GroundedSegmentationModule(
        grounder=lambda: grounder,
        segmenter=FakeSegmenter,
    )

    module.start()
    try:
        result = module.segment(image, ["drawer handle", " drawer handle ", "cabinet"])
    finally:
        module.stop()

    assert grounder.queries == [(image, "drawer handle"), (image, "cabinet")]
    assert [detection.name for detection in result] == ["drawer handle", "cabinet"]
    assert [detection.track_id for detection in result] == [0, 1]
    assert all(detection.ts == 42.0 for detection in result)
    assert all(np.count_nonzero(detection.mask) == 4 for detection in result)


def test_segment_rejects_empty_prompts() -> None:
    image = Image(np.zeros((2, 2, 3), dtype=np.uint8), ImageFormat.RGB, "camera", 1.0)
    module = GroundedSegmentationModule(
        grounder=FakeGrounder,
        segmenter=FakeSegmenter,
    )

    module.start()
    try:
        with pytest.raises(ValueError, match="at least one"):
            module.segment(image, [" "])
    finally:
        module.stop()


def test_segment_best_uses_one_point_grounding() -> None:
    image = Image(np.zeros((4, 4, 3), dtype=np.uint8), ImageFormat.RGB, "camera", 2.0)
    grounder = FakeGrounder()
    module = GroundedSegmentationModule(grounder=lambda: grounder, segmenter=FakeSegmenter)

    module.start()
    try:
        result = module.segment_best(image, " cream cheese package ")
    finally:
        module.stop()

    assert grounder.queries == [(image, "point:cream cheese package")]
    assert len(result) == 1
    assert result[0].name == "cream cheese package"
    assert result[0].track_id == 0
