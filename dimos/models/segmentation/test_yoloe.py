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

from unittest.mock import MagicMock

import numpy as np

from dimos.models.segmentation.yoloe import YoloeBoxSegmenter
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


def _image() -> Image:
    return Image(
        data=np.zeros((32, 48, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera",
        ts=12.5,
    )


def _box(image: Image, *, bbox: tuple[float, float, float, float]) -> Detection2DBBox:
    return Detection2DBBox(
        bbox=bbox,
        track_id=42,
        class_id=7,
        confidence=0.73,
        name="white mug",
        ts=image.ts,
        image=image,
    )


def _mask(
    image: Image,
    *,
    bbox: tuple[float, float, float, float],
    prompt_id: int,
    value: int,
) -> Detection2DSeg:
    return Detection2DSeg(
        bbox=bbox,
        track_id=-1,
        class_id=prompt_id,
        confidence=0.11,
        name=f"prompt-{prompt_id}",
        ts=image.ts,
        image=image,
        mask=np.full((32, 48), value, dtype=np.uint8),
    )


def test_box_prompt_mask_preserves_owlv2_metadata_and_selects_best_overlap() -> None:
    image = _image()
    detection = _box(image, bbox=(10.0, 8.0, 24.0, 28.0))
    low_overlap = _mask(
        image,
        bbox=(0.0, 0.0, 12.0, 12.0),
        prompt_id=0,
        value=1,
    )
    best_overlap = _mask(
        image,
        bbox=(11.0, 9.0, 23.0, 27.0),
        prompt_id=0,
        value=255,
    )
    detector = MagicMock()
    detector.predict_image.return_value = ImageDetections2D(image, [low_overlap, best_overlap])
    segmenter = YoloeBoxSegmenter(detector=detector)

    result = segmenter.segment(ImageDetections2D(image, [detection]))

    np.testing.assert_array_equal(
        detector.set_prompts.call_args.kwargs["bboxes"],
        np.asarray([detection.bbox], dtype=np.float64),
    )
    detector.predict_image.assert_called_once_with(image)
    assert len(result) == 1
    assert result[0].bbox == best_overlap.bbox
    assert (
        result[0].track_id,
        result[0].class_id,
        result[0].confidence,
        result[0].name,
        result[0].ts,
        result[0].image,
    ) == (42, 7, 0.73, "white mug", 12.5, image)
    np.testing.assert_array_equal(result[0].mask, best_overlap.mask)


def test_missing_yoloe_mask_drops_box_instead_of_using_rectangle() -> None:
    image = _image()
    detector = MagicMock()
    detector.predict_image.return_value = ImageDetections2D(image, [])
    segmenter = YoloeBoxSegmenter(detector=detector)

    result = segmenter.segment(
        ImageDetections2D(image, [_box(image, bbox=(10.0, 8.0, 24.0, 28.0))])
    )

    assert result.image is image
    assert result.detections == []
