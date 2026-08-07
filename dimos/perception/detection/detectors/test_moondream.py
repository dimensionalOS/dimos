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
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.detectors.moondream import Moondream2DDetector
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


def test_moondream_detector_queries_each_text_prompt() -> None:
    image = Image(data=np.zeros((4, 4, 3), dtype=np.uint8), format=ImageFormat.BGR)
    model = MagicMock()
    model.query_detections.side_effect = [
        ImageDetections2D(
            image,
            [
                Detection2DBBox(
                    (0.0, 0.0, 2.0, 2.0),
                    track_id=0,
                    class_id=-1,
                    confidence=1.0,
                    name="cup",
                    ts=image.ts,
                    image=image,
                )
            ],
        ),
        ImageDetections2D(
            image,
            [
                Detection2DBBox(
                    (2.0, 2.0, 3.0, 3.0),
                    track_id=1,
                    class_id=-1,
                    confidence=1.0,
                    name="bottle",
                    ts=image.ts,
                    image=image,
                )
            ],
        ),
    ]
    detector = Moondream2DDetector(model=model)

    detector.set_prompts(text=["cup", "bottle"])
    detections = detector.process_image(image)

    assert [detection.name for detection in detections] == ["cup", "bottle"]
    assert model.query_detections.call_args_list[0].args[1] == "cup"
    assert model.query_detections.call_args_list[1].args[1] == "bottle"
    detector.stop()
    model.start.assert_called_once_with()
    model.stop.assert_called_once_with()


def test_moondream_detector_rejects_empty_and_visual_prompts() -> None:
    detector = Moondream2DDetector(model=MagicMock())

    with pytest.raises(ValueError, match="nonempty"):
        detector.set_prompts(text=[" "])
    with pytest.raises(ValueError, match="text prompts"):
        detector.set_prompts(text=["cup"], bboxes=object())
