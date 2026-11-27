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
import torch

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.detectors.dfine import DFineDetector


def test_process_image_converts_the_model_output() -> None:
    """COCO names, boxes clipped to the image, and no box without an area."""
    image = Image(data=np.zeros((48, 64, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=12.5)
    detector = DFineDetector(device="cpu", threshold=0.4)
    model = MagicMock()
    model.config.id2label = {0: "person", 62: "tvmonitor"}
    processor = MagicMock()
    processor.post_process_object_detection.return_value = [
        {
            "boxes": torch.tensor(
                [[-3.0, 2.0, 30.0, 60.0], [40.0, 10.0, 60.0, 20.0], [70.0, 5.0, 90.0, 9.0]]
            ),
            "scores": torch.tensor([0.9, 0.7, 0.8]),
            "labels": torch.tensor([0, 62, 0]),
        }
    ]
    # Both are cached properties. Setting them skips the download.
    detector.__dict__["_model"] = model
    detector.__dict__["_processor"] = processor

    detections = detector.process_image(image)

    assert processor.post_process_object_detection.call_args.kwargs["threshold"] == 0.4
    assert processor.post_process_object_detection.call_args.kwargs["target_sizes"] == [(48, 64)]
    person, tv = detections.detections
    assert (person.name, person.class_id, person.track_id) == ("person", 0, -1)
    assert person.bbox == (0.0, 2.0, 30.0, 48.0)
    assert person.confidence == pytest.approx(0.9)
    assert (person.ts, person.image) == (12.5, image)
    assert (tv.name, tv.class_id) == ("tv", 62)
    assert tv.bbox == (40.0, 10.0, 60.0, 20.0)
