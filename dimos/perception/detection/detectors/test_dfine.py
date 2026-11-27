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

from typing import Any

import numpy as np
import pytest
import torch

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.detectors.dfine import DFineDetector
from dimos.perception.detection.module2D import _default_detector


def test_default_detector_reports_missing_processor_at_startup(mocker: Any) -> None:
    mocker.patch("transformers.DFineForObjectDetection.from_pretrained")
    mocker.patch("transformers.AutoImageProcessor.from_pretrained", side_effect=OSError("offline"))

    with pytest.raises(OSError, match="offline"):
        _default_detector()


@pytest.mark.parametrize("dtype", [torch.float16, torch.bfloat16])
def test_rejects_low_precision_on_cpu(dtype: torch.dtype) -> None:
    with pytest.raises(ValueError, match="Use float32"):
        DFineDetector(device="cpu", dtype=dtype, autostart=True)


def test_process_image_converts_the_model_output(mocker: Any) -> None:
    """COCO names, boxes clipped to the image, and no box without an area."""
    image = Image(data=np.zeros((48, 64, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=12.5)
    model_loader = mocker.patch("transformers.DFineForObjectDetection.from_pretrained")
    model = model_loader.return_value.eval.return_value.to.return_value
    model.config.id2label = {0: "person", 62: "tvmonitor"}
    processor = mocker.patch("transformers.AutoImageProcessor.from_pretrained").return_value
    processor.return_value = {
        "pixel_values": torch.ones(1),
        "pixel_mask": torch.ones(1, dtype=torch.long),
    }
    processor.post_process_object_detection.return_value = [
        {
            "boxes": torch.tensor(
                [[-3.0, 2.0, 30.0, 60.0], [40.0, 10.0, 60.0, 20.0], [70.0, 5.0, 90.0, 9.0]]
            ),
            "scores": torch.tensor([0.9, 0.7, 0.8]),
            "labels": torch.tensor([0, 62, 0]),
        }
    ]
    detector = DFineDetector(device="cpu", dtype=torch.float64, threshold=0.4)

    detections = detector.process_image(image)

    model_loader.assert_called_once_with(detector.model_name, torch_dtype=torch.float64)
    assert model.call_args.kwargs["pixel_values"].dtype == torch.float64
    assert model.call_args.kwargs["pixel_mask"].dtype == torch.long
    assert processor.post_process_object_detection.call_args.kwargs["threshold"] == 0.4
    assert processor.post_process_object_detection.call_args.kwargs["target_sizes"] == [(48, 64)]
    person, tv = detections.detections
    assert (person.name, person.class_id, person.track_id) == ("person", 0, -1)
    assert person.bbox == (0.0, 2.0, 30.0, 48.0)
    assert person.confidence == pytest.approx(0.9)
    assert (person.ts, person.image) == (12.5, image)
    assert (tv.name, tv.class_id) == ("tv", 62)
    assert tv.bbox == (40.0, 10.0, 60.0, 20.0)
