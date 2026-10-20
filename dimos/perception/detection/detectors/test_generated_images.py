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

from copy import deepcopy
from unittest.mock import PropertyMock

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest
import torch
from transformers.feature_extraction_utils import BatchFeature

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.std_msgs.msg import Header
from dimos.messages.vision_msgs.msg import Detection2DArray
from dimos.msgs.image import image_from_array, image_view
from dimos.perception.detection.detectors.omdet import OmDetDetector
from dimos.perception.detection.detectors.owlv2 import Owlv2Detector


@pytest.fixture
def source_image():
    return image_from_array(
        np.full((4, 8, 3), [0, 0, 255], dtype=np.uint8),
        encoding="bgr8",
        header=Header(frame_id="camera", stamp=Time(sec=1700000000, nanosec=123456789)),
    )


@pytest.mark.parametrize("detector_type", [OmDetDetector, Owlv2Detector])
def test_model_boundary_receives_rgb_and_exports_exact_source_header(
    mocker, source_image, detector_type
):
    processor, network = mocker.Mock(), mocker.Mock()
    processor.return_value = BatchFeature(data={"pixel_values": torch.zeros(1, 3, 4, 8)})
    processor.post_process_grounded_object_detection.return_value = [
        {
            "boxes": torch.tensor([[1, 1, 5, 3]]),
            "scores": torch.tensor([0.9]),
            "labels": torch.tensor([0]),
        }
    ]
    mocker.patch.object(
        detector_type, "_processor", new_callable=PropertyMock, return_value=processor
    )
    mocker.patch.object(detector_type, "_model", new_callable=PropertyMock, return_value=network)
    detector = detector_type(device="cpu")
    detections = detector.query_detections(source_image, ["target"])
    assert len(detections) == 1
    assert detections[0].bbox == (1, 1, 5, 3)
    assert detections[0].name == "target"
    processed = processor.call_args.kwargs["images"]
    pil = processed[0] if isinstance(processed, list) else processed
    assert pil.getpixel((0, 0)) == (255, 0, 0)
    message = decode(encode(detections.to_ros_detection2d_array()), Detection2DArray)
    assert message.header == source_image.header
    assert message.detections[0].header == source_image.header
    np.testing.assert_array_equal(image_view(source_image)[0, 0], [0, 0, 255])


def test_owl_feature_cache_distinguishes_adjacent_source_nanoseconds(mocker, source_image):
    processor, network = mocker.Mock(), mocker.Mock()
    processor.return_value = BatchFeature(data={"pixel_values": torch.zeros(1, 3, 4, 8)})
    network.image_embedder.return_value = (torch.ones(1, 1, 1, 2),)
    network.box_predictor.return_value = torch.tensor([[[0.5, 0.5, 0.5, 0.5]]])
    network.class_head.dense0.return_value = torch.ones(1, 1, 2)
    network.class_head.logit_shift.return_value = torch.zeros(1, 1, 1)
    network.class_head.logit_scale.return_value = torch.zeros(1, 1, 1)
    network.class_head.elu.return_value = torch.zeros(1, 1, 1)
    mocker.patch.object(
        Owlv2Detector, "_processor", new_callable=PropertyMock, return_value=processor
    )
    mocker.patch.object(Owlv2Detector, "_model", new_callable=PropertyMock, return_value=network)
    detector = Owlv2Detector(device="cpu")
    second = deepcopy(source_image)
    second.header.stamp.nanosec += 1
    first = detector._frame_features(source_image)
    assert detector._frame_features(source_image) is first
    detector._frame_features(second)
    assert network.image_embedder.call_count == 2
    assert len(detector._features) == 2
    np.testing.assert_array_equal(first[3].numpy(), [[2, 2, 6, 4]])
