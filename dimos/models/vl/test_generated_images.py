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

import base64
from unittest.mock import MagicMock

import cv2
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import Detection2DArray
import numpy as np
import pytest

from dimos.models.vl.base import VlModel
from dimos.models.vl.openai import OpenAIVlModel
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.image import image_from_array


class StaticModel(VlModel):
    def stop(self) -> None:
        pass

    def query(self, image: Image, query: str, **kwargs) -> str:
        return '[["target", 2, 4, 10, 12]]'


def test_generated_vl_resize_and_detection():
    image = image_from_array(
        np.zeros((40, 60, 3), dtype=np.uint8),
        encoding="rgb8",
        header=Header(frame_id="camera", stamp=Time(sec=1700000000, nanosec=123456789)),
    )
    model = StaticModel(auto_resize=(30, 20))
    resized, scale = model._prepare_image(image)
    assert scale == 0.5
    assert (resized.width, resized.height) == (30, 20)
    assert resized.header == image.header
    detections = model.query_detections(image, "target")
    wire = Detection2DArray.decode(detections.to_ros_detection2d_array().encode())
    assert len(wire.detections) == 1
    assert wire.header == image.header
    assert wire.detections[0].results[0].hypothesis.class_id == "-1"


@pytest.mark.parametrize("model_type", [QwenVlModel, OpenAIVlModel])
@pytest.mark.parametrize("batch", [False, True])
def test_provider_encodes_generated_images_as_jpeg(model_type, batch):
    pixels = np.zeros((40, 60, 3), dtype=np.uint8)
    pixels[:, :, 0] = 255
    image = image_from_array(pixels, encoding="rgb8")
    model = model_type(auto_resize=(30, 20))
    client = MagicMock()
    client.chat.completions.create.return_value.choices[0].message.content = "target"
    model.__dict__["_client"] = client
    if batch:
        assert model.query_batch([image, image], "find target") == ["target", "target"]
    else:
        assert model.query(image, "find target") == "target"
    request = client.chat.completions.create.call_args.kwargs
    content = request["messages"][0]["content"]
    images = [item for item in content if item["type"] == "image_url"]
    assert len(images) == (2 if batch else 1)
    for item in images:
        prefix, encoded = item["image_url"]["url"].split(",", 1)
        assert prefix == "data:image/jpeg;base64"
        jpeg = base64.b64decode(encoded)
        assert jpeg[:2] == b"\xff\xd8"
        decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape == (20, 30, 3)
        assert decoded[10, 15, 2] > 240
        assert decoded[10, 15, 0] < 10
    model.stop()
