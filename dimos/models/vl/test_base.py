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
from contextlib import nullcontext

import cv2
import numpy as np
from openai.types.chat import ChatCompletion
import pytest

from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.image import image_from_array
from dimos.msgs.time import header_now
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

# Captured actual response from Qwen API for cafe.jpg with query "humans"
# Added garbage around JSON to ensure we are robustly extracting it
MOCK_QWEN_RESPONSE = """
   Locating humans for you 😊😊

   [
    ["humans", 76, 368, 219, 580],
    ["humans", 354, 372, 512, 525],
    ["humans", 409, 370, 615, 748],
    ["humans", 628, 350, 762, 528],
    ["humans", 785, 323, 960, 650]
   ]

   Here is some trash at the end of the response :)
   Let me know if you need anything else 😀😊
   """


def test_query_detections_mocked(mocker) -> None:
    """Test query_detections with mocked API response (no API key required)."""
    image = image_from_array(
        np.zeros((864, 1152, 3), dtype=np.uint8), encoding="rgb8", header=header_now("camera")
    )

    # Create model and mock the query method
    client = mocker.patch("dimos.models.vl.qwen.OpenAI").return_value
    client.chat.completions.create.return_value = ChatCompletion(
        id="offline",
        object="chat.completion",
        created=0,
        model="test",
        choices=[
            {
                "index": 0,
                "finish_reason": "stop",
                "message": {"role": "assistant", "content": MOCK_QWEN_RESPONSE},
            }
        ],
    )
    model = QwenVlModel(api_key="offline-test")

    # Query for humans in the image
    query = "humans"
    detections = model.query_detections(image, query)

    # Verify the return type
    assert isinstance(detections, ImageDetections2D)

    # Should have 5 detections based on our mock data
    assert len(detections.detections) == 5, (
        f"Expected 5 detections, got {len(detections.detections)}"
    )

    # Verify each detection
    img_height, img_width = image.height, image.width

    for i, detection in enumerate(detections.detections):
        # Verify attributes
        assert detection.name == "humans"
        assert detection.confidence == 1.0
        assert detection.class_id == -1  # VLM detections use -1 for class_id
        assert detection.track_id == i
        assert len(detection.bbox) == 4

        assert detection.is_valid()

        # Verify bbox coordinates are valid (out-of-bounds detections are discarded)
        x1, y1, x2, y2 = detection.bbox
        assert x2 > x1, f"Detection {i}: Invalid x coordinates: x1={x1}, x2={x2}"
        assert y2 > y1, f"Detection {i}: Invalid y coordinates: y1={y1}, y2={y2}"

        # Check bounds (out-of-bounds detections would have been discarded)
        assert 0 <= x1 <= img_width, f"Detection {i}: x1={x1} out of bounds"
        assert 0 <= x2 <= img_width, f"Detection {i}: x2={x2} out of bounds"
        assert 0 <= y1 <= img_height, f"Detection {i}: y1={y1} out of bounds"
        assert 0 <= y2 <= img_height, f"Detection {i}: y2={y2} out of bounds"


@pytest.mark.parametrize("as_array", [False, True])
def test_qwen_encodes_source_pixels_and_keeps_numpy_compatibility(mocker, as_array):
    client = mocker.patch("dimos.models.vl.qwen.OpenAI").return_value
    client.chat.completions.create.return_value = ChatCompletion(
        id="offline",
        object="chat.completion",
        created=0,
        model="test",
        choices=[
            {
                "index": 0,
                "finish_reason": "stop",
                "message": {"role": "assistant", "content": "red"},
            }
        ],
    )
    pixels = np.full((20, 40, 3), [0, 0, 255], dtype=np.uint8)
    image = image_from_array(pixels, encoding="bgr8", header=header_now("camera"))
    model = QwenVlModel(api_key="offline-test", auto_resize=(20, 20))
    with pytest.warns(DeprecationWarning) if as_array else nullcontext():
        assert model.query(pixels if as_array else image, "color?") == "red"
    request = client.chat.completions.create.call_args.kwargs
    url = request["messages"][0]["content"][0]["image_url"]["url"]
    prefix, encoded = url.split(",", 1)
    assert prefix == "data:image/jpeg;base64"
    decoded = cv2.imdecode(
        np.frombuffer(base64.b64decode(encoded), dtype=np.uint8), cv2.IMREAD_COLOR
    )
    assert decoded.shape == (10, 20, 3)
    np.testing.assert_allclose(decoded[0, 0], [0, 0, 255], atol=2)
    assert image.header.frame_id == "camera"
    assert (image.width, image.height) == (40, 20)
