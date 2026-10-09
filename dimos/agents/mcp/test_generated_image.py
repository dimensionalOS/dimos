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

"""A generated CDR image survives MCP serialization and agent history conversion."""

import asyncio
import base64

import cv2
from dimos_generated.sensor_msgs.msg import Image
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

from dimos.agents.mcp.mcp_client import _append_image_to_history
from dimos.agents.mcp.mcp_server import handle_request
from dimos.msgs.image import image_from_array


def test_generated_image_mcp_result_reaches_model_history(mocker):
    pixels = np.zeros((12, 16, 3), dtype=np.uint8)
    pixels[..., 0] = 255
    original = image_from_array(pixels, encoding="rgb8")
    response = asyncio.run(
        handle_request(
            {"method": "tools/call", "id": 1, "params": {"name": "observe"}},
            [],
            {"observe": lambda: cdr_decode(cdr_encode(original), Image)},
        )
    )
    assert response is not None
    content = response["result"]["content"]
    assert len(content) == 1
    image = content[0]
    assert image["type"] == "image"
    assert image["mimeType"] == "image/jpeg"
    jpeg = base64.b64decode(image["data"])
    decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (12, 16, 3)
    assert decoded[0, 0].tolist() == [0, 0, 254]
    client = mocker.Mock()
    _append_image_to_history(client, "observe", "test-image", image)
    message = client.add_message.call_args.args[0]
    assert message.content[1] == {
        "type": "image_url",
        "image_url": {"url": "data:image/jpeg;base64," + image["data"]},
    }
