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

"""Verify generated camera messages reach VLM requests through stream and RPC."""

import base64

import cv2
from dimos_generated.sensor_msgs.msg import Image
from langchain_core.messages import AIMessage, HumanMessage
import numpy as np
import pytest

from dimos.agents.vlm_agent import VLMAgent
from dimos.msgs.image import image_from_array


@pytest.fixture
def agent(mocker):
    model = mocker.Mock()
    model.invoke.return_value = AIMessage(content="red image")
    mocker.patch("langchain.chat_models.init_chat_model", return_value=model)
    value = VLMAgent()
    try:
        yield value, model
    finally:
        value.stop()


@pytest.mark.parametrize("use_stream", [False, True])
def test_generated_image_reaches_model_as_jpeg(agent, use_stream):
    value, model = agent
    pixels = np.zeros((12, 16, 3), dtype=np.uint8)
    pixels[..., 0] = 255
    original = image_from_array(pixels, encoding="rgb8")
    image = Image.decode(original.encode())
    answers = []
    unsubscribe = value.answer_stream.subscribe(answers.append)
    try:
        if use_stream:
            value._on_image(image)
            value._on_query(HumanMessage(content="What color?"))
            assert answers[0].content == "red image"
        else:
            assert value.query_image(image, "What color?") == "red image"
        request = model.invoke.call_args.args[0][-1]
        assert request.content[0] == {"type": "text", "text": "What color?"}
        prefix, encoded = request.content[1]["image_url"]["url"].split(",", 1)
        assert prefix == "data:image/jpeg;base64"
        decoded = cv2.imdecode(
            np.frombuffer(base64.b64decode(encoded), dtype=np.uint8), cv2.IMREAD_COLOR
        )
        assert decoded.shape == (12, 16, 3)
        assert decoded[0, 0].tolist() == [0, 0, 254]
        assert image.encode() == original.encode()
    finally:
        unsubscribe()


def test_query_before_image_does_not_invoke_model(agent):
    value, model = agent
    answers = []
    unsubscribe = value.answer_stream.subscribe(answers.append)
    try:
        value._on_query(HumanMessage(content="What color?"))
        assert answers[0].content == "No image available yet."
        model.invoke.assert_not_called()
    finally:
        unsubscribe()
