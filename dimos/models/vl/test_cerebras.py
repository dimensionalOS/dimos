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

import json

import httpx
import numpy as np
from openai import OpenAI
import pytest

from dimos.models.vl.cerebras import CerebrasVlModel
from dimos.models.vl.create import create
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.visual.query import get_object_bbox_from_image


def test_cerebras_requires_its_own_credential(monkeypatch):
    monkeypatch.delenv("CEREBRAS_API_KEY", raising=False)
    monkeypatch.setenv("OPENAI_API_KEY", "unrelated-key")
    model = create("cerebras")
    with pytest.raises(ValueError, match="CEREBRAS_API_KEY"):
        model.query(Image.from_numpy(np.zeros((32, 32, 3), dtype=np.uint8)), "Describe")


def test_cerebras_image_and_detection_contract(monkeypatch, mocker):
    monkeypatch.setenv("CEREBRAS_API_KEY", "cerebras-test-key")
    requests = []
    replies = iter(
        [
            '{"name": "square", "bbox": [4, 5, 20, 25]}',
            '[["square", 4, 5, 20, 25]]',
        ]
    )

    def respond(request):
        requests.append(request)
        return httpx.Response(
            200,
            json={
                "id": "test",
                "object": "chat.completion",
                "created": 0,
                "model": "gemma-4-31b",
                "choices": [
                    {
                        "index": 0,
                        "finish_reason": "stop",
                        "message": {"role": "assistant", "content": next(replies)},
                    }
                ],
            },
        )

    with httpx.Client(transport=httpx.MockTransport(respond)) as http_client:

        def client(**kwargs):
            return OpenAI(**kwargs, http_client=http_client)

        mocker.patch("dimos.models.vl.cerebras.OpenAI", side_effect=client)
        model = CerebrasVlModel()
        image = Image.from_numpy(np.zeros((32, 32, 3), dtype=np.uint8))
        try:
            assert get_object_bbox_from_image(model, image, "square") == (4, 5, 20, 25)
            detections = model.query_detections(image, "square").detections
            assert [(d.name, d.bbox) for d in detections] == [("square", (4, 5, 20, 25))]
        finally:
            model.stop()

    assert len(requests) == 2
    for request in requests:
        assert str(request.url) == "https://api.cerebras.ai/v1/chat/completions"
        assert request.headers["authorization"] == "Bearer cerebras-test-key"
        body = json.loads(request.content)
        assert body["model"] == "gemma-4-31b"
        content = body["messages"][0]["content"]
        assert content[0]["image_url"]["url"].startswith("data:image/jpeg;base64,")
