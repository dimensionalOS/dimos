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

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from dimos.models.vl.litellm import LiteLLMVlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


def _make_test_image() -> Image:
    """Create a small synthetic RGB image for unit tests (no LFS required)."""
    return Image.from_numpy(np.zeros((64, 64, 3), dtype=np.uint8))


def _mock_response(content: str = "test response") -> SimpleNamespace:
    """Build a fake litellm.completion() response matching the OpenAI shape."""
    return SimpleNamespace(
        choices=[SimpleNamespace(message=SimpleNamespace(content=content))],
    )


MOCK_DETECTION_RESPONSE = """[
    ["person", 10, 10, 50, 50],
    ["person", 20, 20, 60, 60]
]"""

_COMPLETION_PATH = "dimos.models.vl.litellm.LiteLLMVlModel._completion"


class TestLiteLLMVlModel:
    def test_query_dispatches_to_litellm(self) -> None:
        """_completion is called with the right model, messages, and drop_params=True."""
        model = LiteLLMVlModel(model_name="anthropic/claude-sonnet-4-20250514")
        image = _make_test_image()

        with patch(_COMPLETION_PATH, return_value=_mock_response("a cafe")) as mock_comp:
            result = model.query(image, "What is this?")

        assert result == "a cafe"
        call_kwargs = mock_comp.call_args.kwargs
        assert call_kwargs["model"] == "anthropic/claude-sonnet-4-20250514"
        msgs = call_kwargs["messages"]
        assert len(msgs) == 1
        assert msgs[0]["role"] == "user"
        content_parts = msgs[0]["content"]
        assert any(p["type"] == "image_url" for p in content_parts)
        assert any(p["type"] == "text" and p["text"] == "What is this?" for p in content_parts)

    def test_query_forwards_api_key_when_set(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini", api_key="sk-test-key")
        image = _make_test_image()

        with patch(_COMPLETION_PATH, return_value=_mock_response()):
            model.query(image, "hi")

    def test_query_forwards_api_base(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini", api_base="http://localhost:4000")
        image = _make_test_image()

        with patch(_COMPLETION_PATH, return_value=_mock_response()):
            model.query(image, "hi")

    def test_query_forwards_response_format(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        image = _make_test_image()
        fmt = {"type": "json_object"}

        with patch(_COMPLETION_PATH, return_value=_mock_response('{"a":1}')) as mock_comp:
            model.query(image, "json please", response_format=fmt)

        assert mock_comp.call_args.kwargs["response_format"] == fmt

    def test_query_batch_returns_per_image_response(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        images = [_make_test_image(), _make_test_image()]

        with patch(_COMPLETION_PATH, return_value=_mock_response("batch result")):
            results = model.query_batch(images, "describe")

        assert len(results) == 2
        assert all(r == "batch result" for r in results)

    def test_query_batch_empty_input(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        assert model.query_batch([], "describe") == []

    def test_query_detections_with_mocked_response(self) -> None:
        """End-to-end: query -> litellm -> parse detections."""
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        image = _make_test_image()

        model.query = MagicMock(return_value=MOCK_DETECTION_RESPONSE)
        detections = model.query_detections(image, "person")

        assert isinstance(detections, ImageDetections2D)
        assert len(detections.detections) == 2
        for det in detections.detections:
            assert det.name == "person"
            assert det.confidence == 1.0

    def test_factory_creates_litellm_model(self) -> None:
        from dimos.models.vl.create import create

        model = create("litellm")
        assert isinstance(model, LiteLLMVlModel)

    def test_import_error_without_litellm(self) -> None:
        """Verify a helpful message when litellm is not installed."""
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        image = _make_test_image()

        with patch.dict("sys.modules", {"litellm": None}):
            with pytest.raises(ImportError, match="litellm is required"):
                model.query(image, "hi")

