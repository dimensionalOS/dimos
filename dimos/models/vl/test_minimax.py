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

"""Tests for the MiniMax VLM provider."""

from unittest.mock import MagicMock, patch

import pytest

from dimos.models.vl.base import VlModel
from dimos.models.vl.minimax import (
    DEFAULT_MINIMAX_BASE_URL,
    DEFAULT_MINIMAX_MODEL,
    MiniMaxVlModel,
    MiniMaxVlModelConfig,
)
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.data import get_data


def test_config_defaults() -> None:
    """The default model and base URL point at MiniMax-M3 on the overseas endpoint."""
    config = MiniMaxVlModelConfig()
    assert config.model_name == DEFAULT_MINIMAX_MODEL
    assert DEFAULT_MINIMAX_MODEL == "MiniMax-M3"
    assert DEFAULT_MINIMAX_BASE_URL == "https://api.minimax.io/v1"


def test_missing_api_key_raises() -> None:
    """Constructing the client without an API key must fail loudly."""
    with patch.dict("os.environ", {}, clear=True):
        model = MiniMaxVlModel()
        # ``_client`` is a cached_property — it's only built on first access,
        # so trigger it explicitly here.
        with pytest.raises(ValueError, match="MINIMAX_API_KEY"):
            _ = model._client


def test_query_uses_openai_compatible_client() -> None:
    """The query call should hit ``chat.completions.create`` on the MiniMax base URL."""
    image = Image.from_file(get_data("cafe.jpg"))

    model = MiniMaxVlModel(api_key="test-key")

    fake_response = MagicMock()
    fake_response.choices = [MagicMock(message=MagicMock(content="a person sitting"))]

    with patch.object(
        model.__class__, "_client", new_callable=lambda: MagicMock()
    ) as mock_client_prop:
        # Replace the cached_property with a MagicMock that returns our fake OpenAI client.
        mock_client = MagicMock()
        mock_client.chat.completions.create.return_value = fake_response
        model.__dict__["_client"] = mock_client

        result = model.query(image, "What is in the image?")

    assert result == "a person sitting"
    mock_client.chat.completions.create.assert_called_once()
    call_kwargs = mock_client.chat.completions.create.call_args.kwargs
    assert call_kwargs["model"] == "MiniMax-M3"
    # MiniMax rejects temperature=0 — default to 1.0.
    assert call_kwargs["temperature"] == 1.0
    # Should NOT pass response_format (unsupported on MiniMax).
    assert "response_format" not in call_kwargs


def test_query_batch_returns_one_response_per_image() -> None:
    """query_batch returns the same model response replicated per image."""
    images = [
        Image.from_file(get_data("cafe.jpg")),
        Image.from_file(get_data("cafe.jpg")),
    ]

    model = MiniMaxVlModel(api_key="test-key")

    fake_response = MagicMock()
    fake_response.choices = [MagicMock(message=MagicMock(content="cafe interior"))]

    mock_client = MagicMock()
    mock_client.chat.completions.create.return_value = fake_response
    model.__dict__["_client"] = mock_client

    results = model.query_batch(images, "describe the scene")

    assert results == ["cafe interior", "cafe interior"]
    mock_client.chat.completions.create.assert_called_once()


def test_minimax_inherits_vlmodel_interface() -> None:
    """MiniMaxVlModel must satisfy the abstract VlModel interface."""
    model = MiniMaxVlModel(api_key="test-key")
    assert isinstance(model, VlModel)
