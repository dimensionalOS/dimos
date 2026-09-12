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

"""Provider SDK requests preserve each model's reasoning protocol."""

import json
from typing import Any

import anthropic
import httpx
from langchain_anthropic import ChatAnthropic
from langchain_openai import ChatOpenAI
import pytest

from dimos.agents.mcp.mcp_client import init_model


@pytest.mark.parametrize("name", ["gpt-6-astra", "openai:gpt-6-astra", "gpt-5.6-sol"])
def test_openai_reasoning_models_use_responses(name: str, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    model = init_model(name)
    assert isinstance(model, ChatOpenAI)
    assert model.use_responses_api
    assert model.model_name == name.removeprefix("openai:")
    assert model.reasoning == {"effort": "medium", "summary": "auto"}


@pytest.mark.parametrize("name", ["claude-fable-5-1", "anthropic:claude-fable-5-1"])
def test_fable_native_sdk_payload_and_usage(name: str, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test-key")
    model = init_model(name)
    assert isinstance(model, ChatAnthropic)
    requests: list[dict[str, Any]] = []

    def handle(request: httpx.Request) -> httpx.Response:
        requests.append(json.loads(request.content))
        return httpx.Response(
            200,
            json={
                "id": "msg_1",
                "type": "message",
                "role": "assistant",
                "model": "claude-fable-5-1",
                "content": [{"type": "text", "text": "yes"}],
                "stop_reason": "end_turn",
                "stop_sequence": None,
                "usage": {
                    "input_tokens": 10,
                    "output_tokens": 2,
                    "cache_read_input_tokens": 3,
                    "cache_creation_input_tokens": 5,
                },
            },
        )

    with (
        httpx.Client(transport=httpx.MockTransport(handle)) as http,
        anthropic.Anthropic(api_key="test-key", http_client=http) as client,
    ):
        model._client = client
        result = model.invoke("Question")
    assert requests[0]["thinking"] == {"type": "adaptive"}
    assert requests[0]["output_config"]["effort"] == "medium"
    assert requests[0]["model"] == "claude-fable-5-1"
    assert result.text == "yes"
    assert result.usage_metadata is not None
    assert result.usage_metadata["input_tokens"] == 18
