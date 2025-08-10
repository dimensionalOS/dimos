# Copyright 2025 Dimensional Inc.
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

"""Test gateway functionality."""

import asyncio
import os

import pytest
from dotenv import load_dotenv

from dimos.agents.modules.gateway import UnifiedGatewayClient


@pytest.mark.asyncio
async def test_gateway_basic():
    """Test basic gateway functionality."""
    load_dotenv()

    # Check for at least one API key
    has_api_key = any(
        [os.getenv("OPENAI_API_KEY"), os.getenv("ANTHROPIC_API_KEY"), os.getenv("CEREBRAS_API_KEY")]
    )

    if not has_api_key:
        pytest.skip("No API keys found for gateway test")

    gateway = UnifiedGatewayClient()

    try:
        # Test with available provider
        if os.getenv("OPENAI_API_KEY"):
            model = "openai::gpt-4o-mini"
        elif os.getenv("ANTHROPIC_API_KEY"):
            model = "anthropic::claude-3-haiku-20240307"
        else:
            model = "cerebras::llama3.1-8b"

        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": "Say 'Hello Gateway' and nothing else."},
        ]

        # Test non-streaming
        response = await gateway.ainference(
            model=model, messages=messages, temperature=0.0, max_tokens=10
        )

        assert "choices" in response
        assert len(response["choices"]) > 0
        assert "message" in response["choices"][0]
        assert "content" in response["choices"][0]["message"]

        content = response["choices"][0]["message"]["content"]
        assert "hello" in content.lower() or "gateway" in content.lower()

    finally:
        gateway.close()


@pytest.mark.asyncio
async def test_gateway_streaming():
    """Test gateway streaming functionality."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("OpenAI API key required for streaming test")

    gateway = UnifiedGatewayClient()

    try:
        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": "Count from 1 to 3"},
        ]

        # Test streaming
        chunks = []
        async for chunk in await gateway.ainference(
            model="openai::gpt-4o-mini", messages=messages, temperature=0.0, stream=True
        ):
            chunks.append(chunk)

        assert len(chunks) > 0, "Should receive stream chunks"

        # Reconstruct content
        content = ""
        for chunk in chunks:
            if "choices" in chunk and chunk["choices"]:
                delta = chunk["choices"][0].get("delta", {})
                chunk_content = delta.get("content")
                if chunk_content is not None:
                    content += chunk_content

        assert any(str(i) in content for i in [1, 2, 3]), "Should count numbers"

    finally:
        gateway.close()


@pytest.mark.asyncio
async def test_gateway_tools():
    """Test gateway can pass tool definitions to LLM and get responses."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("OpenAI API key required for tools test")

    gateway = UnifiedGatewayClient()

    try:
        # Just test that gateway accepts tools parameter and returns valid response
        tools = [
            {
                "type": "function",
                "function": {
                    "name": "test_function",
                    "description": "A test function",
                    "parameters": {
                        "type": "object",
                        "properties": {"param": {"type": "string"}},
                    },
                },
            }
        ]

        messages = [
            {"role": "user", "content": "Hello, just testing the gateway"},
        ]

        # Just verify gateway doesn't crash when tools are provided
        response = await gateway.ainference(
            model="openai::gpt-4o-mini", messages=messages, tools=tools, temperature=0.0
        )

        # Basic validation - gateway returned something
        assert "choices" in response
        assert len(response["choices"]) > 0
        assert "message" in response["choices"][0]

    finally:
        gateway.close()


@pytest.mark.asyncio
async def test_gateway_providers():
    """Test gateway with different providers."""
    load_dotenv()

    gateway = UnifiedGatewayClient()

    providers_tested = 0

    try:
        # Test each available provider
        test_cases = [
            ("openai::gpt-4o-mini", "OPENAI_API_KEY"),
            ("anthropic::claude-3-haiku-20240307", "ANTHROPIC_API_KEY"),
            # ("cerebras::llama3.1-8b", "CEREBRAS_API_KEY"),
            ("qwen::qwen-turbo", "DASHSCOPE_API_KEY"),
        ]

        for model, env_var in test_cases:
            if not os.getenv(env_var):
                continue

            providers_tested += 1

            messages = [{"role": "user", "content": "Reply with just the word 'OK'"}]

            response = await gateway.ainference(
                model=model, messages=messages, temperature=0.0, max_tokens=10
            )

            assert "choices" in response
            content = response["choices"][0]["message"]["content"]
            assert len(content) > 0, f"{model} should return content"

        if providers_tested == 0:
            pytest.skip("No API keys found for provider test")

    finally:
        gateway.close()


if __name__ == "__main__":
    load_dotenv()
    asyncio.run(test_gateway_basic())
