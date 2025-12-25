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

"""TensorZero embedded gateway client with correct config format."""

from collections.abc import AsyncIterator, Iterator
import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


class TensorZeroEmbeddedGateway:
    """TensorZero embedded gateway using patch_openai_client."""

    def __init__(self) -> None:
        """Initialize TensorZero embedded gateway."""
        self._client = None
        self._config_path = None
        self._setup_config()
        self._initialize_client()  # type: ignore[no-untyped-call]

    def _setup_config(self) -> None:
        """Create TensorZero configuration with correct format."""
        config_dir = Path("/tmp/tensorzero_embedded")
        config_dir.mkdir(exist_ok=True)
        self._config_path = config_dir / "tensorzero.toml"  # type: ignore[assignment]

        # Create config using the correct format from working example
        config_content = """
# OpenAI Models
[models.gpt_4o_mini]
routing = ["openai"]

[models.gpt_4o_mini.providers.openai]
type = "openai"
model_name = "gpt-4o-mini"

[models.gpt_4o]
routing = ["openai"]

[models.gpt_4o.providers.openai]
type = "openai"
model_name = "gpt-4o"

# Claude Models
[models.claude_3_haiku]
routing = ["anthropic"]

[models.claude_3_haiku.providers.anthropic]
type = "anthropic"
model_name = "claude-3-haiku-20240307"

[models.claude_3_sonnet]
routing = ["anthropic"]

[models.claude_3_sonnet.providers.anthropic]
type = "anthropic"
model_name = "claude-3-5-sonnet-20241022"

[models.claude_3_opus]
routing = ["anthropic"]

[models.claude_3_opus.providers.anthropic]
type = "anthropic"
model_name = "claude-3-opus-20240229"

# Cerebras Models - disabled for CI (no API key)
# [models.llama_3_3_70b]
# routing = ["cerebras"]
#
# [models.llama_3_3_70b.providers.cerebras]
# type = "openai"
# model_name = "llama-3.3-70b"
# api_base = "https://api.cerebras.ai/v1"
# api_key_location = "env::CEREBRAS_API_KEY"

# Qwen Models
[models.qwen_plus]
routing = ["qwen"]

[models.qwen_plus.providers.qwen]
type = "openai"
model_name = "qwen-plus"
api_base = "https://dashscope-intl.aliyuncs.com/compatible-mode/v1"
api_key_location = "env::ALIBABA_API_KEY"

[models.qwen_vl_plus]
routing = ["qwen"]

[models.qwen_vl_plus.providers.qwen]
type = "openai"
model_name = "qwen-vl-plus"
api_base = "https://dashscope-intl.aliyuncs.com/compatible-mode/v1"
api_key_location = "env::ALIBABA_API_KEY"

# Object storage - disable for embedded mode
[object_storage]
type = "disabled"

# Single chat function with all models
# TensorZero will automatically skip models that don't support the input type
[functions.chat]
type = "chat"

[functions.chat.variants.openai]
type = "chat_completion"
model = "gpt_4o_mini"
weight = 1.0

[functions.chat.variants.claude]
type = "chat_completion"
model = "claude_3_haiku"
weight = 0.5

# Cerebras disabled for CI (no API key)
# [functions.chat.variants.cerebras]
# type = "chat_completion"
# model = "llama_3_3_70b"
# weight = 0.0

[functions.chat.variants.qwen]
type = "chat_completion"
model = "qwen_plus"
weight = 0.3

# For vision queries, Qwen VL can be used
[functions.chat.variants.qwen_vision]
type = "chat_completion"
model = "qwen_vl_plus"
weight = 0.4
"""

        with open(self._config_path, "w") as f:  # type: ignore[call-overload]
            f.write(config_content)

        logger.info(f"Created TensorZero config at {self._config_path}")

    def _initialize_client(self):  # type: ignore[no-untyped-def]
        """Initialize OpenAI client with TensorZero patch."""
        try:
            from openai import OpenAI
            from tensorzero import patch_openai_client

            self._client = OpenAI()  # type: ignore[assignment]

            # Patch with TensorZero embedded gateway
            patch_openai_client(
                self._client,
                clickhouse_url=None,  # In-memory storage
                config_file=str(self._config_path),
                async_setup=False,
            )

            logger.info("TensorZero embedded gateway initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize TensorZero: {e}")
            raise

    def _map_model_to_tensorzero(self, model: str) -> str:
        """Map provider::model format to TensorZero function format."""
        # Always use the chat function - TensorZero will handle model selection
        # based on input type and model capabilities automatically
        return "tensorzero::function_name::chat"

    def inference(  # type: ignore[no-untyped-def]
        self,
        model: str,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]] | None = None,
        temperature: float = 0.0,
        max_tokens: int | None = None,
        stream: bool = False,
        **kwargs,
    ) -> dict[str, Any] | Iterator[dict[str, Any]]:
        """Synchronous inference call through TensorZero."""

        # Map model to TensorZero function
        tz_model = self._map_model_to_tensorzero(model)

        # Prepare parameters
        params = {
            "model": tz_model,
            "messages": messages,
            "temperature": temperature,
        }

        if max_tokens:
            params["max_tokens"] = max_tokens

        if tools:
            params["tools"] = tools

        if stream:
            params["stream"] = True

        # Add any extra kwargs
        params.update(kwargs)

        try:
            # Make the call through patched client
            if stream:
                # Return streaming iterator
                stream_response = self._client.chat.completions.create(**params)  # type: ignore[attr-defined]

                def stream_generator():  # type: ignore[no-untyped-def]
                    for chunk in stream_response:
                        yield chunk.model_dump()

                return stream_generator()  # type: ignore[no-any-return, no-untyped-call]
            else:
                response = self._client.chat.completions.create(**params)  # type: ignore[attr-defined]
                return response.model_dump()  # type: ignore[no-any-return]

        except Exception as e:
            logger.error(f"TensorZero inference failed: {e}")
            raise

    async def ainference(  # type: ignore[no-untyped-def]
        self,
        model: str,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]] | None = None,
        temperature: float = 0.0,
        max_tokens: int | None = None,
        stream: bool = False,
        **kwargs,
    ) -> dict[str, Any] | AsyncIterator[dict[str, Any]]:
        """Async inference with streaming support."""
        import asyncio

        loop = asyncio.get_event_loop()

        if stream:
            # Create async generator from sync streaming
            async def stream_generator():  # type: ignore[no-untyped-def]
                # Run sync streaming in executor
                sync_stream = await loop.run_in_executor(
                    None,
                    lambda: self.inference(
                        model, messages, tools, temperature, max_tokens, stream=True, **kwargs
                    ),
                )

                # Convert sync iterator to async
                for chunk in sync_stream:
                    yield chunk

            return stream_generator()  # type: ignore[no-any-return, no-untyped-call]
        else:
            result = await loop.run_in_executor(
                None,
                lambda: self.inference(
                    model, messages, tools, temperature, max_tokens, stream, **kwargs
                ),
            )
            return result  # type: ignore[return-value]

    def close(self) -> None:
        """Close the client."""
        # TensorZero embedded doesn't need explicit cleanup
        pass

    async def aclose(self) -> None:
        """Async close."""
        # TensorZero embedded doesn't need explicit cleanup
        pass
