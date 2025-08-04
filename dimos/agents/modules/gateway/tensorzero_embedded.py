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

import os
import json
import logging
from typing import Any, AsyncIterator, Dict, Iterator, List, Optional, Union
from pathlib import Path

logger = logging.getLogger(__name__)


class TensorZeroEmbeddedGateway:
    """TensorZero embedded gateway using patch_openai_client."""

    def __init__(self):
        """Initialize TensorZero embedded gateway."""
        self._client = None
        self._config_path = None
        self._setup_config()
        self._initialize_client()

    def _setup_config(self):
        """Create TensorZero configuration with correct format."""
        config_dir = Path("/tmp/tensorzero_embedded")
        config_dir.mkdir(exist_ok=True)
        self._config_path = config_dir / "tensorzero.toml"

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

# Cerebras Models
[models.llama_3_3_70b]
routing = ["cerebras"]

[models.llama_3_3_70b.providers.cerebras]
type = "openai"
model_name = "llama-3.3-70b"
api_base = "https://api.cerebras.ai/v1"
api_key_location = "env::CEREBRAS_API_KEY"

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

# Functions
[functions.chat]
type = "chat"

[functions.chat.variants.openai]
type = "chat_completion"
model = "gpt_4o_mini"

[functions.chat.variants.claude]
type = "chat_completion"
model = "claude_3_haiku"

[functions.chat.variants.cerebras]
type = "chat_completion"
model = "llama_3_3_70b"

[functions.chat.variants.qwen]
type = "chat_completion"
model = "qwen_plus"

[functions.vision]
type = "chat"

[functions.vision.variants.openai]
type = "chat_completion"
model = "gpt_4o_mini"

[functions.vision.variants.claude]
type = "chat_completion"
model = "claude_3_haiku"

[functions.vision.variants.qwen]
type = "chat_completion"
model = "qwen_vl_plus"
"""

        with open(self._config_path, "w") as f:
            f.write(config_content)

        logger.info(f"Created TensorZero config at {self._config_path}")

    def _initialize_client(self):
        """Initialize OpenAI client with TensorZero patch."""
        try:
            from openai import OpenAI
            from tensorzero import patch_openai_client

            # Create base OpenAI client
            self._client = OpenAI()

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
        # Map common models to TensorZero functions
        model_mapping = {
            # OpenAI models
            "openai::gpt-4o-mini": "tensorzero::function_name::chat",
            "openai::gpt-4o": "tensorzero::function_name::chat",
            # Claude models
            "anthropic::claude-3-haiku-20240307": "tensorzero::function_name::chat",
            "anthropic::claude-3-5-sonnet-20241022": "tensorzero::function_name::chat",
            "anthropic::claude-3-opus-20240229": "tensorzero::function_name::chat",
            # Cerebras models
            "cerebras::llama-3.3-70b": "tensorzero::function_name::chat",
            "cerebras::llama3.1-8b": "tensorzero::function_name::chat",
            # Qwen models
            "qwen::qwen-plus": "tensorzero::function_name::chat",
            "qwen::qwen-vl-plus": "tensorzero::function_name::vision",
        }

        # Check if it's already in TensorZero format
        if model.startswith("tensorzero::"):
            return model

        # Try to map the model
        mapped = model_mapping.get(model)
        if mapped:
            # Append variant based on provider
            if "::" in model:
                provider = model.split("::")[0]
                if "vision" in mapped:
                    # For vision models, use provider-specific variant
                    if provider == "qwen":
                        return mapped  # Use qwen vision variant
                    else:
                        return mapped  # Use openai/claude vision variant
                else:
                    # For chat models, always use chat function
                    return mapped

        # Default to chat function
        logger.warning(f"Unknown model format: {model}, defaulting to chat")
        return "tensorzero::function_name::chat"

    def inference(
        self,
        model: str,
        messages: List[Dict[str, Any]],
        tools: Optional[List[Dict[str, Any]]] = None,
        temperature: float = 0.0,
        max_tokens: Optional[int] = None,
        stream: bool = False,
        **kwargs,
    ) -> Union[Dict[str, Any], Iterator[Dict[str, Any]]]:
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
                stream_response = self._client.chat.completions.create(**params)

                def stream_generator():
                    for chunk in stream_response:
                        yield chunk.model_dump()

                return stream_generator()
            else:
                response = self._client.chat.completions.create(**params)
                return response.model_dump()

        except Exception as e:
            logger.error(f"TensorZero inference failed: {e}")
            raise

    async def ainference(
        self,
        model: str,
        messages: List[Dict[str, Any]],
        tools: Optional[List[Dict[str, Any]]] = None,
        temperature: float = 0.0,
        max_tokens: Optional[int] = None,
        stream: bool = False,
        **kwargs,
    ) -> Union[Dict[str, Any], AsyncIterator[Dict[str, Any]]]:
        """Async inference - wraps sync for now."""

        # TensorZero embedded doesn't have async support yet
        # Run sync version in executor
        import asyncio

        loop = asyncio.get_event_loop()

        if stream:
            # Streaming not supported in async wrapper yet
            raise NotImplementedError("Async streaming not yet supported with TensorZero embedded")
        else:
            result = await loop.run_in_executor(
                None,
                lambda: self.inference(
                    model, messages, tools, temperature, max_tokens, stream, **kwargs
                ),
            )
            return result

    def close(self):
        """Close the client."""
        # TensorZero embedded doesn't need explicit cleanup
        pass

    async def aclose(self):
        """Async close."""
        # TensorZero embedded doesn't need explicit cleanup
        pass
