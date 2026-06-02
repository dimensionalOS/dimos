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

from typing import Any

import numpy as np

from dimos.models.vl.base import VlModel, VlModelConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class LiteLLMVlModelConfig(VlModelConfig):
    """Configuration for LiteLLM-backed vision-language model.

    LiteLLM provides a unified interface to 100+ LLM providers (OpenAI, Anthropic,
    Google, Azure, Bedrock, etc.) through a single API.
    """

    model_name: str = "gpt-4o-mini"
    api_key: str | None = None
    api_base: str | None = None


class LiteLLMVlModel(VlModel):
    """Vision-language model backed by LiteLLM.

    Supports any vision-capable model available through LiteLLM, including
    OpenAI GPT-4o, Anthropic Claude, Google Gemini, Azure OpenAI, AWS Bedrock,
    and many more through a single unified interface.

    See https://docs.litellm.ai/docs/providers for the full list of supported
    providers and model identifiers.
    """

    config: LiteLLMVlModelConfig

    def _completion(self, **kwargs: Any) -> Any:
        try:
            import litellm
        except ImportError as e:
            raise ImportError(
                "litellm is required for LiteLLMVlModel. "
                "Install it with: pip install 'dimos[litellm]'"
            ) from e

        if self.config.api_key:
            kwargs["api_key"] = self.config.api_key

        if self.config.api_base:
            kwargs["api_base"] = self.config.api_base

        return litellm.completion(drop_params=True, **kwargs)

    def query(
        self,
        image: Image | np.ndarray,
        query: str,
        response_format: dict[str, Any] | None = None,
        **kwargs: Any,
    ) -> str:
        if isinstance(image, np.ndarray):
            import warnings

            warnings.warn(
                "LiteLLMVlModel.query should receive standard dimos Image type, not a numpy array",
                DeprecationWarning,
                stacklevel=2,
            )
            image = Image.from_numpy(image)

        image, _ = self._prepare_image(image)
        img_base64 = image.to_base64()

        api_kwargs: dict[str, Any] = {
            "model": self.config.model_name,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"},
                        },
                        {"type": "text", "text": query},
                    ],
                }
            ],
        }

        if response_format:
            api_kwargs["response_format"] = response_format

        response = self._completion(**api_kwargs)
        return response.choices[0].message.content or ""

    def query_batch(
        self,
        images: list[Image],
        query: str,
        response_format: dict[str, Any] | None = None,
        **kwargs: Any,
    ) -> list[str]:
        """Query VLM with multiple images using a single API call."""
        if not images:
            return []

        content: list[dict[str, Any]] = [
            {
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{self._prepare_image(img)[0].to_base64()}"
                },
            }
            for img in images
        ]
        content.append({"type": "text", "text": query})

        api_kwargs: dict[str, Any] = {
            "model": self.config.model_name,
            "messages": [{"role": "user", "content": content}],
        }
        if response_format:
            api_kwargs["response_format"] = response_format

        response = self._completion(**api_kwargs)
        response_text = response.choices[0].message.content or ""
        return [response_text] * len(images)

    def stop(self) -> None:
        pass
