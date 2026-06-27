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

"""MiniMax vision-language model.

Uses MiniMax's OpenAI-compatible chat completions API. MiniMax exposes
``https://api.minimax.io/v1`` and accepts the same ``client.chat.completions.create``
shape as OpenAI, so the same ``openai`` SDK can target it via a custom
``base_url``.

Reference: https://platform.minimax.io/docs/api-reference/text-openai-api
"""

from functools import cached_property
import os
from typing import Any

import numpy as np
from openai import OpenAI

from dimos.models.vl.base import VlModel, VlModelConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Default MiniMax API base URL. Override with ``MINIMAX_BASE_URL`` if you
# need to use the domestic endpoint or a proxy.
DEFAULT_MINIMAX_BASE_URL = "https://api.minimax.io/v1"

# Only M3 is supported — the latest MiniMax chat model with image input.
DEFAULT_MINIMAX_MODEL = "MiniMax-M3"


class MiniMaxVlModelConfig(VlModelConfig):
    """Configuration for the MiniMax VLM."""

    model_name: str = DEFAULT_MINIMAX_MODEL
    api_key: str | None = None
    base_url: str | None = None


class MiniMaxVlModel(VlModel):
    """Vision-language model backed by MiniMax's OpenAI-compatible endpoint.

    Auth: set ``MINIMAX_API_KEY`` (or pass ``api_key`` explicitly).
    Optional override: ``MINIMAX_BASE_URL`` (default: ``https://api.minimax.io/v1``).
    """

    config: MiniMaxVlModelConfig

    @cached_property
    def _client(self) -> OpenAI:
        api_key = self.config.api_key or os.getenv("MINIMAX_API_KEY")
        if not api_key:
            raise ValueError(
                "MiniMax API key must be provided or set in MINIMAX_API_KEY environment variable"
            )

        base_url = self.config.base_url or os.getenv("MINIMAX_BASE_URL", DEFAULT_MINIMAX_BASE_URL)

        return OpenAI(api_key=api_key, base_url=base_url)

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
                "MiniMaxVlModel.query should receive standard dimos Image type, not a numpy array",
                DeprecationWarning,
                stacklevel=2,
            )

            image = Image.from_numpy(image)

        # Apply auto_resize if configured
        image, _ = self._prepare_image(image)

        img_base64 = image.to_base64()

        # ``response_format`` is not supported by MiniMax and will be rejected
        # with a 4xx; if a caller asked for JSON output we have to fall back
        # to a prompt-driven approach at a higher layer.
        if response_format:
            logger.warning(
                "MiniMax does not support response_format; ignoring and relying on prompt."
            )

        api_kwargs: dict[str, Any] = {
            "model": self.config.model_name,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/png;base64,{img_base64}"},
                        },
                        {"type": "text", "text": query},
                    ],
                }
            ],
            # MiniMax requires temperature in (0.0, 1.0]; 0 is rejected.
            "temperature": 1.0,
        }

        response = self._client.chat.completions.create(**api_kwargs)

        return response.choices[0].message.content  # type: ignore[no-any-return]

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

        if response_format:
            logger.warning(
                "MiniMax does not support response_format; ignoring and relying on prompt."
            )

        content: list[dict[str, Any]] = [
            {
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/png;base64,{self._prepare_image(img)[0].to_base64()}"
                },
            }
            for img in images
        ]
        content.append({"type": "text", "text": query})

        messages = [{"role": "user", "content": content}]
        api_kwargs: dict[str, Any] = {
            "model": self.config.model_name,
            "messages": messages,
            "temperature": 1.0,
        }

        response = self._client.chat.completions.create(**api_kwargs)
        response_text = response.choices[0].message.content or ""
        # Return one response per image (same response since API analyzes all images together)
        return [response_text] * len(images)

    def stop(self) -> None:
        """Release the OpenAI client."""
        if "_client" in self.__dict__:
            del self.__dict__["_client"]
