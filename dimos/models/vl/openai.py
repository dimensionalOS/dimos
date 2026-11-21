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

from functools import cached_property
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.models.vl.base import HostedVlModelConfig, VlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from openai import OpenAI

logger = setup_logger()


def _client_type() -> type[OpenAI]:
    # Keep blueprint discovery independent of the optional SDK.
    from openai import OpenAI

    return OpenAI


class OpenAIVlModelConfig(HostedVlModelConfig):
    model_name: str = "gpt-4o-mini"
    api_key_env = "OPENAI_API_KEY"

    def validate_runtime(self) -> None:
        super().validate_runtime()
        try:
            _client_type()
        except ImportError as exc:
            raise ValueError("The OpenAI vision model requires the openai SDK") from exc


class OpenAIVlModel(VlModel):
    config: OpenAIVlModelConfig

    @cached_property
    def _client(self) -> OpenAI:
        return _client_type()(api_key=self.config.require_api_key())

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
                "OpenAIVlModel.query should receive standard dimos Image type, not a numpy array",
                DeprecationWarning,
                stacklevel=2,
            )

            image = Image.from_numpy(image)

        # Apply auto_resize if configured
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

        messages = [{"role": "user", "content": content}]
        api_kwargs: dict[str, Any] = {"model": self.config.model_name, "messages": messages}
        if response_format:
            api_kwargs["response_format"] = response_format

        response = self._client.chat.completions.create(**api_kwargs)
        response_text = response.choices[0].message.content or ""
        # Return one response per image (same response since API analyzes all images together)
        return [response_text] * len(images)

    def stop(self) -> None:
        """Release the OpenAI client."""
        if "_client" in self.__dict__:
            del self.__dict__["_client"]
