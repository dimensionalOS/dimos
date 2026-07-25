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

from functools import cached_property
import os
from typing import Any

import numpy as np
from openai import OpenAI

from dimos.models.vl.base import VlModel, VlModelConfig
from dimos.msgs.sensor_msgs.Image import Image


class QwenVlModelConfig(VlModelConfig):
    """Configuration for Qwen VL model."""

    model_name: str = "qwen2.5-vl-72b-instruct"
    api_key: str | None = None
    base_url: str = "https://dashscope-intl.aliyuncs.com/compatible-mode/v1"


class QwenVlModel(VlModel):
    config: QwenVlModelConfig

    @cached_property
    def _client(self) -> OpenAI:
        api_key = (
            self.config.api_key
            or os.getenv("DIMOS_QWEN_VL_API_KEY")
            or os.getenv("ALIBABA_API_KEY")
        )
        if not api_key:
            raise ValueError(
                "Qwen VL API key must be provided or set in "
                "DIMOS_QWEN_VL_API_KEY/ALIBABA_API_KEY"
            )

        return OpenAI(
            base_url=os.getenv("DIMOS_QWEN_VL_BASE_URL", self.config.base_url),
            api_key=api_key,
        )

    @property
    def model_name(self) -> str:
        """Resolved provider model ID, allowing deployment-level overrides."""

        return os.getenv("DIMOS_QWEN_VL_MODEL", self.config.model_name)

    def query(self, image: Image | np.ndarray, query: str) -> str:  # type: ignore[override]
        if isinstance(image, np.ndarray):
            import warnings

            warnings.warn(
                "QwenVlModel.query should receive standard dimos Image type, not a numpy array",
                DeprecationWarning,
                stacklevel=2,
            )

            image = Image.from_numpy(image)

        # Apply auto_resize if configured
        image, _ = self._prepare_image(image)

        img_base64 = image.to_base64()

        response = self._client.chat.completions.create(
            model=self.model_name,
            messages=[
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
        )

        return response.choices[0].message.content  # type: ignore[return-value]

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
                    "url": f"data:image/png;base64,{self._prepare_image(img)[0].to_base64()}"
                },
            }
            for img in images
        ]
        content.append({"type": "text", "text": query})

        messages = [{"role": "user", "content": content}]
        api_kwargs: dict[str, Any] = {"model": self.model_name, "messages": messages}
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
