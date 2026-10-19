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

import base64
from functools import cached_property
import os
from typing import Any

from dimos_generated.sensor_msgs.msg import Image
from openai import OpenAI

from dimos.models.vl.base import VlModel, VlModelConfig
from dimos.msgs.image import image_to_jpeg

# Must be a model that returns ABSOLUTE pixel coordinates. The qwen3-vl-* models return
# normalized 0-1000 coordinates and ignore prompts asking for pixels, which silently breaks
# query_detections, query_points and get_bbox_from_qwen_frame.
DEFAULT_QWEN_VL_MODEL = "qwen-vl-max"


class QwenVlModelConfig(VlModelConfig):
    """Configuration for Qwen VL model."""

    model_name: str = DEFAULT_QWEN_VL_MODEL
    api_key: str | None = None


class QwenVlModel(VlModel):
    config: QwenVlModelConfig

    @cached_property
    def _client(self) -> OpenAI:
        api_key = self.config.api_key or os.getenv("ALIBABA_API_KEY")
        if not api_key:
            raise ValueError(
                "Alibaba API key must be provided or set in ALIBABA_API_KEY environment variable"
            )

        return OpenAI(
            base_url="https://dashscope-intl.aliyuncs.com/compatible-mode/v1",
            api_key=api_key,
        )

    def query(self, image: Image, query: str) -> str:  # type: ignore[override]
        # Apply auto_resize if configured
        image, _ = self._prepare_image(image)

        img_base64 = base64.b64encode(image_to_jpeg(image)).decode("ascii")

        response = self._client.chat.completions.create(
            model=self.config.model_name,
            messages=[
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
                    "url": f"data:image/jpeg;base64,{base64.b64encode(image_to_jpeg(self._prepare_image(img)[0])).decode('ascii')}"
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
