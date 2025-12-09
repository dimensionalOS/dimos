from dataclasses import dataclass
from functools import cached_property
import os

import numpy as np
from openai import OpenAI

from dimos.models.vl.base import VlModel, VlModelConfig
from dimos.msgs.sensor_msgs import Image


@dataclass
class QwenVlModelConfig(VlModelConfig):
    """Configuration for Qwen VL model."""

    model_name: str = "qwen2.5-vl-72b-instruct"
    api_key: str | None = None


class QwenVlModel(VlModel):
    default_config = QwenVlModelConfig
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

    def query(self, image: Image | np.ndarray, query: str) -> str:  # type: ignore[override, type-arg]
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
            model=self.config.model_name,
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

    def stop(self) -> None:
        """Release the OpenAI client."""
        if "_client" in self.__dict__:
            del self.__dict__["_client"]
