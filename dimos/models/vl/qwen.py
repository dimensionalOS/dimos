import os
from typing import Optional

import numpy as np
from openai import OpenAI

from dimos.models.vl.base import VlModel
from dimos.msgs.sensor_msgs import Image


class QwenVlModel(VlModel):
    _client: OpenAI
    _model_name: str

    def __init__(self, api_key: Optional[str] = None, model_name: str = "qwen2.5-vl-72b-instruct"):
        self._model_name = model_name

        api_key = api_key or os.getenv("ALIBABA_API_KEY")
        if not api_key:
            raise ValueError(
                "Alibaba API key must be provided or set in ALIBABA_API_KEY environment variable"
            )

        self._client = OpenAI(
            base_url="https://dashscope-intl.aliyuncs.com/compatible-mode/v1",
            api_key=api_key,
        )

    def query(self, image: Image | np.ndarray, query: str) -> str:
        if isinstance(image, np.ndarray):
            import warnings

            warnings.warn(
                "QwenVlModel.query should receive standard dimos Image type, not a numpy array",
                DeprecationWarning,
                stacklevel=2,
            )

            image = Image.from_numpy(image)

        img_base64 = image.to_base64()

        response = self._client.chat.completions.create(
            model=self._model_name,
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

        return response.choices[0].message.content
