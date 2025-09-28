import os
import base64
import io
from typing import Optional
import numpy as np
from openai import OpenAI
from PIL import Image

from dimos.models.vl.base import VlModel


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

    def query(self, image: np.ndarray, query: str) -> str:
        pil_image = Image.fromarray(image.astype("uint8"))
        buffered = io.BytesIO()
        pil_image.save(buffered, format="PNG")
        img_base64 = base64.b64encode(buffered.getvalue()).decode("utf-8")

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
