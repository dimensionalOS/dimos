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
from typing import TYPE_CHECKING, Any
import warnings

import numpy as np
from PIL import Image as PILImage

from dimos.models.vl.base import VlModel, VlModelConfig
from dimos.msgs.sensor_msgs.Image import Image

if TYPE_CHECKING:
    from google import genai


class GeminiVlModelConfig(VlModelConfig):
    """Configuration for the Gemini VL model."""

    model_name: str = "gemini-2.5-flash"
    api_key: str | None = None


class GeminiVlModel(VlModel):
    """VL model backed by Google's Gemini API (google-genai).

    Reuses the same ``GOOGLE_API_KEY`` / ``GEMINI_API_KEY`` auth as the Gemini
    TTS and embedding components, so no extra key or dependency is required.
    """

    config: GeminiVlModelConfig

    @cached_property
    def _client(self) -> "genai.Client":
        from google import genai

        api_key = (
            self.config.api_key or os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY")
        )
        if not api_key:
            raise ValueError(
                "Gemini VL model requires GOOGLE_API_KEY or GEMINI_API_KEY to be set"
            )

        return genai.Client(api_key=api_key)

    def _to_pil(self, image: Image | np.ndarray[Any, Any]) -> PILImage.Image:
        """Convert dimos Image or numpy array to PIL Image, applying auto_resize."""
        if isinstance(image, np.ndarray):
            warnings.warn(
                "GeminiVlModel.query should receive standard dimos Image type, not a numpy array",
                DeprecationWarning,
                stacklevel=2,
            )
            image = Image.from_numpy(image)

        image, _ = self._prepare_image(image)
        rgb_image = image.to_rgb()
        return PILImage.fromarray(rgb_image.data)

    def query(self, image: Image | np.ndarray, query: str) -> str:  # type: ignore[override]
        pil_image = self._to_pil(image)

        response = self._client.models.generate_content(
            model=self.config.model_name,
            contents=[pil_image, query],
        )

        return response.text or ""

    def stop(self) -> None:
        """Release the Gemini client."""
        if "_client" in self.__dict__:
            del self.__dict__["_client"]
