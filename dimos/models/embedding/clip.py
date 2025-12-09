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

from dataclasses import dataclass
from functools import cached_property

from PIL import Image as PILImage
import torch
import torch.nn.functional as F
from transformers import CLIPModel as HFCLIPModel, CLIPProcessor  # type: ignore[import-untyped]

from dimos.models.base import HuggingFaceModel
from dimos.models.embedding.base import Embedding, EmbeddingModel, HuggingFaceEmbeddingModelConfig
from dimos.msgs.sensor_msgs import Image


class CLIPEmbedding(Embedding): ...


@dataclass
class CLIPModelConfig(HuggingFaceEmbeddingModelConfig):
    model_name: str = "openai/clip-vit-base-patch32"
    dtype: torch.dtype = torch.float32


class CLIPModel(EmbeddingModel[CLIPEmbedding], HuggingFaceModel):
    """CLIP embedding model for vision-language re-identification."""

    default_config = CLIPModelConfig
    config: CLIPModelConfig
    _model_class = HFCLIPModel

    @cached_property
    def _model(self) -> HFCLIPModel:
        self._ensure_cuda_initialized()
        return HFCLIPModel.from_pretrained(self.config.model_name).eval().to(self.config.device)

    @cached_property
    def _processor(self) -> CLIPProcessor:
        return CLIPProcessor.from_pretrained(self.config.model_name)

    def embed(self, *images: Image) -> CLIPEmbedding | list[CLIPEmbedding]:
        """Embed one or more images.

        Returns embeddings as torch.Tensor on device for efficient GPU comparisons.
        """
        # Convert to PIL images
        pil_images = [PILImage.fromarray(img.to_opencv()) for img in images]

        # Process images
        with torch.inference_mode():
            inputs = self._processor(images=pil_images, return_tensors="pt").to(self.config.device)
            image_features = self._model.get_image_features(**inputs)

            if self.config.normalize:
                image_features = F.normalize(image_features, dim=-1)

        # Create embeddings (keep as torch.Tensor on device)
        embeddings = []
        for i, feat in enumerate(image_features):
            timestamp = images[i].ts
            embeddings.append(CLIPEmbedding(vector=feat, timestamp=timestamp))

        return embeddings[0] if len(images) == 1 else embeddings

    def embed_text(self, *texts: str) -> CLIPEmbedding | list[CLIPEmbedding]:
        """Embed one or more text strings.

        Returns embeddings as torch.Tensor on device for efficient GPU comparisons.
        """
        with torch.inference_mode():
            inputs = self._processor(text=list(texts), return_tensors="pt", padding=True).to(
                self.config.device
            )
            text_features = self._model.get_text_features(**inputs)

            if self.config.normalize:
                text_features = F.normalize(text_features, dim=-1)

        # Create embeddings (keep as torch.Tensor on device)
        embeddings = []
        for feat in text_features:
            embeddings.append(CLIPEmbedding(vector=feat))

        return embeddings[0] if len(texts) == 1 else embeddings

    def start(self) -> None:
        """Start the model with a dummy forward pass."""
        super().start()

        dummy_image = torch.randn(1, 3, 224, 224).to(self.config.device)
        dummy_text_inputs = self._processor(text=["warmup"], return_tensors="pt", padding=True).to(
            self.config.device
        )

        with torch.inference_mode():
            self._model.get_image_features(pixel_values=dummy_image)
            self._model.get_text_features(**dummy_text_inputs)

    def stop(self) -> None:
        """Release model and free GPU memory."""
        if "_processor" in self.__dict__:
            del self.__dict__["_processor"]
        super().stop()
