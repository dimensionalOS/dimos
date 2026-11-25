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
from typing import overload

from PIL import Image as PILImage
import torch
from torch.nn import functional
from transformers import SiglipModel as HFSiglipModel, SiglipProcessor

from dimos.models.base import HuggingFaceModel
from dimos.models.embedding.base import Embedding, EmbeddingModel, HuggingFaceEmbeddingModelConfig
from dimos.msgs.sensor_msgs.Image import Image

TEXT_MAX_LENGTH = 64


class SigLIP2ModelConfig(HuggingFaceEmbeddingModelConfig):
    model_name: str = "google/siglip2-base-patch16-224"
    dtype: torch.dtype = torch.float32


class SigLIP2Model(EmbeddingModel, HuggingFaceModel):
    """SigLIP 2 image/text embedding model for frame-level semantic retrieval.

    Takes the fixed-resolution checkpoints, which keep the SigLIP architecture
    with a Gemma tokenizer. That tokenizer has no length of its own, so text is
    padded to 64 tokens explicitly, and lowercased, as the model was trained.
    """

    config: SigLIP2ModelConfig
    _model_class = HFSiglipModel

    @cached_property
    def _model(self) -> HFSiglipModel:
        self._ensure_cuda_initialized()
        model = HFSiglipModel.from_pretrained(self.config.model_name, torch_dtype=self.config.dtype)
        return model.eval().to(self.config.device)

    @cached_property
    def _processor(self) -> SiglipProcessor:
        return SiglipProcessor.from_pretrained(self.config.model_name, use_fast=True)

    @overload
    def embed(self, image: Image, /) -> Embedding: ...
    @overload
    def embed(self, *images: Image) -> list[Embedding]: ...
    def embed(self, *images: Image) -> Embedding | list[Embedding]:
        """Embed one or more images into the shared image-text space."""
        pil_images = [PILImage.fromarray(img.to_rgb().data) for img in images]

        with torch.inference_mode():
            inputs = self._processor(images=pil_images, return_tensors="pt")
            inputs = self._move_inputs_to_device(inputs)
            image_features = self._model.get_image_features(**inputs)
            if self.config.normalize:
                image_features = functional.normalize(image_features, dim=-1)

        embeddings = [
            Embedding(vector=feat, timestamp=images[i].ts) for i, feat in enumerate(image_features)
        ]
        return embeddings[0] if len(images) == 1 else embeddings

    @overload
    def embed_text(self, text: str, /) -> Embedding: ...
    @overload
    def embed_text(self, *texts: str) -> list[Embedding]: ...
    def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
        """Embed one or more text strings into the shared image-text space."""
        with torch.inference_mode():
            inputs = self._processor(
                text=[text.lower() for text in texts],
                return_tensors="pt",
                padding="max_length",
                max_length=TEXT_MAX_LENGTH,
                truncation=True,
            ).to(self.config.device)
            text_features = self._model.get_text_features(**inputs)
            if self.config.normalize:
                text_features = functional.normalize(text_features, dim=-1)

        embeddings = [Embedding(vector=feat) for feat in text_features]
        return embeddings[0] if len(texts) == 1 else embeddings

    def stop(self) -> None:
        """Release model and free GPU memory."""
        if "_processor" in self.__dict__:
            del self.__dict__["_processor"]
        super().stop()
