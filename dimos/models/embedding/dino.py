# Copyright 2025-2026 Dimensional Inc.
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
from typing import Any, overload

from PIL import Image as PILImage
import torch
import torch.nn.functional as functional
from transformers import AutoImageProcessor, AutoModel

from dimos.models.base import HuggingFaceModel
from dimos.models.embedding.base import Embedding, EmbeddingModel, HuggingFaceEmbeddingModelConfig
from dimos.msgs.sensor_msgs.Image import Image


class DINOModelConfig(HuggingFaceEmbeddingModelConfig):
    model_name: str = "facebook/dinov2-base"
    dtype: torch.dtype = torch.float32


class DINOModel(EmbeddingModel, HuggingFaceModel):
    """DINOv2 image embedding model for visual instance identity."""

    config: DINOModelConfig
    _model_class = AutoModel

    @cached_property
    def _model(self) -> Any:
        return super()._model.eval()

    @cached_property
    def _processor(self) -> AutoImageProcessor:
        return AutoImageProcessor.from_pretrained(self.config.model_name)

    @overload
    def embed(self, image: Image, /) -> Embedding: ...
    @overload
    def embed(self, *images: Image) -> list[Embedding]: ...
    def embed(self, *images: Image) -> Embedding | list[Embedding]:
        pil_images = [PILImage.fromarray(img.to_rgb().data) for img in images]
        with torch.inference_mode():
            inputs = self._processor(images=pil_images, return_tensors="pt").to(self.config.device)
            outputs = self._model(**inputs)
            feats = getattr(outputs, "pooler_output", None)
            if feats is None:
                last_hidden = getattr(outputs, "last_hidden_state", None)
                if last_hidden is None:
                    raise RuntimeError(
                        "DINO model did not return pooler_output or last_hidden_state"
                    )
                feats = last_hidden[:, 0]
            if self.config.normalize:
                feats = functional.normalize(feats, dim=-1)

        embeddings: list[Embedding] = []
        for i, feat in enumerate(feats):
            embeddings.append(Embedding(vector=feat, timestamp=images[i].ts))
        return embeddings[0] if len(images) == 1 else embeddings

    @overload
    def embed_text(self, text: str, /) -> Embedding: ...
    @overload
    def embed_text(self, *texts: str) -> list[Embedding]: ...
    def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
        raise NotImplementedError("DINOModel does not support text embeddings")

    def stop(self) -> None:
        if "_processor" in self.__dict__:
            del self.__dict__["_processor"]
        super().stop()
