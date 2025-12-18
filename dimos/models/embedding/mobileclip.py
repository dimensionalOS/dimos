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
from typing import Any

import open_clip  # type: ignore[import-untyped]
from PIL import Image as PILImage
import torch
import torch.nn.functional as F

from dimos.models.base import LocalModel
from dimos.models.embedding.base import Embedding, EmbeddingModel, EmbeddingModelConfig
from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data


class MobileCLIPEmbedding(Embedding): ...


@dataclass
class MobileCLIPModelConfig(EmbeddingModelConfig):
    model_name: str = "MobileCLIP2-S4"


class MobileCLIPModel(EmbeddingModel[MobileCLIPEmbedding], LocalModel):
    """MobileCLIP embedding model for vision-language re-identification."""

    default_config = MobileCLIPModelConfig
    config: MobileCLIPModelConfig

    @cached_property
    def _model_and_preprocess(self) -> tuple[Any, Any]:
        """Load model and transforms (open_clip returns them together)."""
        model_path = get_data("models_mobileclip") / (self.config.model_name + ".pt")
        model, _, preprocess = open_clip.create_model_and_transforms(
            self.config.model_name, pretrained=str(model_path)
        )
        return model.eval().to(self.config.device), preprocess

    @cached_property
    def _model(self) -> Any:
        return self._model_and_preprocess[0]

    @cached_property
    def _preprocess(self) -> Any:
        return self._model_and_preprocess[1]

    @cached_property
    def _tokenizer(self) -> Any:
        return open_clip.get_tokenizer(self.config.model_name)

    def embed(self, *images: Image) -> MobileCLIPEmbedding | list[MobileCLIPEmbedding]:
        """Embed one or more images.

        Returns embeddings as torch.Tensor on device for efficient GPU comparisons.
        """
        # Convert to PIL images
        pil_images = [PILImage.fromarray(img.to_opencv()) for img in images]

        # Preprocess and batch
        with torch.inference_mode():
            batch = torch.stack([self._preprocess(img) for img in pil_images]).to(
                self.config.device
            )
            feats = self._model.encode_image(batch)
            if self.config.normalize:
                feats = F.normalize(feats, dim=-1)

        # Create embeddings (keep as torch.Tensor on device)
        embeddings = []
        for i, feat in enumerate(feats):
            timestamp = images[i].ts
            embeddings.append(MobileCLIPEmbedding(vector=feat, timestamp=timestamp))

        return embeddings[0] if len(images) == 1 else embeddings

    def embed_text(self, *texts: str) -> MobileCLIPEmbedding | list[MobileCLIPEmbedding]:
        """Embed one or more text strings.

        Returns embeddings as torch.Tensor on device for efficient GPU comparisons.
        """
        with torch.inference_mode():
            text_tokens = self._tokenizer(list(texts)).to(self.config.device)
            feats = self._model.encode_text(text_tokens)
            if self.config.normalize:
                feats = F.normalize(feats, dim=-1)

        # Create embeddings (keep as torch.Tensor on device)
        embeddings = []
        for feat in feats:
            embeddings.append(MobileCLIPEmbedding(vector=feat))

        return embeddings[0] if len(texts) == 1 else embeddings

    def start(self) -> None:
        """Start the model with a dummy forward pass."""
        super().start()
        dummy_image = torch.randn(1, 3, 224, 224).to(self.config.device)
        dummy_text = self._tokenizer(["warmup"]).to(self.config.device)
        with torch.inference_mode():
            self._model.encode_image(dummy_image)
            self._model.encode_text(dummy_text)

    def stop(self) -> None:
        """Release model and free GPU memory."""
        for attr in ("_model_and_preprocess", "_model", "_preprocess", "_tokenizer"):
            if attr in self.__dict__:
                del self.__dict__[attr]
        super().stop()
