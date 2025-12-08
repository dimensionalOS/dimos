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

from pathlib import Path
from typing import Any

import open_clip  # type: ignore[import-not-found]
from PIL import Image as PILImage
import torch
import torch.nn.functional as F

from dimos.models.base import LocalModel
from dimos.models.embedding.base import Embedding, EmbeddingModel
from dimos.msgs.sensor_msgs import Image


class MobileCLIPEmbedding(Embedding): ...


class MobileCLIPModel(EmbeddingModel[MobileCLIPEmbedding], LocalModel):
    """MobileCLIP embedding model for vision-language re-identification."""

    _model_name: str
    _model_path: Path | str | None
    _preprocess: Any
    _tokenizer: Any

    def __init__(
        self,
        model_name: str = "MobileCLIP2-S4",
        model_path: Path | str | None = None,
        device: str | None = None,
        normalize: bool = True,
        warmup: bool = False,
    ) -> None:
        """
        Initialize MobileCLIP model.

        Args:
            model_name: Name of the model architecture
            model_path: Path to pretrained weights
            device: Device to run on (cuda/cpu), auto-detects if None
            normalize: Whether to L2 normalize embeddings
            warmup: If True, immediately load and warmup the model.
        """
        self._model_name = model_name
        self._model_path = model_path
        self.normalize = normalize
        LocalModel.__init__(self, device=device, warmup=warmup)

    def _load_model(self) -> Any:
        pretrained = str(self._model_path) if self._model_path else None
        model, _, self._preprocess = open_clip.create_model_and_transforms(
            self._model_name, pretrained=pretrained
        )
        self._tokenizer = open_clip.get_tokenizer(self._model_name)
        return model.eval().to(self._device)

    def embed(self, *images: Image) -> MobileCLIPEmbedding | list[MobileCLIPEmbedding]:
        """Embed one or more images.

        Returns embeddings as torch.Tensor on device for efficient GPU comparisons.
        """
        # Convert to PIL images
        pil_images = [PILImage.fromarray(img.to_opencv()) for img in images]

        # Preprocess and batch
        with torch.inference_mode():
            batch = torch.stack([self._preprocess(img) for img in pil_images]).to(self._device)
            feats = self._model.encode_image(batch)
            if self.normalize:
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
            text_tokens = self._tokenizer(list(texts)).to(self._device)
            feats = self._model.encode_text(text_tokens)
            if self.normalize:
                feats = F.normalize(feats, dim=-1)

        # Create embeddings (keep as torch.Tensor on device)
        embeddings = []
        for feat in feats:
            embeddings.append(MobileCLIPEmbedding(vector=feat))

        return embeddings[0] if len(texts) == 1 else embeddings

    def warmup(self) -> None:
        """Warmup the model with a dummy forward pass."""
        super().warmup()
        dummy_image = torch.randn(1, 3, 224, 224).to(self._device)
        dummy_text = self._tokenizer(["warmup"]).to(self._device)
        with torch.inference_mode():
            self._model.encode_image(dummy_image)
            self._model.encode_text(dummy_text)
