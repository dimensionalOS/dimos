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

import open_clip
from PIL import Image as PILImage
import torch
import torch.nn.functional as F

from dimos.models.embedding.base import Embedding, EmbeddingModel
from dimos.msgs.sensor_msgs import Image


class MobileCLIPEmbedding(Embedding): ...


class MobileCLIPModel(EmbeddingModel[MobileCLIPEmbedding]):
    """MobileCLIP embedding model for vision-language re-identification."""

    def __init__(
        self,
        model_name: str = "MobileCLIP2-S4",
        model_path: Path | str | None = None,
        device: str | None = None,
        normalize: bool = True,
    ) -> None:
        """
        Initialize MobileCLIP model.

        Args:
            model_name: Name of the model architecture
            model_path: Path to pretrained weights
            device: Device to run on (cuda/cpu), auto-detects if None
            normalize: Whether to L2 normalize embeddings
        """
        if not OPEN_CLIP_AVAILABLE:
            raise ImportError(
                "open_clip is required for MobileCLIPModel. "
                "Install it with: pip install open-clip-torch"
            )

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.normalize = normalize

        # Load model
        pretrained = str(model_path) if model_path else None
        self.model, _, self.preprocess = open_clip.create_model_and_transforms(
            model_name, pretrained=pretrained
        )
        self.tokenizer = open_clip.get_tokenizer(model_name)
        self.model = self.model.eval().to(self.device)

    def embed(self, *images: Image) -> MobileCLIPEmbedding | list[MobileCLIPEmbedding]:
        """Embed one or more images.

        Returns embeddings as torch.Tensor on device for efficient GPU comparisons.
        """
        # Convert to PIL images
        pil_images = [PILImage.fromarray(img.to_opencv()) for img in images]

        # Preprocess and batch
        with torch.inference_mode():
            batch = torch.stack([self.preprocess(img) for img in pil_images]).to(self.device)
            feats = self.model.encode_image(batch)
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
            text_tokens = self.tokenizer(list(texts)).to(self.device)
            feats = self.model.encode_text(text_tokens)
            if self.normalize:
                feats = F.normalize(feats, dim=-1)

        # Create embeddings (keep as torch.Tensor on device)
        embeddings = []
        for feat in feats:
            embeddings.append(MobileCLIPEmbedding(vector=feat))

        return embeddings[0] if len(texts) == 1 else embeddings

    def warmup(self) -> None:
        """Warmup the model with a dummy forward pass."""
        dummy_image = torch.randn(1, 3, 224, 224).to(self.device)
        dummy_text = self.tokenizer(["warmup"]).to(self.device)
        with torch.inference_mode():
            self.model.encode_image(dummy_image)
            self.model.encode_text(dummy_text)
