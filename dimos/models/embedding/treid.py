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

import torch
import torch.nn.functional as F
from torchreid import utils as torchreid_utils  # type: ignore[import-not-found]

from dimos.models.base import LocalModel
from dimos.models.embedding.base import Embedding, EmbeddingModel
from dimos.msgs.sensor_msgs import Image


class TorchReIDEmbedding(Embedding): ...


class TorchReIDModel(EmbeddingModel[TorchReIDEmbedding], LocalModel):
    """TorchReID embedding model for person re-identification."""

    _model_name: str
    _model_path: Path | str | None

    def __init__(
        self,
        model_name: str = "se_resnext101_32x4d",
        model_path: Path | str | None = None,
        device: str | None = None,
        normalize: bool = False,
        warmup: bool = False,
    ) -> None:
        """
        Initialize TorchReID model.

        Args:
            model_name: Name of the model architecture (e.g., "osnet_x1_0", "osnet_x0_75")
            model_path: Path to pretrained weights (.pth.tar file)
            device: Device to run on (cuda/cpu), auto-detects if None
            normalize: Whether to L2 normalize embeddings
            warmup: If True, immediately load and warmup the model.
        """
        self._model_name = model_name
        self._model_path = model_path
        self.normalize = normalize
        LocalModel.__init__(self, device=device, warmup=warmup)

    def _load_model(self) -> torchreid_utils.FeatureExtractor:
        self._ensure_cuda_initialized()
        model_path_str = str(self._model_path) if self._model_path else ""
        return torchreid_utils.FeatureExtractor(
            model_name=self._model_name,
            model_path=model_path_str,
            device=self._device,
        )

    def embed(self, *images: Image) -> TorchReIDEmbedding | list[TorchReIDEmbedding]:
        """Embed one or more images.

        Returns embeddings as torch.Tensor on device for efficient GPU comparisons.
        """
        # Convert to numpy arrays - torchreid expects numpy arrays or file paths
        np_images = [img.to_opencv() for img in images]

        # Extract features
        with torch.inference_mode():
            features = self._model(np_images)

            # torchreid may return either numpy array or torch tensor depending on configuration
            if isinstance(features, torch.Tensor):
                features_tensor = features.to(self._device)
            else:
                features_tensor = torch.from_numpy(features).to(self._device)

            if self.normalize:
                features_tensor = F.normalize(features_tensor, dim=-1)

        # Create embeddings (keep as torch.Tensor on device)
        embeddings = []
        for i, feat in enumerate(features_tensor):
            timestamp = images[i].ts
            embeddings.append(TorchReIDEmbedding(vector=feat, timestamp=timestamp))

        return embeddings[0] if len(images) == 1 else embeddings

    def embed_text(self, *texts: str) -> TorchReIDEmbedding | list[TorchReIDEmbedding]:
        """Text embedding not supported for ReID models.

        TorchReID models are vision-only person re-identification models
        and do not support text embeddings.
        """
        raise NotImplementedError(
            "TorchReID models are vision-only and do not support text embeddings. "
            "Use CLIP or MobileCLIP for text-image similarity."
        )

    def warmup(self) -> None:
        """Warmup the model with a dummy forward pass."""
        super().warmup()

        # Create a dummy 256x128 image (typical person ReID input size) as numpy array
        import numpy as np

        dummy_image = np.random.randint(0, 256, (256, 128, 3), dtype=np.uint8)
        with torch.inference_mode():
            _ = self._model([dummy_image])
