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

import torch
import torch.nn.functional as F
from torchreid import utils as torchreid_utils  # type: ignore[import-untyped]

from dimos.models.base import LocalModel
from dimos.models.embedding.base import Embedding, EmbeddingModel, EmbeddingModelConfig
from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data


class TorchReIDEmbedding(Embedding): ...


# osnet models downloaded from https://kaiyangzhou.github.io/deep-person-reid/MODEL_ZOO.html
# into dimos/data/models_torchreid/
# feel free to add more
@dataclass
class TorchReIDModelConfig(EmbeddingModelConfig):
    model_name: str = "osnet_x1_0"


class TorchReIDModel(EmbeddingModel[TorchReIDEmbedding], LocalModel):
    """TorchReID embedding model for person re-identification."""

    default_config = TorchReIDModelConfig
    config: TorchReIDModelConfig

    @cached_property
    def _model(self) -> torchreid_utils.FeatureExtractor:
        self._ensure_cuda_initialized()
        return torchreid_utils.FeatureExtractor(
            model_name=self.config.model_name,
            model_path=str(get_data("models_torchreid") / (self.config.model_name + ".pth")),
            device=self.config.device,
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
                features_tensor = features.to(self.config.device)
            else:
                features_tensor = torch.from_numpy(features).to(self.config.device)

            if self.config.normalize:
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

    def start(self) -> None:
        """Start the model with a dummy forward pass."""
        super().start()

        # Create a dummy 256x128 image (typical person ReID input size) as numpy array
        import numpy as np

        dummy_image = np.random.randint(0, 256, (256, 128, 3), dtype=np.uint8)
        with torch.inference_mode():
            _ = self._model([dummy_image])

    def stop(self) -> None:
        """Release model and free GPU memory."""
        super().stop()
