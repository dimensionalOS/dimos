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

from PIL import Image as PILImage
import torch
import torch.nn.functional as F
from transformers import CLIPModel as HFCLIPModel, CLIPProcessor  # type: ignore[import-untyped]

from dimos.models.embedding.base import Embedding, EmbeddingModel
from dimos.msgs.sensor_msgs import Image

_CUDA_INITIALIZED = False


class CLIPEmbedding(Embedding): ...


class CLIPModel(EmbeddingModel[CLIPEmbedding]):
    """CLIP embedding model for vision-language re-identification."""

    def __init__(
        self,
        model_name: str = "openai/clip-vit-base-patch32",
        device: str | None = None,
        normalize: bool = False,
    ) -> None:
        """
        Initialize CLIP model.

        Args:
            model_name: HuggingFace model name (e.g., "openai/clip-vit-base-patch32")
            device: Device to run on (cuda/cpu), auto-detects if None
            normalize: Whether to L2 normalize embeddings
        """
        # Use GPU if available, otherwise fall back to CPU
        if torch.cuda.is_available():
            self.device = "cuda"
        # MacOS Metal performance shaders
        elif torch.backends.mps.is_available() and torch.backends.mps.is_built():
            self.device = "mps"
        else:
            self.device = "cpu"
        
        self.normalize = normalize

        # Load model and processor
        self.model = HFCLIPModel.from_pretrained(model_name).eval().to(self.device)
        self.processor = CLIPProcessor.from_pretrained(model_name)

    def embed(self, *images: Image) -> CLIPEmbedding | list[CLIPEmbedding]:
        """Embed one or more images.

        Returns embeddings as torch.Tensor on device for efficient GPU comparisons.
        """
        # Convert to PIL images
        pil_images = [PILImage.fromarray(img.to_opencv()) for img in images]

        # Process images
        with torch.inference_mode():
            inputs = self.processor(images=pil_images, return_tensors="pt").to(self.device)
            image_features = self.model.get_image_features(**inputs)

            if self.normalize:
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
            inputs = self.processor(text=list(texts), return_tensors="pt", padding=True).to(
                self.device
            )
            text_features = self.model.get_text_features(**inputs)

            if self.normalize:
                text_features = F.normalize(text_features, dim=-1)

        # Create embeddings (keep as torch.Tensor on device)
        embeddings = []
        for feat in text_features:
            embeddings.append(CLIPEmbedding(vector=feat))

        return embeddings[0] if len(texts) == 1 else embeddings

    def warmup(self) -> None:
        """Warmup the model with a dummy forward pass."""
        # WORKAROUND: HuggingFace CLIP fails with CUBLAS_STATUS_ALLOC_FAILED when it's
        # the first model to use CUDA. Initialize CUDA context with a dummy operation.
        # This only needs to happen once per process.
        global _CUDA_INITIALIZED
        if self.device == "cuda" and not _CUDA_INITIALIZED:
            try:
                # Initialize CUDA with a small matmul operation to setup cuBLAS properly
                _ = torch.zeros(1, 1, device="cuda") @ torch.zeros(1, 1, device="cuda")
                torch.cuda.synchronize()
                _CUDA_INITIALIZED = True
            except Exception:
                # If initialization fails, continue anyway - the warmup might still work
                pass

        dummy_image = torch.randn(1, 3, 224, 224).to(self.device)
        dummy_text_inputs = self.processor(text=["warmup"], return_tensors="pt", padding=True).to(
            self.device
        )

        with torch.inference_mode():
            # Use pixel_values directly for image warmup
            self.model.get_image_features(pixel_values=dummy_image)
            self.model.get_text_features(**dummy_text_inputs)
