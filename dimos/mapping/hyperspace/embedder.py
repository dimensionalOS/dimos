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

"""SigLIP2 with a text-aligned embedding per patch, not just per image.

SigLIP's raw patch tokens are not aligned with its text tower; only the
attention-pooled image vector is. Running every patch token through that
pooling head as its own length-1 sequence (the MaskCLIP trick) gives 576
vectors per 384x384 image that *are* text-aligned, and those are what
hyperspace stores and scores.
"""

from __future__ import annotations

from functools import cached_property
from typing import TYPE_CHECKING, Any, Literal

import numpy as np
import torch
from torch.nn import functional
from transformers import SiglipModel, SiglipTextModel, SiglipVisionModel

from dimos.models.embedding.siglip import SigLIPModel, SigLIPModelConfig

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.sensor_msgs.Image import Image

# Fixed-resolution SigLIP2: 384 / 16 = 24 patches per side = 576 patches, 1152-d.
SIGLIP2_MODEL_NAME = "google/siglip2-so400m-patch16-384"


class SigLIP2PatchesConfig(SigLIPModelConfig):
    model_name: str = SIGLIP2_MODEL_NAME
    # Which tower(s) to load. The full model is 4.3 GB in f32; a module that
    # only embeds images (the patch writer) or only text (the query side)
    # loads about half of that with "vision" or "text".
    towers: Literal["both", "vision", "text"] = "both"
    # Kept for callers that predate `towers`; same as towers="text".
    text_only: bool = False


class SigLIP2Patches(SigLIPModel):
    """SigLIP2 that also returns one text-aligned embedding per image patch."""

    config: SigLIP2PatchesConfig

    @property
    def towers(self) -> str:
        return "text" if self.config.text_only else self.config.towers

    @cached_property
    def _model(self) -> Any:  # type: ignore[override]
        self._ensure_cuda_initialized()
        loader = {"both": SiglipModel, "vision": SiglipVisionModel, "text": SiglipTextModel}[
            self.towers
        ]
        return loader.from_pretrained(self.config.model_name).eval().to(self.config.device)

    @cached_property
    def _vision(self) -> Any:
        """The vision transformer (with its attention-pooling head), whichever class holds it."""
        if self.towers == "text":
            raise RuntimeError("SigLIP2Patches loaded only the text tower; it cannot embed images")
        return self._model.vision_model

    @cached_property
    def _vision_config(self) -> Any:
        config = self._model.config
        return getattr(config, "vision_config", config)

    @cached_property
    def patches_per_side(self) -> int:
        vision = self._vision_config
        return int(vision.image_size // vision.patch_size)

    @cached_property
    def dim(self) -> int:
        return int(self._vision_config.hidden_size)

    def embed_patches(self, *images: Image) -> list[NDArray[np.float32]]:
        """Per-patch L2-normalized embeddings, one ``[patches, dim]`` array per image."""
        from PIL import Image as PILImage

        pil_images = [PILImage.fromarray(img.to_rgb().data) for img in images]
        with torch.inference_mode():
            inputs = self._processor(images=pil_images, return_tensors="pt").to(self.config.device)
            return self.patches_from_pixels(inputs["pixel_values"])

    def patches_from_pixels(self, pixel_values: torch.Tensor) -> list[NDArray[np.float32]]:
        """Same as :meth:`embed_patches` from an already preprocessed ``[B,3,S,S]`` tensor."""
        vision = self._vision
        with torch.inference_mode():
            pixel_values = pixel_values.to(
                self.config.device, dtype=next(vision.parameters()).dtype
            )
            hidden = vision(pixel_values=pixel_values).last_hidden_state
            batch, patches, dim = hidden.shape
            # MaskCLIP: each patch token pooled alone, so it lands in the text-aligned space.
            per_patch = vision.head(hidden.reshape(batch * patches, 1, dim)).reshape(
                batch, patches, dim
            )
            per_patch = functional.normalize(per_patch.float(), dim=-1).cpu().numpy()
        return [np.ascontiguousarray(grid, dtype=np.float32) for grid in per_patch]

    # SigLIP2 was trained on 64-token, max-length padded text; its tokenizer
    # snapshot does not always carry that length, so pass it.
    TEXT_LENGTH = 64

    def embed_text_array(self, *texts: str) -> NDArray[np.float32]:
        """L2-normalized text embeddings as a ``[len(texts), dim]`` array."""
        if self.towers == "vision":
            raise RuntimeError("SigLIP2Patches loaded only the vision tower; it cannot embed text")
        with torch.inference_mode():
            inputs = self._processor(
                text=list(texts),
                return_tensors="pt",
                padding="max_length",
                max_length=self.TEXT_LENGTH,
                truncation=True,
            ).to(self.config.device)
            if self.towers == "text":
                features = self._model(**inputs).pooler_output
            else:
                features = self._model.get_text_features(**inputs)
            features = functional.normalize(features.float(), dim=-1)
        return np.ascontiguousarray(features.cpu().numpy(), dtype=np.float32)
