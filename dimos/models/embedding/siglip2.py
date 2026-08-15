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
import math
from typing import Any, cast, overload

from PIL import Image as PILImage
import torch
import torch.nn.functional as functional
from transformers import AutoModel, AutoProcessor

from dimos.models.base import HuggingFaceModel
from dimos.models.embedding.base import (
    Embedding,
    EmbeddingModel,
    HuggingFaceEmbeddingModelConfig,
    PatchEmbeddings,
)
from dimos.msgs.sensor_msgs.Image import Image


class SigLIP2ModelConfig(HuggingFaceEmbeddingModelConfig):
    model_name: str = "google/siglip2-base-patch16-256"
    #: fp16 halves VRAM (so400m: 4.6 -> 2.3 GB) at no visible quality cost.
    #: Pass float32 explicitly for CPU inference — fp16 has no fast CPU path.
    dtype: torch.dtype = torch.float16
    #: SigLIP text towers are trained on fixed-length padded input (64 tokens).
    text_max_length: int = 64
    #: Run each patch token through the vision attention-pooling head so patch
    #: embeddings live in the text-aligned space. Raw vision-tower tokens are
    #: NOT aligned with text embeddings — cosine against them is noise.
    pooled_patches: bool = True
    #: Token budget per image for NaFlex checkpoints (e.g. 64 -> ~8x8 grid).
    #: Fewer tokens = quadratically cheaper vision tower at coarser spatial
    #: resolution. Ignored by fixed-resolution checkpoints.
    max_num_patches: int | None = None


class SigLIP2Model(EmbeddingModel, HuggingFaceModel):
    """SigLIP 2 vision-language embedding model.

    Supports pooled image/text embeddings (``embed`` / ``embed_text``) and
    per-patch embedding grids (``embed_patches``). Works with both the
    fixed-resolution SigLIP 2 checkpoints and the NaFlex variants
    (``google/siglip2-*-naflex``), whose patch grid follows the input
    aspect ratio.
    """

    config: SigLIP2ModelConfig
    _model_class = AutoModel

    @cached_property
    def _model(self) -> Any:
        self._ensure_cuda_initialized()
        return super()._model.eval()

    @cached_property
    def _processor(self) -> Any:
        return AutoProcessor.from_pretrained(self.config.model_name, use_fast=True)

    @overload
    def embed(self, image: Image, /) -> Embedding: ...
    @overload
    def embed(self, *images: Image) -> list[Embedding]: ...
    def embed(self, *images: Image) -> Embedding | list[Embedding]:
        """Embed one or more images into pooled image-level vectors."""
        pil_images = [PILImage.fromarray(img.to_rgb().data) for img in images]

        with torch.inference_mode():
            inputs = self._processor(images=pil_images, return_tensors="pt")
            inputs = self._move_inputs_to_device(dict(inputs))
            image_features = self._model.get_image_features(**inputs)

            if self.config.normalize:
                image_features = functional.normalize(image_features, dim=-1)

        embeddings: list[Embedding] = []
        for i, feat in enumerate(image_features):
            embeddings.append(Embedding(vector=feat, timestamp=images[i].ts))

        return embeddings[0] if len(images) == 1 else embeddings

    @overload
    def embed_patches(self, image: Image, /) -> PatchEmbeddings: ...
    @overload
    def embed_patches(self, *images: Image) -> list[PatchEmbeddings]: ...
    def embed_patches(self, *images: Image) -> PatchEmbeddings | list[PatchEmbeddings]:
        """Embed one or more images into per-patch embedding grids.

        Returns one (grid_h, grid_w, dim) grid per image from the vision
        tower's final hidden state. For fixed-resolution checkpoints the grid
        is square (e.g. 16x16 for patch16-256); for NaFlex checkpoints it
        follows the processor's spatial_shapes.
        """
        pil_images = [PILImage.fromarray(img.to_rgb().data) for img in images]

        with torch.inference_mode():
            processor_kwargs: dict[str, Any] = {}
            if self.config.max_num_patches is not None:
                processor_kwargs["max_num_patches"] = self.config.max_num_patches
            inputs = self._processor(images=pil_images, return_tensors="pt", **processor_kwargs)
            # NaFlex processors emit per-image patch grid shapes; fixed-res ones don't.
            spatial_shapes = inputs.get("spatial_shapes")
            inputs = self._move_inputs_to_device(dict(inputs))
            # The NaFlex processor names the mask pixel_attention_mask; the
            # vision tower's forward takes it as attention_mask.
            if "pixel_attention_mask" in inputs:
                inputs["attention_mask"] = inputs.pop("pixel_attention_mask")
            vision_outputs = self._model.vision_model(**inputs)
            hidden = vision_outputs.last_hidden_state

            if self.config.pooled_patches:
                hidden = self._pool_patch_tokens(hidden)
            if self.config.normalize:
                hidden = functional.normalize(hidden, dim=-1)

        results: list[PatchEmbeddings] = []
        for i, image in enumerate(images):
            patches = hidden[i]
            if spatial_shapes is not None:
                grid_h, grid_w = (int(v) for v in spatial_shapes[i])
                patches = patches[: grid_h * grid_w]  # drop NaFlex padding
            else:
                side = math.isqrt(patches.shape[0])
                if side * side != patches.shape[0]:
                    raise ValueError(
                        f"Cannot infer square patch grid from {patches.shape[0]} patches "
                        f"for model {self.config.model_name}"
                    )
                grid_h, grid_w = side, side
            results.append(
                PatchEmbeddings(
                    vector=patches.reshape(grid_h, grid_w, -1),
                    frame_id=image.frame_id,
                    ts=image.ts,
                    source_width=image.width,
                    source_height=image.height,
                )
            )

        return results[0] if len(images) == 1 else results

    def _pool_patch_tokens(self, hidden: torch.Tensor) -> torch.Tensor:
        """Map (B, L, D) vision tokens into the text-aligned pooled space.

        Applies the model's attention-pooling head to every token as its own
        length-one sequence (attention over one token reduces to its value
        projection), mirroring the head's probe -> attention -> residual MLP
        computation. This is the MaskCLIP trick: it preserves per-patch
        locality while landing in the space text embeddings are trained
        against.
        """
        head = self._model.vision_model.head
        batch, length, dim = hidden.shape
        tokens = hidden.reshape(batch * length, 1, dim)
        probe = head.probe.repeat(tokens.shape[0], 1, 1)
        attn = head.attention(probe, tokens, tokens)[0]
        pooled = attn + head.mlp(head.layernorm(attn))
        return cast("torch.Tensor", pooled[:, 0].reshape(batch, length, dim))

    @overload
    def embed_text(self, text: str, /) -> Embedding: ...
    @overload
    def embed_text(self, *texts: str) -> list[Embedding]: ...
    def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
        """Embed one or more text strings."""
        with torch.inference_mode():
            inputs = self._processor(
                text=list(texts),
                return_tensors="pt",
                padding="max_length",
                max_length=self.config.text_max_length,
                truncation=True,
            )
            inputs = self._move_inputs_to_device(dict(inputs))
            text_features = self._model.get_text_features(**inputs)

            if self.config.normalize:
                text_features = functional.normalize(text_features, dim=-1)

        embeddings: list[Embedding] = []
        for feat in text_features:
            embeddings.append(Embedding(vector=feat))

        return embeddings[0] if len(texts) == 1 else embeddings

    def stop(self) -> None:
        """Release model and free GPU memory."""
        if "_processor" in self.__dict__:
            del self.__dict__["_processor"]
        super().stop()
