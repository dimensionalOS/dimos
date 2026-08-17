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
from typing import Any, overload

import torch
import torch.nn.functional as functional

from dimos.models.base import LocalModel
from dimos.models.embedding.base import (
    Embedding,
    EmbeddingModel,
    EmbeddingModelConfig,
    PatchEmbeddings,
)
from dimos.msgs.sensor_msgs.Image import Image


class DINOv3ModelConfig(EmbeddingModelConfig):
    #: timm model name. Weights on the HF hub under timm/ are ungated (Meta's
    #: facebook/dinov3-* originals require a license grant). Heaviest that fits
    #: the 8 GB laptop GPU is vit_huge_plus (840M); vit_7b needs ~13.4 GB fp16.
    model_name: str = "vit_huge_plus_patch16_dinov3"
    dtype: torch.dtype = torch.float16
    #: Inference resolution as (width, height); the input image is resized to
    #: this before embedding, and the patch grid is (height/16, width/16).
    #: Cost scales ~quadratically with token count, so this is the fps knob.
    #: vit_huge_plus at 848x480 (30x53 grid) runs 145ms/frame on the 5070
    #: laptop — the heaviest DINOv3 config that clears 5 fps.
    input_size: tuple[int, int] = (848, 480)


class DINOv3Model(EmbeddingModel, LocalModel):
    """DINOv3 dense-feature embedding model (vision only — no text tower).

    ``embed_patches`` returns a (grid_h, grid_w, dim) grid of patch features;
    ``embed`` returns the pooled CLS feature. Text queries need a separate
    text-aligned head (dino.txt); ``embed_text`` raises until we have one.
    """

    config: DINOv3ModelConfig

    @cached_property
    def _model(self) -> Any:
        import timm

        self._ensure_cuda_initialized()
        model = timm.create_model(
            self.config.model_name, pretrained=True, num_classes=0, dynamic_img_size=True
        )
        model = model.eval().to(device=self.config.device, dtype=self.config.dtype)

        from timm.data import resolve_model_data_config  # type: ignore[attr-defined]

        data_config = resolve_model_data_config(model)  # type: ignore[no-untyped-call]
        self._mean = torch.tensor(
            data_config["mean"], device=self.config.device, dtype=self.config.dtype
        ).view(1, 3, 1, 1)
        self._std = torch.tensor(
            data_config["std"], device=self.config.device, dtype=self.config.dtype
        ).view(1, 3, 1, 1)
        return model

    def _preprocess(self, *images: Image) -> torch.Tensor:
        """Stack images into a normalized (B, 3, H, W) batch at input_size."""
        width, height = self.config.input_size
        frames = [torch.from_numpy(img.to_rgb().data.copy()) for img in images]
        batch = torch.stack(frames).to(device=self.config.device, dtype=self.config.dtype)
        batch = batch.permute(0, 3, 1, 2) / 255.0
        if batch.shape[-2:] != (height, width):
            batch = functional.interpolate(
                batch, size=(height, width), mode="bilinear", align_corners=False
            )
        return (batch - self._mean) / self._std

    @overload
    def embed(self, image: Image, /) -> Embedding: ...
    @overload
    def embed(self, *images: Image) -> list[Embedding]: ...
    def embed(self, *images: Image) -> Embedding | list[Embedding]:
        """Embed one or more images into pooled image-level vectors."""
        model = self._model
        with torch.inference_mode():
            feats = model(self._preprocess(*images))
            if self.config.normalize:
                feats = functional.normalize(feats, dim=-1)

        embeddings = [
            Embedding(vector=feat, timestamp=images[i].ts) for i, feat in enumerate(feats)
        ]
        return embeddings[0] if len(images) == 1 else embeddings

    @overload
    def embed_patches(self, image: Image, /) -> PatchEmbeddings: ...
    @overload
    def embed_patches(self, *images: Image) -> list[PatchEmbeddings]: ...
    def embed_patches(self, *images: Image) -> PatchEmbeddings | list[PatchEmbeddings]:
        """Embed one or more images into per-patch feature grids."""
        model = self._model
        width, height = self.config.input_size
        grid_h, grid_w = (
            height // model.patch_embed.patch_size[0],
            width // model.patch_embed.patch_size[1],
        )

        with torch.inference_mode():
            tokens = model.forward_features(self._preprocess(*images))
            patches = tokens[:, model.num_prefix_tokens :]  # drop CLS + register tokens
            if self.config.normalize:
                patches = functional.normalize(patches, dim=-1)

        results: list[PatchEmbeddings] = []
        for i, image in enumerate(images):
            results.append(
                PatchEmbeddings(
                    vector=patches[i].reshape(grid_h, grid_w, -1),
                    frame_id=image.frame_id,
                    ts=image.ts,
                    source_width=image.width,
                    source_height=image.height,
                )
            )
        return results[0] if len(images) == 1 else results

    @overload
    def embed_text(self, text: str, /) -> Embedding: ...
    @overload
    def embed_text(self, *texts: str) -> list[Embedding]: ...
    def embed_text(self, *texts: str) -> Embedding | list[Embedding]:
        raise NotImplementedError(
            "DINOv3 has no text tower; text queries need a dino.txt-aligned head. "
            "Query DINOv3 maps by image-patch similarity or PCA instead."
        )

    def stop(self) -> None:
        """Release model and free GPU memory."""
        super().stop()
