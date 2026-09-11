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

Two families of checkpoint are handled. Fixed-resolution ones
(``siglip2-<size>-patch<p>-<px>``) squash the image to a square and give a
``side x side`` grid. NaFlex ones (``siglip2-<size>-patch16-naflex``) keep the
aspect ratio and give an ``h x w`` grid whose size depends on the image and on a
patch budget; ``<id>@<budget>`` names one with its budget, e.g.
``google/siglip2-base-patch16-naflex@576``.

:class:`PatchEnsemble` runs several checkpoints over the same frame. Each
one hallucinates in its own places (the floor next to a cone for one, a
reflection for another), so a per-cell *minimum* over their scores keeps
what they agree on and drops the rest (sweep of 2026-09-11, plan.md step 7).
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
# The ensemble the size sweep picked: two small checkpoints whose per-cell
# minimum beats the single So400m above on precision and recall at 2-5x the
# frame rate and 60% of the memory (sf_office, 302 keyframes, 4 queries).
DEFAULT_MEMBERS = ["google/siglip2-base-patch16-224", "google/siglip2-base-patch16-256"]


def parse_member(spec: str) -> tuple[str, int | None]:
    """``"google/siglip2-base-patch16-naflex@576"`` -> (id, 576); no ``@`` -> (id, None)."""
    name, _, budget = spec.partition("@")
    return name.strip(), int(budget) if budget else None


def member_tag(spec: str) -> str:
    """A short, file-safe name: ``google/siglip2-base-patch16-naflex@576`` ->
    ``base-patch16-naflex-576``. Stored with every keyframe so the query side
    knows which text towers to load."""
    name, budget = parse_member(spec)
    tag = name.rstrip("/").rsplit("/", 1)[-1].removeprefix("siglip2-")
    return f"{tag}-{budget}" if budget else tag


class SigLIP2PatchesConfig(SigLIPModelConfig):
    model_name: str = SIGLIP2_MODEL_NAME
    # Which tower(s) to load. The full model is 4.3 GB in f32; a module that
    # only embeds images (the patch writer) or only text (the query side)
    # loads about half of that with "vision" or "text".
    towers: Literal["both", "vision", "text"] = "both"
    # Kept for callers that predate `towers`; same as towers="text".
    text_only: bool = False
    # NaFlex checkpoints only: patches per image (the aspect ratio is kept, so
    # an 848x480 frame gets 18x32 = 576 of them). None = the checkpoint's own
    # default, 256. Ignored by fixed-resolution checkpoints.
    max_patches: int | None = None


class SigLIP2Patches(SigLIPModel):
    """SigLIP2 that also returns one text-aligned embedding per image patch."""

    config: SigLIP2PatchesConfig

    @property
    def towers(self) -> str:
        return "text" if self.config.text_only else self.config.towers

    @cached_property
    def naflex(self) -> bool:
        """NaFlex checkpoints are a different architecture (``model_type``
        "siglip2"): flattened patches through a linear embed, a position grid
        resized per image, and a patch budget instead of a fixed square."""
        from transformers import AutoConfig

        return AutoConfig.from_pretrained(self.config.model_name).model_type == "siglip2"

    @cached_property
    def _model(self) -> Any:  # type: ignore[override]
        self._ensure_cuda_initialized()
        if self.naflex:
            from transformers import Siglip2Model, Siglip2TextModel, Siglip2VisionModel

            loaders = {"both": Siglip2Model, "vision": Siglip2VisionModel, "text": Siglip2TextModel}
        else:
            loaders = {"both": SiglipModel, "vision": SiglipVisionModel, "text": SiglipTextModel}
        return (
            loaders[self.towers]
            .from_pretrained(self.config.model_name)
            .eval()
            .to(self.config.device)
        )

    @cached_property
    def _processor(self) -> Any:  # type: ignore[override]
        # AutoProcessor picks the Siglip2 (NaFlex) processor when the
        # checkpoint needs it; the base class hard-codes the SigLIP one.
        from transformers import AutoProcessor

        return AutoProcessor.from_pretrained(self.config.model_name)

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
        """Side of the square patch grid. Fixed-resolution checkpoints only;
        a NaFlex grid depends on the image, see :meth:`embed_grids`."""
        if self.naflex:
            raise RuntimeError(f"{self.config.model_name} is NaFlex: its grid depends on the image")
        vision = self._vision_config
        return int(vision.image_size // vision.patch_size)

    @cached_property
    def dim(self) -> int:
        return int(self._vision_config.hidden_size)

    def embed_patches(self, *images: Image) -> list[NDArray[np.float32]]:
        """Per-patch L2-normalized embeddings, one ``[patches, dim]`` array per image."""
        return [grid for grid, _ in self.embed_grids(*images)]

    def embed_grids(self, *images: Image) -> list[tuple[NDArray[np.float32], tuple[int, int]]]:
        """Per-patch embeddings with the grid they form: ``([rows*cols, dim],
        (rows, cols))`` per image, row-major."""
        from PIL import Image as PILImage

        pil_images = [PILImage.fromarray(img.to_rgb().data) for img in images]
        if not self.naflex:
            with torch.inference_mode():
                inputs = self._processor(images=pil_images, return_tensors="pt")
                grids = self.patches_from_pixels(inputs["pixel_values"])
            side = round(grids[0].shape[0] ** 0.5) if grids else 0
            return [(grid, (side, side)) for grid in grids]
        # NaFlex frames of one aspect ratio share a grid, but the processor
        # pads every image to the budget; one image per call keeps the
        # bookkeeping simple and the cost is the same.
        out = []
        for image in pil_images:
            extra = (
                {}
                if self.config.max_patches is None
                else {"max_num_patches": self.config.max_patches}
            )
            with torch.inference_mode():
                inputs = self._processor(images=[image], return_tensors="pt", **extra).to(
                    self.config.device
                )
                vision = self._vision
                hidden = vision(
                    pixel_values=inputs["pixel_values"].to(next(vision.parameters()).dtype),
                    attention_mask=inputs["pixel_attention_mask"],
                    spatial_shapes=inputs["spatial_shapes"],
                ).last_hidden_state
                rows, cols = (int(v) for v in inputs["spatial_shapes"][0])
                hidden = hidden[0, : rows * cols]
                per_patch = vision.head(hidden.reshape(rows * cols, 1, hidden.shape[-1]))
                per_patch = functional.normalize(per_patch.reshape(rows * cols, -1).float(), dim=-1)
            out.append(
                (np.ascontiguousarray(per_patch.cpu().numpy(), dtype=np.float32), (rows, cols))
            )
        return out

    def patches_from_pixels(self, pixel_values: torch.Tensor) -> list[NDArray[np.float32]]:
        """Same as :meth:`embed_patches` from an already preprocessed ``[B,3,S,S]``
        tensor (fixed-resolution checkpoints)."""
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


class PatchEnsemble:
    """Several :class:`SigLIP2Patches` over the same frames, one grid each.

    ``specs`` are checkpoint ids, NaFlex ones optionally with ``@<budget>``.
    The first member is the *primary*: its vectors also go into the store's
    vector index, so tools that only understand one grid per keyframe keep
    working. Scores are pooled per cell at query time (see
    ``HyperspaceQuery.pooled_hot_patches``); nothing here mixes embeddings.
    """

    def __init__(
        self, specs: list[str], device: str, towers: Literal["both", "vision", "text"] = "both"
    ) -> None:
        if not specs:
            raise ValueError("PatchEnsemble needs at least one checkpoint")
        self.specs = list(specs)
        self.tags = [member_tag(spec) for spec in specs]
        self.members = [
            SigLIP2Patches(model_name=name, max_patches=budget, device=device, towers=towers)
            for name, budget in map(parse_member, specs)
        ]

    @property
    def primary(self) -> SigLIP2Patches:
        return self.members[0]

    def start(self) -> None:
        for member in self.members:
            member.start()

    def stop(self) -> None:
        for member in self.members:
            member.stop()

    def embed_grids(self, image: Image) -> list[tuple[NDArray[np.float32], tuple[int, int]]]:
        """One ``(grid, (rows, cols))`` per member, in member order."""
        return [member.embed_grids(image)[0] for member in self.members]

    def embed_text_array(self, *texts: str) -> list[NDArray[np.float32]]:
        """One ``[len(texts), dim]`` array per member: each has its own text tower."""
        return [member.embed_text_array(*texts) for member in self.members]

    def embed_text(self, text: str) -> list[NDArray[np.float32]]:
        """The query-side callable: one vector per member."""
        return [vectors[0] for vectors in self.embed_text_array(text)]
