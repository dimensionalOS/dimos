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

"""Meta's Perception Encoder, with a text-aligned embedding per patch.

The same shape as :mod:`siglip_embedder`, for a different family of weights, so a
`PatchEnsemble` can hold one of each and the rest of hyperspace cannot tell the
difference. Name one with a ``pe:`` prefix -- ``pe:PE-Core-B-16``.

WHY IT LOOKS LIKE THE SigLIP ONE. PE-Core is a CLIP: an image tower and a text tower
in one embedding space, so a query is a dot product exactly as before. And like
SigLIP, its raw patch tokens are NOT in that space -- only the attention-pooled image
vector is. Running each patch token through the pooling head as its own length-one
sequence (the MaskCLIP trick) gives one text-aligned vector per patch, which is what
hyperspace stores and scores. The plumbing differs because the checkpoint is a timm
`Eva` behind open_clip rather than a transformers `SiglipModel`; the idea does not.

WHERE THE WEIGHTS COME FROM. open_clip ships PE-Core with Meta's own weights
(``PE-Core-T-16-384``, ``-S-16-384``, ``-B-16``, ``-L-14-336``, ``-bigG-14-448``), so
nothing here needs the ``perception_models`` repository that PE's own README reaches
for. That matters: a checkpoint that costs a git clone and a second package is a
checkpoint nobody swaps in to try.

PE-Spatial is NOT used. It is the dense tower and its patch tokens are supervised to
be local, which is the exact failing this whole line of work is about -- but its text
tower lives in PE-Core, and two towers from two checkpoints are not one embedding
space until someone shows that they are. PE-Core answers text on its own, so that is
what is wired up.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
from typing import TYPE_CHECKING, Any, Literal

import numpy as np
import torch

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.sensor_msgs.Image import Image

logger = setup_logger()

PREFIX = "pe:"
# How a patch token is put into the text's space.
#
# POOLED is the MaskCLIP route: the token goes through the attention-pooling head on
# its own, because that head is what the image vector passes through and therefore
# what the text was trained against. It is a trick -- the head was built to summarise
# a whole sequence and it is being handed one token -- so it is worth knowing what the
# obvious alternative does.
#
# DIRECT skips the pooling head and projects the token straight through the output
# layer. Cheaper, and honest about what a token is, but the token never went through
# the operation the text is aligned to.
POOLED = "pooled"
DIRECT = "direct"
# What open_clip calls the weights. Every PE-Core checkpoint is published by Meta
# under this one tag.
PRETRAINED = "meta"


def is_pe(spec: str) -> bool:
    """True for a member spec naming a Perception Encoder checkpoint."""
    return spec.strip().lower().startswith(PREFIX)


def pe_name(spec: str) -> str:
    """``pe:PE-Core-B-16`` -> ``PE-Core-B-16``, dropping any pooling suffix."""
    return parse_pe(spec)[0]


def parse_pe(spec: str) -> tuple[str, str]:
    """``pe:PE-Core-B-16@direct`` -> ``("PE-Core-B-16", "direct")``.

    The suffix names how a patch token is put into the text's space. Without one it
    is `POOLED`, which is the MaskCLIP route and the default.
    """
    body = spec.strip()[len(PREFIX) :].strip()
    name, _, how = body.partition("@")
    how = how.strip().lower() or POOLED
    if how not in (POOLED, DIRECT):
        raise ValueError(f"{spec!r}: pooling must be {POOLED!r} or {DIRECT!r}, not {how!r}")
    return name.strip(), how


@dataclass
class PEPatchesConfig:
    model_name: str = "PE-Core-B-16"
    device: str = "cpu"
    # Which tower to load. The query side needs only the text tower and the ingest
    # only the vision one; loading both doubles the memory for nothing.
    towers: Literal["both", "vision", "text"] = "both"
    pretrained: str = PRETRAINED
    # POOLED (MaskCLIP) or DIRECT. See the note on the constants.
    pooling: str = POOLED


class PEPatches:
    """PE-Core that also returns one text-aligned embedding per image patch."""

    def __init__(self, **kwargs: Any) -> None:
        self.config = PEPatchesConfig(**kwargs)
        self._model: Any = None

    @property
    def towers(self) -> str:
        return self.config.towers

    def start(self) -> None:
        import open_clip

        if self._model is not None:
            return
        self._model = open_clip.create_model(
            self.config.model_name, pretrained=self.config.pretrained
        )
        self._model.eval().to(self.config.device)
        self._tokenizer = open_clip.get_tokenizer(self.config.model_name)
        logger.info(
            f"hyperspace: {self.config.model_name} ({self.config.towers}, "
            f"{self.config.pooling}) on {self.config.device}, "
            f"{self.grid[0]}x{self.grid[1]} patches, {self.dim}-d"
        )

    def stop(self) -> None:
        self._model = None

    @property
    def model(self) -> Any:
        if self._model is None:
            self.start()
        return self._model

    @cached_property
    def trunk(self) -> Any:
        """The timm ViT under open_clip's wrapper, which is where the tokens are."""
        visual = self.model.visual
        trunk = getattr(visual, "trunk", None)
        if trunk is None or not hasattr(trunk, "attn_pool"):
            raise RuntimeError(
                f"{self.config.model_name} is not the attention-pooled PE tower this "
                "expects; per-patch embeddings would not be text-aligned"
            )
        return trunk

    @cached_property
    def side(self) -> tuple[int, int]:
        """Pixels the checkpoint takes, (height, width)."""
        size = self.model.visual.image_size
        return (int(size[0]), int(size[1])) if isinstance(size, tuple | list) else (size, size)

    @cached_property
    def grid(self) -> tuple[int, int]:
        """Patches down and across."""
        patch = self.trunk.patch_embed.patch_size
        patch = patch[0] if isinstance(patch, tuple | list) else patch
        return (self.side[0] // int(patch), self.side[1] // int(patch))

    @cached_property
    def dim(self) -> int:
        """Width of the shared image/text space."""
        return int(self.trunk.head.out_features)

    @cached_property
    def _preprocess(self) -> Any:
        from torchvision import transforms

        return transforms.Compose(
            [
                transforms.Resize(self.side, antialias=True),
                transforms.ToTensor(),
                transforms.Normalize(
                    mean=self.model.visual.image_mean, std=self.model.visual.image_std
                ),
            ]
        )

    def embed_patches(self, *images: Image) -> list[NDArray[np.float32]]:
        """Per-patch L2-normalized embeddings, one ``[patches, dim]`` array per image."""
        return [grid for grid, _ in self.embed_grids(*images)]

    def embed_grids(self, *images: Image) -> list[tuple[NDArray[np.float32], tuple[int, int]]]:
        """Per-patch embeddings with the grid they form, row-major."""
        from PIL import Image as PILImage

        if self.towers == "text":
            raise RuntimeError(f"{self.config.model_name} loaded only its text tower")
        pictures = [PILImage.fromarray(image.to_rgb().data) for image in images]
        batch = torch.stack([self._preprocess(picture) for picture in pictures]).to(
            self.config.device
        )
        with torch.inference_mode():
            tokens = self.trunk.forward_features(batch)
            # The prefix tokens are the class token and friends. They are a summary of
            # the whole picture, not a place in it, and letting one through would put a
            # patch's worth of score on a cell that does not exist.
            prefix = int(getattr(self.trunk, "num_prefix_tokens", 1))
            patches = tokens[:, prefix:, :]
            count, length, width = patches.shape
            rows, cols = self.grid
            if length != rows * cols:
                raise RuntimeError(
                    f"{self.config.model_name} gave {length} patch tokens for a "
                    f"{rows}x{cols} grid; the grid would be misread"
                )
            # Each patch through the pooling head on its own: what the head does to the
            # whole sequence is what puts a vector in the text's space, so a patch that
            # skips it is not comparable to a word.
            if self.config.pooling == POOLED:
                alone = patches.reshape(count * length, 1, width)
                vectors = self.trunk.attn_pool(alone)
                if vectors.ndim == 3:
                    vectors = vectors[:, 0, :]
            else:
                vectors = patches.reshape(count * length, width)
            vectors = self.trunk.head(self.trunk.fc_norm(vectors))
            vectors = torch.nn.functional.normalize(vectors, dim=-1)
            grids = vectors.reshape(count, length, -1).float().cpu().numpy()
        return [(grid.astype(np.float32), (rows, cols)) for grid in grids]

    def embed_text_array(self, *texts: str) -> NDArray[np.float32]:
        """L2-normalized text embeddings as a ``[len(texts), dim]`` array."""
        if self.towers == "vision":
            raise RuntimeError(f"{self.config.model_name} loaded only its vision tower")
        with torch.inference_mode():
            tokens = self._tokenizer(list(texts)).to(self.config.device)
            features = self.model.encode_text(tokens)
            features = torch.nn.functional.normalize(features, dim=-1)
        return features.float().cpu().numpy().astype(np.float32)

    def embed_text(self, text: str) -> NDArray[np.float32]:
        return self.embed_text_array(text)[0]
