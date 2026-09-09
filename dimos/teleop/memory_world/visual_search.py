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

"""Text-to-place lookup over a recording's images, using SigLIP 2 patch embeddings.

"Where did I see a traffic cone" has to answer in under a couple of seconds, so
the expensive half is precomputed: every sampled camera frame is embedded once
into an index stream. At query time only the *text* is embedded.

The index keeps one embedding **per patch** (a 24x24 grid for a 384 px
checkpoint), not one pooled vector per frame. A pooled whole-frame vector is a
mean over everything in view, and a small object contributes almost nothing to
it: on an office recording the pooled score for "a traffic cone" (0.077) sat
below "a bicycle" (0.087) even though the cone was plainly in view, while the
best single patch scored 0.159 and landed on the cone. Each patch token is run
through the vision tower's attention-pooling head on its own (the MaskCLIP
trick), which is what puts it into the text-aligned space; raw patch tokens are
not comparable to text.

A frame's score is the maximum over its patches after subtracting the best
match to a fixed set of background prompts ("an office", "a wall", ...), which
removes the floor that every indoor frame shares. A background prompt that is
nearly a synonym of the query (text-text cosine above ``BACKGROUND_SYNONYM_CUTOFF``)
is dropped for that query, otherwise "furniture" would erase "a desk".

The index stream stores the source observation id as part of its payload
rather than a copy of the image, and the model name in its tags: the grid
shape and vector width are fixed by the checkpoint, so an index built with one
model cannot be searched with another.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np
import torch

from dimos.models.embedding.siglip import SigLIPModel
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

    from dimos.memory.store.sqlite import SqliteStore

logger = setup_logger()

# Highest-accuracy SigLIP 2 checkpoint with a text tower (85.0 zero-shot
# ImageNet, 1536-dim). 1.87B params = 3.74 GB in fp16, which fits an 8 GB GPU
# with room for activations.
SIGLIP2_MODEL_NAME = "google/siglip2-giant-opt-patch16-384"

BACKGROUND_PROMPTS = (
    "a photo",
    "an office",
    "a room",
    "an indoor scene",
    "a wall",
    "a floor",
    "a ceiling",
    "furniture",
)
BACKGROUND_SYNONYM_CUTOFF = 0.85
# Frames scored per matmul. The index stays fp16 in memory (5 fps of 848x480
# for four minutes is ~2 GB); each chunk is widened to fp32 for the product.
SCORE_CHUNK_FRAMES = 64


@dataclass(frozen=True)
class PatchGrid:
    """Per-patch, text-aligned embeddings of one camera frame (the index payload)."""

    source_id: int
    rows: int
    cols: int
    patches: np.ndarray  # (rows * cols, dims) float16, L2-normalised


@dataclass(frozen=True)
class Place:
    """One distinct location where the query was seen."""

    position: tuple[float, float, float]
    similarity: float
    source_id: int
    ts: float
    # Where in the matching frame the best patch sits, as fractions of width
    # and height, so a later step can raycast it into the map.
    image_uv: tuple[float, float] = (0.5, 0.5)


def cluster_places(
    candidates: Iterable[Place],
    radius: float,
    max_places: int,
) -> list[Place]:
    """Reduce ranked candidates to the best hit per distinct location.

    A robot pointed at one cone produces dozens of near-identical high-scoring
    frames. Keeping the strongest candidate and rejecting everything within
    *radius* of an already-accepted place turns that into the "handful of
    places" the user asked about. Candidates need not be pre-sorted.
    """
    if radius <= 0:
        raise ValueError(f"radius must be positive, got {radius}")
    if max_places <= 0:
        raise ValueError(f"max_places must be positive, got {max_places}")

    places: list[Place] = []
    for candidate in sorted(candidates, key=lambda p: p.similarity, reverse=True):
        if len(places) >= max_places:
            break
        far_from_all = all(
            sum((a - b) ** 2 for a, b in zip(candidate.position, place.position, strict=True))
            >= radius**2
            for place in places
        )
        if far_from_all:
            places.append(candidate)
    return places


def score_frames(
    patches: torch.Tensor,
    query: torch.Tensor,
    background: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Score every frame by its best background-contrasted patch.

    ``patches`` is (frames, patches, dims), ``query`` (dims,), ``background``
    (prompts, dims); all L2-normalised. Returns (frame score, index of the
    winning patch), each (frames,). With no background prompts the score is
    the plain max patch-text cosine.
    """
    scores: list[torch.Tensor] = []
    best: list[torch.Tensor] = []
    for start in range(0, patches.shape[0], SCORE_CHUNK_FRAMES):
        chunk = patches[start : start + SCORE_CHUNK_FRAMES].to(torch.float32)
        similarity = chunk @ query  # frames, patches
        if background.shape[0] > 0:
            similarity = similarity - (chunk @ background.T).amax(dim=-1)
        chunk_scores, chunk_best = similarity.max(dim=-1)
        scores.append(chunk_scores)
        best.append(chunk_best)
    return torch.cat(scores), torch.cat(best)


def posed_frames(images: Any, poses: Any | None, tolerance_s: float) -> Iterator[tuple[Any, Any]]:
    """Yield (image observation, pose) in time order, skipping frames with no pose.

    Some recordings stamp a pose on every image; others (the RealSense ones)
    carry poses only on odometry. With *poses* given, an image that has no pose
    of its own takes the nearest odometry pose within *tolerance_s*.
    """
    if poses is None:
        for obs in images.order_by("ts"):
            if obs.pose_tuple is not None:
                yield obs, obs.pose
        return
    for pair in images.order_by("ts").align(poses.order_by("ts"), tolerance=tolerance_s):
        image_obs, pose_obs = pair.data[0], pair.data[1]
        pose = image_obs.pose if image_obs.pose_tuple is not None else pose_obs.pose
        if pose is not None:
            yield image_obs, pose


def per_patch_embeddings(model: SigLIPModel, pixel_values: torch.Tensor) -> torch.Tensor:
    """Run each patch token through the attention-pooling head on its own.

    Returns (batch, patches, dims), L2-normalised. Pooling a length-1 sequence
    is the MaskCLIP trick: the head's cross-attention collapses to a projection
    of that single token, which lands it in the text-aligned space the pooled
    vector lives in.
    """
    vision = model._model.vision_model
    hidden = vision(pixel_values=pixel_values).last_hidden_state
    batch, n_patches, dims = hidden.shape
    pooled = vision.head(hidden.reshape(batch * n_patches, 1, dims))
    return torch.nn.functional.normalize(pooled.reshape(batch, n_patches, dims), dim=-1)


class VisualMemoryIndex:
    """A SigLIP 2 per-patch embedding index over one image stream of a recording."""

    def __init__(
        self,
        store: SqliteStore,
        image_stream_name: str = "color_image",
        index_stream_name: str = "image_siglip2_patches",
        pose_stream_name: str | None = None,
        pose_tolerance_s: float = 0.1,
        model_name: str = SIGLIP2_MODEL_NAME,
        device: str | None = None,
        dtype: torch.dtype = torch.float16,
    ) -> None:
        """*pose_stream_name* supplies poses for image streams that carry none
        (nearest observation within *pose_tolerance_s*); an image's own pose
        wins when it has one."""
        self.store = store
        self.image_stream_name = image_stream_name
        self.index_stream_name = index_stream_name
        self.pose_stream_name = pose_stream_name
        self.pose_tolerance_s = pose_tolerance_s
        self.model_name = model_name
        self._device = device
        self._dtype = dtype
        self._model: SigLIPModel | None = None
        self._index_stream: Any = None
        self._loaded: _LoadedIndex | None = None
        self._background: torch.Tensor | None = None

    @property
    def model(self) -> SigLIPModel:
        if self._model is None:
            settings: dict[str, Any] = {"model_name": self.model_name, "dtype": self._dtype}
            if self._device is not None:
                settings["device"] = self._device
            self._model = SigLIPModel(**settings)
            self._model.start()
            logger.info("loaded %s for visual memory search", self.model_name)
        return self._model

    @property
    def index_stream(self) -> Any:
        if self._index_stream is None:
            stream = self.store.stream(self.index_stream_name, PatchGrid)
            if stream.count() > 0:
                built_with = stream.first().tags.get("model")
                if built_with != self.model_name:
                    raise ValueError(
                        f"index stream {self.index_stream_name!r} was built with {built_with}, "
                        f"not {self.model_name}; rebuild it or pass model_name={built_with!r}"
                    )
            self._index_stream = stream
        return self._index_stream

    def count(self) -> int:
        """How many frames are already indexed."""
        return int(self.index_stream.count())

    def _posed_frames(self) -> Iterator[tuple[Any, Any]]:
        poses = None if self.pose_stream_name is None else self.store.streams[self.pose_stream_name]
        return posed_frames(
            self.store.streams[self.image_stream_name], poses, self.pose_tolerance_s
        )

    def build(self, stride: int = 1, batch_size: int = 8) -> int:
        """Embed every *stride*-th posed frame of the image stream into the index.

        Returns the number of frames added. Existing index rows are kept, so a
        rebuild after adding frames only costs the new ones.
        """
        if stride < 1:
            raise ValueError(f"stride must be at least 1, got {stride}")

        from PIL import Image as PILImage

        target = self.index_stream
        already_indexed = {obs.data.source_id for obs in target}
        wanted = (
            (obs, pose)
            for index, (obs, pose) in enumerate(self._posed_frames())
            if index % stride == 0 and int(obs.id) not in already_indexed
        )

        added = 0
        vision_config = self.model._model.config.vision_config
        grid_side = vision_config.image_size // vision_config.patch_size
        for batch in _batched(wanted, batch_size):
            pil_images = [PILImage.fromarray(obs.data.to_rgb().data) for obs, _ in batch]
            with torch.inference_mode():
                inputs = self.model._move_inputs_to_device(
                    dict(self.model._processor(images=pil_images, return_tensors="pt"))
                )
                embeddings = per_patch_embeddings(self.model, inputs["pixel_values"])
            for (obs, pose), patches in zip(batch, embeddings, strict=True):
                target.append(
                    PatchGrid(
                        source_id=int(obs.id),
                        rows=grid_side,
                        cols=grid_side,
                        patches=patches.to(torch.float16).cpu().numpy(),
                    ),
                    ts=obs.ts,
                    pose=pose,
                    tags={"model": self.model_name},
                )
                added += 1
            logger.info("indexed %d frames of %s", added, self.image_stream_name)
        self._loaded = None
        return added

    def _load(self) -> _LoadedIndex:
        """Pull the whole index into memory once, as stored (fp16)."""
        if self._loaded is None:
            observations = [obs for obs in self.index_stream if obs.pose_tuple is not None]
            if not observations:
                raise LookupError(f"index stream {self.index_stream_name!r} is empty")
            grid = observations[0].data
            self._loaded = _LoadedIndex(
                patches=torch.from_numpy(np.stack([obs.data.patches for obs in observations])),
                rows=grid.rows,
                cols=grid.cols,
                source_ids=[obs.data.source_id for obs in observations],
                timestamps=[float(obs.ts) for obs in observations],
                positions=[
                    (float(obs.pose_tuple[0]), float(obs.pose_tuple[1]), float(obs.pose_tuple[2]))
                    for obs in observations
                ],
            )
        return self._loaded

    def _embed(self, text: str) -> torch.Tensor:
        return self.model.embed_text(text).to_torch("cpu").to(torch.float32)

    def _background_for(self, query: torch.Tensor) -> torch.Tensor:
        if self._background is None:
            self._background = torch.stack([self._embed(prompt) for prompt in BACKGROUND_PROMPTS])
        keep = (self._background @ query) < BACKGROUND_SYNONYM_CUTOFF
        return self._background[keep]

    def search(self, text: str, k: int = 200) -> list[Place]:
        """Rank indexed frames by similarity to *text*, most similar first."""
        loaded = self._load()
        query = self._embed(text)
        scores, best_patch = score_frames(loaded.patches, query, self._background_for(query))
        top = torch.topk(scores, k=min(k, scores.shape[0]))
        return [
            Place(
                position=loaded.positions[frame],
                similarity=float(score),
                source_id=loaded.source_ids[frame],
                ts=loaded.timestamps[frame],
                image_uv=(
                    (int(best_patch[frame]) % loaded.cols + 0.5) / loaded.cols,
                    (int(best_patch[frame]) // loaded.cols + 0.5) / loaded.rows,
                ),
            )
            for score, frame in zip(top.values.tolist(), top.indices.tolist(), strict=True)
        ]

    def stop(self) -> None:
        if self._model is not None:
            self._model.stop()
            self._model = None
        self._loaded = None
        self._background = None


@dataclass(frozen=True)
class _LoadedIndex:
    patches: torch.Tensor  # frames, patches, dims (float16, as stored)
    rows: int
    cols: int
    source_ids: list[int]
    timestamps: list[float]
    positions: list[tuple[float, float, float]]


def _batched(iterator: Iterable[Any], size: int) -> Iterator[list[Any]]:
    batch: list[Any] = []
    for item in iterator:
        batch.append(item)
        if len(batch) == size:
            yield batch
            batch = []
    if batch:
        yield batch


def main() -> None:
    """Build the index for a recording: ``python -m ...visual_search <db>``."""
    import argparse

    from dimos.memory.store.sqlite import SqliteStore

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("store_path")
    parser.add_argument("--image-stream", default="color_image")
    parser.add_argument("--index-stream", default="image_siglip2_patches")
    parser.add_argument(
        "--pose-stream", default=None, help="odometry stream for images that carry no pose"
    )
    parser.add_argument("--model", default=SIGLIP2_MODEL_NAME)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--device", default=None)
    parser.add_argument("--search", default=None, help="run this query after building")
    args = parser.parse_args()

    store = SqliteStore(path=args.store_path, must_exist=True)
    store.start()
    index = VisualMemoryIndex(
        store,
        image_stream_name=args.image_stream,
        index_stream_name=args.index_stream,
        pose_stream_name=args.pose_stream,
        model_name=args.model,
        device=args.device,
    )
    try:
        added = index.build(stride=args.stride, batch_size=args.batch_size)
        print(f"added {added} frames; index now holds {index.count()}")
        if args.search:
            for place in cluster_places(index.search(args.search), radius=2.5, max_places=6):
                print(
                    f"  {place.similarity:+.4f}  {place.position}  id={place.source_id}"
                    f"  uv={place.image_uv[0]:.2f},{place.image_uv[1]:.2f}"
                )
    finally:
        index.stop()
        store.stop()


if __name__ == "__main__":
    # Under ``python -m`` this file runs as ``__main__``, and the store records
    # payload classes by module path, so an index built here would be typed
    # ``__main__.PatchGrid`` and unreadable everywhere else. Run the properly
    # imported module instead.
    from dimos.teleop.memory_world.visual_search import main as installed_main

    installed_main()
