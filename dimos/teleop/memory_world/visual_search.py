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

A recording that siglipify has already embedded (a
``<image stream>_<model>`` stream of ``std_msgs/msg/Float32MultiArray`` rows) is
searched from those vectors instead of building anything. siglipify applies
the same per-patch head trick and marks the stream ``text_aligned``; a stream
without the mark holds raw tower tokens and is refused rather than searched.

The index stream stores the source observation id as part of its payload
rather than a copy of the image, and the model name, image stream and world
frame in its tags: the grid
shape and vector width are fixed by the checkpoint, so an index built with one
model cannot be searched with another. Each row's pose is the camera's
*optical* frame in the world (z forward, x right, y down), supplied by the
caller's ``pose_of`` (the recording's tf tree); rows are tagged with that
convention and an index built any other way is refused.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any

import numpy as np
import torch

from dimos.models.embedding.siglip import SigLIPModel
from dimos.teleop.memory_world.recording import (
    StoredEmbeddings,
    embedding_stream_name,
    grid_side,
)
from dimos.teleop.memory_world.tf_tree import quaternion_from_matrix
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable, Iterator

    from dimos.memory.store.base import Store

# How index rows say which frame their pose describes.
POSE_FRAME_TAG = "camera_optical"


def pose_tag_for(tree: Any) -> str:
    """The pose convention tag for poses placed through *tree*.

    Index rows store poses as computed and are never re-placed on read, so the tag
    has to name the extrinsic that made them -- and name WHICH one, since replacing
    one measurement with another moves every pose just as much as the first did.
    """
    mount = getattr(tree, "mount_fingerprint", None) if tree is not None else None
    return POSE_FRAME_TAG + (f"+mount_{mount}" if mount else "")


logger = setup_logger()

# Highest-accuracy SigLIP 2 checkpoint with a text tower (85.0 zero-shot
# ImageNet, 1536-dim). 1.87B params = 3.74 GB in fp16, which fits an 8 GB GPU
# with room for activations.
SIGLIP2_MODEL_NAME = "google/siglip2-giant-opt-patch16-384"


def model_slug(model_name: str) -> str:
    """``google/siglip2-giant-opt-patch16-384`` -> ``siglip2_giant_opt_p16_384``.

    The same abbreviation siglipify uses, so its streams and this module's are
    named alike and found by the model that made them.
    """
    tail = model_name.rsplit("/", 1)[-1].replace("patch", "p")
    return re.sub(r"[^a-z0-9]+", "_", tail.lower()).strip("_")


def index_stream_name_of(model_name: str, image_stream_name: str) -> str:
    """Name the index stream after the images and the model that built it.

    Two models' embeddings are not comparable, and two cameras' frames are not
    the same evidence, so neither pair may share a stream: ``color_image`` with
    ``google/siglip2-giant-opt-patch16-384`` -> ``color_image_index_siglip2_giant_opt_p16_384``
    (siglipify's own vectors are ``color_image_siglip2_giant_opt_p16_384``: a
    different format, so a different name).
    """
    return f"{image_stream_name}_index_{model_slug(model_name)}"


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
# A patch is "hot" when it scores at least this much and at least this fraction
# of its frame's best patch. The scores are background-contrasted (patch cosine
# minus its best background cosine; raw aligned cosines peak around 0.10-0.17),
# so the floor is loose and the ratio does most of the work.
HOT_PATCH_FLOOR = 0.10
HOT_PATCH_RATIO = 0.75
# Frames scored per matmul. The index stays fp16 in memory (5 fps of 848x480
# for four minutes is ~2 GB); each chunk is widened to fp32 for the product.
SCORE_CHUNK_FRAMES = 64
# An mcap embedding names its frame only by stamp; this is how close it must be.
STAMP_MATCH_TOLERANCE_S = 1e-3


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
    # Pose the frame was captured from: (qx, qy, qz, qw), and the position when
    # it differs from ``position`` (a located object keeps its camera here).
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    camera_position: tuple[float, float, float] | None = None
    # Distinct viewing directions that saw this place (1 for a single frame).
    views: int = 1


@dataclass(frozen=True)
class FramePatches:
    """One indexed frame's patch scores for a query, with where it was taken from."""

    source_id: int
    ts: float
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    similarity: torch.Tensor  # (patches,)
    rows: int
    cols: int


_QUESTION_PREFIXES = (
    "where did i see",
    "where did you see",
    "where have i seen",
    "where have you seen",
    "where is",
    "where are",
    "where was",
    "where were",
    "have you seen",
    "did you see",
    "did i see",
    "show me",
    "find",
)


def search_phrase(spoken: str) -> str:
    """Reduce a spoken question to the thing being asked about.

    "Where did I see a traffic cone?" scores fine as-is, but the answer text
    and the markers should read "a traffic cone", not the whole question.
    """
    phrase = " ".join(spoken.split()).strip(" ?.!,")[:400]  # MemoryQueryResult.query_text's cap
    lowered = phrase.lower()
    for prefix in _QUESTION_PREFIXES:
        if lowered.startswith(prefix + " "):
            return phrase[len(prefix) + 1 :]
    return phrase


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


def patch_similarity(
    patches: torch.Tensor,
    query: torch.Tensor,
    background: torch.Tensor,
) -> torch.Tensor:
    """Background-contrasted patch-text cosine for every patch of every frame.

    ``patches`` is (frames, patches, dims), ``query`` (dims,), ``background``
    (prompts, dims); all L2-normalised. Returns (frames, patches). With no
    background prompts this is the plain cosine.
    """
    chunks: list[torch.Tensor] = []
    for start in range(0, patches.shape[0], SCORE_CHUNK_FRAMES):
        chunk = patches[start : start + SCORE_CHUNK_FRAMES].to(torch.float32)
        similarity = chunk @ query  # frames, patches
        if background.shape[0] > 0:
            similarity = similarity - (chunk @ background.T).amax(dim=-1)
        chunks.append(similarity)
    return torch.cat(chunks)


def score_frames(
    patches: torch.Tensor,
    query: torch.Tensor,
    background: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Score every frame by its best patch. Returns (score, winning patch), each (frames,)."""
    return patch_similarity(patches, query, background).max(dim=-1)


def hot_patches(
    similarity: torch.Tensor,
    rows: int,
    cols: int,
    floor: float = HOT_PATCH_FLOOR,
    ratio: float = HOT_PATCH_RATIO,
) -> list[tuple[tuple[float, float], float]]:
    """The patches of one frame worth raycasting: (image_uv, score) pairs.

    A frame's single best patch is often a stray (a picture frame scoring a
    hair above the cone next to it), so every patch within *ratio* of the
    frame's maximum and above the absolute *floor* is kept. The object itself
    spans several patches; the stray does not.
    """
    best = float(similarity.max())
    threshold = max(floor, best * ratio)
    return [
        ((((index % cols) + 0.5) / cols, ((index // cols) + 0.5) / rows), float(score))
        for index, score in enumerate(similarity.tolist())
        if score >= threshold
    ]


@dataclass(frozen=True)
class PatchHit:
    """One hot patch of one frame, raycast into the world."""

    position: tuple[float, float, float]
    similarity: float
    source_id: int
    ts: float
    camera_position: tuple[float, float, float]
    # No default: a hit that forgets its camera's orientation would hang the
    # answer frame facing straight up.
    camera_orientation: tuple[float, float, float, float]


def cluster_hits(hits: Iterable[PatchHit], radius: float, max_places: int) -> list[Place]:
    """Group raycast patch hits into objects, ranked by how many directions saw them.

    Hits within *radius* of a cluster's running centroid join it. A cluster's
    rank is the number of distinct viewing bearings (45 degree bins) it was
    seen from, then its best similarity: a real object is hot from several
    directions, a look-alike patch in one frame is not. Consecutive frames
    from the same spot share a bearing and count once.
    """
    if radius <= 0:
        raise ValueError(f"radius must be positive, got {radius}")
    if max_places <= 0:
        raise ValueError(f"max_places must be positive, got {max_places}")

    clusters: list[dict[str, Any]] = []
    for hit in sorted(hits, key=lambda h: h.similarity, reverse=True):
        point = np.asarray(hit.position)
        nearest = min(
            (c for c in clusters if np.linalg.norm(c["centroid"] - point) <= radius),
            key=lambda c: float(np.linalg.norm(c["centroid"] - point)),
            default=None,
        )
        if nearest is None:
            nearest = {"centroid": point.copy(), "weight": 0.0, "bearings": set(), "best": hit}
            clusters.append(nearest)
        weight = max(hit.similarity, 1e-6)
        nearest["centroid"] = (nearest["centroid"] * nearest["weight"] + point * weight) / (
            nearest["weight"] + weight
        )
        nearest["weight"] += weight
        dx, dy = point[0] - hit.camera_position[0], point[1] - hit.camera_position[1]
        nearest["bearings"].add(int(np.degrees(np.arctan2(dy, dx)) // 45))

    ranked = sorted(
        clusters, key=lambda c: (len(c["bearings"]), c["best"].similarity), reverse=True
    )
    return [
        Place(
            position=tuple(float(v) for v in c["centroid"]),  # type: ignore[arg-type]
            similarity=c["best"].similarity,
            source_id=c["best"].source_id,
            ts=c["best"].ts,
            orientation=c["best"].camera_orientation,
            camera_position=c["best"].camera_position,
            views=len(c["bearings"]),
        )
        for c in ranked[:max_places]
    ]


def body_style_quaternion(optical: np.ndarray) -> tuple[float, float, float, float]:
    """Quaternion of a frame with x along the optical axis and z the image's up.

    The viewer orients capture markers assuming a robot body (x forward, z
    up); a camera's optical frame is z forward and y down.
    """
    forward = optical[:3, 2] / np.linalg.norm(optical[:3, 2])
    up = -optical[:3, 1] / np.linalg.norm(optical[:3, 1])
    left = np.cross(up, forward)
    return quaternion_from_matrix(np.stack([forward, left, up], axis=1))


def patch_world_position(
    image_uv: tuple[float, float],
    depth_mm: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    camera_to_world: np.ndarray,
    window_px: int = 16,
) -> tuple[float, float, float] | None:
    """Back-project the centre of the winning patch through the depth image.

    Takes the median valid depth in a *window_px* square around the patch
    centre (zeros are holes), lifts it with the pinhole model in the optical
    frame (x right, y down, z forward), and moves it into the world with
    *camera_to_world*. None when the window holds no valid depth.
    """
    if depth_mm.dtype.kind == "f":  # 32FC1 depth is metres
        depth_mm = depth_mm * 1000.0
    height, width = depth_mm.shape
    u = round(image_uv[0] * width)
    v = round(image_uv[1] * height)
    half = window_px // 2
    window = depth_mm[max(v - half, 0) : v + half, max(u - half, 0) : u + half]
    valid = window[np.isfinite(window) & (window > 0)]
    if valid.size == 0:
        return None
    depth_m = float(np.median(valid)) / 1000.0
    fx, fy, cx, cy = intrinsics
    optical = np.array([(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m, 1.0])
    world = camera_to_world @ optical
    return (float(world[0]), float(world[1]), float(world[2]))


def align_patch_tokens(model: SigLIPModel, tokens: torch.Tensor) -> torch.Tensor:
    """Run each patch token through the attention-pooling head on its own.

    *tokens* is (batch, patches, dims) straight out of the vision tower (after
    its final layernorm); returns the same shape, L2-normalised. Pooling a
    length-1 sequence is the MaskCLIP trick: the head's cross-attention
    collapses to a projection of that single token, which lands it in the
    text-aligned space the pooled vector lives in.
    """
    head = model._model.vision_model.head
    batch, n_patches, dims = tokens.shape
    weight = next(head.parameters())
    pooled = head(tokens.to(weight.device, weight.dtype).reshape(batch * n_patches, 1, dims))
    return torch.nn.functional.normalize(pooled.reshape(batch, n_patches, dims), dim=-1)


def per_patch_embeddings(model: SigLIPModel, pixel_values: torch.Tensor) -> torch.Tensor:
    """Embed images to (batch, patches, dims) text-aligned, L2-normalised patch vectors."""
    hidden = model._model.vision_model(pixel_values=pixel_values).last_hidden_state
    return align_patch_tokens(model, hidden)


class VisualMemoryIndex:
    """A SigLIP 2 per-patch embedding index over one image stream of a recording."""

    def __init__(
        self,
        store: Store,
        pose_of: Callable[[Any], np.ndarray | None],
        image_stream_name: str = "color_image",
        index_stream_name: str = "",  # default: named after the images and the model
        model_name: str = SIGLIP2_MODEL_NAME,
        device: str | None = None,
        dtype: torch.dtype = torch.float16,
        world_frame: str | None = None,
        pose_tag: str = POSE_FRAME_TAG,
    ) -> None:
        """*pose_of* maps an image observation to its camera's optical pose in
        the world as a 4x4 matrix, or None to skip the frame. *world_frame* names
        that world: an index built in another one is refused, not reused."""
        self.store = store
        self.pose_of = pose_of
        self.world_frame = world_frame
        # Rows carry poses as they were computed, never recomputed on read, so this
        # records which extrinsic produced them. A recording whose camera mount is
        # later measured moves every pose by the whole correction, and an index built
        # before that has to be rebuilt rather than quietly believed.
        self.pose_tag = pose_tag
        self.image_stream_name = image_stream_name
        self.index_stream_name = index_stream_name or index_stream_name_of(
            model_name, image_stream_name
        )
        self.model_name = model_name
        self._device = device
        self._dtype = dtype
        self._model: SigLIPModel | None = None
        self._index_stream: Any = None
        self._precomputed: str | None | _Unresolved = _UNRESOLVED
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
                tags = stream.first().tags
                built_with = tags.get("model")
                if built_with != self.model_name:
                    raise ValueError(
                        f"index stream {self.index_stream_name!r} was built with {built_with}, "
                        f"not {self.model_name}; rebuild it or pass model_name={built_with!r}"
                    )
                if tags.get("image_stream", self.image_stream_name) != self.image_stream_name:
                    raise ValueError(
                        f"index stream {self.index_stream_name!r} indexes "
                        f"{tags.get('image_stream')!r}, not {self.image_stream_name!r}"
                    )
                built_in = tags.get("world_frame")
                if built_in and self.world_frame and built_in != self.world_frame:
                    raise ValueError(
                        f"index stream {self.index_stream_name!r} holds poses in {built_in!r}, "
                        f"not {self.world_frame!r}; rebuild it"
                    )
                if tags.get("pose_frame") != self.pose_tag:
                    raise ValueError(
                        f"index stream {self.index_stream_name!r} stores "
                        f"{tags.get('pose_frame')!r} poses, not {self.pose_tag!r}; rebuild it"
                    )
            self._index_stream = stream
        return self._index_stream

    @property
    def precomputed_stream_name(self) -> str | None:
        """The stream siglipify wrote for this image stream and model, if the recording has one.

        When it does, nothing is built: those vectors are the index.
        """
        if isinstance(self._precomputed, _Unresolved):
            name = embedding_stream_name(self.image_stream_name, self.model_name)
            self._precomputed = name if name in self.store.list_streams() else None
        return self._precomputed

    def count(self) -> int:
        """How many frames are already indexed."""
        if self.precomputed_stream_name is not None:
            return StoredEmbeddings(self.store, self.precomputed_stream_name).count()
        return int(self.index_stream.count())

    def _posed_frames(self) -> Iterator[tuple[Any, np.ndarray]]:
        """(image observation, world_T_optical) in time order, skipping frames tf cannot place."""
        for obs in self.store.streams[self.image_stream_name].order_by("ts"):
            matrix = self.pose_of(obs)
            if matrix is not None:
                yield obs, matrix

    def build(self, stride: int = 1, batch_size: int = 8) -> int:
        """Embed every *stride*-th posed frame of the image stream into the index.

        Returns the number of frames added. Existing index rows are kept, so a
        rebuild after adding frames only costs the new ones.
        """
        if stride < 1:
            raise ValueError(f"stride must be at least 1, got {stride}")
        if self.precomputed_stream_name is not None:
            logger.info(
                "using the %d frames siglipify embedded into %r; nothing to build",
                self.count(),
                self.precomputed_stream_name,
            )
            return 0

        from PIL import Image as PILImage

        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        try:
            target: Any | None = self.index_stream
            stale = ""
        except ValueError as mismatch:  # another model, camera or pose convention
            target, stale = None, str(mismatch)  # dropped below, once vectors replace it
        already_indexed = set() if target is None else {obs.data.source_id for obs in target}
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
            if target is None:  # the replacement is in hand now, so the stale rows can go
                logger.warning("dropping %r and rebuilding: %s", self.index_stream_name, stale)
                self.store.delete_stream(self.index_stream_name)
                self._index_stream = None
                target = self.index_stream
            for (obs, matrix), patches in zip(batch, embeddings, strict=True):
                target.append(
                    PatchGrid(
                        source_id=int(obs.id),
                        rows=grid_side,
                        cols=grid_side,
                        patches=patches.to(torch.float16).cpu().numpy(),
                    ),
                    ts=obs.ts,
                    pose=PoseStamped(
                        position=Vector3(*matrix[:3, 3]),
                        orientation=Quaternion(*quaternion_from_matrix(matrix[:3, :3])),
                    ),
                    tags={
                        "model": self.model_name,
                        "image_stream": self.image_stream_name,
                        "world_frame": self.world_frame,
                        "pose_frame": self.pose_tag,
                    },
                )
                added += 1
            logger.info("indexed %d frames of %s", added, self.image_stream_name)
        if target is None and stale:
            # Nothing was placeable, so no replacement is coming. Keeping the mismatched
            # rows would make every later count() raise and tell the user to run the
            # rebuild that just ran; an empty index is at least honest.
            logger.warning(
                "dropping %r, which nothing could replace: %s", self.index_stream_name, stale
            )
            self.store.delete_stream(self.index_stream_name)
            self._index_stream = None
        self._loaded = None
        return added

    def load(self) -> None:
        """Bring the index into memory now, so the first query does not pay for it."""
        self._load()

    def _load(self) -> _LoadedIndex:
        """Pull the whole index into memory once, as stored (fp16)."""
        if self._loaded is None and self.precomputed_stream_name is not None:
            self._loaded = self._load_precomputed(self.precomputed_stream_name)
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
                orientations=[
                    tuple(float(value) for value in obs.pose_tuple[3:7])  # type: ignore[misc]
                    if len(obs.pose_tuple) >= 7
                    else (0.0, 0.0, 0.0, 1.0)
                    for obs in observations
                ],
            )
        return self._loaded

    def _load_precomputed(self, name: str) -> _LoadedIndex:
        """Read siglipify's rows and place each frame with ``pose_of``.

        A mem2 row names its source frame; an mcap row only shares its stamp.
        Rows whose frame cannot be found or placed are dropped.
        """
        rows = StoredEmbeddings(self.store, name)
        total = rows.count()
        if total == 0:
            raise LookupError(f"embedding stream {name!r} is empty")
        if rows.text_aligned() is False:
            raise ValueError(
                f"embedding stream {name!r} holds raw vision-tower tokens, which text cannot "
                "score; re-run siglipify (it now applies the pooling head per patch)"
            )
        # Ids and stamps only: an mcap observation holds its image bytes, and a
        # recording has tens of thousands of them.
        frames = [
            SimpleNamespace(id=int(obs.id), ts=float(obs.ts), pose_tuple=obs.pose_tuple)
            for obs in self.store.streams[self.image_stream_name].order_by("ts")
        ]
        by_id = {obs.id: obs for obs in frames}
        stamps = np.array([obs.ts for obs in frames])

        def frame_of(row: Any) -> Any | None:
            if row.source_id is not None:
                return by_id.get(row.source_id)
            after = int(np.searchsorted(stamps, row.ts))
            nearest = min(
                (i for i in (after - 1, after) if 0 <= i < len(frames)),
                key=lambda i: abs(stamps[i] - row.ts),
                default=None,
            )
            if nearest is None or abs(stamps[nearest] - row.ts) > STAMP_MATCH_TOLERANCE_S:
                return None
            return frames[nearest]

        patches: torch.Tensor | None = None
        side = 1
        source_ids: list[int] = []
        timestamps: list[float] = []
        positions: list[tuple[float, float, float]] = []
        orientations: list[tuple[float, float, float, float]] = []
        dropped = 0
        for row in rows:
            if row.model is not None and row.model != self.model_name:
                raise ValueError(
                    f"embedding stream {name!r} was built with {row.model}, not "
                    f"{self.model_name}; pass model_name={row.model!r}"
                )
            obs = frame_of(row)
            matrix = None if obs is None else self.pose_of(obs)
            if obs is None or matrix is None:
                dropped += 1
                continue
            if patches is None:
                side = grid_side(row.vectors.shape[0])
                patches = torch.empty((total, *row.vectors.shape), dtype=torch.float16)
            if row.vectors.shape != patches.shape[1:]:
                raise ValueError(
                    f"embedding stream {name!r} mixes shapes: {row.vectors.shape} after "
                    f"{tuple(patches.shape[1:])}"
                )
            slot = len(source_ids)
            source_ids.append(int(obs.id))
            timestamps.append(float(obs.ts))
            positions.append((float(matrix[0, 3]), float(matrix[1, 3]), float(matrix[2, 3])))
            orientations.append(quaternion_from_matrix(matrix[:3, :3]))
            patches[slot] = torch.nn.functional.normalize(torch.from_numpy(row.vectors), dim=-1).to(
                torch.float16
            )
            if slot % 256 == 255:
                logger.info("loaded %d/%d precomputed frames of %r", slot + 1, total, name)
        if patches is None:
            raise LookupError(f"none of the {total} frames in {name!r} could be placed")
        if dropped:
            logger.warning("%d of %d rows of %r have no placeable frame", dropped, total, name)
        return _LoadedIndex(
            patches=patches[: len(source_ids)],
            rows=side,
            cols=side,
            source_ids=source_ids,
            timestamps=timestamps,
            positions=positions,
            orientations=orientations,
        )

    def _embed(self, text: str) -> torch.Tensor:
        return self.model.embed_text(text).to_torch("cpu").to(torch.float32)

    def _background_for(self, query: torch.Tensor) -> torch.Tensor:
        if self._background is None:
            self._background = torch.stack([self._embed(prompt) for prompt in BACKGROUND_PROMPTS])
        keep = (self._background @ query) < BACKGROUND_SYNONYM_CUTOFF
        return self._background[keep]

    def frame_patches(self, text: str, k: int = 12) -> list[FramePatches]:
        """The *k* best frames for *text* with their full patch score grids."""
        loaded = self._load()
        query = self._embed(text)
        similarity = patch_similarity(loaded.patches, query, self._background_for(query))
        top = torch.topk(similarity.amax(dim=-1), k=min(k, similarity.shape[0]))
        return [
            FramePatches(
                source_id=loaded.source_ids[frame],
                ts=loaded.timestamps[frame],
                position=loaded.positions[frame],
                orientation=loaded.orientations[frame],
                similarity=similarity[frame],
                rows=loaded.rows,
                cols=loaded.cols,
            )
            for frame in top.indices.tolist()
        ]

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
                orientation=loaded.orientations[frame],
            )
            for score, frame in zip(top.values.tolist(), top.indices.tolist(), strict=True)
        ]

    def stop(self) -> None:
        if self._model is not None:
            self._model.stop()
            self._model = None
        self._loaded = None
        self._background = None


class _Unresolved:
    """Marker for "not looked up yet" where None means "the recording has none"."""


_UNRESOLVED = _Unresolved()


@dataclass(frozen=True)
class _LoadedIndex:
    patches: torch.Tensor  # frames, patches, dims (float16, as stored)
    rows: int
    cols: int
    source_ids: list[int]
    timestamps: list[float]
    positions: list[tuple[float, float, float]]
    orientations: list[tuple[float, float, float, float]]


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

    from dimos.teleop.memory_world.recording import open_recording

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("store_path")
    parser.add_argument("--image-stream", default="color_image")
    parser.add_argument(
        "--index-stream", default="", help="default: named after the images and the model"
    )
    parser.add_argument("--tf-stream", default="tf")
    parser.add_argument(
        "--world-frame",
        default="world",
        help="the frame to place poses in; one tf lacks means its root",
    )
    parser.add_argument(
        "--camera-frame",
        default=None,
        help="optical frame of the images (default: the image stream's own frame_id)",
    )
    parser.add_argument("--model", default=SIGLIP2_MODEL_NAME)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--device", default=None)
    parser.add_argument("--search", default=None, help="run this query after building")
    args = parser.parse_args()

    from dimos.teleop.memory_world.recording import build_tf_tree, tf_root

    store = open_recording(args.store_path)
    store.start()
    tree = build_tf_tree(store, args.tf_stream, args.world_frame)
    # Like the module: a world frame tf does not know means the tf root.
    world = (
        args.world_frame if args.world_frame in tree.frames else tf_root(tree) or args.world_frame
    )
    camera_frame = args.camera_frame or str(
        getattr(store.streams[args.image_stream].first().data, "frame_id", "")
    )
    index = VisualMemoryIndex(
        store,
        pose_of=lambda obs: tree.lookup(world, camera_frame, float(obs.ts)),
        image_stream_name=args.image_stream,
        index_stream_name=args.index_stream,
        model_name=args.model,
        device=args.device,
        world_frame=world,
        pose_tag=pose_tag_for(tree),
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
