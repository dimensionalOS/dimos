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

"""Text-to-place lookup over a recording's images, using SigLIP 2 image embeddings.

"Where did I see a traffic cone" has to answer in under a couple of seconds, so
the expensive half is precomputed: every sampled camera frame is embedded once
into an index stream. At query time only the *text* is embedded.

**One vector per image, and nothing finer.** The vector is whatever
``EmbeddingModel.embed`` returns -- DimOS's own image-embedding call, the same
one ``dimos.memory.embed.EmbedImages`` runs over a recording -- so the index
holds exactly what the platform already produces for any image. There is no
patch grid and no attempt to work out *where in the picture* the match sits:
a frame either looks like the sentence or it does not.

That is also what an answer can honestly say. A matching frame fixes **when**
the thing was seen, and tf turns that into **where the camera stood** -- so a
place is a spot the robot saw the thing FROM, not the thing's own coordinates.
Nothing here back-projects a match into the map.

**The lookup is DimOS's own vector database.** Each frame's embedding is appended to
the index stream with ``embedding=``, which puts it in the store's vector index
(``SqliteVectorStore``), and a question is answered by ``Stream.search(query_vec, k)``
from ``dimos/memory/stream.py`` -- plain cosine, ranked by the store. Nothing in this
module scores vectors itself, and there is no in-memory copy of the index.

Two things fill that index and both end up in the same place, so there is ONE query
path. ``build()`` embeds the frames here. ``_import_precomputed()`` copies the rows
siglipify wrote into the recording (a ``<image stream>_<model>`` stream of
``std_msgs/msg/Float32MultiArray``); siglipify must have been run with
``embedding = "pooled"``, and a stream of per-patch rows is refused rather than pooled
after the fact -- averaging patch tokens is not the vector the model's own head
produces, and quietly substituting one for the other is how a score stops meaning
anything.

The index stream stores the source observation id as part of its payload
rather than a copy of the image, and the model name, image stream and world
frame in its tags: the vector width is fixed by the checkpoint, so an index
built with one model cannot be searched with another. Each row's pose is the
camera's *optical* frame in the world (z forward, x right, y down), supplied by
the caller's ``pose_of`` (the recording's tf tree); rows are tagged with that
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
from dimos.teleop.memory_world.recording import StoredEmbeddings, embedding_stream_name
from dimos.teleop.memory_world.tf_tree import quaternion_from_matrix
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable, Iterator

    from dimos.memory.store.base import Store

# How index rows say which frame their pose describes.
POSE_FRAME_TAG = "camera_optical"
# What KIND of index a row belongs to. An older index of this package stored a per-patch
# grid as the row's payload and scored it in memory; those rows carry the same model,
# camera and world tags as these, decode to a different payload, and have no entry in the
# store's vector index -- so without this tag a stale one is adopted and every question
# fails on it. A row without the tag is an old row.
INDEX_KIND_TAG = "image-embedding-v1"

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


# The most a knn query may ask the vector store for. sqlite-vec refuses a larger k
# outright -- "k value in knn query too large, provided 4819 and the limit is 4096" -- so
# a recording with more indexed frames than this cannot be asked for all of them at once,
# which is what a windowed search would like to do.
MAX_VECTOR_SEARCH_K = 4096
# An mcap embedding names its frame only by stamp; this is how close it must be.
STAMP_MATCH_TOLERANCE_S = 1e-3


@dataclass(frozen=True)
class FrameEmbedding:
    """One indexed camera frame: which frame it was.

    The vector itself is NOT here. It goes to the store's vector index via
    ``append(..., embedding=...)``, which is what makes ``Stream.search`` able to rank
    these rows at all; keeping a second copy in the payload would be a copy nothing reads.
    The row's pose and timestamp are the observation's own.
    """

    source_id: int


@dataclass(frozen=True)
class Place:
    """One spot the query was seen FROM: a matching frame's camera pose.

    ``position`` is where the camera stood, not where the thing is. With one
    vector per image that is the whole of what the match knows, and the wording
    everywhere downstream says so.
    """

    position: tuple[float, float, float]
    similarity: float
    source_id: int
    ts: float
    # Pose the frame was captured from: (qx, qy, qz, qw).
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


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


def _unit(vector: np.ndarray) -> np.ndarray:
    """L2-normalise, so the cosine the vector store reports is comparable between runs.

    A locally built index and one imported from siglipify do not otherwise agree on
    scale, and the same question would score differently on the same recording depending
    on which way its embeddings got there.
    """
    flat = np.asarray(vector, dtype=np.float32).reshape(-1)
    norm = float(np.linalg.norm(flat))
    return flat if norm == 0.0 else flat / norm


def body_style_quaternion(optical: np.ndarray) -> tuple[float, float, float, float]:
    """Quaternion of a frame with x along the optical axis and z the image's up.

    The viewer orients capture markers assuming a robot body (x forward, z
    up); a camera's optical frame is z forward and y down.
    """
    forward = optical[:3, 2] / np.linalg.norm(optical[:3, 2])
    up = -optical[:3, 1] / np.linalg.norm(optical[:3, 1])
    left = np.cross(up, forward)
    return quaternion_from_matrix(np.stack([forward, left, up], axis=1))


def sensor_intrinsics(info: Any) -> tuple[tuple[float, float, float, float], tuple[int, int]]:
    """``(fx, fy, cx, cy)`` and the raster they address, after ROI and binning.

    A camera_info's K is solved for the FULL calibrated frame; ``roi`` and ``binning``
    say which part of it the published image actually is, and the published image is
    ``roi / binning``. This is the adjustment ROS's own image_proc makes, and it is not
    the same as scaling by the size ratio: a top-left 640x480 crop of a 1280x960
    calibration keeps fx exactly and only moves the principal point, so inferring a
    resize from the dimensions alone moved the sampled pixel from (480, 240) to
    (240, 120) and read the background instead of the object -- 5 m where 2 m was right.

    Returning the size alongside is what lets the caller tell the remaining difference
    apart: anything still mismatched after this IS a resize, and scales.
    """
    bx = max(int(getattr(info, "binning_x", 0) or 1), 1)
    by = max(int(getattr(info, "binning_y", 0) or 1), 1)
    rx = float(getattr(info, "roi_x_offset", 0) or 0)
    ry = float(getattr(info, "roi_y_offset", 0) or 0)
    rw = int(getattr(info, "roi_width", 0) or 0) or int(info.width)
    rh = int(getattr(info, "roi_height", 0) or 0) or int(info.height)
    k = info.K
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    return (fx / bx, fy / by, (cx - rx) / bx, (cy - ry) / by), (rw // bx, rh // by)


class VisualMemoryIndex:
    """A SigLIP 2 image-embedding index over one image stream of a recording."""

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
    ) -> None:
        """*pose_of* maps an image observation to its camera's optical pose in
        the world as a 4x4 matrix, or None to skip the frame. *world_frame* names
        that world: an index built in another one is refused, not reused."""
        self.store = store
        self.pose_of = pose_of
        self.world_frame = world_frame
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
        self._span: tuple[float, float] | None = None

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
            stream = self.store.stream(self.index_stream_name, FrameEmbedding)
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
                if self.world_frame and built_in != self.world_frame:
                    raise ValueError(
                        f"index stream {self.index_stream_name!r} holds poses in {built_in!r}, "
                        f"not {self.world_frame!r}; rebuild it"  # no tag means old, not any
                    )
                if tags.get("pose_frame") != POSE_FRAME_TAG:
                    raise ValueError(
                        f"index stream {self.index_stream_name!r} stores "
                        f"{tags.get('pose_frame')!r} poses, not {POSE_FRAME_TAG!r}; rebuild it"
                    )
                if tags.get("index_kind") != INDEX_KIND_TAG:
                    raise ValueError(
                        f"index stream {self.index_stream_name!r} is a "
                        f"{tags.get('index_kind') or 'pre-vector-store'} index, not "
                        f"{INDEX_KIND_TAG!r}; rebuild it"
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
            # An empty stream cannot fill a role, the same rule `detect_streams` applies
            # and for the same reason: a killed ingest leaves the NAME behind. Adopted on
            # the name alone, an empty one is a trap with no way out from inside the
            # product -- `count()` reads 0 so `build()` says "nothing to build" and never
            # builds, while `_index_status` reports the vectors PRESENT, which hides the
            # viewer's "Add embeddings" button. And this module makes that artifact
            # itself: `stop()` terminates the embed job, so a `memworld --stop` during an
            # "Add embeddings" run kills siglipify mid-write.
            # Counted, not iterated: these rows are siglipify's cdr-encoded vectors, and
            # walking the stream DECODES them, which raises `Unknown codec: 'cdr'` wherever
            # that codec is not registered. `StoredEmbeddings.count()` is what `count()`
            # below already uses on this same stream, so it is known to work on the real
            # thing.
            # And a stream this build cannot COUNT is not one either -- the same clause
            # `detect_streams` ends with. On a SqliteStore that count is a raw
            # `SELECT count(*) FROM "<name>"`, so a registry entry whose table the killed
            # writer never created raises `no such table` here, where the old name lookup
            # simply returned. Turning a dormant trap into a live crash is how the FIRST
            # version of this fix went wrong; this is the same mistake one layer out.
            has_rows = name in self.store.list_streams() and self._usable(name)
            if not has_rows:
                name = self._embeddings_under_another_prefix() or name
                has_rows = name in self.store.list_streams() and self._usable(name)
            self._precomputed = name if has_rows else None
        return self._precomputed

    def _usable(self, name: str) -> bool:
        """Rows AND vectors `_load_precomputed` will accept -- not just rows.

        Counting alone said "present" for a stream that load() then refuses, so
        `_index_status` told the viewer search was ready while every query failed. It also
        let a stale siglipify stream of raw vision-tower tokens outrank a freshly built
        index in this same recording: 1108 unusable vectors adopted over 5538 good ones.
        """
        try:
            rows = StoredEmbeddings(self.store, name)
            if rows.count() <= 0:
                return False
            if rows.text_aligned() is False:
                logger.warning(
                    "embeddings stream %r holds raw vision-tower tokens; not adopting", name
                )
                return False
            return True
        except Exception:
            logger.warning("embeddings stream %r cannot be read; not adopting", name)
            return False

    def _embeddings_under_another_prefix(self) -> str | None:
        """A siglipify stream for this MODEL whose prefix is not this image stream's.

        siglipify names its stream after the image stream it was pointed at, and that need
        not be the name this recording's images carry: sf_office1_2 holds 1108 vectors in
        `image_siglip2_giant_opt_p16_384` while the images are `realsense_color_image`, so
        the strict name found nothing and the viewer offered to build an index that was
        already sitting in the file.

        Only when there is exactly ONE candidate. The strict rule exists because two
        models' vectors are not comparable and two cameras' frames are not the same
        evidence; with a single embeddings stream for this model in the recording there is
        nothing it could be confused with, and with more than one there is, so this
        declines rather than guessing which camera a prefix meant.
        """
        suffix = f"_{model_slug(self.model_name)}"
        found = [
            name
            for name in self.store.list_streams()
            if name.endswith(suffix) and "_index_" not in name and self._usable(name)
        ]
        if len(found) != 1:
            if found:
                logger.warning(
                    "%d embeddings streams for %s and none named for %r; not guessing: %s",
                    len(found),
                    self.model_name,
                    self.image_stream_name,
                    ", ".join(sorted(found)),
                )
            return None
        logger.info(
            "adopting %r for images %r: the only embeddings stream for this model",
            found[0],
            self.image_stream_name,
        )
        return found[0]

    def count(self) -> int:
        """How many frames are searchable.

        Before siglipify's rows have been imported they are still the count: the answer
        to "is this recording searchable" must not change just because nothing has asked
        it a question yet.
        """
        try:
            indexed = int(self.index_stream.count())
        except ValueError as mismatch:
            # An index this build cannot search is not one. Reporting its size would tell
            # the viewer search is ready and hide the "Add embeddings" button, and every
            # question would then fail against it; `build()` drops and replaces it.
            logger.warning("ignoring %r: %s", self.index_stream_name, mismatch)
            indexed = 0
        if indexed:
            return indexed
        precomputed = self.precomputed_stream_name
        if precomputed is not None:
            return StoredEmbeddings(self.store, precomputed).count()
        return 0

    def _index_tags(self) -> dict[str, Any]:
        """What every index row is stamped with, so a mismatched one is refused not reused."""
        return {
            "model": self.model_name,
            "image_stream": self.image_stream_name,
            "world_frame": self.world_frame,
            "pose_frame": POSE_FRAME_TAG,
            "index_kind": INDEX_KIND_TAG,
        }

    def _posed_frames(self, stride: int = 1) -> Iterator[tuple[Any, np.ndarray]]:
        """(image observation, world_T_optical) in time order, skipping frames tf cannot place.

        The stride counts frames of the IMAGE STREAM, not of the posed ones that survive
        the skip, and that distinction is the whole point. Striding the survivors makes
        the sample set a function of how many frames tf happened to place, so a run that
        places a few more or fewer re-phases every later pick: the index already in the
        recording then matches almost nothing the next build wants, and it re-embeds most
        of the recording -- on cpu, holding the store lock, while the viewer says only
        "Search not ready". Measured on grocery.db: 24,110 frames, 14,478 posed, stride 3
        over the survivors is the 4,826 rows stored, and a fresh build of the same file
        wanted 6,431 it did not have. Counting raw frames makes the set depend on the
        recording alone, so a second build asks for exactly what the first one wrote.
        """
        for index, obs in enumerate(self.store.streams[self.image_stream_name].order_by("ts")):
            if index % stride:
                continue
            matrix = self.pose_of(obs)
            if matrix is not None:
                yield obs, matrix

    def build(self, stride: int = 1, batch_size: int = 8) -> int:
        """Embed every *stride*-th frame of the image stream that tf can place.

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

        from dimos.models.embedding.base import Embedding
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        try:
            target: Any | None = self.index_stream
            stale = ""
        except ValueError as mismatch:  # another model, camera or pose convention
            target, stale = None, str(mismatch)  # dropped below, once vectors replace it
        already_indexed = set() if target is None else {obs.data.source_id for obs in target}
        # A frame tf cannot place is skipped, not consumed: it stays wanted, and a later
        # build over a repaired tf picks it up. That retry costs a pose lookup, never an
        # embedding, because the skip happens before the model is asked for anything.
        wanted = (
            (obs, pose)
            for obs, pose in self._posed_frames(stride=stride)
            if int(obs.id) not in already_indexed
        )

        added = 0
        for batch in _batched(wanted, batch_size):
            # The platform's own image-embedding call -- the same one
            # `dimos.memory.embed.EmbedImages` runs over a recording -- handed the
            # recording's own `Image` messages, which is what it takes. (It does the
            # `to_rgb()` itself; converting to PIL first got "'Image' object has no
            # attribute 'to_rgb'" from inside the model.) Whatever DimOS gives any other
            # consumer of this model for a picture is exactly what the index holds.
            embedded = self.model.embed(*[obs.data for obs, _ in batch])
            if not isinstance(embedded, list):
                embedded = [embedded]
            if target is None:  # the replacement is in hand now, so the stale rows can go
                logger.warning("dropping %r and rebuilding: %s", self.index_stream_name, stale)
                self.store.delete_stream(self.index_stream_name)
                self._index_stream = None
                target = self.index_stream
            for (obs, matrix), vector in zip(batch, embedded, strict=True):
                target.append(
                    FrameEmbedding(source_id=int(obs.id)),
                    ts=obs.ts,
                    pose=PoseStamped(
                        position=Vector3(*matrix[:3, 3]),
                        orientation=Quaternion(*quaternion_from_matrix(matrix[:3, :3])),
                    ),
                    tags=self._index_tags(),
                    # Into the store's VECTOR index, not into the payload: this is what
                    # makes `Stream.search` able to answer over these frames at all.
                    embedding=Embedding(_unit(vector.to_numpy())),
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
        self._span = None
        return added

    def load(self) -> None:
        """Make the index searchable now, so the first question does not pay for it.

        With a vector store that means importing whatever siglipify left in the
        recording; there is no in-memory copy of the vectors to warm.
        """
        self._ensure_searchable()

    def _ensure_searchable(self) -> Any:
        """The index stream, with siglipify's vectors imported into it if that is where
        this recording's embeddings live. Returns the stream to search."""
        precomputed = self.precomputed_stream_name
        if precomputed is not None and int(self.index_stream.count()) == 0:
            self._import_precomputed(precomputed)
        return self.index_stream

    def _import_precomputed(self, name: str) -> int:
        """Copy siglipify's rows into the index stream, as embeddings the store can search.

        siglipify writes plain ``Float32MultiArray`` rows into the recording; they carry a
        stamp and sometimes a source id, but the recording is read-only and its rows are
        not in any vector index. Copying them once into the derived database's index
        stream is what makes them searchable by the same path as a locally built index --
        so there is ONE query path, not two.

        A mem2 row names its source frame; an mcap row only shares its stamp. Rows whose
        frame cannot be found or placed are dropped.
        """
        from dimos.models.embedding.base import Embedding
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        rows = StoredEmbeddings(self.store, name)
        total = rows.count()
        if total == 0:
            raise LookupError(f"embedding stream {name!r} is empty")
        if rows.text_aligned() is False:
            raise ValueError(
                f"embedding stream {name!r} holds raw vision-tower tokens, which text cannot "
                'score; re-run siglipify with embedding = "pooled"'
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

        target = self.index_stream
        imported = 0
        dropped = 0
        for row in rows:
            if row.model is not None and row.model != self.model_name:
                raise ValueError(
                    f"embedding stream {name!r} was built with {row.model}, not "
                    f"{self.model_name}; pass model_name={row.model!r}"
                )
            # One row per frame, and one VECTOR per row. A per-patch stream is a
            # different kind of index, not a wider one: mean-pooling its tokens here
            # would search on a vector the model never produced, so it is refused.
            if row.vectors.shape[0] != 1:
                raise ValueError(
                    f"embedding stream {name!r} holds {row.vectors.shape[0]} vectors per frame; "
                    'this index is one-per-image -- re-run siglipify with embedding = "pooled"'
                )
            obs = frame_of(row)
            matrix = None if obs is None else self.pose_of(obs)
            if obs is None or matrix is None:
                dropped += 1
                continue
            target.append(
                FrameEmbedding(source_id=int(obs.id)),
                ts=float(obs.ts),
                pose=PoseStamped(
                    position=Vector3(*matrix[:3, 3]),
                    orientation=Quaternion(*quaternion_from_matrix(matrix[:3, :3])),
                ),
                tags=self._index_tags(),
                embedding=Embedding(_unit(row.vectors.reshape(-1))),
            )
            imported += 1
            if imported % 256 == 0:
                logger.info("imported %d/%d embedded frames of %r", imported, total, name)
        if imported == 0:
            raise LookupError(f"none of the {total} frames in {name!r} could be placed")
        if dropped:
            logger.warning("%d of %d rows of %r have no placeable frame", dropped, total, name)
        logger.info("imported %d embedded frames of %r into the index", imported, name)
        self._span = None
        return imported

    def time_span(self) -> tuple[float, float]:
        """(first, last) timestamp among the indexed frames.

        What "the first half of the recording" is measured against. It is the span of
        what was INDEXED, not of the whole recording: with a stride, or with frames tf
        could not place, those are not the same, and a fraction of a span that holds no
        searchable frames would be a window the answer could never look in.
        """
        if self._span is None:
            # The two ENDS, not every row. Every question asks for this to turn "the
            # first half" into stamps, and reading the whole index to do it is a table
            # scan per question over thousands of rows.
            stream = self._ensure_searchable()
            try:
                # `last()` orders by ts descending itself; `first()` RAISES LookupError on
                # an empty stream rather than returning None, so the empty case is caught,
                # not tested for -- "No matching observation" names nothing the reader can
                # act on.
                first, last = stream.order_by("ts").first(), stream.last()
            except LookupError:
                raise LookupError(f"index stream {self.index_stream_name!r} is empty") from None
            self._span = (float(first.ts), float(last.ts))
        return self._span

    def search(
        self,
        text: str,
        k: int = 200,
        window: tuple[float, float] | None = None,
    ) -> list[Place]:
        """The *k* frames most like *text*, most similar first.

        This is DimOS's own vector-database lookup: ``Stream.search`` over the embeddings
        recorded into the store, scored by cosine and ranked by the vector store. Each
        result is placed at the camera pose the frame was taken from -- with one vector
        per image that is the only location the match supports.

        *window* restricts the answer to frames stamped within ``(since, until)``, which
        is how a question about part of a recording is asked.
        """
        stream = self._ensure_searchable()
        query = self.model.embed_text(text)
        if window is None:
            hits: Iterable[Any] = stream.search(query, k=min(k, MAX_VECTOR_SEARCH_K))
        else:
            hits = self._search_window(stream, query, k, window)
        places: list[Place] = []
        for obs in hits:
            pose = obs.pose_tuple
            if pose is None:  # not placeable: an answer cannot point at it
                continue
            places.append(
                Place(
                    position=(float(pose[0]), float(pose[1]), float(pose[2])),
                    similarity=float(obs.similarity),
                    source_id=int(obs.data.source_id),
                    ts=float(obs.ts),
                    orientation=(
                        tuple(float(value) for value in pose[3:7])  # type: ignore[misc]
                        if len(pose) >= 7
                        else (0.0, 0.0, 0.0, 1.0)
                    ),
                )
            )
            if len(places) >= k:
                break
        return places

    def _search_window(
        self,
        stream: Any,
        query: Any,
        k: int,
        window: tuple[float, float],
    ) -> list[Any]:
        """The *k* best hits stamped inside *window*, asking the store for as few as will do.

        Search FIRST, then narrow: `time_range(...).search(...)` reads as the natural
        order and silently returns NOTHING, because the range never reaches the vector
        store, which ranks the whole stream regardless. Since the narrowing therefore
        happens AFTER the ranking, a plain top-`k` that all falls outside the window would
        answer "nothing there" about a window it never looked in.

        So the ask widens until the window is filled -- rather than asking for every frame
        at once, which is both wasteful and, past `MAX_VECTOR_SEARCH_K`, refused outright
        by the store. When the ceiling is reached with fewer than *k* in hand, that is the
        honest answer: the store has been asked for as much as it will rank.
        """
        total = int(stream.count())
        ceiling = min(total, MAX_VECTOR_SEARCH_K)
        asked = min(max(k, 1) * 4, ceiling)
        while True:
            hits = list(stream.search(query, k=asked).time_range(*window))
            if len(hits) >= k or asked >= ceiling:
                return hits
            asked = min(asked * 4, ceiling)

    def stop(self) -> None:
        if self._model is not None:
            self._model.stop()
            self._model = None
        self._span = None


class _Unresolved:
    """Marker for "not looked up yet" where None means "the recording has none"."""


_UNRESOLVED = _Unresolved()


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
    parser.add_argument(
        "--tf-tolerance",
        type=float,
        default=0.1,
        help="how far a transform may be from an image's stamp and still place it",
    )
    parser.add_argument("--model", default=SIGLIP2_MODEL_NAME)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--device", default=None)
    parser.add_argument("--search", default=None, help="run this query after building")
    args = parser.parse_args()

    from dimos.teleop.memory_world.recording import tf_root, tf_tree_if_usable
    from dimos.teleop.memory_world.tf_tree import body_camera_pose

    store = open_recording(args.store_path)
    store.start()
    # Like the module, a recording with NO tf places its images by the body pose stamped
    # on each one. Asking for the tf stream regardless died with `KeyError: 'tf'` and
    # indexed nothing -- on exactly the recordings this indexer exists for, since the
    # module only reaches for it when siglipify cannot read the file. A tf stream that is
    # THERE AND EMPTY is that same recording: it loads as a tree, answers every lookup
    # with None, and `added 0 frames` was the whole of the report.
    tree = tf_tree_if_usable(store, args.tf_stream)
    if tree is not None:
        # Like the module: a world frame tf does not know means the tf root.
        world = (
            args.world_frame
            if tree.has_frame(args.world_frame)
            else tf_root(tree) or args.world_frame
        )
        camera_frame = args.camera_frame or str(
            getattr(store.streams[args.image_stream].first().data, "frame_id", "")
        )

        def pose_of(obs: Any) -> Any:
            return tree.lookup(world, camera_frame, float(obs.ts), args.tf_tolerance)
    else:
        # No tf tree to name a world with; the poses stamped on the images are already in
        # whatever world the recording was made in, which is the one the caller asked for.
        world = args.world_frame
        pose_of = body_camera_pose

    index = VisualMemoryIndex(
        store,
        pose_of=pose_of,
        image_stream_name=args.image_stream,
        index_stream_name=args.index_stream,
        model_name=args.model,
        device=args.device,
        world_frame=world,
    )
    try:
        added = index.build(stride=args.stride, batch_size=args.batch_size)
        print(f"added {added} frames; index now holds {index.count()}")
        if args.search:
            first, _ = index.time_span()
            for place in cluster_places(index.search(args.search), radius=2.5, max_places=6):
                print(
                    f"  {place.similarity:+.4f}  {place.position}  id={place.source_id}"
                    f"  {place.ts - first:.1f}s in"
                )
    finally:
        index.stop()
        store.stop()


if __name__ == "__main__":
    # Under ``python -m`` this file runs as ``__main__``, and the store records
    # payload classes by module path, so an index built here would be typed
    # ``__main__.FrameEmbedding`` and unreadable everywhere else. Run the properly
    # imported module instead.
    from dimos.teleop.memory_world.visual_search import main as installed_main

    installed_main()
