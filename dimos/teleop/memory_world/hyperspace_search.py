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

"""Answers from Hyperspace's voxel heat map, shaped for the viewer.

Hyperspace (``dimos.mapping.hyperspace``) scores a text query against every
stored camera patch, places the hot ones as pyramids through tf and pools
them into a voxel heat map. This module turns one such map into what the
viewer walks through:

* **clusters** - connected blobs of hot voxels, best first, each with a
  centre and radius the camera can fly to and orbit;
* **evidence** - the keyframes whose hot patches landed in each cluster, so
  the picture behind an answer can be hung where its camera stood;
* **pyramids** - the hot patches' frusta themselves, for the slide that
  shows how the map was made.

The arithmetic runs on :mod:`hyperspace_fast` (resident arrays, ~100 ms a
query); Hyperspace's own engine only loads the keyframes and embeds text.
Nothing here embeds images: ``hyperspace_keyframes`` and ``hyperspace_patches``
must already be there. A ``.db`` recording holds them itself; only an mcap,
which `McapStore` opens read-only, gets a ``<recording>.hyperspace.db`` beside it.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
import sqlite3
import threading
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from dimos.teleop.memory_world.hyperspace_fast import (
    FastQuery,
    FastResult,
    Frames,
    Patches,
    near_scene,
    pack_keys,
    pack_probe_keys,
    patch_rects,
    project_pixels,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

KEYFRAME_STREAM = "hyperspace_keyframes"
PATCH_STREAM = "hyperspace_patches"
# NOT written any more, only deleted. It used to be written last by the ingest, so that
# keyframes arriving one at a time could not be mistaken for a finished run; that was
# dropped deliberately (see hyperspace_ingest, "Nothing marks the in-place index
# finished"). It is still removed wherever it is found, because a recording indexed before
# that change carries one and it would vouch for keyframes that have since been dropped.
COMPLETE_STREAM = "hyperspace_complete"
MEMORY_DB_SUFFIX = ".hyperspace.db"

# The model a memory db is embedded with, in ONE place. An ingest run by hand with a
# different one produces a db the module cannot read at all -- the vectors are a
# different width -- and the only sign is a shape error at query time.
HYPERSPACE_MODEL_NAME = "google/siglip2-so400m-patch16-384"

# Voxels scoring below this fraction of the map's top (1.0 after Hyperspace's
# normalization) are neither drawn nor clustered.
SCORE_CUTOFF = 0.3
# Two hot voxels this many voxels apart (Chebyshev) belong to one cluster.
CLUSTER_GAP_VOXELS = 2
MIN_CLUSTER_VOXELS = 4
MAX_CLUSTERS = 12
# How many places are clustered before the viewpoint ranking picks the MAX_CLUSTERS that
# survive. Capping at MAX_CLUSTERS while clustering means capping by SCORE, and the place
# seen from twenty viewpoints is then thrown away before the ranking it would have won ever
# runs. Evidence is gathered for the whole pool, which is the cost of the wider net.
CLUSTER_POOL = MAX_CLUSTERS * 3
# A cluster whose summed score is under this fraction of the best is dropped.
MIN_CLUSTER_FRACTION = 0.05
EVIDENCE_PER_CLUSTER = 8  # distinct PICTURES shown per place, not distinct records
# Hyperspace's refine chain works on dense grids over the heat's bounding box; on a
# city-scale map a broad question lights thousands of voxels kilometres apart, so
# only the best ones are refined (the rest never made a cluster anyway).
REFINE_MAX_VOXELS = 3000
# ... and its grids cover the heat's bounding box: past this extent (metres, any axis)
# the sparse components here answer instead, or the box would be gigabytes.
REFINE_MAX_EXTENT_M = 60
REFINE_MAX_CELLS = 50_000_000  # the dense box refine grids: 200 MB of float32.0
# The occupancy gate is trusted only when it keeps at least this share of the hot voxels.
# Measured on grocery_stitch.db, the grounded share runs 0.59 to 0.91 across six queries, so
# this has never fired: it is insurance against a query whose true place was never seen in
# depth, not a tuned threshold. A depth thumbnail is decimated by depth_thumbnail_stride (4),
# so it is far sparser than the depth that was available -- if grounding ever rejects
# something that plainly has depth, that stride is the first suspect.
GROUNDED_FLOOR = 0.2
MAX_PYRAMIDS = 240


@dataclass
class Evidence:
    """One keyframe that saw a cluster: where its camera stood and which patch hit."""

    keyframe_id: int
    camera_frame: str
    ts: float
    # 4x4 target_from_camera (optical: z forward, y down).
    pose: NDArray[np.float64]
    score: float
    # Centre of the hot patch, image pixels, for marking it on the picture.
    image_uv: tuple[float, float]
    # Where that patch landed in the world.
    point: tuple[float, float, float]
    channel: str = "patches"

    @property
    def position(self) -> tuple[float, float, float]:
        return tuple(float(v) for v in self.pose[:3, 3])  # type: ignore[return-value]

    @property
    def forward(self) -> tuple[float, float, float]:
        return tuple(float(v) for v in self.pose[:3, 2])  # type: ignore[return-value]

    @property
    def up(self) -> tuple[float, float, float]:
        return tuple(float(-v) for v in self.pose[:3, 1])  # type: ignore[return-value]


@dataclass
class Cluster:
    index: int
    centre: tuple[float, float, float]
    # Distance holding 90% of the cluster's voxels (>= half a voxel).
    radius: float
    # The ranking key: the summed voxel score from cluster_voxels, or, after
    # Hyperspace's refine, the cluster's share of the best one (0-1).
    score: float
    peak: float
    n_voxels: int
    # Distinct keyframes that saw this place, UNCAPPED. `evidence` is the handful of them
    # worth showing, so its length says how many pictures are on screen, not how many
    # viewpoints agreed -- which is what a person means by "seen from 8 places", and what
    # the ranking has to sort on.
    views: int = 0
    evidence: list[Evidence] = field(default_factory=list)

    def summary(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "centre": [round(v, 3) for v in self.centre],
            "radius": round(self.radius, 3),
            "score": round(self.score, 3),
            "peak": round(self.peak, 3),
            "n_voxels": self.n_voxels,
            "n_views": self.views,
            "n_evidence": len(self.evidence),
        }


@dataclass
class Pyramid:
    """A hot patch's frustum slice: apex at the camera, near and far rectangles."""

    apex: tuple[float, float, float]
    near: list[tuple[float, float, float]]
    far: list[tuple[float, float, float]]
    score: float
    keyframe_id: int
    cluster: int = -1

    def summary(self) -> dict[str, Any]:
        def rnd(pts: list[tuple[float, float, float]]) -> list[list[float]]:
            return [[round(v, 3) for v in p] for p in pts]

        return {
            "apex": [round(v, 3) for v in self.apex],
            "near": rnd(self.near),
            "far": rnd(self.far),
            "score": round(self.score, 3),
            "keyframe": self.keyframe_id,
            "cluster": self.cluster,
        }


@dataclass
class HeatmapAnswer:
    text: str
    frame: str
    voxel_size: float
    # Every voxel above the cutoff: centres (N, 3), scores (N,), cluster index (N,) or -1.
    centres: NDArray[np.float32]
    scores: NDArray[np.float32]
    cluster_of: NDArray[np.int16]
    clusters: list[Cluster]
    pyramids: list[Pyramid]
    stats: dict[str, Any]
    seconds: float

    @property
    def n_voxels(self) -> int:
        return len(self.scores)


def cluster_voxels(
    indices: NDArray[np.integer],
    scores: NDArray[np.floating],
    voxel_size: float,
    *,
    gap: int = CLUSTER_GAP_VOXELS,
    min_voxels: int = MIN_CLUSTER_VOXELS,
    max_clusters: int = MAX_CLUSTERS,
    min_fraction: float = MIN_CLUSTER_FRACTION,
) -> tuple[list[Cluster], NDArray[np.int16]]:
    """Group hot voxels into connected blobs.

    Voxels within about ``gap`` voxels of each other (Chebyshev) join; blobs
    are ranked by summed score and renumbered 0.. best first. Returns the clusters
    and each input voxel's cluster index (-1 for voxels in dropped blobs).
    """
    labels_of = np.full(len(indices), -1, dtype=np.int16)
    if len(indices) == 0:
        return [], labels_of
    indices = np.asarray(indices, dtype=np.int64)
    scores = np.asarray(scores, dtype=np.float64)
    # Sparse connectivity on a grid coarsened by `gap`: voxels sharing a
    # coarse cell, or in 26-adjacent cells, join. That bridges gaps of up to
    # `gap` voxels (sometimes more) with 26 neighbour lookups per cell, each a
    # binary search over the sorted packed keys, whatever the bounding box.
    coarse = np.floor_divide(indices, max(gap, 1))
    cell_keys, cell_of = np.unique(pack_keys(coarse), return_inverse=True)
    cells = coarse[np.unique(cell_of, return_index=True)[1]]
    m = len(cell_keys)
    src, dst = [np.arange(m)], [np.arange(m)]
    for offset in np.array(
        [
            (x, y, z)
            for x in (-1, 0, 1)
            for y in (-1, 0, 1)
            for z in (-1, 0, 1)
            if (x, y, z) != (0, 0, 0)
        ]
    ):
        # The probe form: a cluster's voxel can sit on the edge of the packed range, and
        # the neighbour one step past it does not exist rather than being an error.
        neighbour = pack_probe_keys(cells + offset)
        pos = np.minimum(np.searchsorted(cell_keys, neighbour), m - 1)
        found = cell_keys[pos] == neighbour
        src.append(np.flatnonzero(found))
        dst.append(pos[found])
    graph = coo_matrix(
        (np.ones(sum(len(a) for a in src)), (np.concatenate(src), np.concatenate(dst))),
        shape=(m, m),
    )
    n_blobs, cell_blob = connected_components(graph, directed=False)
    blob_of = cell_blob[cell_of] + 1

    candidates: list[tuple[float, int]] = []
    for blob in range(1, n_blobs + 1):
        members = np.nonzero(blob_of == blob)[0]
        if len(members) < min_voxels:
            continue
        candidates.append((float(scores[members].sum()), blob))
    candidates.sort(reverse=True)
    if not candidates:
        return [], labels_of
    floor = candidates[0][0] * min_fraction

    clusters: list[Cluster] = []
    for total, blob in candidates[:max_clusters]:
        if total < floor:
            break
        members = np.nonzero(blob_of == blob)[0]
        centres = (indices[members] + 0.5) * voxel_size
        weights = scores[members]
        centre = (centres * weights[:, None]).sum(axis=0) / weights.sum()
        distances = np.linalg.norm(centres - centre, axis=1)
        radius = max(float(np.percentile(distances, 90)), voxel_size / 2)
        rank = len(clusters)
        labels_of[members] = rank
        clusters.append(
            Cluster(
                index=rank,
                centre=tuple(float(v) for v in centre),  # type: ignore[arg-type]
                radius=radius,
                score=total,
                peak=float(weights.max()),
                n_voxels=len(members),
            )
        )
    return clusters, labels_of


def assign_points(
    points: NDArray[np.floating], clusters: list[Cluster], slack_m: float = 0.75
) -> NDArray[np.int64]:
    """Index of the cluster each point belongs to (nearest centre, inside its
    radius plus ``slack_m``), or -1."""
    out = np.full(len(points), -1, dtype=np.int64)
    if not clusters or len(points) == 0:
        return out
    centres = np.asarray([c.centre for c in clusters], dtype=np.float64)
    radii = np.asarray([max(c.radius, 0.5) + slack_m for c in clusters], dtype=np.float64)
    distances = np.linalg.norm(points[:, None, :] - centres[None, :, :], axis=2)
    nearest = distances.argmin(axis=1)
    inside = distances[np.arange(len(points)), nearest] <= radii[nearest]
    out[inside] = nearest[inside]
    return out


def _by_viewpoints(
    clusters: list[Cluster],
    owner: NDArray[np.int64],
    cluster_of: NDArray[np.int64],
    limit: int = MAX_CLUSTERS,
) -> tuple[list[Cluster], NDArray[np.int64], NDArray[np.int64]]:
    """Rank the clusters by viewpoints, keep the best *limit*, and renumber what points at them.

    The cut is made HERE and not while clustering, because a cut made there is made by
    score: a place seen from twenty viewpoints, scoring less than twelve one-view blobs,
    was dropped before this ranking ever saw it.

    Ranked on `views`, not on `len(evidence)`: evidence is capped at EVIDENCE_PER_CLUSTER,
    so ranking by it ties every cluster seen from that many places or more -- which is
    exactly the clusters most likely to be the answer, so the ranking would be a no-op
    where it matters most.

    ``index`` is not a name, it is a position: the viewer steps through clusters by it,
    the voxel labels carry it, and the pictures are filtered by it. So renumbering means
    rewriting all three together or the answer comes apart.
    """
    order = sorted(range(len(clusters)), key=lambda i: (-clusters[i].views, -clusters[i].score))
    order = order[:limit]
    renumbered = np.full(len(clusters) + 1, -1, dtype=np.int64)  # -1 for "no cluster"
    for rank, old in enumerate(order):
        renumbered[old] = rank
    ranked = [clusters[old] for old in order]
    for rank, cluster in enumerate(ranked):
        cluster.index = rank
    return ranked, renumbered[owner], renumbered[cluster_of]


def memory_db_for(recording: str | Path) -> Path:
    """Where Hyperspace's ingest puts a recording's keyframes and patches.

    A memory2 recording holds them itself: one file, and one tf tree that the map, the
    markers and the search all read. Only an mcap, which `McapStore` does not write, needs a
    companion -- and then that companion carries a copy of the recording's tf.
    """
    path = Path(recording)
    return path if path.suffix == ".db" else path.with_suffix(MEMORY_DB_SUFFIX)


def memory_db_ready(recording: str | Path) -> bool:
    """True when the index has keyframes AND patches; either alone cannot answer.

    A companion db is built beside its final name and moved into place, so its mere
    existence says the ingest finished. Keyframes written into the recording itself have
    no such moment -- they appear one at a time -- and nothing now distinguishes a
    half-written index from a whole one. This used to require :data:`COMPLETE_STREAM`,
    and that requirement was removed deliberately: the operator does not want the index
    marked complete. The cost is real and is not hidden here -- an ingest killed outright
    leaves an index that reads as finished and is merely short, and the recovery is a
    rerun. Nothing writes the marker any more, and the ingest deletes any it finds along
    with the keyframes when it rebuilds, so an old marker cannot vouch for keyframes that
    have since been dropped. (There is no `drop_index` function; dropping an index means
    deleting those streams, which is what `refuse_if_a_rebuild_is_half_done` guards.)
    """
    return memory_db_index_stamp(recording)[0] > 0


def memory_db_index_stamp(recording: str | Path) -> tuple[int, float]:
    """``(keyframes, latest keyframe stamp)`` for the recording's index; ``(0, 0.0)``
    when it has none.

    Identity, not just size, because without a completion marker this is all a reader
    has to tell one index from another. A count alone cannot see a drop and re-ingest
    that lands on the same number of keyframes, nor one that lands on fewer --
    and a reader comparing only "did it grow" would serve the deleted index for ever.
    The newest stamp moves whenever the keyframes are rewritten, so the pair changes
    whenever the index does.
    """
    path = memory_db_for(recording)
    if not path.is_file():
        return (0, 0.0)
    wanted = {KEYFRAME_STREAM, PATCH_STREAM}
    try:
        connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        try:
            names = {row[0] for row in connection.execute("SELECT name FROM _streams")}
            if not wanted <= names:
                return (0, 0.0)
            # BOTH halves, not just the keyframes. It takes two streams to answer a
            # question -- the patches are the searchable content and the keyframes only
            # say where each was seen from -- so keyframes alone is not an index, it is
            # the half that cannot answer anything.
            #
            # Seen for real: an ingest reported a clean summary ending `kept: 3462`,
            # wrote 3,462 keyframes and ZERO patches, and left a stream whose name,
            # presence and plausible count all said "index". Counting keyframes alone
            # would have loaded it and reported `ready: true` with 3,462 keyframes while
            # answering nothing at all.
            #
            # That particular index was not broken -- it was an ENSEMBLE one, whose
            # keyframes carry every member's grid inline and which is scored without ever
            # opening the patch stream, so leaving it empty was deliberate. Do not loosen
            # this check on that account: `hyperspace_fast` REFUSES an ensemble store
            # outright (it holds one grid per keyframe), so for this reader an index it
            # cannot search is an index it should decline at the door rather than load
            # and fail on every question.
            (patches,) = connection.execute(f'SELECT count(*) FROM "{PATCH_STREAM}"').fetchone()
            if int(patches) <= 0:
                return (0, 0.0)
            (count,) = connection.execute(f'SELECT count(*) FROM "{KEYFRAME_STREAM}"').fetchone()
            latest = 0.0
            try:
                (newest,) = connection.execute(
                    f'SELECT max(ts) FROM "{KEYFRAME_STREAM}"'
                ).fetchone()
                latest = float(newest or 0.0)
            except sqlite3.Error:
                # No `ts` column. Every store this package writes has one, so this is a
                # hand-made or foreign table -- the count still identifies it, and
                # reporting "no index" for a table full of keyframes would be worse.
                pass
            return (int(count), latest)
        finally:
            connection.close()
    except sqlite3.Error:
        return (0, 0.0)


def _use_the_cores() -> None:
    """A dimos worker starts torch on one thread; the text tower on cpu takes
    ~2 s that way and ~0.15 s on the machine's cores."""
    import os

    import torch

    cores = os.cpu_count() or 4
    if torch.get_num_threads() < min(cores, 8):
        before = torch.get_num_threads()
        torch.set_num_threads(min(cores, 8))
        logger.info("torch threads %d -> %d", before, torch.get_num_threads())


class HyperspaceSearch:
    """Text -> clustered heat map over one recording's Hyperspace memory db.

    Loads lazily on the first :meth:`query` (or :meth:`warm`): the text tower
    (cpu, ~3 s), every keyframe's patch grid and, when present, the segments
    (~20 s for 300 keyframes + 8k segments). Queries after that take about a
    tenth of a second. Thread-safe: one query at a time.
    """

    def __init__(
        self,
        memory_db: str | Path,
        *,
        model_name: str,
        world_frame: str,
        voxel_size: float = 0.1,
        device: str = "cpu",
        config: Any | None = None,
        use_segments: bool = True,
        refine: str = "default",
    ) -> None:
        self.memory_db = Path(memory_db)
        self.model_name = model_name
        self.world_frame = world_frame
        self.voxel_size = voxel_size
        self.device = device
        self.use_segments = use_segments
        # "default" (Hyperspace's QueryConfig.refine chain, e.g. "occupancy,support,prior") or any
        # such chain; "occupancy": heat within a voxel of the map + connected components here;
        # "none": the raw map. When a chain keeps nothing, the occupancy path answers instead.
        self.refine = refine
        self._config = config
        self._scene_keys: NDArray[np.int64] | None = None
        self._lock = threading.Lock()
        self._store: Any = None
        self._model: Any = None
        self._engine: Any = None
        self._fast: FastQuery | None = None
        self.warm_seconds: float | None = None

    # ---- lifecycle ---------------------------------------------------------

    def _load_scene(self, engine: Any) -> None:
        """Where the keyframes saw a surface: what grounds a hot voxel.

        NOT the ray-traced map, which is what this used to be. That map clears a voxel
        the moment a later scan sees through it, so a basket that was moved has its
        original place cleared -- and the grounding test then throws away the true
        location and keeps whatever wrong one happens to sit on a surface. The
        keyframes' own depth thumbnails are a union over time that nothing clears, so
        a thing that was somewhere at some point stays grounded there.
        """
        voxels = engine.scene_voxels(self.world_frame)
        if not voxels:
            # Said out loud: with no scene the occupancy gate does nothing at all, and an
            # answer that floats in mid-air then has nothing to catch it.
            logger.warning("no scene voxels from the keyframes: heat will not be grounded")
            self._scene_keys = None
            return
        indices = np.asarray([index for index, _ in voxels], dtype=np.int64)
        self._scene_keys = np.unique(pack_keys(indices))
        logger.info("scene: %d voxels seen by the keyframes' depth", len(self._scene_keys))

    def warm(self) -> None:
        """Load the model, every keyframe and the segments so the first answer is not slow."""
        with self._lock:
            self._ensure_fast()

    def close(self) -> None:
        with self._lock:
            self._release()

    def _release(self) -> None:
        for obj in (self._model, self._store):
            try:
                if obj is not None and hasattr(obj, "stop"):
                    obj.stop()
            except Exception:
                logger.exception("closing %s", type(obj).__name__)
        self._model = self._store = self._engine = self._fast = None

    @property
    def keyframe_count(self) -> int:
        fast = self._fast
        return len(fast.patches.frames) if fast is not None else 0

    @property
    def segment_count(self) -> int:
        fast = self._fast
        return len(fast.segments) if fast is not None and fast.segments is not None else 0

    def _ensure_fast(self) -> FastQuery:
        if self._fast is not None:
            return self._fast
        from dimos.mapping.hyperspace import patches as hs
        from dimos.mapping.hyperspace.embedder import SigLIP2Patches
        from dimos.mapping.hyperspace.query import HyperspaceQuery
        from dimos.memory.store.sqlite import SqliteStore

        started = time.monotonic()
        # Published as they open, so a failure part-way leaves nothing for close() to miss.
        self._store = store = SqliteStore(path=str(self.memory_db), must_exist=True)
        try:
            store.start()
            _use_the_cores()
            self._model = model = SigLIP2Patches(
                model_name=self.model_name, device=self.device, towers="text"
            )
            model.start()
            config = self._config or hs.QueryConfig()
            engine = HyperspaceQuery(
                store,
                lambda text: model.embed_text_array(text)[0],
                config,
                world_frame=self.world_frame,
                voxel_size=self.voxel_size,
            )
            self._load_scene(engine)
            fast = FastQuery(
                engine,
                world_frame=self.world_frame,
                voxel_size=self.voxel_size,
                embed_texts=lambda texts: model.embed_text_array(*texts),
                with_segments=self.use_segments,
            )
        except BaseException:
            # BaseException: this package reports an expected failure with SystemExit, and
            # letting that past here leaks the open store and a loaded text tower.
            self._release()
            raise
        self._engine, self._fast = engine, fast
        self.warm_seconds = time.monotonic() - started
        logger.info(
            "hyperspace: %d keyframes, %d segments from %s ready in %.1f s",
            len(fast.patches.frames),
            len(fast.segments) if fast.segments else 0,
            self.memory_db.name,
            self.warm_seconds,
        )
        return fast

    # ---- querying ----------------------------------------------------------

    def query(self, text: str) -> HeatmapAnswer:
        with self._lock:
            return self._query(text)

    def _query(self, text: str) -> HeatmapAnswer:
        started = time.monotonic()
        fast = self._ensure_fast()
        result = fast.query(text)

        keep = result.score >= SCORE_CUTOFF
        if self.refine != "none" and self._scene_keys is not None:
            # Heat that floats in free space is a pyramid slice that missed its surface.
            # Done here on sorted keys (sparse, fast) rather than by refine's dense grid,
            # which on a city-scale map would span kilometres.
            grounded = near_scene(result.index, self._scene_keys)
            # Only when enough of the heat is grounded to believe the gate. A depth
            # thumbnail misses glass, dark shelves and anything out of range, so a
            # location can be real and ungrounded; firing whenever a single voxel
            # survives turns "the true place was never seen in depth" into "keep only
            # the wrong places", which is worse than not gating at all.
            surviving = int((keep & grounded).sum())
            if surviving and surviving >= GROUNDED_FLOOR * int(keep.sum()):
                keep &= grounded
        refined = (
            self._refine(text, result, keep) if self.refine not in ("occupancy", "none") else None
        )
        indices = result.index[keep]
        scores = result.score[keep].astype(np.float32)
        if refined is not None:
            indices, scores, clusters, cluster_of = refined
        else:
            clusters, cluster_of = cluster_voxels(
                indices, scores, self.voxel_size, max_clusters=CLUSTER_POOL
            )
        centres = ((indices + 0.5) * self.voxel_size).astype(np.float32)

        points = np.concatenate([result.patch_points, result.segment_points]).reshape(-1, 3)
        owner = assign_points(points, clusters)
        members = np.flatnonzero(owner >= 0)
        hits = _hits_of(fast, result, members)
        for cluster in clusters:
            saw_it = [hits[i] for i in members[owner[members] == cluster.index]]
            cluster.views = viewpoints_of(saw_it)
            cluster.evidence = _pick_evidence(saw_it)
        # Ranked by how many viewpoints saw it, which is the question a person is really
        # asking: a thing seen from eight places is more likely to be the thing than one
        # bright patch seen once. Summed score breaks the ties. Only now, because the
        # evidence is what says how many saw it, and it is attached above.
        clusters, owner, cluster_of = _by_viewpoints(clusters, owner, cluster_of)
        pyramids = _pyramids(
            fast.patches.frames, result.patches, owner[: len(result.patches)], fast.config
        )

        stats = dict(result.stats)
        stats.update(
            {
                "voxels_total": len(result.score),
                "voxels_kept": len(scores),
                "refined": refined is not None,
                "clusters": len(clusters),
                "hot_patches_placed": len(points),
            }
        )
        return HeatmapAnswer(
            text=text,
            frame=fast.world_frame,
            voxel_size=self.voxel_size,
            centres=centres,
            scores=scores,
            cluster_of=cluster_of,
            clusters=clusters,
            pyramids=pyramids,
            stats=stats,
            seconds=time.monotonic() - started,
        )

    def _refine(
        self, text: str, result: FastResult, keep: NDArray[np.bool_]
    ) -> tuple[NDArray[np.int64], NDArray[np.float32], list[Cluster], NDArray[np.int16]] | None:
        """Hyperspace's refinement (occupancy, support, size prior, components)
        over the fast map; None when it is switched off or keeps nothing."""
        from dimos.mapping.hyperspace import patches as hs, refine as rf

        fast = self._fast
        assert fast is not None
        config = rf.refine_config_of(
            self.refine, getattr(fast.config, "refine", ""), cutoff=SCORE_CUTOFF
        )
        if config is None or not keep.any():
            return None
        # Occupancy always comes out of the dense chain. _query applies it on sparse keys
        # when there is a scene, and the dense method is handed `scene=[]` below, on which
        # it keeps NOTHING -- so leaving it in for the no-scene case, which is what this
        # used to do to avoid grounding an answer nowhere, silently deleted the entire
        # answer instead. With no scene nothing can ground anything; _load_scene says so
        # when that happens.
        config = replace(config, methods=[m for m in config.methods if m != "occupancy"])
        if int(keep.sum()) > REFINE_MAX_VOXELS:
            ranked = np.flatnonzero(keep)
            ranked = ranked[np.argsort(-result.score[ranked])[:REFINE_MAX_VOXELS]]
            keep = np.zeros_like(keep)
            keep[ranked] = True
        index = result.index[keep]
        span = index.max(axis=0) - index.min(axis=0) + 1  # in voxels; refine grids the box
        extent = span * self.voxel_size
        if float(extent.max()) > REFINE_MAX_EXTENT_M or float(np.prod(span)) > REFINE_MAX_CELLS:
            logger.info(
                "refine skipped: heat spans %.0f m, %.0f M cells (limits %.0f m, %.0f M); "
                "sparse components instead",
                float(extent.max()),
                float(np.prod(span)) / 1e6,
                REFINE_MAX_EXTENT_M,
                REFINE_MAX_CELLS / 1e6,
            )
            return None
        heat = hs.Heatmap(
            frame=self.world_frame,
            voxel_size=self.voxel_size,
            voxels=[
                (tuple(int(v) for v in ijk), float(sc))
                for ijk, sc in zip(index, result.score[keep], strict=True)
            ],
            stats={},
            support={
                tuple(int(v) for v in ijk): (int(f), int(b))
                for ijk, f, b in zip(index, result.frames[keep], result.bins[keep], strict=True)
            },
        )
        scene: list[tuple[int, int, int]] = []  # occupancy was already applied on sparse keys
        refined = rf.refine(heat, config, scene=scene, text=text)
        if len(refined.clusters) < 2 and config.min_frames > 1:
            # A sparse ingest (one keyframe per voxel) gives "support" nothing to count and
            # the answer collapses to nothing or a single blob; the rest of the chain
            # (occupancy, size prior, merge) still shapes the places. Decided per question,
            # so an answer never depends on what was asked before it.
            refined = rf.refine(heat, replace(config, min_frames=1), scene=scene, text=text)
        if not refined.voxels or not refined.clusters:
            return None
        # Best first, and no more places than the viewer steps through (a broad
        # question in a shop can return 70+ blobs; the rest stay as dim heat).
        ranked = sorted(refined.clusters, key=lambda c: c.rank)[:CLUSTER_POOL]
        rank_to_index = {c.rank: i for i, c in enumerate(ranked)}
        score_of = dict(refined.voxels)
        out_index = np.asarray([ijk for ijk, _ in refined.voxels], dtype=np.int64).reshape(-1, 3)
        out_score = np.asarray([sc for _, sc in refined.voxels], dtype=np.float32)
        cluster_of = np.asarray(
            [rank_to_index.get(refined.cluster_of.get(ijk, -1), -1) for ijk, _ in refined.voxels],
            dtype=np.int16,
        )
        clusters = [
            Cluster(
                index=i,
                centre=tuple(float(v) for v in c.centre),  # type: ignore[arg-type]
                radius=max(float(max(c.extent)) / 2, self.voxel_size / 2),
                score=float(c.score),
                peak=float(score_of.get(tuple(c.peak), out_score.max() if len(out_score) else 1.0)),
                n_voxels=int(c.voxels),
            )
            for i, c in enumerate(ranked)
        ]
        return out_index, out_score, clusters, cluster_of


def _hits_of(fast: FastQuery, result: FastResult, wanted: NDArray[np.int64]) -> dict[int, Evidence]:
    """Evidence for the hits numbered in *wanted*: camera patches first, then
    segment cells, numbered the way ``patch_points`` + ``segment_points`` concatenate."""
    out: dict[int, Evidence] = {}
    base = 0
    for frames, patches, points, channel in (
        (fast.patches.frames, result.patches, result.patch_points, "patches"),
        (
            fast.segments.frames if fast.segments else None,
            result.segments,
            result.segment_points,
            "segments",
        ),
    ):
        if frames is None or len(patches) == 0:
            continue
        local = wanted[(wanted >= base) & (wanted < base + len(patches))] - base
        if len(local):
            u0, u1, v0, v1 = patch_rects(frames, patches)
            for i in local:
                f = int(patches.frame[i])
                out[base + int(i)] = Evidence(
                    keyframe_id=int(frames.ids[f]),
                    camera_frame=frames.camera_frame[f],
                    ts=float(frames.ts[f]),
                    pose=frames.poses[f],
                    score=float(patches.score[i]),
                    image_uv=(float((u0[i] + u1[i]) / 2), float((v0[i] + v1[i]) / 2)),
                    point=tuple(float(v) for v in points[i]),  # type: ignore[arg-type]
                    channel=channel,
                )
        base += len(patches)
    return out


def viewpoints_of(hits: list[Evidence]) -> int:
    """How many distinct viewpoints saw a place: a camera at a moment, not a record.

    Segments carry their own synthetic ids, so counting ids made one camera looking once --
    a patch hit and three segment hits from the same frame -- read as four viewpoints, and
    that place then outranked one seen in two real photographs.

    Named rather than inline so a test can call THIS, instead of a copy of the expression
    in a test file, which is a test of itself.
    """
    return len({(hit.camera_frame, hit.ts) for hit in hits})


def _pick_evidence(members: list[Evidence]) -> list[Evidence]:
    """The best-scoring hit per distinct PICTURE, up to ``EVIDENCE_PER_CLUSTER``.

    Keyed the same way :func:`viewpoints_of` counts, and for the same reason: segments
    carry their own synthetic ids, so one photograph that produced a patch hit and three
    segment cells looked like four keyframes here. `_publish_cluster_images` then fetches
    each by `ts`, so it encodes the SAME frame four times and hangs four coincident planes
    in the world -- and a place genuinely seen from eight viewpoints can spend six of its
    eight slots on copies of one picture.
    """
    members = sorted(members, key=lambda e: -e.score)
    out: list[Evidence] = []
    seen: set[tuple[str, float]] = set()
    for evidence in members:
        picture = (evidence.camera_frame, evidence.ts)
        if picture in seen:
            continue
        seen.add(picture)
        out.append(evidence)
        if len(out) >= EVIDENCE_PER_CLUSTER:
            break
    return out


def _pyramids(
    frames: Frames, patches: Patches, owner: NDArray[np.int64], config: Any
) -> list[Pyramid]:
    """The frusta of the best ``MAX_PYRAMIDS`` camera patches, corners in the world frame."""
    usable = np.flatnonzero(np.isfinite(patches.depth) & (patches.depth > 0))
    if len(usable) == 0:
        return []
    # NaN corners would make the JSON unreadable; a patch without depth has no pyramid.
    order = usable[np.argsort(-patches.score[usable])[:MAX_PYRAMIDS]]
    sub = Patches(
        patches.frame[order], patches.cell[order], patches.depth[order], patches.score[order]
    )
    u0, u1, v0, v1 = patch_rects(frames, sub)
    rep = np.repeat(np.arange(len(sub)), 4)
    us = np.stack([u0, u1, u1, u0], axis=1).ravel()
    vs = np.stack([v0, v0, v1, v1], axis=1).ravel()
    near = project_pixels(
        frames, sub.frame[rep], us, vs, np.repeat(sub.depth * config.cap_near, 4)
    ).reshape(-1, 4, 3)
    far = project_pixels(
        frames, sub.frame[rep], us, vs, np.repeat(sub.depth * config.cap_far, 4)
    ).reshape(-1, 4, 3)
    out: list[Pyramid] = []
    for i, p in enumerate(order):
        f = int(sub.frame[i])
        out.append(
            Pyramid(
                apex=tuple(float(v) for v in frames.poses[f][:3, 3]),  # type: ignore[arg-type]
                near=[tuple(float(v) for v in c) for c in near[i]],  # type: ignore[misc]
                far=[tuple(float(v) for v in c) for c in far[i]],  # type: ignore[misc]
                score=float(sub.score[i]),
                keyframe_id=int(frames.ids[f]),
                cluster=int(owner[p]),
            )
        )
    return out
