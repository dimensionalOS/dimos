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

"""Text in, scored voxels out, against a memory store that
:class:`~dimos.mapping.hyperspace.ingest.PatchIngestor` filled.

Shared by the live ``Hyperspace`` module and the offline CLI.
"""

from __future__ import annotations

import math
import sqlite3
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.mapping.hyperspace import patches as hs
from dimos.mapping.hyperspace.ingest import (
    KEYFRAME_STREAM,
    PATCH_STREAM,
    TF_STREAM,
    transform_to_matrix,
)
from dimos.mapping.hyperspace.refine import STRUCTURAL_LABELS, RefineConfig, refine
from dimos.mapping.hyperspace.segments import SEGMENT_STREAM
from dimos.models.embedding.base import Embedding
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable

    from numpy.typing import NDArray

    from dimos.memory.store.base import Store

# sqlite-vec refuses knn queries with k above this.
VEC0_MAX_K = 4096


class TfCache:
    """The recorded tf, decoded once and kept, where a transform written later
    for the same (parent, child, stamp) replaces the earlier one.

    That replacement is how a loop closure rewrites the past: keyframes store
    no pose, so re-publishing corrected transforms moves every answer that
    depends on them. New observations are picked up incrementally; a
    replacement rebuilds the buffer from the surviving transforms.
    """

    def __init__(self, store: Store, stream: str = TF_STREAM) -> None:
        self.store = store
        self.stream_name = stream
        self.buffer = MultiTBuffer(buffer_size=math.inf)
        self.latest: dict[tuple[str, str, float], Any] = {}
        self.last_id = -1

    def update(self) -> None:
        if self.stream_name not in self.store.list_streams():
            return
        # Newest id first, stopping at the last one seen, so a pass over an
        # unchanging recording reads one row rather than the whole stream --
        # answering a query used to re-scan every tf observation. Ids are
        # assigned in write order, so a transform republished for an old stamp
        # still arrives here (that is how a loop closure rewrites the past).
        stream = self.store.stream(self.stream_name, TFMessage)
        batch: Iterable[Any]
        if self.last_id < 0:
            batch = stream.order_by("ts")  # first pass: read it all, in order
        else:
            tail = []
            for obs in stream.order_by("id", desc=True):
                if obs.id <= self.last_id:
                    break
                tail.append(obs)
            if not tail:
                return
            batch = sorted(tail, key=lambda o: o.ts)
        fresh, replaced = [], False
        for obs in batch:
            self.last_id = max(self.last_id, obs.id)
            for transform in obs.data.transforms:
                key = (transform.frame_id, transform.child_frame_id, float(transform.ts))
                if key in self.latest:
                    replaced = True
                self.latest[key] = transform
                fresh.append(transform)
        if replaced:
            self.buffer = MultiTBuffer(buffer_size=math.inf)
            self.buffer.receive_transform(*self.latest.values())
        elif fresh:
            self.buffer.receive_transform(*fresh)

    def get(self, target: str, source: str, ts: float) -> NDArray[np.float64] | None:
        transform = self.buffer.get(target, source, ts, warn=False)
        return None if transform is None else transform_to_matrix(transform)


class HyperspaceQuery:
    """Search the store's patch vectors, place the hot ones as pyramids through
    the recorded tf, rasterize and pool them into a voxel heat map."""

    def __init__(
        self,
        store: Store,
        embed_text: Callable[[str], NDArray[np.float32]],
        config: hs.QueryConfig,
        world_frame: str = "odom",
        voxel_size: float = 0.1,
        refine_config: RefineConfig | None = None,
    ) -> None:
        self.store = store
        self.embed_text = embed_text
        self.config = config
        # None = raw map. Set (or pass) to get ranked clusters; see refine.py.
        self.refine_config = refine_config
        self._scene: tuple[int, list[tuple[int, int, int]]] | None = None
        self._surfaces: tuple[int, list[tuple[int, int, int]]] | None = None
        self._structural: tuple[int, dict[str, list[tuple[float, set[int]]]]] | None = None
        self._query_text = ""
        self._last_gated = 0
        self.world_frame = world_frame
        self.voxel_size = voxel_size
        self._backgrounds: list[NDArray[np.float32]] | None = None
        self._keyframes: dict[int, tuple[hs.Keyframe, NDArray[np.float16]]] = {}
        # Ensemble stores: per keyframe id, the members' grids (primary first)
        # with their shapes; empty for single-grid stores.
        self._member_grids: dict[int, list[tuple[NDArray[np.float16], tuple[int, int]]]] = {}
        self._members: list[str] = []
        self._labels: dict[str, NDArray[np.float32]] = {}
        self._labels_last_id = -1
        self.tf = TfCache(store)

    def keyframe(self, keyframe_id: int) -> tuple[hs.Keyframe, NDArray[np.float16]] | None:
        """A keyframe and its patch grid. The first miss loads every keyframe in
        one pass: grids are ~1.3 MB each and a per-id scan would unpickle all
        of them for every hit."""
        if keyframe_id not in self._keyframes:
            for obs in self.store.stream(KEYFRAME_STREAM, dict).order_by("ts"):
                if obs.id in self._keyframes:
                    continue
                payload = obs.data
                keyframe = hs.Keyframe(
                    id=obs.id,
                    camera_frame=payload["camera_frame"],
                    ts=float(payload["ts"]),
                    rows=int(payload["rows"]),
                    cols=int(payload["cols"]),
                    intrinsics=hs.Intrinsics(**payload["intrinsics"]),
                    patch_depth=np.asarray(payload["patch_depth"], dtype=np.float32),
                )
                self._keyframes[obs.id] = (keyframe, np.asarray(payload["grid"], dtype=np.float16))
                if "grids" in payload:
                    self._member_grids[obs.id] = [
                        (np.asarray(grid, dtype=np.float16), (int(shape[0]), int(shape[1])))
                        for grid, shape in zip(
                            payload["grids"], payload["grid_shapes"], strict=True
                        )
                    ]
                    self._members = list(payload.get("members", self._members))
        return self._keyframes.get(keyframe_id)

    def members(self) -> list[str]:
        """The ensemble members the store was written with (tags, primary
        first); empty for a single-grid store."""
        self.keyframe(-1)
        return self._members

    def text_vectors(self, text: str) -> list[NDArray[np.float32]]:
        """One unit vector per ensemble member (a single one for a single model)."""
        vectors = self.embed_text(text)
        if isinstance(vectors, np.ndarray) and vectors.ndim == 1:
            vectors = [vectors]
        return [np.asarray(v, dtype=np.float32) for v in vectors]

    def backgrounds(self) -> list[NDArray[np.float32]]:
        """Per member, the background prompt vectors as a ``[prompts, dim]`` array."""
        if self._backgrounds is None:
            per_prompt = [self.text_vectors(prompt) for prompt in self.config.background_prompts]
            if not per_prompt:
                self._backgrounds = [np.zeros((0, 1), np.float32)]
            else:
                self._backgrounds = [
                    np.stack([vectors[m] for vectors in per_prompt]).astype(np.float32)
                    for m in range(len(per_prompt[0]))
                ]
        return self._backgrounds

    def _relevant_backgrounds(self, member: int, query: NDArray[np.float32]) -> NDArray[np.float32]:
        backgrounds = self.backgrounds()[min(member, len(self.backgrounds()) - 1)]
        if len(backgrounds):
            backgrounds = backgrounds[(backgrounds @ query) < self.config.background_synonym_cutoff]
        return backgrounds

    def hot_patches(self, queries: list[NDArray[np.float32]]) -> tuple[list[hs.HotPatch], int]:
        """Patches whose query score beats their best background prompt by the
        hot threshold, one text vector per ensemble member. Returns (hot
        patches, patches searched). An ensemble store scores every keyframe's
        grids directly; a single-grid store goes through the vector index."""
        self.keyframe(-1)
        if self._member_grids:
            return self.pooled_hot_patches(queries)
        query = queries[0]
        backgrounds = self._relevant_backgrounds(0, query)
        try:
            hits = (
                self.store.stream(PATCH_STREAM, dict)
                .search(Embedding(vector=query), k=min(self.config.max_hot_patches, VEC0_MAX_K))
                .to_list()
            )
        except sqlite3.OperationalError as error:
            if "imension" not in str(error):
                raise
            # A store written before member specs were recorded, read back with
            # a differently shaped model: say so instead of leaking sqlite-vec's
            # "expected 1152 received 768".
            raise ValueError(
                f"{error}: this store was embedded with differently shaped "
                "checkpoints than the ones querying it. Pass the checkpoints it "
                "was written with (--models), or re-embed it (--no-reuse)."
            ) from error
        hot: list[hs.HotPatch] = []
        gate = self.structural_cells() if self.config.structural_gate else None
        exempt = any(label in self._query_text.lower() for label in STRUCTURAL_LABELS)
        gated = 0
        for hit in hits:
            entry = self.keyframe(int(hit.data["keyframe"]))
            if entry is None:
                continue
            keyframe, grid = entry
            index = int(hit.data["patch"])
            vector = grid[index].astype(np.float32)
            background = float((backgrounds @ vector).max()) if len(backgrounds) else 0.0
            contrast = float(vector @ query) - background
            if contrast <= self.config.hot_threshold:
                continue
            if gate is not None and not exempt and self.is_structural(gate, keyframe, index):
                gated += 1
                continue
            hot.append(hs.HotPatch(keyframe=keyframe, patch=index, score=contrast))
        self._last_gated = gated
        return hot, len(hits)

    def pooled_hot_patches(
        self, queries: list[NDArray[np.float32]]
    ) -> tuple[list[hs.HotPatch], int]:
        """Every keyframe, every member: contrast on the member's own grid,
        spread onto the keyframe's cell grid, then pooled across members
        (``config.pool``); cells above ``pooled_hot_threshold`` are hot."""
        members = len(next(iter(self._member_grids.values())))
        if len(queries) != members:
            raise ValueError(
                f"the store was written with {members} ensemble members {self._members} "
                f"but the query side embeds text with {len(queries)}"
            )
        backgrounds = [self._relevant_backgrounds(m, q) for m, q in enumerate(queries)]
        gate = self.structural_cells() if self.config.structural_gate else None
        exempt = any(label in self._query_text.lower() for label in STRUCTURAL_LABELS)
        hot: list[hs.HotPatch] = []
        searched = gated = 0
        for keyframe_id, grids in self._member_grids.items():
            keyframe, _ = self._keyframes[keyframe_id]
            cells = (keyframe.rows, keyframe.cols)
            contrasts = []
            for (grid, shape), query, background in zip(grids, queries, backgrounds, strict=True):
                vectors = grid.astype(np.float32)
                searched += len(vectors)
                contrast = vectors @ query
                if len(background):
                    contrast -= (background @ vectors.T).max(axis=0)
                contrasts.append(hs.cell_matrix(shape, cells) @ contrast)
            pooled = hs.pool_cells(contrasts, self.config.pool)
            for index in np.flatnonzero(pooled > self.config.pooled_hot_threshold):
                if (
                    gate is not None
                    and not exempt
                    and self.is_structural(gate, keyframe, int(index))
                ):
                    gated += 1
                    continue
                hot.append(
                    hs.HotPatch(keyframe=keyframe, patch=int(index), score=float(pooled[index]))
                )
        self._last_gated = gated
        hot.sort(key=lambda h: -h.score)
        return hot[: self.config.max_hot_patches], searched

    def structural_cells(self) -> dict[str, list[tuple[float, set[int]]]]:
        """Per camera frame, the (ts, grid cells) the segmenter labelled
        floor/wall/ceiling, sorted by ts; rebuilt when the segment count
        changes."""
        if SEGMENT_STREAM not in self.store.list_streams():
            return {}
        count = self.store.stream(SEGMENT_STREAM, dict).count()
        if self._structural is not None and self._structural[0] == count:
            return self._structural[1]
        frames: dict[tuple[str, float], set[int]] = {}
        for label in STRUCTURAL_LABELS:
            for hit in self.store.stream(SEGMENT_STREAM, dict).tags(name=label).order_by("ts"):
                record = hit.data
                cells = frames.setdefault((record["camera_frame"], float(record["ts"])), set())
                cells.update(
                    int(index)
                    for index, coverage, _ in record.get("cells", [])
                    if coverage >= self.config.structural_gate_coverage
                )
        by_camera: dict[str, list[tuple[float, set[int]]]] = {}
        for (camera, ts), cells in sorted(frames.items()):
            by_camera.setdefault(camera, []).append((ts, cells))
        self._structural = (count, by_camera)
        return by_camera

    def is_structural(
        self, gate: dict[str, list[tuple[float, set[int]]]], keyframe: hs.Keyframe, index: int
    ) -> bool:
        """Whether the nearest segment frame (same camera, within
        structural_gate_dt) labelled this patch's cell floor/wall/ceiling."""
        frames = gate.get(keyframe.camera_frame)
        if not frames:
            return False
        stamps = [ts for ts, _ in frames]
        position = int(np.searchsorted(stamps, keyframe.ts))
        candidates = [i for i in (position - 1, position) if 0 <= i < len(frames)]
        nearest = min(candidates, key=lambda i: abs(stamps[i] - keyframe.ts))
        if abs(stamps[nearest] - keyframe.ts) > self.config.structural_gate_dt:
            return False
        return index in frames[nearest][1]

    def labels(self) -> dict[str, NDArray[np.float32]]:
        """Every label the segments carry, with its text embedding. Read from
        the tags alone (payloads stay unread); new labels are picked up as
        they appear."""
        if SEGMENT_STREAM not in self.store.list_streams():
            return self._labels
        for obs in self.store.stream(SEGMENT_STREAM, dict).order_by("ts"):
            if obs.id <= self._labels_last_id:
                continue
            self._labels_last_id = max(self._labels_last_id, obs.id)
            name = obs.tags.get("name")
            if name and name not in self._labels:
                self._labels[name] = self.text_vectors(name)[0]
        return self._labels

    def label_scores(self, query: NDArray[np.float32]) -> dict[str, float]:
        """Word score per label: its z-scored cosine to the query, 0 at
        ``segment_min_z`` and 1 at twice that; only the labels scoring > 0."""
        labels = self.labels()
        if not labels:
            return {}
        names = list(labels)
        cosine = np.stack([labels[n] for n in names]) @ query
        z = (cosine - cosine.mean()) / max(float(cosine.std()), 1e-6)
        floor = max(self.config.segment_min_z, 1e-6)
        word = np.clip((z - floor) / floor, 0.0, 1.0)
        return {n: float(w) for n, w in zip(names, word, strict=True) if w > 0}

    def hot_segments(self, query: NDArray[np.float32]) -> tuple[list[hs.HotPatch], int]:
        """Segments whose label matches the query, as hot patches: one per grid
        cell the segment covers, scored word x confidence x coverage. Returns
        (hot patches, segment records read)."""
        if self.config.segment_weight <= 0:
            return [], 0
        hot: list[hs.HotPatch] = []
        read = 0
        for name, word in sorted(self.label_scores(query).items(), key=lambda kv: -kv[1]):
            for hit in self.store.stream(SEGMENT_STREAM, dict).tags(name=name).order_by("ts"):
                if read >= self.config.max_hot_segments:
                    return hot, read
                read += 1
                record = hit.data
                keyframe = self.segment_keyframe(hit.id, record)
                if keyframe is None:
                    continue
                weight = word * float(record["confidence"])
                for index, coverage, _ in record["cells"]:
                    hot.append(
                        hs.HotPatch(keyframe=keyframe, patch=int(index), score=weight * coverage)
                    )
        return hot, read

    def placer(self, target: str) -> Callable[[hs.Keyframe], NDArray[np.float64] | None]:
        self.tf.update()
        return lambda keyframe: self.tf.get(target, keyframe.camera_frame, keyframe.ts)

    def heatmap(self, text: str, frame: str | None = None) -> hs.Heatmap:
        target = frame or self.world_frame
        queries = self.text_vectors(text)
        # The segment channel compares label text to query text within one
        # tower: the primary member's.
        query = queries[0]
        self._query_text = text
        hot, searched = self.hot_patches(queries)
        place = self.placer(target)
        result = hs.heatmap(hot, place, target, self.voxel_size, self.config)
        result.stats["patches_searched"] = searched
        result.stats["patches_gated"] = self._last_gated
        hot_segments, segments_read = self.hot_segments(query)
        if self.config.segment_weight > 0 and SEGMENT_STREAM in self.store.list_streams():
            segments = hs.heatmap(hot_segments, place, target, self.voxel_size, self.config)
            segments.stats["read"] = segments_read
            segments.stats["labels"] = sorted(self.label_scores(query), key=str)
            result = hs.combine(result, segments, self.config)
        if self.refine_config is not None:
            methods = self.refine_config.methods
            scene = self.scene_indices(target) if "occupancy" in methods else None
            surfaces = self.surface_indices(target) if "structural" in methods else None
            result = refine(result, self.refine_config, scene=scene, surfaces=surfaces, text=text)
        return result

    def surface_indices(self, frame: str) -> list[tuple[int, int, int]]:
        """Voxels on floor/wall/ceiling segment surfaces, rasterized once per
        segment count (the segment records already carry cells + depth)."""
        if SEGMENT_STREAM not in self.store.list_streams():
            return []
        count = self.store.stream(SEGMENT_STREAM, dict).count()
        if self._surfaces is not None and self._surfaces[0] == count:
            return self._surfaces[1]
        place = self.placer(frame)
        voxels: set[tuple[int, int, int]] = set()
        for label in STRUCTURAL_LABELS:
            for hit in self.store.stream(SEGMENT_STREAM, dict).tags(name=label).order_by("ts"):
                keyframe = self.segment_keyframe(hit.id, hit.data)
                if keyframe is None:
                    continue
                pose = place(keyframe)
                if pose is None:
                    continue
                for index, _, depth in hit.data["cells"]:
                    if not (depth > 0) or not math.isfinite(depth):
                        continue
                    patch = hs.HotPatch(keyframe=keyframe, patch=int(index), score=1.0)
                    for voxel, _ in hs.rasterize_pyramid(patch, pose, self.voxel_size, self.config):
                        voxels.add(voxel)
        self._surfaces = (count, sorted(voxels))
        return self._surfaces[1]

    @staticmethod
    def segment_keyframe(hit_id: int, record: dict[str, Any]) -> hs.Keyframe | None:
        """A segment record as a placeable keyframe (negative id, its cells'
        depth as patch depth), or None when it has no cells or camera."""
        if not record.get("cells") or not record.get("intrinsics"):
            return None
        rows, cols = int(record["rows"]), int(record["cols"])
        patch_depth = np.full(rows * cols, np.nan, dtype=np.float32)
        for index, _, depth in record["cells"]:
            patch_depth[int(index)] = depth
        # Negative ids keep segment frames apart from keyframes in the pool.
        return hs.Keyframe(
            id=-(hit_id + 1),
            camera_frame=record["camera_frame"],
            ts=float(record["ts"]),
            rows=rows,
            cols=cols,
            intrinsics=hs.Intrinsics(**record["intrinsics"]),
            patch_depth=patch_depth,
        )

    def scene_indices(self, frame: str) -> list[tuple[int, int, int]]:
        """Occupied voxel indices, kept until the keyframe count changes."""
        count = self.store.stream(KEYFRAME_STREAM, dict).count()
        if self._scene is None or self._scene[0] != count:
            self._scene = (count, [index for index, _ in self.scene_voxels(frame)])
        return self._scene[1]

    def scene_voxels(
        self, frame: str | None = None, min_samples: int = 3
    ) -> list[tuple[tuple[int, int, int], int]]:
        """Occupied voxels from the keyframes' depth thumbnails, placed through tf."""
        target = frame or self.world_frame
        place = self.placer(target)
        counts: dict[tuple[int, int, int], int] = {}
        for obs in self.store.stream(KEYFRAME_STREAM, dict).order_by("ts"):
            payload = obs.data
            thumbnail = np.asarray(payload["thumbnail_mm"])
            if thumbnail.size == 0:
                continue
            keyframe = hs.Keyframe(
                id=obs.id,
                camera_frame=payload["camera_frame"],
                ts=float(payload["ts"]),
                rows=int(payload["rows"]),
                cols=int(payload["cols"]),
                intrinsics=hs.Intrinsics(**payload["intrinsics"]),
                patch_depth=np.zeros(0, np.float32),
            )
            pose = place(keyframe)
            if pose is None:
                continue
            camera = keyframe.intrinsics
            stride = int(payload["thumbnail_stride"])
            vs, us = np.nonzero(thumbnail)
            z = thumbnail[vs, us].astype(np.float64) * 0.001
            local = np.stack(
                [
                    (us * stride - camera.cx) / camera.fx * z,
                    (vs * stride - camera.cy) / camera.fy * z,
                    z,
                    np.ones_like(z),
                ]
            )
            world = (pose @ local)[:3].T
            for index in map(tuple, np.floor(world / self.voxel_size).astype(int)):
                counts[index] = counts.get(index, 0) + 1
        return [(index, n) for index, n in counts.items() if n >= min_samples]

    def answer(
        self, text: str, request_id: int, frame: str | None = None, top: int = 10
    ) -> dict[str, Any]:
        """The heat map plus the JSON-able summary the module publishes."""
        result = self.heatmap(text, frame)
        centres = result.centres()
        scores = result.scores()
        best = [
            {"xyz": [round(float(v), 3) for v in centres[i]], "score": round(float(scores[i]), 3)}
            for i in range(min(top, len(centres)))
        ]
        return {
            "id": request_id,
            "text": text,
            "frame": result.frame,
            "voxel_size": result.voxel_size,
            "voxels": len(result.voxels),
            "best": best,
            "stats": result.stats,
            "heatmap": result,
        }
