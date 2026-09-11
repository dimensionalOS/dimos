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
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.mapping.hyperspace import patches as hs
from dimos.mapping.hyperspace.ingest import (
    KEYFRAME_STREAM,
    PATCH_STREAM,
    TF_STREAM,
    transform_to_matrix,
)
from dimos.mapping.hyperspace.segments import SEGMENT_STREAM
from dimos.models.embedding.base import Embedding
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer

if TYPE_CHECKING:
    from collections.abc import Callable

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
        fresh, replaced = [], False
        # Observations past the last seen id are decoded; the rest are skipped
        # before their payload is touched, so this scan is cheap.
        for obs in self.store.stream(self.stream_name, TFMessage).order_by("ts"):
            if obs.id <= self.last_id:
                continue
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
    ) -> None:
        self.store = store
        self.embed_text = embed_text
        self.config = config
        self.world_frame = world_frame
        self.voxel_size = voxel_size
        self._backgrounds: NDArray[np.float32] | None = None
        self._keyframes: dict[int, tuple[hs.Keyframe, NDArray[np.float16]]] = {}
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
        return self._keyframes.get(keyframe_id)

    def backgrounds(self) -> NDArray[np.float32]:
        if self._backgrounds is None:
            vectors = [self.embed_text(prompt) for prompt in self.config.background_prompts]
            self._backgrounds = (
                np.stack(vectors).astype(np.float32) if vectors else np.zeros((0, 1), np.float32)
            )
        return self._backgrounds

    def hot_patches(self, query: NDArray[np.float32]) -> tuple[list[hs.HotPatch], int]:
        """Patches whose query score beats their best background prompt by the
        hot threshold. Returns (hot patches, patches searched)."""
        backgrounds = self.backgrounds()
        if len(backgrounds):
            backgrounds = backgrounds[(backgrounds @ query) < self.config.background_synonym_cutoff]
        hits = (
            self.store.stream(PATCH_STREAM, dict)
            .search(Embedding(vector=query), k=min(self.config.max_hot_patches, VEC0_MAX_K))
            .to_list()
        )
        hot: list[hs.HotPatch] = []
        for hit in hits:
            entry = self.keyframe(int(hit.data["keyframe"]))
            if entry is None:
                continue
            keyframe, grid = entry
            index = int(hit.data["patch"])
            vector = grid[index].astype(np.float32)
            background = float((backgrounds @ vector).max()) if len(backgrounds) else 0.0
            contrast = float(vector @ query) - background
            if contrast > self.config.hot_threshold:
                hot.append(hs.HotPatch(keyframe=keyframe, patch=index, score=contrast))
        return hot, len(hits)

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
                self._labels[name] = np.asarray(self.embed_text(name), dtype=np.float32)
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
                if not record.get("cells") or not record.get("intrinsics"):
                    continue
                rows, cols = int(record["rows"]), int(record["cols"])
                patch_depth = np.full(rows * cols, np.nan, dtype=np.float32)
                for index, _, depth in record["cells"]:
                    patch_depth[int(index)] = depth
                # Negative ids keep segment frames apart from keyframes in the pool.
                keyframe = hs.Keyframe(
                    id=-(hit.id + 1),
                    camera_frame=record["camera_frame"],
                    ts=float(record["ts"]),
                    rows=rows,
                    cols=cols,
                    intrinsics=hs.Intrinsics(**record["intrinsics"]),
                    patch_depth=patch_depth,
                )
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
        query = np.asarray(self.embed_text(text), dtype=np.float32)
        hot, searched = self.hot_patches(query)
        place = self.placer(target)
        result = hs.heatmap(hot, place, target, self.voxel_size, self.config)
        result.stats["patches_searched"] = searched
        hot_segments, segments_read = self.hot_segments(query)
        if self.config.segment_weight > 0 and SEGMENT_STREAM in self.store.list_streams():
            segments = hs.heatmap(hot_segments, place, target, self.voxel_size, self.config)
            segments.stats["read"] = segments_read
            segments.stats["labels"] = sorted(self.label_scores(query), key=str)
            result = hs.combine(result, segments, self.config)
        return result

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
