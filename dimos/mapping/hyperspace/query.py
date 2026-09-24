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

from collections.abc import Sequence
import math
import sqlite3
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.mapping.hyperspace import patches as hs
from dimos.mapping.hyperspace.ingest import (
    TF_STREAM,
    patch_stream_for,
    stream_names,
    transform_to_matrix,
)
from dimos.mapping.hyperspace.refine import RefineConfig, refine
from dimos.models.embedding.base import Embedding
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable

    from numpy.typing import NDArray

    from dimos.memory.store.base import Store

# sqlite-vec refuses knn queries with k above this.
VEC0_MAX_K = 4096


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
        slug: str = "",
    ) -> None:
        self.store = store
        # Which index in the recording to answer from: "" is the canonical one. A
        # recording can hold several, one per model, to compare them on one question.
        self.slug = slug
        self.keyframe_stream, _ = stream_names(slug)
        self.embed_text = embed_text
        self.config = config
        # None = raw map. Set (or pass) to get ranked clusters; see refine.py.
        self.refine_config = refine_config
        self._scene: tuple[int, list[tuple[int, int, int]]] | None = None
        self._query_text = ""
        self.world_frame = world_frame
        self.voxel_size = voxel_size
        self._backgrounds: list[NDArray[np.float32]] | None = None
        self._keyframes: dict[int, tuple[hs.Keyframe, NDArray[np.float16]]] = {}
        # Ensemble stores: per keyframe id, the members' grids (primary first)
        # with their shapes; empty for single-grid stores.
        self._member_grids: dict[int, list[tuple[NDArray[np.float16], tuple[int, int]]]] = {}
        self._members: list[str] = []
        # The recorded transforms, and whatever is published while we run: the
        # live module hands them to the same buffer (see Hyperspace.handle_tf).
        # Unbounded, because a query reaches back to the first keyframe.
        self.tf = MultiTBuffer(buffer_size=math.inf)
        self._tf_last_id = -1

    def patch_stream(self, member: str = "") -> str:
        """The vec0 stream of one model's patch vectors, named after that model.

        Defaults to the primary member, which is the one a single-grid store answers
        from. The keyframes must be loaded first: the member names come off them.
        """
        return patch_stream_for(self.slug, member or (self._members[0] if self._members else ""))

    def keyframe(self, keyframe_id: int) -> tuple[hs.Keyframe, NDArray[np.float16]] | None:
        """A keyframe and its patch grid. The first miss loads every keyframe in
        one pass: grids are ~1.3 MB each and a per-id scan would unpickle all
        of them for every hit."""
        if keyframe_id not in self._keyframes:
            for obs in self.store.stream(self.keyframe_stream, dict).order_by("ts"):
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
                # Provenance is on every keyframe, ensemble or not, and it names the
                # vec0 stream holding this model's vectors.
                self._members = list(payload.get("members", self._members))
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
                self.store.stream(self.patch_stream(), dict)
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
            hot.append(hs.HotPatch(keyframe=keyframe, patch=index, score=contrast))
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
        hot: list[hs.HotPatch] = []
        searched = 0
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
                hot.append(
                    hs.HotPatch(keyframe=keyframe, patch=int(index), score=float(pooled[index]))
                )
        hot.sort(key=lambda h: -h.score)
        return hot[: self.config.max_hot_patches], searched

    def read_tf(self) -> None:
        """Take in the transforms written since the last pass. Newest id first,
        stopping at the last one seen, so an unchanging recording costs one row
        -- this used to re-scan every tf observation on every query."""
        if TF_STREAM not in self.store.list_streams():
            return
        stream = self.store.stream(TF_STREAM, TFMessage)
        batch: Iterable[Any]
        if self._tf_last_id < 0:
            batch = stream.order_by("ts")  # first pass: read it all, in order
        else:
            tail = []
            for obs in stream.order_by("id", desc=True):
                if obs.id <= self._tf_last_id:
                    break
                tail.append(obs)
            if not tail:
                return
            batch = sorted(tail, key=lambda o: o.ts)
        for obs in batch:
            self._tf_last_id = max(self._tf_last_id, obs.id)
            self.tf.receive_tfmessage(obs.data)

    def placer(self, target: str) -> Callable[[hs.Keyframe], NDArray[np.float64] | None]:
        self.read_tf()

        def place(keyframe: hs.Keyframe) -> NDArray[np.float64] | None:
            transform = self.tf.get(target, keyframe.camera_frame, keyframe.ts, warn=False)
            return None if transform is None else transform_to_matrix(transform)

        return place

    def heatmap(
        self,
        text: str,
        frame: str | None = None,
        background_prompts: Sequence[str] | None = None,
    ) -> hs.Heatmap:
        """Where the map looks like *text*, as scored voxels.

        *background_prompts* replaces the contrast for this one call. An area query wants
        to be contrasted against objects rather than against a room, since the usual
        floor/wall/ceiling set would subtract the very thing it is looking for. Restored
        afterwards, so one odd query does not change what the next one means.
        """
        if background_prompts is not None:
            kept, self.config.background_prompts = (
                self.config.background_prompts,
                list(background_prompts),
            )
            self._backgrounds = None
            try:
                return self.heatmap(text, frame)
            finally:
                self.config.background_prompts = kept
                self._backgrounds = None
        target = frame or self.world_frame
        queries = self.text_vectors(text)
        self._query_text = text
        hot, searched = self.hot_patches(queries)
        place = self.placer(target)
        result = hs.heatmap(hot, place, target, self.voxel_size, self.config)
        result.stats["patches_searched"] = searched
        if self.refine_config is not None:
            methods = self.refine_config.methods
            scene = self.scene_indices(target) if "occupancy" in methods else None
            result = refine(result, self.refine_config, scene=scene, text=text)
        return result

    def scene_indices(self, frame: str) -> list[tuple[int, int, int]]:
        """Occupied voxel indices, kept until the keyframe count changes."""
        count = self.store.stream(self.keyframe_stream, dict).count()
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
        for obs in self.store.stream(self.keyframe_stream, dict).order_by("ts"):
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
