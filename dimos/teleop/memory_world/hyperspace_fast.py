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

"""Hyperspace's query, vectorized, for a recording that does not change.

:class:`dimos.mapping.hyperspace.query.HyperspaceQuery` answers in about a
second: the patch search goes through sqlite-vec (0.6 s), the tf stream is
re-scanned for every answer (0.5 s), and each hot pyramid is rasterized and
each voxel pooled in its own Python call. A recording is static, so this
module does the same arithmetic once over resident arrays:

* the patch grids the engine already holds, stacked into one fp16 matrix a
  torch matmul searches in ~6 ms;
* keyframe (and segment) poses looked up through tf once, at load;
* all hot pyramids rasterized in one batch, pooled with sorted reductions.

Results match :func:`patches.rasterize_pyramid` / :func:`patches.pool`
voxel for voxel (see ``test_hyperspace_fast.py``); the one intended
difference is that the top-k pre-filter runs over every patch instead of
sqlite-vec's approximate index.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import itertools
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

VEC0_MAX_K = 4096
SEGMENT_STREAM = "hyperspace_segments"
# Voxel indices are packed into one int64 key; this many voxels each side of
# the origin (105 km at 10 cm) fit.
_KEY_HALF = 1 << 20
_KEY_SPAN = 1 << 21


@dataclass
class Frames:
    """Cameras that took patches: pose and intrinsics per frame, patches by frame."""

    ids: NDArray[np.int64]  # frame id per frame (keyframe id, or -(segment id + 1))
    poses: NDArray[np.float64]  # (F, 4, 4) target_from_camera
    width: NDArray[np.float64]
    height: NDArray[np.float64]
    fx: NDArray[np.float64]
    fy: NDArray[np.float64]
    cx: NDArray[np.float64]
    cy: NDArray[np.float64]
    rows: NDArray[np.int64]
    cols: NDArray[np.int64]
    # Extra per-frame facts the viewer wants back (camera frame, stamp).
    camera_frame: list[str]
    ts: NDArray[np.float64]

    def __len__(self) -> int:
        return len(self.ids)


@dataclass
class Patches:
    """Patches to rasterize: which frame, which cell, its depth and score."""

    frame: NDArray[np.int64]  # index into Frames
    cell: NDArray[np.int64]
    depth: NDArray[np.float64]
    score: NDArray[np.float64]

    def __len__(self) -> int:
        return len(self.frame)


@dataclass
class Rasterized:
    """Voxel evidence: one row per (voxel, patch) the pyramid covered."""

    index: NDArray[np.int64]  # (M, 3)
    frame_id: NDArray[np.int64]
    score: NDArray[np.float64]
    yaw_bin: NDArray[np.int64]


@dataclass
class Pooled:
    index: NDArray[np.int64]  # (V, 3) unique voxels
    score: NDArray[np.float64]  # pooled, unnormalized
    # Support per voxel: frames that saw it, distinct yaw bins (Hyperspace's refine uses both).
    frames: NDArray[np.int64] | None = None
    bins: NDArray[np.int64] | None = None

    def normalized(self, percentile: float) -> Pooled:
        return Pooled(self.index, normalize_scores(self.score, percentile), self.frames, self.bins)


def pack_keys(index: NDArray[np.integer]) -> NDArray[np.int64]:
    index = np.asarray(index, dtype=np.int64) + _KEY_HALF
    return (index[:, 0] * _KEY_SPAN + index[:, 1]) * _KEY_SPAN + index[:, 2]


def unpack_keys(keys: NDArray[np.int64]) -> NDArray[np.int64]:
    z = keys % _KEY_SPAN
    rest = keys // _KEY_SPAN
    y = rest % _KEY_SPAN
    x = rest // _KEY_SPAN
    return np.stack([x, y, z], axis=1) - _KEY_HALF


def patch_rects(frames: Frames, patches: Patches) -> tuple[NDArray[np.float64], ...]:
    """Pixel rectangle (u0, u1, v0, v1) of every patch."""
    f = patches.frame
    cols, rows = frames.cols[f], frames.rows[f]
    row, col = np.divmod(patches.cell, cols)
    u0 = col * frames.width[f] / cols
    u1 = (col + 1) * frames.width[f] / cols
    v0 = row * frames.height[f] / rows
    v1 = (row + 1) * frames.height[f] / rows
    return u0, u1, v0, v1


def project_pixels(
    frames: Frames,
    frame: NDArray[np.int64],
    u: NDArray[np.float64],
    v: NDArray[np.float64],
    z: NDArray[np.float64],
) -> NDArray[np.float64]:
    """World points of pixels (u, v) at camera depth z in frame `frame`. (n, 3)."""
    local = np.stack(
        [
            (u - frames.cx[frame]) / frames.fx[frame] * z,
            (v - frames.cy[frame]) / frames.fy[frame] * z,
            z,
            np.ones_like(z),
        ],
        axis=1,
    )
    return np.einsum("nij,nj->ni", frames.poses[frame][:, :3, :], local)


def patch_points(frames: Frames, patches: Patches) -> NDArray[np.float64]:
    """Where each patch's centre lands in the world at its depth. (n, 3)."""
    u0, u1, v0, v1 = patch_rects(frames, patches)
    return project_pixels(frames, patches.frame, (u0 + u1) / 2, (v0 + v1) / 2, patches.depth)


def rasterize(
    frames: Frames,
    patches: Patches,
    voxel_size: float,
    config: Any,
    *,
    max_candidates: int = 40_000_000,
) -> Rasterized:
    """Every hot pyramid's voxels, in one batch.

    Same test as :func:`patches.rasterize_pyramid`: a voxel counts when its
    centre projects inside the patch's pixel rectangle at a camera depth
    between ``cap_near * d`` and ``cap_far * d``. The yaw bin is the bearing
    from the voxel to the camera in the target frame's XY plane.
    """
    empty = Rasterized(
        np.zeros((0, 3), np.int64), np.zeros(0, np.int64), np.zeros(0), np.zeros(0, np.int64)
    )
    keep = np.isfinite(patches.depth) & (patches.depth > 0)
    if not keep.any():
        return empty
    # Candidates are generated per frame, so order the patches by frame.
    order = np.flatnonzero(keep)[np.argsort(patches.frame[keep], kind="stable")]
    frame = patches.frame[order]
    depth = patches.depth[order]
    score = patches.score[order]
    cell = patches.cell[order]
    sub = Patches(frame=frame, cell=cell, depth=depth, score=score)
    u0, u1, v0, v1 = patch_rects(frames, sub)
    near, far = depth * config.cap_near, depth * config.cap_far

    # Bounding box of the 8 corners, in voxel indices.
    n = len(frame)
    corner_u = np.stack([u0, u1, u0, u1] * 2, axis=1)
    corner_v = np.stack([v0, v0, v1, v1] * 2, axis=1)
    corner_z = np.concatenate(
        [np.repeat(near[:, None], 4, 1), np.repeat(far[:, None], 4, 1)], axis=1
    )
    rep = np.repeat(np.arange(n), 8)
    corners = project_pixels(
        frames, frame[rep], corner_u.ravel(), corner_v.ravel(), corner_z.ravel()
    )
    corners = corners.reshape(n, 8, 3)
    lo = np.floor(corners.min(axis=1) / voxel_size).astype(np.int64)
    hi = np.floor(corners.max(axis=1) / voxel_size).astype(np.int64)
    dims = hi - lo + 1
    counts = dims.prod(axis=1)
    total = int(counts.sum())
    if total > max_candidates:
        raise ValueError(f"{total} candidate voxels for {n} pyramids; raise voxel_size")
    if total == 0:
        return empty

    # Expand every bounding box into its voxels.
    p_of = np.repeat(np.arange(n), counts)
    starts = np.cumsum(counts) - counts
    offset = np.arange(total) - starts[p_of]
    plane = dims[:, 1] * dims[:, 2]
    ix = offset // plane[p_of]
    rem = offset - ix * plane[p_of]
    iy = rem // dims[p_of, 2]
    iz = rem - iy * dims[p_of, 2]
    index = lo[p_of] + np.stack([ix, iy, iz], axis=1)
    centres = (index + 0.5) * voxel_size

    # Project the candidates back into their cameras, one frame at a time
    # (the candidates are grouped by frame already).
    inside = np.zeros(total, dtype=bool)
    yaw_bin = np.zeros(total, dtype=np.int64)
    cand_frame = frame[p_of]
    bounds = np.flatnonzero(np.diff(cand_frame, prepend=-1))
    bounds = np.append(bounds, total)
    bins = max(int(config.yaw_bins), 1)
    for a, b in itertools.pairwise(bounds):
        f = int(cand_frame[a])
        pose = frames.poses[f]
        camera_from_target = np.linalg.inv(pose)
        pts = centres[a:b]
        local = pts @ camera_from_target[:3, :3].T + camera_from_target[:3, 3]
        z = local[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = local[:, 0] / z * frames.fx[f] + frames.cx[f]
            v = local[:, 1] / z * frames.fy[f] + frames.cy[f]
        sel = p_of[a:b]
        ok = (
            (z >= near[sel])
            & (z <= far[sel])
            & (u >= u0[sel])
            & (u < u1[sel])
            & (v >= v0[sel])
            & (v < v1[sel])
        )
        inside[a:b] = ok
        bearing = np.arctan2(pose[1, 3] - pts[:, 1], pose[0, 3] - pts[:, 0])
        yaw_bin[a:b] = np.floor((bearing + np.pi) / (2 * np.pi) * bins).astype(np.int64) % bins

    hit = np.flatnonzero(inside)
    return Rasterized(
        index=index[hit],
        frame_id=frames.ids[cand_frame[hit]],
        score=score[p_of[hit]],
        yaw_bin=yaw_bin[hit],
    )


def pool(evidence: Rasterized, config: Any) -> Pooled:
    """:func:`patches.pool` for every voxel at once: max per frame, log-sum-exp
    across frames, times the square root of the distinct yaw bins whose best
    score beats ``yaw_hot_threshold``."""
    if len(evidence.score) == 0:
        return Pooled(
            np.zeros((0, 3), np.int64), np.zeros(0), np.zeros(0, np.int64), np.zeros(0, np.int64)
        )
    keys = pack_keys(evidence.index)
    # Best entry per (voxel, frame): sort by voxel, frame, score descending; keep the first of each run.
    order = np.lexsort((-evidence.score, evidence.frame_id, keys))
    keys, frame, score, yaw = (
        keys[order],
        evidence.frame_id[order],
        evidence.score[order],
        evidence.yaw_bin[order],
    )
    first = np.ones(len(keys), dtype=bool)
    first[1:] = (keys[1:] != keys[:-1]) | (frame[1:] != frame[:-1])
    all_keys, all_yaw = keys, yaw  # every hit: the bin support counts them all
    keys, score, yaw = keys[first], score[first], yaw[first]

    # Per voxel: log-sum-exp over its frames' bests.
    starts = np.flatnonzero(np.diff(keys, prepend=keys[0] - 1))
    group = np.repeat(np.arange(len(starts)), np.diff(np.append(starts, len(keys))))
    t = max(config.lse_temperature, 1e-6)
    top = np.maximum.reduceat(score, starts)
    total = np.add.reduceat(np.exp((score - top[group]) / t), starts)
    lse = top + t * np.log(total)

    hot = score > config.yaw_hot_threshold
    pairs = np.unique(np.stack([group[hot], yaw[hot]], axis=1), axis=0)
    hot_bins = np.bincount(pairs[:, 0], minlength=len(starts))
    pooled = lse * np.sqrt(np.maximum(hot_bins, 1))
    frames = np.diff(np.append(starts, len(keys)))
    # Support counts every distinct yaw bin over every hit, like Hyperspace's (a
    # frame can see a voxel from two bins): only the score multiplier is about
    # the hot per-frame bests, and refine's min_bins must not drop a voxel the
    # reference keeps.
    seen = np.zeros((len(starts), max(int(config.yaw_bins), 1)), dtype=bool)
    seen[np.searchsorted(keys[starts], all_keys), all_yaw] = True  # dense: 4x np.unique
    bins = seen.sum(axis=1)
    return Pooled(unpack_keys(keys[starts]), pooled, frames.astype(np.int64), bins.astype(np.int64))


def normalize_scores(scores: NDArray[np.float64], percentile: float) -> NDArray[np.float64]:
    """Scaled so the ``percentile`` voxel is 1, clipped to [0, 1] (as :func:`patches.normalize`)."""
    if len(scores) == 0:
        return scores
    values = np.sort(scores)
    rank = round((len(values) - 1) * min(max(percentile, 0.0), 1.0))
    top = max(float(values[rank]), 1e-9)
    return np.clip(scores / top, 0.0, 1.0)


def combine(patch_map: Pooled, segment_map: Pooled, weight: float) -> Pooled:
    """Sum of the two normalized channels per voxel (as :func:`patches.combine`, before renormalizing)."""
    keys = np.concatenate([pack_keys(patch_map.index), pack_keys(segment_map.index)])
    values = np.concatenate([patch_map.score, weight * segment_map.score])
    unique, inverse = np.unique(keys, return_inverse=True)
    summed = np.bincount(inverse, weights=values, minlength=len(unique))
    frames = bins = None
    if patch_map.frames is not None or segment_map.frames is not None:
        # A channel without support counts (an empty one) contributes nothing.
        patch_map = _with_support(patch_map)
        segment_map = _with_support(segment_map)
        # Frames add up across the channels; the bin count is the larger one (as patches.combine).
        frames = np.bincount(
            inverse,
            weights=np.concatenate([patch_map.frames, segment_map.frames]),
            minlength=len(unique),
        ).astype(np.int64)
        bins = np.zeros(len(unique), dtype=np.int64)
        np.maximum.at(bins, inverse, np.concatenate([patch_map.bins, segment_map.bins]))
    return Pooled(unpack_keys(unique), summed, frames, bins)


def near_scene(
    index: NDArray[np.int64], scene_keys: NDArray[np.int64], radius: int = 1
) -> NDArray[np.bool_]:
    """Which voxels lie within *radius* (Chebyshev) of an occupied voxel.
    *scene_keys* are ``pack_keys`` of the map's voxels, sorted. Heat that
    floats in free space (a pyramid slice that missed its surface) is dropped
    by keeping only these; the same test as Hyperspace's ``occupancy`` step."""
    if len(index) == 0 or len(scene_keys) == 0:
        return np.zeros(len(index), dtype=bool)
    hit = np.zeros(len(index), dtype=bool)
    span = range(-radius, radius + 1)
    for dx in span:
        for dy in span:
            for dz in span:
                keys = pack_keys(index + np.array([dx, dy, dz]))
                pos = np.minimum(np.searchsorted(scene_keys, keys), len(scene_keys) - 1)
                hit |= scene_keys[pos] == keys
    return hit


def _with_support(pooled: Pooled) -> Pooled:
    if pooled.frames is not None and pooled.bins is not None:
        return pooled
    n = len(pooled.score)
    return Pooled(pooled.index, pooled.score, np.zeros(n, np.int64), np.zeros(n, np.int64))


def best_first(pooled: Pooled) -> Pooled:
    """Sorted by score descending, then index, like Hyperspace's ``Heatmap.voxels``."""
    order = np.lexsort((pooled.index[:, 2], pooled.index[:, 1], pooled.index[:, 0], -pooled.score))
    return Pooled(
        pooled.index[order],
        pooled.score[order],
        None if pooled.frames is None else pooled.frames[order],
        None if pooled.bins is None else pooled.bins[order],
    )


# ---- resident banks --------------------------------------------------------


class PatchBank:
    """Every keyframe's patch grid stacked for a matmul search, with its geometry."""

    def __init__(
        self, keyframes: list[tuple[Any, NDArray[np.float16]]], place: Callable[[Any], Any]
    ) -> None:
        import torch

        kept = []
        for keyframe, grid in keyframes:
            pose = place(keyframe)
            if pose is not None:
                kept.append((keyframe, grid, np.asarray(pose, dtype=np.float64)))
        self.unplaced = len(keyframes) - len(kept)
        # patch_cell below numbers patches within the MEMBER grid, while patch_depth comes
        # from the keyframe's CELL grid. They are the same grid only when the model's own
        # shape equals the cell grid, and since the cell grid became "the finest member,
        # floored at 24x24" they can differ for a single member too -- a 14x14 model
        # against a 24x24 cell grid. Stacked anyway, the depth of one keyframe is read for
        # the patches of another: no exception, just placements from the wrong frame.
        mismatched = [
            (k.id, len(g), k.rows * k.cols) for (k, g, _) in kept if len(g) != k.rows * k.cols
        ]
        if mismatched:
            keyframe, patches, cells = mismatched[0]
            raise SystemExit(
                f"keyframe {keyframe} holds {patches} patches against a {cells}-cell grid"
                f" ({len(mismatched)} such): this store's model grid is not its cell grid,"
                " and the fast path stacks the two together. Query through the engine."
            )
        self.frames = _frames_of(
            [
                (k.id, pose, k.intrinsics, k.rows, k.cols, k.camera_frame, k.ts)
                for k, _, pose in kept
            ]
        )
        grids = [grid for _, grid, _ in kept]
        self.matrix = (
            torch.from_numpy(np.concatenate(grids, axis=0)) if grids else torch.zeros((0, 1))
        )
        sizes = np.asarray([len(g) for g in grids], dtype=np.int64)
        self.patch_frame = np.repeat(np.arange(len(kept)), sizes)
        self.patch_cell = (
            np.concatenate([np.arange(s) for s in sizes]) if len(sizes) else np.zeros(0, np.int64)
        )
        self.patch_depth = (
            np.concatenate([np.asarray(k.patch_depth, np.float64) for k, _, _ in kept])
            if kept
            else np.zeros(0)
        )
        self._background_sims: NDArray[np.float32] | None = None

    def __len__(self) -> int:
        return int(self.matrix.shape[0]) if len(self.frames) else 0

    def similarities(self, vectors: NDArray[np.float32]) -> NDArray[np.float32]:
        """Cosine of every patch against each row of *vectors*. (N, k)."""
        import torch

        if len(self) == 0:
            return np.zeros((0, len(vectors)), np.float32)
        q = torch.from_numpy(np.ascontiguousarray(vectors, dtype=np.float32)).to(self.matrix.dtype)
        with torch.no_grad():
            if (
                len(q) == 1
            ):  # the matrix-vector kernel is several times faster than a 1-column matmul
                return (self.matrix @ q[0]).float().numpy()[:, None]
            return (self.matrix @ q.T).float().numpy()

    def background_sims(self, backgrounds: NDArray[np.float32]) -> NDArray[np.float32]:
        if self._background_sims is None:
            self._background_sims = (
                self.similarities(backgrounds)
                if len(backgrounds)
                else np.zeros((len(self), 0), np.float32)
            )
        return self._background_sims

    def hot(
        self,
        query: NDArray[np.float32],
        backgrounds: NDArray[np.float32],
        config: Any,
        gate: NDArray[np.bool_] | None = None,
    ) -> tuple[Patches, int]:
        """Patches whose query score beats their best background by ``hot_threshold``,
        among the ``min(max_hot_patches, VEC0_MAX_K)`` most similar; those *gate*
        marks (structural: floor/wall/ceiling) are dropped."""
        if len(self) == 0:
            return Patches(
                np.zeros(0, np.int64), np.zeros(0, np.int64), np.zeros(0), np.zeros(0)
            ), 0
        sims = self.similarities(query[None, :])[:, 0]
        k = min(int(config.max_hot_patches), VEC0_MAX_K, len(sims))
        top = np.argpartition(-sims, k - 1)[:k] if k < len(sims) else np.arange(len(sims))
        contrast = sims[top].astype(np.float64)
        if len(backgrounds):
            usable = (backgrounds @ query) < config.background_synonym_cutoff
            bg = self.background_sims(backgrounds)[top][:, usable]
            if bg.shape[1]:
                contrast = contrast - bg.max(axis=1)
        keep = contrast > config.hot_threshold
        if gate is not None:
            keep &= ~gate[top]
        hot = top[keep]
        hot_score = contrast[keep]
        return (
            Patches(
                frame=self.patch_frame[hot],
                cell=self.patch_cell[hot],
                depth=self.patch_depth[hot],
                score=hot_score,
            ),
            k,
        )


class SegmentBank:
    """Every stored segment as pseudo-frames and their covered cells, placed once."""

    def __init__(
        self,
        store: Any,
        place: Callable[[Any], Any],
        embed_texts: Callable[[list[str]], NDArray[np.float32]],
    ) -> None:
        from dimos.mapping.hyperspace import patches as hs

        frames: list[tuple[int, NDArray[np.float64], Any, int, int, str, float]] = []
        cell_frame: list[int] = []
        cell_index: list[int] = []
        cell_cov: list[float] = []
        cell_depth: list[float] = []
        seg_label: list[int] = []
        seg_conf: list[float] = []
        seg_ts: list[float] = []
        seg_camera: list[str] = []
        labels: dict[str, int] = {}
        self.read = 0
        self.unplaced = 0
        if SEGMENT_STREAM in store.list_streams():
            for obs in store.stream(SEGMENT_STREAM, dict).order_by("ts"):
                self.read += 1
                record = obs.data
                name = (obs.tags or {}).get("name")
                if not name or not record.get("cells") or not record.get("intrinsics"):
                    continue
                keyframe = hs.Keyframe(
                    id=-(obs.id + 1),
                    camera_frame=record["camera_frame"],
                    ts=float(record["ts"]),
                    rows=int(record["rows"]),
                    cols=int(record["cols"]),
                    intrinsics=hs.Intrinsics(**record["intrinsics"]),
                    patch_depth=np.zeros(0, np.float32),
                )
                pose = place(keyframe)
                if pose is None:
                    self.unplaced += 1
                    continue
                f = len(frames)
                frames.append(
                    (
                        keyframe.id,
                        np.asarray(pose, np.float64),
                        keyframe.intrinsics,
                        keyframe.rows,
                        keyframe.cols,
                        keyframe.camera_frame,
                        keyframe.ts,
                    )
                )
                seg_label.append(labels.setdefault(name, len(labels)))
                seg_conf.append(float(record["confidence"]))
                seg_ts.append(float(record["ts"]))
                seg_camera.append(record["camera_frame"])
                for index, coverage, depth in record["cells"]:
                    cell_frame.append(f)
                    cell_index.append(int(index))
                    cell_cov.append(float(coverage))
                    cell_depth.append(float(depth) if depth is not None else np.nan)
        self.frames = _frames_of(frames)
        self.seg_label = np.asarray(seg_label, np.int64)
        self.seg_conf = np.asarray(seg_conf)
        self.seg_ts = np.asarray(seg_ts)
        self.seg_camera = seg_camera
        self.cell_frame = np.asarray(cell_frame, np.int64)
        self.cell_index = np.asarray(cell_index, np.int64)
        self.cell_cov = np.asarray(cell_cov)
        self.cell_depth = np.asarray(cell_depth)
        self.label_names = list(labels)
        self.label_vectors = (
            np.asarray(embed_texts(self.label_names), np.float32)
            if labels
            else np.zeros((0, 1), np.float32)
        )
        # Cells grouped by segment, for the per-query selection.
        self._cells_of = np.argsort(self.cell_frame, kind="stable")
        self._cell_starts = np.searchsorted(
            self.cell_frame[self._cells_of], np.arange(len(frames) + 1)
        )

    def __len__(self) -> int:
        return len(self.frames)

    def label_scores(self, query: NDArray[np.float32], config: Any) -> NDArray[np.float64]:
        """Word score per label (z-scored cosine; 0 at ``segment_min_z``, 1 at twice it)."""
        if not len(self.label_names):
            return np.zeros(0)
        cosine = self.label_vectors @ query
        z = (cosine - cosine.mean()) / max(float(cosine.std()), 1e-6)
        floor = max(config.segment_min_z, 1e-6)
        return np.clip((z - floor) / floor, 0.0, 1.0)

    def hot(self, query: NDArray[np.float32], config: Any) -> tuple[Patches, int, dict[str, float]]:
        """Cells of the segments whose label matches, scored word x confidence x coverage.
        Segments are taken best label first, ``max_hot_segments`` of them, as the engine does."""
        empty = Patches(np.zeros(0, np.int64), np.zeros(0, np.int64), np.zeros(0), np.zeros(0))
        if len(self) == 0 or config.segment_weight <= 0:
            return empty, 0, {}
        word = self.label_scores(query, config)
        words = {n: float(w) for n, w in zip(self.label_names, word, strict=True) if w > 0}
        seg_word = word[self.seg_label]
        chosen = np.flatnonzero(seg_word > 0)
        if len(chosen) == 0:
            return empty, 0, words
        chosen = chosen[np.lexsort((self.seg_ts[chosen], -seg_word[chosen]))][
            : int(config.max_hot_segments)
        ]
        weight = seg_word[chosen] * self.seg_conf[chosen]
        spans = [(self._cell_starts[s], self._cell_starts[s + 1]) for s in chosen]
        cells = (
            np.concatenate([self._cells_of[a:b] for a, b in spans])
            if spans
            else np.zeros(0, np.int64)
        )
        per_cell_weight = np.repeat(weight, [b - a for a, b in spans])
        return (
            Patches(
                frame=self.cell_frame[cells],
                cell=self.cell_index[cells],
                depth=self.cell_depth[cells],
                score=per_cell_weight * self.cell_cov[cells],
            ),
            len(chosen),
            words,
        )


TEXT_CACHE_SIZE = 64  # phrases whose embedding is kept (a demo asks a few dozen)

STRUCTURAL_LABELS = ("floor", "wall", "ceiling")


def structural_mask(patches: PatchBank, segments: SegmentBank, config: Any) -> NDArray[np.bool_]:
    """Per camera patch: whether the nearest segment frame of the same camera
    (within ``structural_gate_dt``) labelled its cell floor, wall or ceiling
    with at least ``structural_gate_coverage``. Mirrors ``HyperspaceQuery.is_structural``
    over the segments the bank kept (those with a pose and intrinsics)."""
    mask = np.zeros(len(patches), dtype=bool)
    if len(segments) == 0 or len(patches) == 0:
        return mask
    coverage_min = float(getattr(config, "structural_gate_coverage", 0.98))
    max_dt = float(getattr(config, "structural_gate_dt", 0.3))
    structural_labels = [
        i for i, name in enumerate(segments.label_names) if name in STRUCTURAL_LABELS
    ]
    # Cells per (camera, stamp) that a structural segment covers well enough.
    cells_of: dict[tuple[str, float], set[int]] = {}
    for seg in np.flatnonzero(np.isin(segments.seg_label, structural_labels)):
        a, b = segments._cell_starts[seg], segments._cell_starts[seg + 1]
        idx = segments._cells_of[a:b]
        good = idx[segments.cell_cov[idx] >= coverage_min]
        # A frame whose structural cells are all below the coverage stays, with no
        # cells: nearest by stamp, it gates nothing, as in the reference.
        key = (segments.seg_camera[seg], float(segments.seg_ts[seg]))
        cells_of.setdefault(key, set()).update(int(c) for c in segments.cell_index[good])
    by_camera: dict[str, tuple[NDArray[np.float64], list[set[int]]]] = {}
    for camera in {c for c, _ in cells_of}:
        stamps = sorted(ts for c, ts in cells_of if c == camera)
        by_camera[camera] = (np.asarray(stamps), [cells_of[(camera, ts)] for ts in stamps])
    frames = patches.frames
    starts = np.searchsorted(patches.patch_frame, np.arange(len(frames) + 1))
    for f in range(len(frames)):
        entry = by_camera.get(frames.camera_frame[f])
        if entry is None:
            continue
        stamps, cell_sets = entry
        ts = float(frames.ts[f])
        position = int(np.searchsorted(stamps, ts))
        candidates = [i for i in (position - 1, position) if 0 <= i < len(stamps)]
        nearest = min(candidates, key=lambda i: abs(stamps[i] - ts))
        if abs(stamps[nearest] - ts) > max_dt or not cell_sets[nearest]:
            continue
        a, b = starts[f], starts[f + 1]
        mask[a:b] = np.isin(patches.patch_cell[a:b], list(cell_sets[nearest]))
    return mask


def _frames_of(rows: list[tuple[int, NDArray[np.float64], Any, int, int, str, float]]) -> Frames:
    if not rows:
        z = np.zeros(0)
        return Frames(
            np.zeros(0, np.int64),
            np.zeros((0, 4, 4)),
            z,
            z,
            z,
            z,
            z,
            z,
            np.zeros(0, np.int64),
            np.zeros(0, np.int64),
            [],
            z,
        )
    return Frames(
        ids=np.asarray([r[0] for r in rows], np.int64),
        poses=np.stack([r[1] for r in rows]),
        width=np.asarray([r[2].width for r in rows], np.float64),
        height=np.asarray([r[2].height for r in rows], np.float64),
        fx=np.asarray([r[2].fx for r in rows], np.float64),
        fy=np.asarray([r[2].fy for r in rows], np.float64),
        cx=np.asarray([r[2].cx for r in rows], np.float64),
        cy=np.asarray([r[2].cy for r in rows], np.float64),
        rows=np.asarray([r[3] for r in rows], np.int64),
        cols=np.asarray([r[4] for r in rows], np.int64),
        camera_frame=[r[5] for r in rows],
        ts=np.asarray([r[6] for r in rows], np.float64),
    )


# ---- the query -------------------------------------------------------------


@dataclass
class FastResult:
    """One answer: normalized voxels best first, plus the hot patches behind them."""

    index: NDArray[np.int64]  # (V, 3)
    score: NDArray[np.float64]  # (V,), 1.0 at the top
    frames: NDArray[np.int64]  # (V,) keyframes (and segments) that saw the voxel
    bins: NDArray[np.int64]  # (V,) distinct hot viewing directions
    patches: Patches  # hot camera patches (frames = bank.frames)
    patch_points: NDArray[np.float64]  # (n, 3) where each hot patch landed
    segments: Patches  # hot segment cells (frames = segment bank frames)
    segment_points: NDArray[np.float64]
    stats: dict[str, Any]


class FastQuery:
    """Text -> heat map over resident banks. Built from a warmed ``HyperspaceQuery``."""

    def __init__(
        self,
        engine: Any,
        *,
        world_frame: str,
        voxel_size: float,
        embed_texts: Callable[[list[str]], NDArray[np.float32]],
        with_segments: bool = True,
    ) -> None:
        started = time.monotonic()
        self.config = engine.config
        self.voxel_size = voxel_size
        self.world_frame = world_frame
        self._embed_texts = embed_texts
        self._text_cache: dict[str, NDArray[np.float32]] = {}
        engine.keyframe(-1)  # loads every keyframe
        place = engine.placer(world_frame)  # one tf pass
        # How many members the STORE holds, which is the question. Counting the arrays
        # engine.backgrounds() returns counts what the EMBEDDER produced, and this module
        # hands it a single-model embedder -- so an ensemble store passed that test and
        # was then searched on its primary grid alone, silently, which is exactly what
        # the check exists to stop. `members()` is empty for a single-grid store.
        members = engine.members()
        if len(members) > 1:
            raise SystemExit(
                f"this store was embedded with {len(members)} ensemble members "
                f"({', '.join(members)}), and the fast path holds one grid per keyframe."
                " Re-ingest with a single model, or query through the engine."
            )
        self.patches = PatchBank(list(engine._keyframes.values()), place)
        backgrounds = engine.backgrounds()
        self.backgrounds = np.asarray(backgrounds[0] if len(backgrounds) else [], np.float32)
        self.patches.background_sims(self.backgrounds)
        # Not gated on segment_weight: the structural gate reads the same bank, and
        # SegmentBank.hot already no-ops at weight 0. Gating here turned the gate off
        # silently, admitting every floor, wall and ceiling patch.
        self.segments = SegmentBank(engine.store, place, embed_texts) if with_segments else None
        self.structural = (
            structural_mask(self.patches, self.segments, self.config)
            if self.segments is not None and getattr(self.config, "structural_gate", False)
            else None
        )
        self.build_seconds = time.monotonic() - started
        logger.info(
            "fast hyperspace: %d patches in %d frames (%d unplaced), %d segments (%d unplaced), built in %.1f s",
            len(self.patches),
            len(self.patches.frames),
            self.patches.unplaced,
            len(self.segments) if self.segments is not None else 0,
            self.segments.unplaced if self.segments is not None else 0,
            self.build_seconds,
        )

    def embed(self, text: str) -> NDArray[np.float32]:
        vector = self._text_cache.get(text)
        if vector is None:
            vector = np.asarray(self._embed_texts([text])[0], np.float32)
            self._text_cache[text] = vector
            if len(self._text_cache) > TEXT_CACHE_SIZE:
                del self._text_cache[next(iter(self._text_cache))]
        return vector

    def query(self, text: str) -> FastResult:
        config = self.config
        timings: dict[str, float] = {}
        clock = time.perf_counter
        t = clock()
        vector = self.embed(text)
        timings["embed"] = clock() - t

        t = clock()
        # Patches the segmenter called floor/wall/ceiling are not answers, unless
        # the question is about those (Hyperspace's structural gate).
        exempt = any(label in text.lower() for label in STRUCTURAL_LABELS)
        gate = None if exempt else self.structural
        hot, searched = self.patches.hot(vector, self.backgrounds, config, gate=gate)
        timings["search"] = clock() - t
        t = clock()
        evidence = rasterize(self.patches.frames, hot, self.voxel_size, config)
        patch_map = pool(evidence, config).normalized(config.normalize_percentile)
        timings["patches"] = clock() - t
        stats: dict[str, Any] = {
            "hot_patches": len(hot),
            "patches_searched": searched,
            "structural_gate": bool(gate is not None),
            "voxels_touched": len(patch_map.score),
        }

        result = patch_map
        seg_hot = Patches(np.zeros(0, np.int64), np.zeros(0, np.int64), np.zeros(0), np.zeros(0))
        seg_points = np.zeros((0, 3))
        if self.segments is not None and len(self.segments):
            t = clock()
            seg_hot, chosen, words = self.segments.hot(vector, config)
            seg_evidence = rasterize(self.segments.frames, seg_hot, self.voxel_size, config)
            seg_map = pool(seg_evidence, config).normalized(config.normalize_percentile)
            summed = combine(patch_map, seg_map, config.segment_weight)
            result = summed.normalized(config.normalize_percentile)
            timings["segments"] = clock() - t
            stats.update(
                {
                    "segment_hot_cells": len(seg_hot),
                    "segments_chosen": chosen,
                    "segment_voxels_touched": len(seg_map.score),
                    "segment_labels": sorted(words, key=lambda k: -words[k])[:6],
                }
            )
            if len(seg_hot):
                seg_points = patch_points(self.segments.frames, seg_hot)
        result = best_first(result)
        stats["timings_ms"] = {k: round(v * 1000, 1) for k, v in timings.items()}
        n_out = len(result.score)
        return FastResult(
            index=result.index,
            score=result.score,
            frames=result.frames if result.frames is not None else np.ones(n_out, np.int64),
            bins=result.bins if result.bins is not None else np.ones(n_out, np.int64),
            patches=hot,
            patch_points=patch_points(self.patches.frames, hot) if len(hot) else np.zeros((0, 3)),
            segments=seg_hot,
            segment_points=seg_points,
            stats=stats,
        )
