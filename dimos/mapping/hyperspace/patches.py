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

"""The math behind hyperspace, in numpy, with no model and no I/O.

A kept keyframe is a grid of text-aligned patch embeddings plus one fused depth
per patch. A query scores patches, and every hot patch becomes a pyramid: the
patch's view frustum cut at 0.9x..1.1x its depth. Pyramids are rasterized into
a sparse voxel grid and pooled per voxel: max over a frame's patches, log-sum-
exp across frames, times the square root of how many distinct yaw bins were
hot. Keyframes store no pose; the caller places them at query time, which is
what lets a loop closure move old answers.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from functools import lru_cache
import math
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable

    from numpy.typing import NDArray


@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class KeyframeGateConfig:
    """Every knob is per camera and individually off-able (``None``)."""

    # Frames held before the middle one is judged; odd.
    buffer_len: int = 11
    # Mean per-patch (1 - cosine) against the last kept keyframe needed to keep one.
    novelty_threshold: float = 0.05
    # Any single patch changing more than this keeps a frame (a new object in a static view).
    patch_novelty_threshold: float | None = 0.5
    max_angular_velocity: float | None = 1.5
    max_linear_velocity: float | None = None
    max_dark_fraction: float | None = 0.6
    dark_level: int = 8
    max_bright_fraction: float | None = None
    bright_level: int = 247
    min_interval: float | None = 0.1


def exposure_stats(
    rgb: NDArray[np.uint8], dark_level: int, bright_level: int
) -> tuple[float, float, float]:
    """(mean luma, dark fraction, bright fraction), sampled every 4th pixel."""
    sample = rgb[::4, ::4].astype(np.uint32)
    luma = (sample[..., 0] * 299 + sample[..., 1] * 587 + sample[..., 2] * 114) // 1000
    return (
        float(luma.mean()),
        float((luma <= dark_level).mean()),
        float((luma >= bright_level).mean()),
    )


def quality_gate(
    config: KeyframeGateConfig, rgb: NDArray[np.uint8], speeds: tuple[float, float] | None
) -> str | None:
    """Why a frame is dropped before the model runs, or None to keep going."""
    if speeds is not None:
        angular, linear = speeds
        if config.max_angular_velocity is not None and angular > config.max_angular_velocity:
            return "too_fast"
        if config.max_linear_velocity is not None and linear > config.max_linear_velocity:
            return "too_fast"
    if config.max_dark_fraction is not None or config.max_bright_fraction is not None:
        _, dark, bright = exposure_stats(rgb, config.dark_level, config.bright_level)
        if config.max_dark_fraction is not None and dark > config.max_dark_fraction:
            return "too_dark"
        if config.max_bright_fraction is not None and bright > config.max_bright_fraction:
            return "too_bright"
    return None


def grid_distance(grid: NDArray[np.floating], other: NDArray[np.floating]) -> tuple[float, float]:
    """(mean, max) per-patch ``1 - cosine`` between two L2-normalized grids."""
    cosine = np.einsum("pd,pd->p", grid.astype(np.float32), other.astype(np.float32))
    distance = 1.0 - cosine
    return float(distance.mean()), float(distance.max())


@lru_cache(maxsize=64)
def cell_matrix(source: tuple[int, int], target: tuple[int, int]) -> NDArray[np.float32]:
    """``[target cells, source cells]`` map from one patch grid to another over
    the same image: each target cell is the area-weighted mean of the source
    cells under it (exact, via a raster both grids divide evenly). Identity
    when the grids match; a 14x14 grid onto 24x24 spreads each patch over the
    cells it covers."""
    (sr, sc), (tr, tc) = source, target
    if source == target:
        return np.eye(tr * tc, dtype=np.float32)
    matrix = np.zeros((tr * tc, sr * sc), dtype=np.float32)
    rows = np.arange(sr * tr)
    cols = np.arange(sc * tc)
    source_index = (rows // tr)[:, None] * sc + (cols // tc)[None, :]
    target_index = (rows // sr)[:, None] * tc + (cols // sc)[None, :]
    np.add.at(matrix, (target_index.ravel(), source_index.ravel()), 1.0)
    return matrix / matrix.sum(axis=1, keepdims=True)


def pool_cells(contrasts: list[NDArray[np.float32]], pool: str) -> NDArray[np.float32]:
    """Combine one ``[cells]`` contrast per ensemble member."""
    stack = np.stack(contrasts)
    if pool == "min" or len(stack) == 1:
        return stack.min(axis=0)
    if pool == "2nd":
        return np.sort(stack, axis=0)[1]
    if pool == "mean":
        return stack.mean(axis=0)
    raise ValueError(f"unknown pool {pool!r}; choose min, 2nd or mean")


@dataclass
class BufferedFrame:
    ts: float
    grid: NDArray[np.float16]
    quality: float
    payload: Any


class RollingBuffer:
    """Hold ``buffer_len`` embedded frames; judge the middle one against the
    frames before and after it. Keep it when it is novel versus the last kept
    keyframe and the best-quality frame among the window's novel frames."""

    def __init__(self, config: KeyframeGateConfig) -> None:
        self.config = config
        self.frames: deque[BufferedFrame] = deque()
        self.last_kept: NDArray[np.float16] | None = None
        self.last_kept_ts: float | None = None

    def _is_novel(self, grid: NDArray[np.float16]) -> bool:
        if self.last_kept is None:
            return True
        mean, patch_max = grid_distance(grid, self.last_kept)
        if (
            self.config.patch_novelty_threshold is not None
            and patch_max > self.config.patch_novelty_threshold
        ):
            return True
        return mean > self.config.novelty_threshold

    def _judge(self, middle: int) -> bool:
        candidate = self.frames[middle]
        if (
            self.config.min_interval is not None
            and self.last_kept_ts is not None
            and candidate.ts - self.last_kept_ts < self.config.min_interval
        ):
            return False
        if not self._is_novel(candidate.grid):
            return False
        return all(
            frame.quality <= candidate.quality
            for index, frame in enumerate(self.frames)
            if index != middle and self._is_novel(frame.grid)
        )

    def _take(self, middle: int) -> BufferedFrame:
        winner = self.frames[middle]
        del self.frames[middle]
        for _ in range(middle):
            self.frames.popleft()
        self.last_kept = winner.grid
        self.last_kept_ts = winner.ts
        return winner

    def push(self, frame: BufferedFrame) -> BufferedFrame | None:
        self.frames.append(frame)
        if len(self.frames) < max(self.config.buffer_len, 1):
            return None
        middle = len(self.frames) // 2
        if self._judge(middle):
            return self._take(middle)
        self.frames.popleft()
        return None

    def flush(self) -> list[BufferedFrame]:
        kept = []
        while self.frames:
            middle = len(self.frames) // 2
            if self._judge(middle):
                kept.append(self._take(middle))
            else:
                self.frames.popleft()
        return kept


def per_patch_depth(depth_m: NDArray[np.floating], rows: int, cols: int) -> NDArray[np.float32]:
    """Median valid depth under each patch of a ``rows x cols`` grid laid over
    the image; NaN where a patch has no valid reading."""
    height, width = depth_m.shape
    out = np.full(rows * cols, np.nan, dtype=np.float32)
    for row in range(rows):
        y0, y1 = row * height // rows, (row + 1) * height // rows
        for col in range(cols):
            x0, x1 = col * width // cols, (col + 1) * width // cols
            cell = depth_m[y0:y1, x0:x1]
            valid = cell[(cell > 0) & np.isfinite(cell)]
            if valid.size:
                out[row * cols + col] = np.median(valid)
    return out


def reproject_depth(
    depth_m: NDArray[np.floating],
    depth: Intrinsics,
    color: Intrinsics,
    color_from_depth: NDArray[np.floating],
) -> NDArray[np.float32]:
    """Re-render a depth image into the colour camera's pixel grid through the
    4x4 ``color_from_depth`` transform. Nearest surface wins per pixel."""
    out = np.zeros((color.height, color.width), dtype=np.float32)
    vs, us = np.nonzero((depth_m > 0) & np.isfinite(depth_m))
    if us.size == 0:
        return out
    z = depth_m[vs, us].astype(np.float64)
    points = np.stack(
        [
            (us + 0.5 - depth.cx) / depth.fx * z,
            (vs + 0.5 - depth.cy) / depth.fy * z,
            z,
            np.ones_like(z),
        ]
    )
    moved = color_from_depth @ points
    zc = moved[2]
    keep = zc > 0
    uc = np.floor(moved[0][keep] / zc[keep] * color.fx + color.cx).astype(int)
    vc = np.floor(moved[1][keep] / zc[keep] * color.fy + color.cy).astype(int)
    zc = zc[keep].astype(np.float32)
    inside = (uc >= 0) & (vc >= 0) & (uc < color.width) & (vc < color.height)
    uc, vc, zc = uc[inside], vc[inside], zc[inside]
    # Farthest first so the nearest write lands last.
    order = np.argsort(-zc)
    out[vc[order], uc[order]] = zc[order]
    return out


@dataclass
class Keyframe:
    """One kept frame: where and when it was taken, never where the camera was."""

    id: int
    camera_frame: str
    ts: float
    rows: int
    cols: int
    intrinsics: Intrinsics
    patch_depth: NDArray[np.float32]


@dataclass
class QueryConfig:
    hot_threshold: float = 0.02
    max_hot_patches: int = 6000
    # Ensemble stores (several grids per keyframe): how the members' per-cell
    # contrasts combine. "min" keeps only what every member sees, "2nd" = the
    # second lowest (a 2-of-3 vote with three members; one may miss), "mean"
    # averages. The threshold applies to the pooled score: a minimum sits
    # below every member's score, so "min" wants 0.005 (the single model's
    # recall at twice its precision, plan.md 7); use 0.02 with "2nd".
    pool: str = "min"
    pooled_hot_threshold: float = 0.005
    # Pyramids span this slice of the patch depth; 0.99-1.01 is a thin shell
    # at the depth itself (Jeff, 2026-09-10: tighter caps read better).
    cap_near: float = 0.99
    cap_far: float = 1.01
    lse_temperature: float = 0.02
    yaw_bins: int = 8
    yaw_hot_threshold: float = 0.04
    normalize_percentile: float = 0.999
    background_prompts: list[str] = field(
        default_factory=lambda: [
            "a photo",
            "an office",
            "a room",
            "an indoor scene",
            "a wall",
            "a floor",
            "a ceiling",
            "furniture",
        ]
    )
    background_synonym_cutoff: float = 0.85
    # The segment channel, added on top of the patch channel. Text-text cosines
    # sit in a narrow band (0.6-0.95, "a chair" vs chair 0.87, vs table 0.75),
    # so a label is scored by how far its cosine to the query stands above the
    # vocabulary's mean, in standard deviations: 0 at segment_min_z, 1 at
    # twice that. A cell then scores word x segment confidence x coverage.
    segment_weight: float = 1.0
    segment_min_z: float = 2.0
    max_hot_segments: int = 4000
    # Default refinement chain (comma separated refine.py methods; "" = raw
    # map). Chosen 2026-09-11 on sf_office: see refine.py and the SacredLocust
    # report; with structural_gate it is what makes the answers object-shaped.
    refine: str = "occupancy,support,prior"
    # Drop hot patches whose grid cell the segmenter labelled floor, wall or
    # ceiling (in the nearest segment frame within structural_gate_dt
    # seconds), unless the query names one of those. SigLIP patches carry
    # image-wide context: the floor beside a cone scores like a cone.
    structural_gate: bool = True
    structural_gate_dt: float = 0.3
    # A cell counts as floor/wall/ceiling only when almost all of it is: a far
    # cone fills a tenth of its cell and must not be gated with the floor.
    structural_gate_coverage: float = 0.98


@dataclass
class HotPatch:
    keyframe: Keyframe
    patch: int
    score: float


def rasterize_pyramid(
    hot: HotPatch,
    target_from_camera: NDArray[np.floating],
    voxel_size: float,
    config: QueryConfig,
) -> list[tuple[tuple[int, int, int], int]]:
    """Voxels whose centres fall inside the patch's pixel rectangle and within
    ``[cap_near*d, cap_far*d]`` along the camera's z axis, with each voxel's yaw
    bin (bearing from the voxel to the camera in the target frame's XY plane)."""
    depth = float(hot.keyframe.patch_depth[hot.patch])
    if not (depth > 0) or not math.isfinite(depth):
        return []
    camera = hot.keyframe.intrinsics
    rows, cols = hot.keyframe.rows, hot.keyframe.cols
    row, col = divmod(hot.patch, cols)
    u0, u1 = col * camera.width / cols, (col + 1) * camera.width / cols
    v0, v1 = row * camera.height / rows, (row + 1) * camera.height / rows
    near, far = depth * config.cap_near, depth * config.cap_far

    corners = []
    for z in (near, far):
        for u, v in ((u0, v0), (u1, v0), (u0, v1), (u1, v1)):
            corners.append(
                [(u - camera.cx) / camera.fx * z, (v - camera.cy) / camera.fy * z, z, 1.0]
            )
    world_corners = (target_from_camera @ np.asarray(corners).T)[:3].T
    lo = np.floor(world_corners.min(axis=0) / voxel_size).astype(int)
    hi = np.floor(world_corners.max(axis=0) / voxel_size).astype(int)

    xs, ys, zs = np.meshgrid(
        np.arange(lo[0], hi[0] + 1),
        np.arange(lo[1], hi[1] + 1),
        np.arange(lo[2], hi[2] + 1),
        indexing="ij",
    )
    index = np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1)
    centres = (index + 0.5) * voxel_size
    camera_from_target = np.linalg.inv(target_from_camera)
    in_camera = (camera_from_target @ np.column_stack([centres, np.ones(len(centres))]).T)[:3].T
    z = in_camera[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = in_camera[:, 0] / z * camera.fx + camera.cx
        v = in_camera[:, 1] / z * camera.fy + camera.cy
    inside = (z >= near) & (z <= far) & (u >= u0) & (u < u1) & (v >= v0) & (v < v1)
    if not inside.any():
        return []
    camera_position = target_from_camera[:3, 3]
    hit_centres = centres[inside]
    bearing = np.arctan2(
        camera_position[1] - hit_centres[:, 1], camera_position[0] - hit_centres[:, 0]
    )
    bins = (np.floor((bearing + np.pi) / (2 * np.pi) * config.yaw_bins).astype(int)) % max(
        config.yaw_bins, 1
    )
    return [
        ((int(x), int(y), int(zz)), int(b))
        for (x, y, zz), b in zip(index[inside], bins, strict=True)
    ]


def pool(evidence: Iterable[tuple[int, float, int]], config: QueryConfig) -> float:
    """max per frame -> log-sum-exp across frames -> x sqrt(hot yaw bins)."""
    best: dict[int, tuple[float, int]] = {}
    for keyframe_id, score, yaw_bin in evidence:
        if keyframe_id not in best or score > best[keyframe_id][0]:
            best[keyframe_id] = (score, yaw_bin)
    scores = np.array([s for s, _ in best.values()], dtype=np.float64)
    t = max(config.lse_temperature, 1e-6)
    top = scores.max()
    lse = top + t * math.log(np.exp((scores - top) / t).sum())
    hot_bins = {b for s, b in best.values() if s > config.yaw_hot_threshold}
    return float(lse * math.sqrt(max(len(hot_bins), 1)))


@dataclass
class Cluster:
    """One refined answer: a connected blob of voxels, ranked by score."""

    rank: int
    score: float
    voxels: int
    centre: tuple[float, float, float]  # metres, score-weighted
    extent: tuple[float, float, float]  # metres, bounding box
    peak: tuple[int, int, int]  # voxel index of the best voxel


@dataclass
class Heatmap:
    frame: str
    voxel_size: float
    # (index, score) best first
    voxels: list[tuple[tuple[int, int, int], float]]
    stats: dict[str, Any]
    # Per voxel (patch score, segment score), each normalized on its own, when
    # the map is the sum of both channels; empty for a single-channel map.
    channels: dict[tuple[int, int, int], tuple[float, float]] = field(default_factory=dict)
    # Per voxel (distinct frames, distinct yaw bins) that put evidence on it.
    support: dict[tuple[int, int, int], tuple[int, int]] = field(default_factory=dict)
    # Voxel -> cluster rank (0 = best) once refined; see refine.py.
    cluster_of: dict[tuple[int, int, int], int] = field(default_factory=dict)
    clusters: list[Cluster] = field(default_factory=list)

    def centres(self) -> NDArray[np.float64]:
        if not self.voxels:
            return np.zeros((0, 3))
        return (np.asarray([v for v, _ in self.voxels], dtype=np.float64) + 0.5) * self.voxel_size

    def scores(self) -> NDArray[np.float64]:
        return np.asarray([s for _, s in self.voxels], dtype=np.float64)


def heatmap(
    hot_patches: list[HotPatch],
    place: Callable[[Keyframe], NDArray[np.floating] | None],
    target_frame: str,
    voxel_size: float,
    config: QueryConfig,
) -> Heatmap:
    """Place every hot patch's pyramid through ``place`` (keyframe -> 4x4
    target_from_camera, or None when tf cannot), rasterize, pool, normalize."""
    poses: dict[int, NDArray[np.floating] | None] = {}
    evidence: dict[tuple[int, int, int], list[tuple[int, float, int]]] = {}
    without_depth = 0
    for hot in hot_patches:
        if hot.keyframe.id not in poses:
            poses[hot.keyframe.id] = place(hot.keyframe)
        pose = poses[hot.keyframe.id]
        if pose is None:
            continue
        depth = float(hot.keyframe.patch_depth[hot.patch])
        if not (depth > 0) or not math.isfinite(depth):
            without_depth += 1
            continue
        for index, yaw_bin in rasterize_pyramid(hot, pose, voxel_size, config):
            evidence.setdefault(index, []).append((hot.keyframe.id, hot.score, yaw_bin))
    scored = normalize([(index, pool(hits, config)) for index, hits in evidence.items()], config)
    support = {
        index: (len({f for f, _, _ in hits}), len({b for _, _, b in hits}))
        for index, hits in evidence.items()
    }
    return Heatmap(
        frame=target_frame,
        voxel_size=voxel_size,
        voxels=scored,
        support=support,
        stats={
            "hot_patches": len(hot_patches),
            "hot_patches_without_depth": without_depth,
            "keyframes_placed": sum(1 for p in poses.values() if p is not None),
            "keyframes_seen": len(poses),
            "voxels_touched": len(evidence),
        },
    )


def normalize(
    scored: list[tuple[tuple[int, int, int], float]], config: QueryConfig
) -> list[tuple[tuple[int, int, int], float]]:
    """Scores scaled so the ``normalize_percentile`` voxel is 1, clipped to
    [0, 1], best first."""
    if scored:
        values = np.sort(np.array([s for _, s in scored]))
        rank = round((len(values) - 1) * min(max(config.normalize_percentile, 0.0), 1.0))
        top = max(float(values[rank]), 1e-9)
        scored = [(index, min(max(s / top, 0.0), 1.0)) for index, s in scored]
    return sorted(scored, key=lambda item: (-item[1], item[0]))


def combine(patches: Heatmap, segments: Heatmap, config: QueryConfig) -> Heatmap:
    """The two channels added: patch score + ``segment_weight`` x segment
    score per voxel, renormalized, so voxels both channels light up rank
    first. Keeps each channel's own score in ``channels``."""
    channels: dict[tuple[int, int, int], tuple[float, float]] = {
        index: (score, 0.0) for index, score in patches.voxels
    }
    for index, score in segments.voxels:
        channels[index] = (channels.get(index, (0.0, 0.0))[0], score)
    summed = [(index, e + config.segment_weight * s) for index, (e, s) in channels.items()]
    stats = dict(patches.stats)
    stats.update({f"segment_{k}": v for k, v in segments.stats.items()})
    stats["voxels_in_both"] = sum(1 for e, s in channels.values() if e > 0 and s > 0)
    support = dict(patches.support)
    for index, (frames, bins) in segments.support.items():
        had = support.get(index, (0, 0))
        support[index] = (had[0] + frames, max(had[1], bins))
    return Heatmap(
        frame=patches.frame,
        voxel_size=patches.voxel_size,
        voxels=normalize(summed, config),
        stats=stats,
        channels=channels,
        support=support,
    )


def transform_matrix(
    translation: NDArray[np.floating], quaternion_xyzw: NDArray[np.floating]
) -> NDArray[np.float64]:
    """4x4 homogeneous matrix from a translation and an xyzw quaternion."""
    x, y, z, w = (float(v) for v in quaternion_xyzw)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix
