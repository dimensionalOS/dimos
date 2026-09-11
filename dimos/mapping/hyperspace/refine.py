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

"""Turning a raw voxel heat map into a few clean, object-shaped answers.

The raw map (:func:`patches.heatmap`) is a sparse cloud of small blobs. Each
function here is ``Heatmap -> Heatmap`` on the voxels at or above the cutoff,
in numpy/scipy, with no model and no I/O:

- ``closing``: inflate then erode (Jeff's ask): fills 1-2 voxel gaps and
  smooths blob surfaces; ``opening`` is the reverse and drops specks.
- ``gaussian``: blur the score field and re-threshold.
- ``support``: keep voxels seen from enough keyframes and directions.
- ``occupancy``: keep voxels next to depth-observed geometry.
- ``structural``: drop voxels lying on floor, wall or ceiling surfaces (from
  the segment records) unless the query is about those: a cone is not a
  patch of floor, however cone-like the floor beside a cone looks to SigLIP.
- ``prior``: common-sense size: a cluster larger than the thing asked for
  is split around its peaks and trimmed to a plausible thickness.
- ``components``: label 26-connected clusters, rank them, cut the small
  and weak ones.

:func:`refine` runs the configured chain and always ends with cluster
labelling, so the result carries ranked :class:`patches.Cluster` answers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np
from scipy import ndimage

from dimos.mapping.hyperspace.patches import Cluster, Heatmap

if TYPE_CHECKING:
    from collections.abc import Iterable

    from numpy.typing import NDArray

Index = tuple[int, int, int]


def _index(cell: NDArray[np.integer] | Iterable[int]) -> Index:
    a, b, c = (int(v) for v in cell)
    return (a, b, c)


def _xyz(values: NDArray[np.floating] | Iterable[float]) -> tuple[float, float, float]:
    a, b, c = (float(v) for v in values)
    return (a, b, c)


# 26-connectivity: touching by face, edge or corner counts.
CONNECTIVITY = ndimage.generate_binary_structure(3, 3)


@dataclass
class SizePrior:
    """How big a thing can be, in metres: longest horizontal side, shortest
    horizontal side (thickness), height."""

    length: float
    thickness: float
    height: float


# Common things a robot is asked about, keyed by a word in the query. The
# generic fallback is what "reduce the output space" means when the query is
# something we have no idea about: nothing indoors is 3 m long and 1.5 m thick.
SIZE_PRIORS: dict[str, SizePrior] = {
    "chair": SizePrior(1.0, 1.0, 1.4),
    "stool": SizePrior(0.6, 0.6, 1.0),
    "sofa": SizePrior(2.5, 1.2, 1.2),
    "couch": SizePrior(2.5, 1.2, 1.2),
    "cone": SizePrior(0.5, 0.5, 1.0),
    "person": SizePrior(0.9, 0.9, 2.0),
    "people": SizePrior(0.9, 0.9, 2.0),
    "table": SizePrior(2.5, 1.5, 1.2),
    "desk": SizePrior(2.5, 1.5, 1.2),
    # Double doors are ~1.8 m wide; a door is never thick (Jeff, 2026-09-10).
    "door": SizePrior(2.0, 0.3, 2.4),
    "monitor": SizePrior(0.8, 0.3, 0.6),
    "screen": SizePrior(1.2, 0.3, 0.8),
    "tv": SizePrior(1.5, 0.3, 1.0),
    "laptop": SizePrior(0.5, 0.4, 0.4),
    "plant": SizePrior(1.2, 1.2, 2.2),
    "trash": SizePrior(0.7, 0.7, 1.3),
    "bin": SizePrior(0.7, 0.7, 1.3),
    "box": SizePrior(1.2, 1.2, 1.2),
    "bag": SizePrior(0.7, 0.7, 0.8),
    "bottle": SizePrior(0.2, 0.2, 0.4),
    "cup": SizePrior(0.2, 0.2, 0.2),
    "fire extinguisher": SizePrior(0.3, 0.3, 0.8),
    "cabinet": SizePrior(2.0, 0.8, 2.2),
    "shelf": SizePrior(2.5, 0.6, 2.4),
    "window": SizePrior(3.0, 0.3, 2.0),
    "dog": SizePrior(1.2, 0.6, 1.0),
    "robot": SizePrior(1.2, 1.0, 1.5),
}
GENERIC_PRIOR = SizePrior(2.5, 1.2, 2.5)


def size_prior(text: str) -> SizePrior:
    """The prior for a query: the longest matching key wins, else generic."""
    lowered = text.lower()
    matches = [key for key in SIZE_PRIORS if key in lowered]
    if not matches:
        return GENERIC_PRIOR
    return SIZE_PRIORS[max(matches, key=len)]


@dataclass
class RefineConfig:
    # Steps in order; "components" always runs last whether listed or not.
    methods: list[str] = field(default_factory=list)
    # Voxels below this (0..1) are ignored by every step.
    cutoff: float = 0.3
    closing_radius: int = 1
    opening_radius: int = 1
    gaussian_sigma: float = 1.0
    # Support: voxels seen from fewer keyframes / yaw bins than this go.
    min_frames: int = 2
    min_bins: int = 1
    # Occupancy: a voxel must be within this many voxels of observed depth.
    occupancy_radius: int = 1
    # Structural: voxels within this many voxels of a floor/wall/ceiling
    # surface go, unless the query names one of those.
    structural_radius: int = 1
    # Components: clusters smaller than this, or scoring under min_ratio x
    # the best cluster, are dropped; top_k 0 keeps every survivor.
    min_cluster: int = 6
    min_ratio: float = 0.0
    top_k: int = 0
    # Clusters whose centres are closer than this (m) are one answer: the
    # same object placed twice by depth error from different distances.
    merge_distance: float = 0.6


METHODS = (
    "closing",
    "opening",
    "gaussian",
    "support",
    "occupancy",
    "structural",
    "prior",
    "components",
)
# Segment labels whose surfaces the ``structural`` step removes heat from.
STRUCTURAL_LABELS = ("floor", "wall", "ceiling")


class Grid:
    """A dense box around a set of voxel indices, for scipy."""

    def __init__(self, indices: Iterable[Index], pad: int = 3) -> None:
        points = np.asarray(list(indices), dtype=int).reshape(-1, 3)
        self.lo = points.min(axis=0) - pad
        self.shape = tuple(int(v) for v in points.max(axis=0) - self.lo + 1 + pad)

    def field(self, voxels: Iterable[tuple[Index, float]]) -> NDArray[np.float32]:
        out = np.zeros(self.shape, dtype=np.float32)
        for index, score in voxels:
            out[tuple(np.asarray(index) - self.lo)] = score
        return out

    def voxels(self, scores: NDArray[np.floating]) -> list[tuple[Index, float]]:
        cells = np.argwhere(scores > 0)
        return [(_index(cell + self.lo), float(scores[tuple(cell)])) for cell in cells]

    def local(self, index: Index) -> Index:
        return _index(np.asarray(index) - self.lo)


def ball(radius: int) -> NDArray[np.bool_]:
    """Every voxel within ``radius`` steps (Chebyshev): a (2r+1)^3 box;
    radius 0 is the voxel itself."""
    if radius <= 0:
        return np.ones((1, 1, 1), dtype=bool)
    if radius == 1:
        return CONNECTIVITY
    return ndimage.iterate_structure(CONNECTIVITY, radius)


def _hot(heat: Heatmap, cutoff: float) -> list[tuple[Index, float]]:
    return [(index, score) for index, score in heat.voxels if score >= cutoff]


def _with(heat: Heatmap, voxels: list[tuple[Index, float]]) -> Heatmap:
    kept = {index for index, _ in voxels}
    return Heatmap(
        frame=heat.frame,
        voxel_size=heat.voxel_size,
        voxels=sorted(voxels, key=lambda item: (-item[1], item[0])),
        stats=dict(heat.stats),
        channels={i: c for i, c in heat.channels.items() if i in kept},
        support={i: s for i, s in heat.support.items() if i in kept},
    )


def closing(heat: Heatmap, config: RefineConfig) -> Heatmap:
    """Inflate then erode: gaps up to 2x the radius close, surfaces smooth.
    Filled voxels get the grey-scale closing of the score field."""
    hot = _hot(heat, config.cutoff)
    if not hot:
        return _with(heat, [])
    grid = Grid((i for i, _ in hot), pad=config.closing_radius + 2)
    scores = grid.field(hot)
    footprint = ball(config.closing_radius)
    mask = ndimage.binary_erosion(ndimage.binary_dilation(scores > 0, footprint), footprint)
    grey = ndimage.grey_erosion(
        ndimage.grey_dilation(scores, footprint=footprint), footprint=footprint
    )
    return _with(heat, grid.voxels(np.where(mask, np.maximum(grey, scores), 0.0)))


def opening(heat: Heatmap, config: RefineConfig) -> Heatmap:
    """Erode then inflate: specks thinner than 2x the radius vanish."""
    hot = _hot(heat, config.cutoff)
    if not hot:
        return _with(heat, [])
    grid = Grid((i for i, _ in hot), pad=config.opening_radius + 2)
    scores = grid.field(hot)
    footprint = ball(config.opening_radius)
    mask = ndimage.binary_dilation(ndimage.binary_erosion(scores > 0, footprint), footprint)
    return _with(heat, grid.voxels(np.where(mask, scores, 0.0)))


def gaussian(heat: Heatmap, config: RefineConfig) -> Heatmap:
    """Blur the score field, rescale so the top is 1, re-apply the cutoff."""
    hot = _hot(heat, config.cutoff)
    if not hot:
        return _with(heat, [])
    grid = Grid((i for i, _ in hot), pad=int(3 * config.gaussian_sigma) + 2)
    blurred = ndimage.gaussian_filter(grid.field(hot), config.gaussian_sigma)
    blurred /= max(float(blurred.max()), 1e-9)
    return _with(heat, grid.voxels(np.where(blurred >= config.cutoff, blurred, 0.0)))


def support(heat: Heatmap, config: RefineConfig) -> Heatmap:
    """Keep voxels seen from at least ``min_frames`` keyframes and
    ``min_bins`` yaw bins; a single view cannot make an answer."""
    kept = [
        (index, score)
        for index, score in _hot(heat, config.cutoff)
        if heat.support.get(index, (1, 1))[0] >= config.min_frames
        and heat.support.get(index, (1, 1))[1] >= config.min_bins
    ]
    return _with(heat, kept)


def occupancy(heat: Heatmap, config: RefineConfig, scene: Iterable[Index]) -> Heatmap:
    """Keep voxels within ``occupancy_radius`` of depth-observed geometry."""
    hot = _hot(heat, config.cutoff)
    scene_points = np.asarray(list(scene), dtype=int).reshape(-1, 3)
    if not hot or not len(scene_points):
        return _with(heat, [])
    grid = Grid((i for i, _ in hot), pad=config.occupancy_radius + 2)
    local = scene_points - grid.lo
    inside = np.all((local >= 0) & (local < np.asarray(grid.shape)), axis=1)
    occupied = np.zeros(grid.shape, dtype=bool)
    occupied[tuple(local[inside].T)] = True
    occupied = ndimage.binary_dilation(occupied, ball(config.occupancy_radius))
    return _with(heat, [(i, s) for i, s in hot if occupied[grid.local(i)]])


def structural(
    heat: Heatmap, config: RefineConfig, surfaces: Iterable[Index], text: str = ""
) -> Heatmap:
    """Drop voxels within ``structural_radius`` of a floor/wall/ceiling
    surface, unless the query is about one of those."""
    if any(label in text.lower() for label in STRUCTURAL_LABELS):
        return _with(heat, _hot(heat, config.cutoff))
    hot = _hot(heat, config.cutoff)
    points = np.asarray(list(surfaces), dtype=int).reshape(-1, 3)
    if not hot or not len(points):
        return _with(heat, hot)
    grid = Grid((i for i, _ in hot), pad=config.structural_radius + 2)
    local = points - grid.lo
    inside = np.all((local >= 0) & (local < np.asarray(grid.shape)), axis=1)
    covered = np.zeros(grid.shape, dtype=bool)
    covered[tuple(local[inside].T)] = True
    covered = ndimage.binary_dilation(covered, ball(config.structural_radius))
    return _with(heat, [(i, s) for i, s in hot if not covered[grid.local(i)]])


def prior(heat: Heatmap, config: RefineConfig, size: SizePrior) -> Heatmap:
    """Common-sense size. Each cluster larger than the prior is carved into
    boxes of the prior's size around successive peaks (a run of 12 chairs
    becomes 12 answers, not one), and each piece is trimmed across its minor
    horizontal axis to the prior's thickness (a wall of "door" 4 ft thick
    keeps its front face)."""
    hot = _hot(heat, config.cutoff)
    if not hot:
        return _with(heat, [])
    grid = Grid((i for i, _ in hot))
    scores = grid.field(hot)
    labels, count = ndimage.label(scores > 0, structure=CONNECTIVITY)
    size_vox = np.array([size.length, size.length, size.height]) / heat.voxel_size
    half = np.maximum(np.floor(size_vox / 2).astype(int), 1)
    kept: list[tuple[Index, float]] = []
    for label in range(1, count + 1):
        remaining = scores.copy()
        remaining[labels != label] = 0.0
        while (remaining > 0).sum() >= max(config.min_cluster, 1):
            centre = np.asarray(np.unravel_index(int(np.argmax(remaining)), remaining.shape))
            # A few mean-shift steps: slide the window onto the local mass so
            # a piece is centred on an object, not on whichever voxel scored
            # highest at its edge.
            for _ in range(4):
                lo = np.maximum(centre - half, 0)
                hi = np.minimum(centre + half + 1, np.asarray(remaining.shape))
                cells = np.argwhere(remaining[lo[0] : hi[0], lo[1] : hi[1], lo[2] : hi[2]] > 0)
                if not len(cells):
                    break
                weights = remaining[tuple((cells + lo).T)]
                centre = np.round(np.average(cells + lo, axis=0, weights=weights)).astype(int)
            lo = np.maximum(centre - half, 0)
            hi = np.minimum(centre + half + 1, np.asarray(remaining.shape))
            window = np.zeros_like(remaining, dtype=bool)
            window[lo[0] : hi[0], lo[1] : hi[1], lo[2] : hi[2]] = True
            # The piece keeps one voxel of margin inside its window, so two
            # neighbouring pieces come out as two clusters, not one.
            inner = np.zeros_like(window)
            inner[lo[0] + 1 : hi[0] - 1, lo[1] + 1 : hi[1] - 1, lo[2] : hi[2]] = True
            piece = np.argwhere(inner & (remaining > 0))
            kept.extend(_trim_thickness(piece, remaining, size.thickness / heat.voxel_size, grid))
            remaining[window] = 0.0
    return _with(heat, kept)


def _trim_thickness(
    piece: NDArray[np.integer], scores: NDArray[np.floating], thickness_vox: float, grid: Grid
) -> list[tuple[Index, float]]:
    """Drop the voxels of ``piece`` farther than half the thickness from its
    horizontal principal line, so a slab keeps one face."""
    if len(piece) < 3:
        return [(_index(c + grid.lo), float(scores[tuple(c)])) for c in piece]
    weights = np.array([scores[tuple(c)] for c in piece], dtype=np.float64)
    xy = piece[:, :2].astype(np.float64)
    centre = np.average(xy, axis=0, weights=weights)
    centred = xy - centre
    covariance = (centred * weights[:, None]).T @ centred / max(weights.sum(), 1e-9)
    _, vectors = np.linalg.eigh(covariance)
    minor = vectors[:, 0]  # eigh sorts ascending: first is the thin direction
    offset = np.abs(centred @ minor)
    keep = offset <= max(thickness_vox / 2, 0.5)
    return [(_index(c + grid.lo), float(scores[tuple(c)])) for c in piece[keep]]


def _merge_close(
    labels: NDArray[np.integer],
    count: int,
    scores: NDArray[np.floating],
    voxel_size: float,
    distance: float,
) -> tuple[NDArray[np.integer], int]:
    """Relabel so clusters whose score-weighted centres lie within
    ``distance`` metres share a label (single linkage), renumbered 1..n."""
    if count < 2 or distance <= 0:
        return labels, count
    ids = np.arange(1, count + 1)
    centres = np.asarray(ndimage.center_of_mass(scores, labels, ids), dtype=np.float64)
    parent = list(range(count))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    limit = distance / voxel_size
    for a in range(count):
        near = np.flatnonzero(np.linalg.norm(centres[a + 1 :] - centres[a], axis=1) <= limit)
        for b in near + a + 1:
            parent[find(int(b))] = find(a)
    roots = sorted({find(i) for i in range(count)})
    renumber = np.zeros(count + 1, dtype=labels.dtype)
    for i in range(count):
        renumber[i + 1] = roots.index(find(i)) + 1
    return renumber[labels], len(roots)


def components(heat: Heatmap, config: RefineConfig) -> Heatmap:
    """Label 26-connected clusters, rank by summed score, cut the small and
    weak ones, and write ranked :class:`Cluster` answers onto the map."""
    hot = _hot(heat, config.cutoff)
    if not hot:
        return _with(heat, [])
    grid = Grid((i for i, _ in hot))
    scores = grid.field(hot)
    labels, count = ndimage.label(scores > 0, structure=CONNECTIVITY)
    labels, count = _merge_close(labels, count, scores, heat.voxel_size, config.merge_distance)
    ids = np.arange(1, count + 1)
    sums = ndimage.sum_labels(scores, labels, index=ids)
    sizes = ndimage.sum_labels(scores > 0, labels, index=ids)
    order = [int(i) for i in np.argsort(-sums)]
    best = float(sums[order[0]]) if order else 0.0
    clusters: list[Cluster] = []
    cluster_of: dict[Index, int] = {}
    kept: list[tuple[Index, float]] = []
    for position in order:
        if sizes[position] < config.min_cluster or sums[position] < config.min_ratio * best:
            continue
        if config.top_k and len(clusters) >= config.top_k:
            break
        cells = np.argwhere(labels == position + 1)
        weights = scores[tuple(cells.T)].astype(np.float64)
        world = (cells + grid.lo + 0.5) * heat.voxel_size
        centre = np.average(world, axis=0, weights=weights)
        extent = (cells.max(axis=0) - cells.min(axis=0) + 1) * heat.voxel_size
        peak = cells[int(np.argmax(weights))] + grid.lo
        rank = len(clusters)
        clusters.append(
            Cluster(
                rank=rank,
                score=float(sums[position] / best) if best else 0.0,
                voxels=int(sizes[position]),
                centre=_xyz(centre),
                extent=_xyz(extent),
                peak=_index(peak),
            )
        )
        for cell, weight in zip(cells, weights, strict=True):
            index = _index(cell + grid.lo)
            cluster_of[index] = rank
            kept.append((index, float(weight)))
    result = _with(heat, kept)
    result.clusters = clusters
    result.cluster_of = cluster_of
    return result


def refine(
    heat: Heatmap,
    config: RefineConfig,
    *,
    scene: Iterable[Index] | None = None,
    surfaces: Iterable[Index] | None = None,
    text: str = "",
) -> Heatmap:
    """Run ``config.methods`` in order, then label clusters. ``scene`` (the
    depth-observed voxels) is needed by ``occupancy``, ``surfaces`` (the
    floor/wall/ceiling voxels) by ``structural``; ``text`` picks the size
    prior and exempts structural queries."""
    result = heat
    for name in config.methods:
        if name == "components":
            continue
        if name == "closing":
            result = closing(result, config)
        elif name == "opening":
            result = opening(result, config)
        elif name == "gaussian":
            result = gaussian(result, config)
        elif name == "support":
            result = support(result, config)
        elif name == "occupancy":
            result = occupancy(result, config, scene or [])
        elif name == "structural":
            result = structural(result, config, surfaces or [], text)
        elif name == "prior":
            result = prior(result, config, size_prior(text))
        else:
            raise ValueError(f"unknown refine method {name!r}; choose from {METHODS}")
    result = components(result, config)
    result.stats["refine"] = [*config.methods, "components"]
    result.stats["clusters"] = len(result.clusters)
    return result


def refine_config_of(spec: str, default: str, cutoff: float = 0.3) -> RefineConfig | None:
    """The refinement a config string asks for: "default" means ``default``
    (normally ``QueryConfig.refine``), "" or "none" means the raw map, else
    a comma separated chain of methods."""
    if spec == "default":
        spec = default
    if spec in ("", "none"):
        return None
    return RefineConfig(methods=[m.strip() for m in spec.split(",") if m.strip()], cutoff=cutoff)
