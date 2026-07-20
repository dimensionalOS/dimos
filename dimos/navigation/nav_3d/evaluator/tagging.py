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

"""Geometric tags for a case: elevation, narrow, doorway, corridor.

Elevation tags come from the endpoints. Shape tags measure corridor width along
the demonstrated route and apply only when it runs roughly straight from start
to goal. Thresholds come from the robot's dimensions in EvalConfig.
"""

from __future__ import annotations

import itertools
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator.metrics import (
    MARGIN_CAP_M,
    arc_lengths,
    body_frames,
    densify,
    path_length,
)
from dimos.navigation.nav_3d.evaluator.voxel_keys import keys_contain, voxel_keys

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.config import EvalConfig

# A pair of endpoints this far apart in z or beyond is a climb, not a flat
# traverse. Half the body height, the smallest step the elevation tags care to
# call stairs.
STAIRS_DZ_M = 0.5
# A climb earns "long" past either bound: a tall total rise or a long walk.
LONG_STAIRS_DZ_M = 1.5
LONG_STAIRS_WALKED_M = 20.0
# A sustained narrow stretch this long is a corridor, not a doorway.
CORRIDOR_RUN_M = 2.0
# A doorway pinch is no longer than this. Beyond it the passage is a corridor.
DOORWAY_MAX_RUN_M = 1.2
# Open space must reappear within this arc on both sides of a pinch for it to be
# a doorway rather than a dead-end narrowing.
DOORWAY_FLANK_M = 1.4
# A door frame makes the width wobble across the threshold, splitting one pinch
# into fragments. Merge narrow runs separated by gaps this small so a sharp
# doorway reads as one passage, not several.
NARROW_MERGE_GAP_M = 0.2
# A narrow run shorter than this is a single stray voxel, not a passage.
NARROW_MIN_RUN_M = 0.15
# Path-shape tags describe the local terrain between the endpoints, so they only
# apply when the demonstrated route runs roughly straight from start to goal. A
# route far longer than the straight line is a detour through the building, and
# one far shorter is a stub that never spans the endpoints. Neither describes
# the case. Kept strict for precision: a doorway tag the filter can trust is
# worth more than catching every winding-route doorway.
LOCAL_DETOUR_MAX = 2.0
LOCAL_SPAN_MIN_FRAC = 0.8

# The tags this module owns and recomputes. Everything else on a case, such as
# auto, manual, negative, or dynamic provenance, is left untouched by a retag.
GEOMETRIC_TAGS = frozenset(
    {"flat", "up", "down", "stairs", "long", "narrow", "doorway", "corridor"}
)


def elevation_tags(
    start: tuple[float, float, float], goal: tuple[float, float, float]
) -> list[str]:
    """Elevation from the case endpoints, matching how generation labels them.

    Endpoints, not the recovered route: the route can wander far off the
    straight line, and a case's climb is defined by where it starts and ends.
    """
    dz = goal[2] - start[2]
    euclid = float(np.linalg.norm(np.asarray(goal) - np.asarray(start)))
    if abs(dz) < STAIRS_DZ_M:
        return ["flat"]
    tags = ["stairs", "up" if dz > 0 else "down"]
    if abs(dz) >= LONG_STAIRS_DZ_M or euclid >= LONG_STAIRS_WALKED_M:
        tags.append("long")
    return tags


def _corridor_width(
    samples: NDArray[np.float32], occupied_keys: NDArray[np.int64], cfg: EvalConfig
) -> NDArray[np.float64]:
    """Free lateral width at each densified sample, at body height.

    Probes outward along both body-lateral directions from a point chest-high
    over the path and returns left-plus-right distance to the nearest occupied
    voxel, capped when the passage is open. This is the room the body has to
    pass, not the room the feet have to stand.
    """
    _, lateral, _ = body_frames(samples, cfg.robot_length)
    mid_z = (cfg.ground_margin + cfg.body_clearance) / 2.0
    origin = samples.astype(np.float64) + np.array([0.0, 0.0, mid_z])
    max_probe = cfg.robot_length + MARGIN_CAP_M
    steps = np.arange(cfg.voxel_size, max_probe + cfg.voxel_size, cfg.voxel_size)

    def side_dist(sign: float) -> NDArray[np.float64]:
        pts = origin[:, None, :] + sign * steps[None, :, None] * lateral[:, None, :]
        hit = keys_contain(occupied_keys, voxel_keys(pts.reshape(-1, 3), cfg.voxel_size))
        hit = hit.reshape(len(samples), len(steps))
        any_hit = hit.any(axis=1)
        first = np.where(any_hit, steps[hit.argmax(axis=1)], max_probe)
        return first

    return side_dist(1.0) + side_dist(-1.0)


def _runs(mask: NDArray[np.bool_], arc: NDArray[np.float64]) -> list[tuple[float, float]]:
    """Arc-length (start, end) of every maximal True run in mask."""
    out: list[tuple[float, float]] = []
    i = 0
    for value, group in itertools.groupby(mask):
        n = sum(1 for _ in group)
        if value:
            out.append((float(arc[i]), float(arc[i + n - 1])))
        i += n
    return out


def _merge(runs: list[tuple[float, float]], gap: float) -> list[tuple[float, float]]:
    """Join runs separated by less than gap of arc length."""
    merged: list[tuple[float, float]] = []
    for lo, hi in runs:
        if merged and lo - merged[-1][1] <= gap:
            merged[-1] = (merged[-1][0], hi)
        else:
            merged.append((lo, hi))
    return merged


def _corridor_tags(
    route: NDArray[np.float32], occupied_keys: NDArray[np.int64], cfg: EvalConfig
) -> list[str]:
    if len(occupied_keys) == 0 or len(route) < 2:
        return []
    samples = densify(route, cfg.voxel_size)
    width = _corridor_width(samples, occupied_keys, cfg)
    tight = cfg.robot_width + 2.0 * MARGIN_CAP_M
    roomy = cfg.robot_width + 2.0 * cfg.robot_length
    arc = arc_lengths(samples)
    # A real passage is at least the robot's own body wide. Anything tighter is
    # furniture or map noise the robot could not have walked through, so it does
    # not count as a passage.
    narrow = (width >= cfg.robot_width) & (width < tight)
    runs = [
        (lo, hi)
        for lo, hi in _merge(_runs(narrow, arc), NARROW_MERGE_GAP_M)
        if hi - lo >= NARROW_MIN_RUN_M
    ]
    if not runs:
        return []
    if max(hi - lo for lo, hi in runs) >= CORRIDOR_RUN_M:
        return ["corridor", "narrow"]
    # A doorway is a short pinch with open space reappearing on both sides.
    for lo, hi in runs:
        before = width[(arc >= lo - DOORWAY_FLANK_M) & (arc < lo)]
        after = width[(arc > hi) & (arc <= hi + DOORWAY_FLANK_M)]
        if (
            hi - lo <= DOORWAY_MAX_RUN_M
            and bool((before >= roomy).any())
            and bool((after >= roomy).any())
        ):
            return ["doorway", "narrow"]
    return ["narrow"]


def _is_local(
    route: NDArray[np.float32],
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    cfg: EvalConfig,
) -> bool:
    """True when the route runs roughly straight from start to goal.

    Endpoints closer than a body length have no terrain to describe. Otherwise
    the walked route must span the straight line without detouring far past it.
    """
    eucl = float(np.linalg.norm(np.asarray(goal) - np.asarray(start)))
    if eucl < cfg.robot_length:
        return False
    arc = path_length(route)
    return LOCAL_SPAN_MIN_FRAC * eucl <= arc <= LOCAL_DETOUR_MAX * eucl + cfg.robot_length


def route_tags(
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    route: NDArray[np.float32] | None,
    occupied_keys: NDArray[np.int64],
    cfg: EvalConfig,
) -> list[str]:
    """Geometric tags for a case, in a stable order.

    Elevation comes from the endpoints. Path-shape tags describe the local
    terrain between them and need a walked route that runs roughly straight from
    start to goal, so they are skipped when the route is off the trajectory or a
    long detour. Deterministic and free of provenance: the caller prepends auto
    or manual.
    """
    tags = elevation_tags(start, goal)
    if route is not None and len(route) >= 2 and _is_local(route, start, goal, cfg):
        tags += _corridor_tags(route, occupied_keys, cfg)
    return tags
