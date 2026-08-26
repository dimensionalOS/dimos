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

"""Cases from a recorded trajectory. Endpoints come off the walked path."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator.cases import Case
from dimos.navigation.nav_3d.evaluator.tagging import STAIRS_DZ_M, elevation_tags

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.config import EvalConfig
    from dimos.navigation.nav_3d.evaluator.recording import Trajectory


# Beyond this a near-straight flat pair is trivial whatever the map holds.
MAX_TRIVIAL_SPAN_M = 30.0


# Candidate pairs must be this far apart along the walk and in a straight line.
MIN_SEPARATION_M = 3.0
MIN_EUCLID_M = 2.0
# A flat pair is only interesting if the walk detoured this much over the
# straight line.
DETOUR_RATIO_MIN = 1.3
# Endpoint pairs are binned this coarsely before ranking, so near-identical
# pairs compete for one slot.
BIN_SIZE_M = 2.0
WAYPOINT_SPACING_M = 1.0
# Two cases are duplicates when both endpoints land within this radius.
DEDUPE_RADIUS_M = 1.5
# Share of slots reserved for flat cases when the recording has them.
FLAT_FRACTION = 0.25
# Endpoint spread stops earning score past this distance, and is worth this
# much next to a candidate's own priority.
SPREAD_CAP_M = 16.0
SPREAD_WEIGHT = 0.4
# A waypoint counts as revisited when the walk comes back within this.
REVISIT_RADIUS_M = 1.0
# Floor on the case count. When strict selection falls short, a relaxed pass
# ignores the sector caps and the flat quota to reach it.
MIN_CASES = 10
# An unpinned case count scales with the walk, between these bounds.
METERS_PER_CASE = 25.0
MIN_AUTO_CASES = 16
MAX_AUTO_CASES = 48


def resolve_max_cases(max_cases: int | None, walked_total_m: float) -> int:
    """Case count, scaled with the walked distance when not pinned."""
    if max_cases is not None:
        return max_cases
    return int(np.clip(walked_total_m / METERS_PER_CASE, MIN_AUTO_CASES, MAX_AUTO_CASES))


@dataclass
class Candidate:
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    walked_m: float
    detour_ratio: float
    dz: float

    @property
    def priority(self) -> float:
        return (
            min(self.detour_ratio, 3.0)
            + 2.0 * min(abs(self.dz), 3.0)
            + 0.5 * min(self.walked_m / 50.0, 2.0)
        )


def _subsample_indices(trajectory: Trajectory, spacing_m: float) -> NDArray[np.int64]:
    arcs = trajectory.arc_lengths()
    targets = np.arange(0.0, arcs[-1], spacing_m)
    return np.unique(np.searchsorted(arcs, targets))


def _candidates(
    trajectory: Trajectory,
    idx: NDArray[np.int64],
    points: NDArray[np.float32],
    arcs: NDArray[np.float64],
    cfg: EvalConfig,
) -> dict[tuple[int, ...], Candidate]:
    """The best candidate pair per spatial bin, over every ordered waypoint pair."""
    foot = trajectory.foot(cfg.robot_height)
    way_arcs = arcs[idx]
    candidates: dict[tuple[int, ...], Candidate] = {}
    for ai in range(len(idx)):
        sa = points[ai]
        near_a = np.linalg.norm(foot - sa, axis=1) <= REVISIT_RADIUS_M
        last_visit_a = float(trajectory.ts[near_a].max()) if near_a.any() else -np.inf
        later = np.arange(ai + 1, len(idx))
        if not len(later):
            continue
        walked = way_arcs[later] - way_arcs[ai]
        deltas = points[later] - sa
        euclid = np.linalg.norm(deltas, axis=1)
        keep = (walked >= MIN_SEPARATION_M) & (euclid >= MIN_EUCLID_M)
        for bi, w, e in zip(later[keep], walked[keep], euclid[keep], strict=True):
            sb = points[bi]
            dz = float(sb[2] - sa[2])
            detour = float(w / e)
            # Backward in time is always causal. Forward only when the start
            # spot is revisited after the goal visit.
            directed = [(sb, sa, -dz)]
            if last_visit_a >= float(trajectory.ts[idx[bi]]):
                directed.append((sa, sb, dz))
            proposed = [
                (
                    Candidate(
                        start=(float(p_start[0]), float(p_start[1]), float(p_start[2])),
                        goal=(float(p_goal[0]), float(p_goal[1]), float(p_goal[2])),
                        walked_m=float(w),
                        detour_ratio=detour,
                        dz=d_dz,
                    ),
                    _bin_key(p_start, p_goal, d_dz, BIN_SIZE_M),
                )
                for p_start, p_goal, d_dz in directed
            ]
            # The sweep only decides admission, never priority, so a pair that
            # cannot win any of its bins never has to pay for one.
            if all(
                (best := candidates.get(key)) is not None and best.priority >= cand.priority
                for cand, key in proposed
            ):
                continue
            # A long near-straight flat pair demonstrates nothing.
            if detour < DETOUR_RATIO_MIN and abs(dz) < STAIRS_DZ_M and e > MAX_TRIVIAL_SPAN_M:
                continue
            for cand, key in proposed:
                best = candidates.get(key)
                if best is None or cand.priority > best.priority:
                    candidates[key] = cand
    return candidates


def generate_cases(
    trajectory: Trajectory,
    cfg: EvalConfig,
    max_cases: int | None = None,
    min_cases: int = MIN_CASES,
) -> list[Case]:
    """A diverse spread of demonstrated routes, endpoints straight off the odometry."""
    arcs = trajectory.arc_lengths()
    idx = _subsample_indices(trajectory, WAYPOINT_SPACING_M)
    points = trajectory.foot(cfg.robot_height)[idx]
    candidates = _candidates(trajectory, idx, points, arcs, cfg)
    ranked = sorted(candidates.values(), key=lambda c: (-c.priority, c.start, c.goal))
    selected = _select_diverse(ranked, resolve_max_cases(max_cases, float(arcs[-1])), min_cases)
    return [
        _to_case(cand, n, elevation_tags(cand.start, cand.goal)) for n, cand in enumerate(selected)
    ]


def _bin_key(
    start: NDArray[np.float32], goal: NDArray[np.float32], dz: float, bin_size_m: float
) -> tuple[int, int, int, int, int]:
    return (
        int(start[0] // bin_size_m),
        int(start[1] // bin_size_m),
        int(goal[0] // bin_size_m),
        int(goal[1] // bin_size_m),
        (1 if dz > 0 else -1) if abs(dz) >= STAIRS_DZ_M else 0,
    )


def _is_duplicate(cand: Candidate, accepted: list[Candidate], radius: float) -> bool:
    a = np.array([*cand.start, *cand.goal])
    for other in accepted:
        b = np.array([*other.start, *other.goal])
        if np.linalg.norm(a[:3] - b[:3]) < radius and np.linalg.norm(a[3:] - b[3:]) < radius:
            return True
    return False


class _DiverseSelector:
    """Spread-greedy selection state: availability, endpoint distance, flat/stairs quota."""

    def __init__(self, ranked: list[Candidate], max_cases: int) -> None:
        self.ranked = ranked
        self.flat_target = int(max_cases * FLAT_FRACTION)
        self.stairs_cap = max_cases - self.flat_target
        self.starts = np.array([c.start for c in ranked], dtype=np.float32)
        self.goals = np.array([c.goal for c in ranked], dtype=np.float32)
        self.priorities = np.array([c.priority for c in ranked], dtype=np.float32)
        self.is_stairs = np.array([abs(c.dz) >= STAIRS_DZ_M for c in ranked])
        self.alive = np.ones(len(ranked), dtype=bool)
        self.stairs: list[Candidate] = []
        self.flats: list[Candidate] = []
        # Running minima over the selected endpoints. Seeded to infinity so the
        # first pick sees the uniform spread an empty selection gives.
        self.d_start = np.full(len(ranked), np.inf, dtype=np.float32)
        self.d_goal = np.full(len(ranked), np.inf, dtype=np.float32)

    @property
    def selected(self) -> list[Candidate]:
        return self.stairs + self.flats

    def _accept(self, n: int) -> None:
        """Record a selection and fold its endpoints into the running minima."""
        for point in (self.starts[n], self.goals[n]):
            np.minimum(self.d_start, _distance_to(self.starts, point), out=self.d_start)
            np.minimum(self.d_goal, _distance_to(self.goals, point), out=self.d_goal)
        bucket = self.stairs if self.is_stairs[n] else self.flats
        bucket.append(self.ranked[n])

    def _scores(self, relax: bool) -> NDArray[np.float32]:
        spread = np.minimum(self.d_start, SPREAD_CAP_M) + np.minimum(self.d_goal, SPREAD_CAP_M)
        score = self.priorities + SPREAD_WEIGHT * spread
        score[~self.alive] = -np.inf
        if not relax and len(self.stairs) >= self.stairs_cap:
            score[self.is_stairs] = -np.inf
        return np.asarray(score, dtype=np.float32)

    def fill(self, target: int, relax: bool) -> None:
        """Take the highest-scoring live candidate until target is reached."""
        while self.alive.any() and len(self.selected) < target:
            score = self._scores(relax)
            if not np.isfinite(score).any():
                break
            n = int(score.argmax())
            self.alive[n] = False
            bucket = self.stairs if self.is_stairs[n] else self.flats
            if _is_duplicate(self.ranked[n], bucket, DEDUPE_RADIUS_M):
                continue
            self._accept(n)


def _distance_to(points: NDArray[np.float32], target: NDArray[np.float32]) -> NDArray[np.float32]:
    dist: NDArray[np.float32] = np.linalg.norm(points - target, axis=1)
    return dist


def _select_diverse(
    ranked: list[Candidate], max_cases: int, min_cases: int = MIN_CASES
) -> list[Candidate]:
    """Spread-greedy selection under a flat/stairs quota, relaxed to reach the floor."""
    if not ranked:
        return []
    selector = _DiverseSelector(ranked, max_cases)
    selector.fill(max_cases, relax=False)
    if len(selector.selected) < min(min_cases, max_cases):
        selector.fill(min(min_cases, max_cases), relax=True)
    return selector.selected[:max_cases]


def _to_case(cand: Candidate, n: int, tags: list[str]) -> Case:
    kind = "up" if cand.dz >= STAIRS_DZ_M else "down" if cand.dz <= -STAIRS_DZ_M else "flat"
    return Case(id=f"auto_{n:02d}_{kind}", start=cand.start, goal=cand.goal, tags=["auto", *tags])
