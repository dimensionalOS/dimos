#!/usr/bin/env python3
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

"""Pluggable relocalization priors -- candidate PROPOSERS feeding the shared
fine-ICP judge in relocalize.py (``refine_candidates``). No prior is trusted:
a candidate wins the pool by surviving the wall-only fine-fitness rerank,
never by its source's own reported confidence. Three priors live here:
``RansacPrior`` (wrapping relocalize.py's existing multi-scale FPFH+RANSAC
search), ``LastPosePrior`` (the last accepted answer, carried forward) and
``FiducialPrior`` (visual marker fixes, age-decayed). The same invariant
binds any future prior: it competes through the judge, never bypasses it.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import time
from typing import Protocol

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.mapping.relocalization.relocalize import generate_ransac_candidates, refine_candidates


@dataclass
class Candidate:
    """One proposed local_map->global_map transform, pre-judging.

    ``T`` uses relocalize()'s own convention: the 4x4 transform that maps
    ``local_map`` into ``global_map``'s frame (same as generate_ransac_
    candidates()'s and refine_candidates()'s transforms -- see
    ``relocalize()``'s docstring). A candidate carries only WHICH transform and
    WHICH source proposed it -- no self-reported confidence: ``refine_candidates``
    ranks every candidate purely on wall-only fine-scale fitness, so a candidate
    from a "trusted" prior still loses to one that actually fits the walls. (A
    calibrated composite confidence is the Phase-4 fusion arbiter's job, read
    from measured signals downstream of the judge, not asserted by the proposer.)
    """

    T: np.ndarray
    source: str


class RelocPrior(Protocol):
    """A relocalization candidate proposer.

    Implementations MUST NOT self-select a winner -- that is
    ``refine_candidates``'s job, always. Returning zero candidates (e.g. no
    marker in view, no prior pose seeded yet) is a valid, expected response,
    not an error.
    """

    name: str

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]: ...


class RansacPrior:
    """Wraps relocalize.py's existing multi-scale FPFH+RANSAC global search --
    a real geometric search over the full map, proposing its raw candidate pool
    into the judge like any other source."""

    name = "ransac"

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        transforms = generate_ransac_candidates(global_map, local_map)
        return [Candidate(T=T, source=self.name) for T in transforms]


class LastPosePrior:
    """Carries the last accepted relocalization forward as a single seed
    candidate for the next call -- cheap continuous tracking to compete
    alongside (not replace) a fresh RANSAC search.

    Frame convention (relocalize()'s own, NOT the published TF -- every pose
    here names its frame per repo convention): ``relocalize()``/
    ``refine_candidates()`` return T such that
    ``global_map_points = T @ local_map_points``. In the live pipeline
    (module.py) ``local_map`` is ``VoxelGridMapper``'s world-frame
    accumulated cloud and ``global_map`` is the loaded premap in the ``map``
    frame, so this stored seed is ``map_T_world``. module.py inverts
    relocalize()'s T to ``world_T_map`` only when it publishes the
    world->map TF -- callers of ``update()`` must pass the PRE-inversion T
    (relocalize()/refine_candidates's own return value), never the published
    TF.
    """

    name = "last_pose"

    def __init__(self) -> None:
        self._last_T: np.ndarray | None = None

    def update(self, T: np.ndarray) -> None:
        self._last_T = T

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        if self._last_T is None:
            return []
        return [Candidate(T=self._last_T, source=self.name)]


class FiducialPrior:
    """Fiducial-marker fixes as seed candidates, age-gated.

    ``update()`` takes the fix in relocalize()'s own convention — map_T_world,
    PRE-inversion, the same frame-direction rule as ``LastPosePrior`` (invert
    a published world->map TF before passing it). ``ts`` is the fix time in
    ``now_fn``'s own timebase; None stamps arrival time.

    A marker fix is a fresh measurement of the world->map edge that drifts with
    the odometry accumulated since the sighting, so propose() drops it once it
    is older than ``age_max_s`` (a hard cutoff — no decay curve: nothing in the
    judge weights a fix by age, and a stale fix still has to win on wall fitness
    like every other prior). ``age_max_s`` is an engineering guess, not a tuned
    value; tuning it belongs to #2137's autoresearch harness.
    """

    name = "fiducial"

    def __init__(
        self,
        *,
        age_max_s: float = 120.0,
        now_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        self._age_max_s = age_max_s
        self._now_fn = now_fn
        self._fix_T: np.ndarray | None = None
        self._fix_ts: float = 0.0

    def update(self, T: np.ndarray, ts: float | None = None) -> None:
        self._fix_T = T
        self._fix_ts = self._now_fn() if ts is None else ts

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        if self._fix_T is None:
            return []
        if self._now_fn() - self._fix_ts > self._age_max_s:
            return []
        return [Candidate(T=self._fix_T, source=self.name)]


def relocalize_with_priors(
    global_map: o3d.geometry.PointCloud,
    local_map: o3d.geometry.PointCloud,
    priors: list[RelocPrior],
) -> tuple[np.ndarray, float, str]:
    """Gather candidates from every prior, judge them all through the one
    shared fine-ICP tail (``refine_candidates``), and report which prior's
    candidate actually won.

    No source bypasses the judge: a prior with high self-reported confidence
    whose candidate doesn't fit the walls loses to one that does, same as
    plain ``relocalize()``'s RANSAC-only pool always has.

    Returns ``(T, fitness, winning_source)``. Raises ``ValueError`` if every
    prior proposed zero candidates -- there is nothing for the judge to rank.
    """
    all_transforms: list[np.ndarray] = []
    sources: list[str] = []
    for prior in priors:
        for candidate in prior.propose(global_map, local_map):
            all_transforms.append(candidate.T)
            sources.append(candidate.source)

    if not all_transforms:
        raise ValueError("relocalize_with_priors: no prior proposed any candidate")

    # sources= makes the judge's gravity gate per-source — a lone upright
    # seed must never orphan another source's all-tilted pool (the
    # gravity-gate walkover; see refine_candidates).
    T, fitness, winning_index = refine_candidates(
        global_map, local_map, all_transforms, sources=sources
    )
    return T, fitness, sources[winning_index]
