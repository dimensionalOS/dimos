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
never by its source's own reported confidence. RANSAC (``RansacPrior``,
wrapping relocalize.py's existing multi-scale FPFH+RANSAC search) is the
first prior; a future high-confidence seed (a fiducial marker, a carried-
forward pose) is meant to compete here too, not bypass the judge.
"""

from __future__ import annotations

from dataclasses import dataclass
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
    ``relocalize()``'s docstring). ``confidence`` is a coarse 0..1 tier the
    proposer reports about itself -- informational only. ``refine_candidates``
    never reads it: every candidate is judged purely on wall-only fine-scale
    fitness, so a high-confidence candidate from a bad prior still loses to a
    low-confidence candidate that actually fits the walls.
    """

    T: np.ndarray
    source: str
    confidence: float


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
    """Wraps relocalize.py's existing multi-scale FPFH+RANSAC global search.

    Confidence 0.5: a real geometric search over the full map, but not as
    tight a prior as a fiducial marker sighting or a freshly-carried-forward
    pose would be.
    """

    name = "ransac"

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        transforms = generate_ransac_candidates(global_map, local_map)
        return [Candidate(T=T, source=self.name, confidence=0.5) for T in transforms]


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
        return [Candidate(T=self._last_T, source=self.name, confidence=0.3)]


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

    T, fitness, winning_index = refine_candidates(global_map, local_map, all_transforms)
    return T, fitness, sources[winning_index]
