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

"""Pluggable relocalization priors: candidate proposers feeding the shared
fine-ICP judge in relocalize.py (``refine_candidates``). A prior only proposes;
its trigger lives with its nature -- RANSAC is polled by the module on a timer,
the fiducial is event-driven off a tag burst. Every candidate is accepted only by
surviving the wall-only fine-fitness rerank, never by its source's confidence.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Annotated, Literal, Protocol

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from pydantic import Field

from dimos.mapping.relocalization.relocalize import (
    GRAVITY_TILT_MAX_DEG,
    generate_ransac_candidates,
    refine_candidates,
)
from dimos.perception.fiducial.apriltag_aggregation import AggregationConfig
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# s between RANSAC fires. One FPFH+RANSAC search costs seconds of CPU (4.4-23 s
# measured on the trial's go2/Orin recordings), so the sweep is paced, not per-frame.
DEFAULT_RANSAC_INTERVAL_S = 2.0


# Prior configs: one pydantic config per prior, keyed by a Literal ``type`` into a
# discriminated union. Pattern from dimos/manipulation/planning/kinematics/config.py:26-57.


class PriorConfigBase(BaseConfig):
    """Fields every prior shares: the on/off toggle plus its accept bar."""

    enabled: bool = True
    # Per-prior accept gate: min wall fitness (dimensionless, 0-1) this prior's fix
    # must clear to be accepted.
    fitness_threshold: float = Field(default=0.6, ge=0.0, le=1.0)


class RansacPriorConfig(PriorConfigBase):
    """Multi-scale FPFH+RANSAC global search (``RansacPrior``); search knobs live in
    relocalize.py, this entry owns the accept bar, cadence and geometry floor."""

    type: Literal["ransac"] = "ransac"
    # s between RANSAC fires -- this prior's trigger; a global search has no event to
    # wait for, so it paces itself.
    interval_s: float = Field(default=DEFAULT_RANSAC_INTERVAL_S, gt=0.0)
    # Min local-map points (post VoxelGridMapper) this search needs before it fires:
    # below this FPFH matching + the wall-only rerank have too little geometry, so a
    # sparse frame is skipped and retried on the next dense frame.
    min_local_points: int = Field(default=50_000, ge=0)


class FiducialPriorConfig(PriorConfigBase):
    """Marker sightings Huber-aggregated into one world->map candidate per tag
    (``FiducialPrior``). Owns the whole fiducial parameter surface."""

    type: Literal["fiducial"] = "fiducial"
    # Surveyed marker map (map_T_marker per id), a .json path resolved via
    # resolve_named_path; required -- start() no-ops the prior without it.
    marker_map_file: str | None = None
    # Tag geometry/family/aggregation knobs the DETECTOR needs; none is read here.
    # Blueprints thread all three into MarkerDetectionStreamModule.
    marker_length_m: float = Field(default=0.10, gt=0.0)  # physical tag edge, m
    aruco_dictionary: str = "DICT_APRILTAG_36h11"
    aggregation: AggregationConfig = Field(default_factory=AggregationConfig)


# Discriminated on ``type`` (kinematics/config.py:54 is the exemplar).
PriorConfig = Annotated[
    RansacPriorConfig | FiducialPriorConfig,
    Field(discriminator="type"),
]


@dataclass
class Candidate:
    """One proposed transform, pre-judging. ``T`` is the 4x4 mapping ``local_map``
    into ``global_map``'s frame; no self-reported confidence (the judge ranks it)."""

    T: np.ndarray
    source: str


class RelocPrior(Protocol):
    """A relocalization candidate proposer. The module owns the trigger.

    Implementations must not self-select a winner (``refine_candidates``'s job);
    returning zero candidates is a valid, expected response.
    """

    name: str

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]: ...


class RansacPrior:
    """Wraps relocalize.py's FPFH+RANSAC global search. A pure candidate source: the
    module polls it on a paced interval (a global search waits on no event, so the pace
    bounds its cost) and owns that timer state."""

    name = "ransac"

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        transforms = generate_ransac_candidates(global_map, local_map)
        return [Candidate(T=T, source=self.name) for T in transforms]


class FiducialPrior:
    """Aggregated fiducial tag poses -> ONE world->map candidate per tag.

    ``observe()`` composes an aggregated ``world_T_marker`` with the surveyed marker
    map; ``propose()`` hands each composed fix to the judge. Frame convention: a
    candidate's ``T`` is map_T_world = ``map_T_marker @ inv(world_T_marker_aggregated)``,
    the same local_map(world)->global_map(map) direction RANSAC uses. A composed fix
    stays pending until ``propose()`` consumes it; ``has_pending`` reports readiness so
    the module fires this event prior on the burst edge.
    """

    name = "fiducial"

    def __init__(self, marker_map: dict[int, np.ndarray]) -> None:
        # marker_id -> map_T_marker (4x4); the surveyed marker map.
        self._map_T_marker = marker_map
        # marker_id -> map_T_world (4x4) awaiting its ONE trip past the judge.
        self._pending: dict[int, np.ndarray] = {}
        # observe() and propose() run on different transport threads (module.py::_fire),
        # so the sighting that reads self._pending and the drain that replaces it are
        # one critical section -- see propose() for what tears without it.
        self._pending_lock = threading.Lock()

    def observe(self, marker_id: int, world_T_marker_aggregated: np.ndarray) -> str | None:
        """Take ONE aggregated tag pose from the detector's ``aggregated_detections`` stream
        and compose this tag's world->map fix. Returns ``unmapped_id`` or ``None``."""
        map_T_marker = self._map_T_marker.get(marker_id)
        if map_T_marker is None:
            return "unmapped_id"
        with self._pending_lock:
            self._pending[marker_id] = map_T_marker @ np.linalg.inv(world_T_marker_aggregated)
        return None

    @property
    def has_pending(self) -> bool:
        """A composed fix is waiting for the judge -- the module's fire signal."""
        return bool(self._pending)

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        # Consume on use: a fix is drained the one time it reaches the judge. Re-offering
        # it on a later fire hands the judge the SAME measurement against a world that has
        # drifted further since, so it can only score worse than it did when fresh.
        # The swap takes the lock because the swap alone is not enough: a store in
        # observe() reads self._pending and writes it as two steps, and a detections
        # thread preempted between them still holds the dict this fire is draining:
        #
        #     observe:  d = self._pending ....................... d[id] = fix
        #     propose:                     self._pending = {}; iterate d
        #                                                              ^ RuntimeError
        #
        # "dictionary changed size during iteration" -- which _try_relocalize's
        # boundary except swallows, dropping the cycle. Under the lock a sighting
        # either lands before the swap or waits and goes out on the next fire.
        with self._pending_lock:
            pending, self._pending = self._pending, {}
        return [Candidate(T=fix_T, source=self.name) for fix_T in pending.values()]


class EmptyProposalError(ValueError):
    """No prior proposed any candidate this fire. Benign race artifact (two threads
    see the fiducial trigger due, the loser drains an empty _pending); treated as a
    no-op, not a crash."""


def relocalize_with_priors(
    global_map: o3d.geometry.PointCloud,
    local_map: o3d.geometry.PointCloud,
    priors: list[RelocPrior],
    gravity_tilt_max_deg: float = GRAVITY_TILT_MAX_DEG,
    verbose_eval_logging: bool = False,
) -> tuple[np.ndarray, float, str]:
    """Gather candidates from the priors this fire runs, judge them through the shared
    fine-ICP tail (``refine_candidates``), report which won. ``gravity_tilt_max_deg``
    threads to the judge's gravity gate; ``verbose_eval_logging`` turns the per-cycle
    proposal census on (``--eval``). Returns ``(T, fitness, winning_source)``; raises
    ``EmptyProposalError`` if every prior proposed zero candidates (a benign race).
    """
    all_transforms: list[np.ndarray] = []
    sources: list[str] = []
    for prior in priors:
        for candidate in prior.propose(global_map, local_map):
            all_transforms.append(candidate.T)
            sources.append(candidate.source)

    if not all_transforms:
        raise EmptyProposalError("relocalize_with_priors: no prior proposed any candidate")

    # Proposal census (eval-only): which prior offered candidates this fire.
    if verbose_eval_logging:
        counts = {s: sources.count(s) for s in sorted(set(sources))}
        logger.info("relocalize candidates", counts=counts)

    T, fitness, winning_index = refine_candidates(
        global_map,
        local_map,
        all_transforms,
        gravity_tilt_max_deg=gravity_tilt_max_deg,
    )
    return T, fitness, sources[winning_index]
