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
a candidate is accepted by surviving the wall-only fine-fitness rerank, never by
its source's own reported confidence. Three priors live here: ``RansacPrior``
(wrapping relocalize.py's existing multi-scale FPFH+RANSAC search),
``LastPosePrior`` (the last accepted answer, carried forward) and
``FiducialPrior`` (the detector's fused tag poses composed into one world->map
candidate per tag). The same invariant binds any future prior: it
goes through the judge on wall fitness, never bypasses it.

Each prior also owns its own TRIGGER (``is_due`` / ``on_fired``), and each fire
judges that prior's candidates ALONE. RANSAC sweeps periodically because a global
search has no event to wait for; the fiducial prior fires on the edge of a
completed tag burst, so a tag fix publishes at the tag's latency instead of
waiting on a RANSAC search that costs seconds. A pool of one is the expected
case: the judge is a validator (wall fitness, gravity tilt, wall evidence), not a
tournament.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Annotated, Literal, Protocol

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from pydantic import Field

from dimos.mapping.relocalization.relocalize import (
    GRAVITY_TILT_MAX_DEG,
    JudgeReport,
    generate_ransac_candidates,
    refine_candidates,
)
from dimos.perception.fiducial.apriltag_aggregation import AggregationConfig
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# s between RANSAC fires. One multi-scale FPFH+RANSAC search costs seconds of CPU
# (4.4-23 s measured on the trial's go2/Orin recordings), so the sweep is paced
# rather than run per frame; 2.0 is the cadence the module throttled at before the
# trigger moved onto this prior.
DEFAULT_RANSAC_INTERVAL_S = 2.0


# ---------------------------------------------------------------------------
# Prior configs -- one pydantic config per prior, keyed by a Literal ``type``
# into a discriminated union, so every prior is an EQUAL, toggleable entry (no
# always-on RANSAC special case). Pattern from the per-backend kinematics configs
# at dimos/manipulation/planning/kinematics/config.py:26-57. A blueprint is just
# a preset of this list.
# ---------------------------------------------------------------------------


class PriorConfigBase(BaseConfig):
    """Fields every prior shares: the on/off toggle plus its accept bar."""

    enabled: bool = True
    # Per-prior accept gate: min wall fitness (dimensionless, 0-1) THIS prior's
    # fix must clear to be accepted. On the base -- like ``enabled`` -- because every
    # prior is judged on its own bar (each fire pools one source). 0.45 is the old
    # single module gate, kept here as the fallback for a prior that states no bar of
    # its own (last_pose, which only re-proposes an already accepted fix). ransac and
    # fiducial DO override it below, each stating its own bar where it is configured.
    fitness_threshold: float = Field(default=0.45, ge=0.0, le=1.0)


class RansacPriorConfig(PriorConfigBase):
    """Multi-scale FPFH+RANSAC global search (``RansacPrior``). The search knobs
    live in relocalize.py; this entry owns the accept bar, the sweep cadence and the
    geometry floor the search needs to be worth firing."""

    type: Literal["ransac"] = "ransac"
    # HIGH bar (dimensionless wall fitness, 0-1). A RANSAC fix is a geometric FPFH
    # search with nothing anchoring it but the walls it landed on, and a repeated
    # corridor hands it a wrong-but-fitting wall that still scores respectably -- the
    # wall overlap IS the whole evidence. 0.6 raises the old single 0.45 module gate,
    # which was set when a lidar-only module had no second source to fall back on.
    fitness_threshold: float = Field(default=0.6, ge=0.0, le=1.0)
    # s between RANSAC fires -- this prior's trigger. Was the module's
    # reloc_interval_s, which no longer exists: a global search has no event to
    # wait for, so it paces itself and the tag prior fires on its own edge.
    interval_s: float = Field(default=DEFAULT_RANSAC_INTERVAL_S, gt=0.0)
    # Min local-map points (post VoxelGridMapper) this search needs before it fires:
    # FPFH matching + the wall-only rerank have too little geometry below this, so a
    # sparse frame is skipped (throttled log) and the search retries on the next
    # dense frame. RANSAC-scoped on purpose -- a fiducial fix comes from the tag, not
    # lidar density, so a tag burst fires regardless (its wall evidence is still
    # validated by refine_candidates' separate, much lower MIN_WALL_POINTS gate).
    min_local_points: int = Field(default=50_000, ge=0)


class LastPosePriorConfig(PriorConfigBase):
    """Last accepted fix carried forward as one seed (``LastPosePrior``). No params
    -- the seed is captured at runtime, not configured."""

    type: Literal["last_pose"] = "last_pose"


class FiducialPriorConfig(PriorConfigBase):
    """Marker sightings Huber-fused into one world->map candidate per tag
    (``FiducialPrior``). Owns the whole fiducial parameter surface."""

    type: Literal["fiducial"] = "fiducial"
    # Same wall-fitness bar as any other source (dimensionless, 0-1): a decoded id
    # names the tag, it does not show the composed pose fits the walls.
    fitness_threshold: float = Field(default=0.6, ge=0.0, le=1.0)
    # Surveyed marker map (map_T_marker per id), a .json path resolved via
    # resolve_named_path; required -- start() no-ops the prior without it.
    marker_map_file: str | None = None
    # Tag geometry, family and per-glimpse gate/fusion knobs the DETECTOR needs; none
    # of the three is read here (this prior consumes poses the detector already gated
    # and fused). Blueprints thread all three into MarkerDetectionStreamModule so the
    # fiducial family has one source of truth.
    marker_length_m: float = Field(default=0.10, gt=0.0)  # physical tag edge, m
    aruco_dictionary: str = "DICT_APRILTAG_36h11"
    aggregation: AggregationConfig = Field(default_factory=AggregationConfig)


# Discriminated on ``type`` (kinematics/config.py:54 is the exemplar).
PriorConfig = Annotated[
    RansacPriorConfig | LastPosePriorConfig | FiducialPriorConfig,
    Field(discriminator="type"),
]


@dataclass
class Candidate:
    """One proposed local_map->global_map transform, pre-judging.

    ``T`` uses relocalize()'s convention: the 4x4 mapping ``local_map`` into
    ``global_map``'s frame. A candidate carries only WHICH transform and WHICH
    source -- no self-reported confidence, since ``refine_candidates`` ranks purely
    on wall-only fine-scale fitness, so a "trusted" prior still loses to one that
    fits the walls.
    """

    T: np.ndarray
    source: str


class RelocPrior(Protocol):
    """A relocalization candidate proposer that owns when it is asked.

    Implementations MUST NOT self-select a winner -- that is
    ``refine_candidates``'s job, always. Returning zero candidates (e.g. no
    marker in view, no prior pose seeded yet) is a valid, expected response,
    not an error.

    ``is_due(now_s)`` is the prior's TRIGGER: True when it wants a relocalization
    fired now. ``on_fired(now_s)`` acks the fire that answered it (restart the
    timer, clear the edge). The module calls ``is_due`` on every enabled prior each
    local-map frame and fires one INDEPENDENT relocalization per prior that said
    yes, so no prior's latency is coupled to another's cost. The pair is split so
    the module can decline a due prior (RANSAC below its min_local_points floor):
    unacked, the trigger stands and fires at the first frame that qualifies.
    ``now_s`` is seconds on the module's single monotonic timebase, passed in so a
    test can drive the trigger without sleeping.
    """

    name: str

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]: ...

    def is_due(self, now_s: float) -> bool: ...

    def on_fired(self, now_s: float) -> None: ...


class RansacPrior:
    """Wraps relocalize.py's multi-scale FPFH+RANSAC global search, proposing its
    raw candidate pool into the judge.

    Trigger: a fixed interval. A global search waits on no event -- there is always
    a scan to match -- so the only sane trigger is a paced sweep, and the pace is
    what bounds its cost.
    """

    name = "ransac"

    def __init__(self, interval_s: float = DEFAULT_RANSAC_INTERVAL_S) -> None:
        self._interval_s = interval_s
        # None == never fired, so the first frame with enough points relocalizes
        # immediately -- the leading emit of the throttle this trigger replaces.
        self._last_fired_s: float | None = None

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        transforms = generate_ransac_candidates(global_map, local_map)
        return [Candidate(T=T, source=self.name) for T in transforms]

    def is_due(self, now_s: float) -> bool:
        return self._last_fired_s is None or now_s - self._last_fired_s >= self._interval_s

    def on_fired(self, now_s: float) -> None:
        self._last_fired_s = now_s


class LastPosePrior:
    """Carries the last accepted relocalization forward as one seed candidate --
    cheap continuous tracking competing alongside (not replacing) fresh RANSAC.

    Frame convention (relocalize()'s own, NOT the published TF):
    ``relocalize()``/``refine_candidates()`` return T with
    ``global_map_points = T @ local_map_points``. In the live pipeline ``local_map``
    is VoxelGridMapper's world-frame cloud and ``global_map`` the premap in the
    ``map`` frame, so this seed is ``map_T_world``. module.py inverts to
    ``world_T_map`` only when publishing the TF -- ``update()`` callers must pass
    the PRE-inversion T, never the published TF.

    Trigger: none. This is a SEED, not a source -- re-judging the pose it just
    published announces nothing new, so it never asks for a fire of its own and
    instead rides along with whichever prior did fire (module.py::_fire_pool).
    It costs one candidate and never a search.
    """

    name = "last_pose"

    def __init__(self) -> None:
        self._last_map_T_world: np.ndarray | None = None

    def update(self, map_T_world: np.ndarray) -> None:
        self._last_map_T_world = map_T_world

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        if self._last_map_T_world is None:
            return []
        return [Candidate(T=self._last_map_T_world, source=self.name)]

    def is_due(self, now_s: float) -> bool:
        return False

    def on_fired(self, now_s: float) -> None:
        """Nothing to ack: this prior never asks for a fire."""


class FiducialPrior:
    """Fused fiducial tag poses -> ONE world->map candidate per tag.

    ``observe()`` takes one already-gated, already-fused ``world_T_marker`` from the
    detector's ``fused_detections`` stream and composes it with the surveyed marker
    map; ``propose()`` hands each composed fix to the judge. No confidence -- the
    judge ranks it on wall fitness like every source.

    Gating and fusion live UPSTREAM, in the detector's ``FuseTagBursts``, because
    that is the only place ``corners_px``, the reprojection error and the camera
    transform still exist -- the wire ``Detection3DArray`` drops all three. This
    prior therefore does composition, nothing else.

    Frame convention: a candidate's ``T`` is map_T_world =
    ``map_T_marker @ inv(world_T_marker_fused)`` -- the same
    local_map(world)->global_map(map) direction RANSAC uses.

    Trigger and payload are ONE fact: a composed fix stays PENDING until
    ``propose()`` consumes it, and pending is what ``is_due`` reports. Every arriving
    fused pose is already one completed burst (``FuseTagBursts`` publishes once per
    marker per visit), so each tag's estimate goes past the judge exactly once and
    the prior then goes quiet until that tag is seen again.
    """

    name = "fiducial"

    def __init__(self, marker_map: dict[int, np.ndarray]) -> None:
        # marker_id -> map_T_marker (4x4); the surveyed marker map.
        self._map_T_marker = marker_map
        # marker_id -> map_T_world (4x4) awaiting its ONE trip past the judge.
        self._pending: dict[int, np.ndarray] = {}

    def observe(self, marker_id: int, world_T_marker_fused: np.ndarray) -> str | None:
        """Take ONE fused tag pose from the detector's ``fused_detections`` stream
        and compose this tag's world->map fix. Returns ``unmapped_id`` or ``None``."""
        map_T_marker = self._map_T_marker.get(marker_id)
        if map_T_marker is None:
            return "unmapped_id"
        self._pending[marker_id] = map_T_marker @ np.linalg.inv(world_T_marker_fused)
        return None

    def is_due(self, now_s: float) -> bool:
        return bool(self._pending)

    def on_fired(self, now_s: float) -> None:
        """Nothing to ack: propose() consumes the pending fix, and pending IS the
        trigger."""

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        # Consume on use, and HERE rather than in on_fired(): the module acks the
        # trigger BEFORE the solve that reaches propose() (module.py::_fire), so an
        # on_fired() clear would drop the fix unjudged. Re-offering a fix on a later
        # fire hands the judge the SAME measurement against a world that has drifted
        # further since, so it can only score worse than it did when fresh.
        # The swap is one statement, so an observe() racing in from the detections
        # transport thread either lands in the drained dict or stays pending for the
        # next fire -- the reload never tears and no fix is read twice.
        pending, self._pending = self._pending, {}
        return [Candidate(T=fix_T, source=self.name) for fix_T in pending.values()]


def relocalize_with_priors(
    global_map: o3d.geometry.PointCloud,
    local_map: o3d.geometry.PointCloud,
    priors: list[RelocPrior],
    gravity_tilt_max_deg: float = GRAVITY_TILT_MAX_DEG,
    verbose_eval_logging: bool = False,
    report: JudgeReport | None = None,
) -> tuple[np.ndarray, float, str]:
    """Gather candidates from the priors this fire runs, judge them through the
    shared fine-ICP tail (``refine_candidates``), report which candidate won.

    Usually ONE prior: triggers are per prior, so this is the validator path for a
    single source as often as it is a comparison. No source bypasses the judge -- a
    candidate that doesn't fit the walls loses even with nothing to lose to.
    ``gravity_tilt_max_deg`` threads to the judge's gravity gate, defaulting to the
    module constant so behavior is unchanged unless overridden.
    ``verbose_eval_logging`` turns the per-cycle proposal census and the judge's
    finalist table back on (``--eval``); off, this path logs nothing per cycle and
    module.py's single accept line carries the evidence.
    ``report`` collects the finalists' post-ICP fitness for that line's margin.

    Returns ``(T, fitness, winning_source)``. Raises ``ValueError`` if every prior
    proposed zero candidates.
    """
    all_transforms: list[np.ndarray] = []
    sources: list[str] = []
    for prior in priors:
        for candidate in prior.propose(global_map, local_map):
            all_transforms.append(candidate.T)
            sources.append(candidate.source)

    if not all_transforms:
        raise ValueError("relocalize_with_priors: no prior proposed any candidate")

    # Proposal census: which prior offered candidates this fire. A fiducial source
    # absent here means no tag reached min_observations -- distinct from proposing
    # then losing the judge below. Eval-only: it is one line per fire forever,
    # which is a debugging trace, not an operating log.
    if verbose_eval_logging:
        counts = {s: sources.count(s) for s in sorted(set(sources))}
        logger.info("relocalize candidates", counts=counts)

    # sources= makes the judge's gravity gate per-source — a lone upright
    # seed must never orphan another source's all-tilted pool (the
    # gravity-gate walkover; see refine_candidates).
    T, fitness, winning_index = refine_candidates(
        global_map,
        local_map,
        all_transforms,
        sources=sources,
        gravity_tilt_max_deg=gravity_tilt_max_deg,
        verbose_eval_logging=verbose_eval_logging,
        report=report,
    )
    return T, fitness, sources[winning_index]
