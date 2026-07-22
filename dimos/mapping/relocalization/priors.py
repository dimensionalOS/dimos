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
``FiducialPrior`` (marker sightings Huber-fused into one robust world->map
candidate per tag, age-gated). The same invariant binds any future prior: it
competes through the judge on wall fitness, never bypasses it.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import time
from typing import Annotated, Literal, Protocol

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from pydantic import Field

from dimos.mapping.relocalization.relocalize import (
    GRAVITY_TILT_MAX_DEG,
    generate_ransac_candidates,
    refine_candidates,
)
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.perception.fiducial.apriltag_aggregation import (
    AggregationConfig,
    TagAggregator,
    TagObservation,
    matrix_from_pose7,
    pose7_from_matrix,
    tag_pixel_size,
    view_quality,
)
from dimos.perception.fiducial.marker_pose import (
    ambiguity_gated_pose,
    camera_info_to_cv_matrices,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# ---------------------------------------------------------------------------
# Prior configs -- one pydantic config per prior, keyed by a Literal ``type``
# into a discriminated union, so every prior is an EQUAL, toggleable entry (no
# always-on RANSAC special case). Pattern from the per-backend kinematics configs
# at dimos/manipulation/planning/kinematics/config.py:26-57. A blueprint is just
# a preset of this list.
# ---------------------------------------------------------------------------


class PriorConfigBase(BaseConfig):
    """Fields every prior shares: the on/off toggle plus the (INERT) fusion surface.

    ``enabled`` alone decides whether a prior proposes into the judge today.
    ``tier``/``weight``/``max_age_s`` are declared but read by NOTHING yet -- the
    judge ranks purely on wall fitness, never on self-reported trust. They are the
    seam the Phase-4 fusion arbiter will read; surfacing them now keeps a preset
    written today working once that arbiter lands.
    """

    enabled: bool = True
    tier: int = Field(default=0, ge=0)  # INERT: fusion-arbiter priority tier (0 = base)
    weight: float = Field(default=1.0, ge=0.0)  # INERT: fusion-arbiter blend weight
    # INERT: fusion-arbiter age-decay half-life (s). NOT a live gate -- the only
    # live age cutoff is FiducialPriorConfig.age_max_s below.
    max_age_s: float = Field(default=120.0, ge=0.0)


class RansacPriorConfig(PriorConfigBase):
    """Multi-scale FPFH+RANSAC global search (``RansacPrior``). No params of its
    own -- the search knobs live in relocalize.py."""

    type: Literal["ransac"] = "ransac"


class LastPosePriorConfig(PriorConfigBase):
    """Last accepted fix carried forward as one seed (``LastPosePrior``). No params
    -- the seed is captured at runtime, not configured."""

    type: Literal["last_pose"] = "last_pose"


class FiducialPriorConfig(PriorConfigBase):
    """Marker sightings Huber-fused into one world->map candidate per tag
    (``FiducialPrior``). Owns the whole fiducial parameter surface."""

    type: Literal["fiducial"] = "fiducial"
    # Surveyed marker map (map_T_marker per id), a .json path resolved via
    # resolve_named_path; required -- start() no-ops the prior without it.
    marker_map_file: str | None = None
    marker_length_m: float = Field(default=0.10, gt=0.0)  # physical tag edge, m
    # Tag family the DETECTOR decodes; not read here (this module fuses decoded
    # sightings). Blueprints thread it into MarkerDetectionStreamModule so detector
    # + prior share one source of truth.
    aruco_dictionary: str = "DICT_APRILTAG_36h11"
    # IPPE mirror-ambiguity gate: best/runner-up reproj ratio a glimpse must beat
    # (1.0 = off). Only bites when corners_px reach the prior (offline harness);
    # the live wire drops the pixels, so the medoid/Huber fusion carries the
    # mirror-flip rejection. Collins & Bartoli 2014
    # https://link.springer.com/article/10.1007/s11263-014-0725-5
    ambiguity_ratio_min: float = Field(default=2.0, ge=1.0)
    # Intrinsics for the ambiguity gate's solvePnP. None -> pixel-gated path skipped.
    camera_info: CameraInfo | None = None
    # LIVE hard cutoff (s): propose() drops a fused fix older than this. Distinct
    # from the INERT base max_age_s (fusion arbiter, unread today).
    age_max_s: float = Field(default=120.0, ge=0.0)
    # Per-glimpse gate + fusion knobs for the aggregator, so a deployment can
    # retune fusion without code changes.
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
    fits the walls. (Composite confidence is the Phase-4 fusion arbiter's job.)
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
    """Wraps relocalize.py's multi-scale FPFH+RANSAC global search, proposing its
    raw candidate pool into the judge like any other source."""

    name = "ransac"

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        transforms = generate_ransac_candidates(global_map, local_map)
        return [Candidate(T=T, source=self.name) for T in transforms]


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
    """Fiducial marker sightings -> ONE robust world->map candidate per tag.

    ``observe()`` gates one sighting and Huber-fuses the visit's sightings (medoid
    seed + IRLS: weighted-mean translation + Markley quaternion mean); ``propose()``
    composes each fused pose with the surveyed marker map into a candidate. No
    confidence -- the judge ranks it on wall fitness like every source.

    Fusing the marker's drift-free WORLD pose (``world_T_marker``) keeps camera
    motion within a visit out of the fused pose. Available inputs drive the gates;
    a gate whose input is absent is skipped (graceful degradation):
     - ``corners_px`` + ``camera_info`` -> IPPE mirror-ambiguity gate (full
       solvePnPGeneric, drops a glimpse whose flip reprojects nearly as well) plus
       reproj/tag-px/view gates;
     - ``world_T_optical`` alone -> distance/view-angle gates;
     - neither (pixel-less wire) -> fuse on min-obs + time window, leaning on the
       medoid/Huber to reject mirror-flip outliers.

    Frame convention: a candidate's ``T`` is map_T_world =
    ``map_T_marker @ inv(world_T_marker_fused)`` -- the same
    local_map(world)->global_map(map) direction RANSAC uses. propose() drops a fix
    older than ``age_max_s`` (hard cutoff, no decay). Defaults are engineering
    guesses; tuning belongs to #2137's autoresearch harness.
    """

    name = "fiducial"

    def __init__(
        self,
        marker_map: dict[int, np.ndarray],
        *,
        camera_info: CameraInfo | None = None,
        marker_length_m: float = 0.10,  # 10 cm tag
        ambiguity_ratio_min: float = 2.0,  # flip must reproject >=2x worse to keep
        config: AggregationConfig | None = None,
        age_max_s: float = 120.0,
        now_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        # marker_id -> map_T_marker (4x4); the surveyed marker map.
        self._map_T_marker = marker_map
        self._camera_info = camera_info
        self._marker_length_m = marker_length_m
        self._ambiguity_ratio_min = ambiguity_ratio_min
        self._age_max_s = age_max_s
        self._now_fn = now_fn
        self._aggregator = TagAggregator(config if config is not None else AggregationConfig())
        # marker_id -> (map_T_world 4x4, arrival ts). Single-timebase invariant: ts
        # is stamped by now_fn here and MUST be compared only against now_fn() in
        # propose() -- mixing clocks makes age negative and neuters the age gate.
        self._fixes: dict[int, tuple[np.ndarray, float]] = {}

    def observe(
        self,
        marker_id: int,
        world_T_marker: np.ndarray,
        ts: float,
        *,
        world_T_optical: np.ndarray | None = None,
        corners_px: np.ndarray | None = None,
    ) -> str | None:
        """Feed one marker sighting; refresh this tag's fused fix if the visit is
        rich enough. Returns a rejection reason (``unmapped_id`` /
        ``mirror_ambiguous`` / a gate name) or ``None`` when the glimpse is kept.
        """
        map_T_marker = self._map_T_marker.get(marker_id)
        if map_T_marker is None:
            return "unmapped_id"

        distance_m: float | None = None
        view_angle_deg: float | None = None
        reproj_px: float | None = None
        tag_px: float | None = None
        optical_T_marker: np.ndarray | None = None
        if corners_px is not None and self._camera_info is not None:
            camera_matrix, dist_coeffs = camera_info_to_cv_matrices(self._camera_info)
            gated = ambiguity_gated_pose(
                corners_px,
                self._marker_length_m,
                camera_matrix,
                dist_coeffs,
                distortion_model=self._camera_info.distortion_model,
                ambiguity_ratio_min=self._ambiguity_ratio_min,
            )
            if gated is None:
                return "mirror_ambiguous"
            optical_T_marker, reproj_px = gated
            tag_px = tag_pixel_size(corners_px)
        elif world_T_optical is not None:
            optical_T_marker = np.linalg.inv(world_T_optical) @ world_T_marker
        if optical_T_marker is not None:
            distance_m, view_angle_deg = view_quality(pose7_from_matrix(optical_T_marker))

        reason = self._aggregator.observe(
            TagObservation(
                ts=ts,
                marker_id=marker_id,
                pose=pose7_from_matrix(world_T_marker),
                distance_m=distance_m,
                view_angle_deg=view_angle_deg,
                reproj_px=reproj_px,
                tag_px=tag_px,
            )
        )
        if reason is not None:
            return reason
        estimate = self._aggregator.robust_estimate(marker_id)
        if estimate is not None:
            world_T_marker_fused = matrix_from_pose7(estimate.pose)
            self._fixes[marker_id] = (
                map_T_marker @ np.linalg.inv(world_T_marker_fused),
                self._now_fn(),
            )
        return None

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        now = self._now_fn()
        candidates: list[Candidate] = []
        for fix_T, fix_ts in self._fixes.values():
            age_s = now - fix_ts
            # Negative age == fix_ts and now came from different clocks. Left as-is,
            # `<= age_max_s` is always true and a stale fix proposes forever (age gate
            # disabled). Clamp to fresh and warn so the misconfig surfaces loudly.
            if age_s < 0:
                logger.warning(
                    "fiducial fix age negative; clamping",
                    age_s=round(age_s, 2),
                    hint="fix_ts and now must share one clock (now_fn)",
                )
                age_s = 0.0
            if age_s <= self._age_max_s:
                candidates.append(Candidate(T=fix_T, source=self.name))
        return candidates


def relocalize_with_priors(
    global_map: o3d.geometry.PointCloud,
    local_map: o3d.geometry.PointCloud,
    priors: list[RelocPrior],
    gravity_tilt_max_deg: float = GRAVITY_TILT_MAX_DEG,
) -> tuple[np.ndarray, float, str]:
    """Gather candidates from every prior, judge them through the shared fine-ICP
    tail (``refine_candidates``), report which prior's candidate won.

    No source bypasses the judge -- a candidate that doesn't fit the walls loses.
    ``gravity_tilt_max_deg`` threads to the judge's gravity gate, defaulting to the
    module constant so behavior is unchanged unless overridden.

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

    # Proposal census: which prior offered candidates this cycle. A fiducial source
    # absent here means no tag reached min_observations -- distinct from proposing
    # then losing the judge below.
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
    )
    return T, fitness, sources[winning_index]
