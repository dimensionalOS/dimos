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
from typing import Protocol

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]

from dimos.mapping.relocalization.relocalize import generate_ransac_candidates, refine_candidates
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
    """Fiducial marker sightings -> ONE robust world->map candidate per tag.

    The aggregation pipeline lives here (apriltag_aggregation, ported from
    jnav): ``observe()`` takes one marker sighting, gates it, and Huber-fuses
    the visit's sightings (medoid seed + IRLS: weighted-mean translation +
    Markley quaternion mean); ``propose()`` composes each fused pose with the
    surveyed marker map into a candidate for the judge. No confidence -- the
    judge ranks it on wall fitness like every source.

    Fusing the drift-free WORLD pose of the marker (``world_T_marker``, which the
    detector already computes) means camera motion within a 5 s visit never
    enters the fused pose. What the caller can supply drives the gates; a gate
    whose input is absent is skipped (graceful degradation, gate_reason):
     - ``corners_px`` + ``camera_info`` -> the IPPE mirror-ambiguity gate (ours;
       jnav lacks it) runs a full solvePnPGeneric and drops a glimpse whose
       flipped pose reprojects nearly as well, plus reproj/tag-px/view gates;
     - ``world_T_optical`` alone -> the distance/view-angle gates;
     - neither (a pixel-less wire delivery) -> fuse on min-obs + time window,
       leaning on the medoid/Huber to reject the mirror-flip outliers instead.

    Frame convention (relocalize()'s own, per repo rule): a candidate's ``T`` is
    map_T_world = ``map_T_marker @ inv(world_T_marker_fused)`` -- the same
    local_map(world)->global_map(map) direction the RANSAC pool uses. propose()
    drops a tag's fix once it is older than ``age_max_s`` (hard cutoff, no decay:
    nothing downstream weights a fix by age). Defaults are engineering guesses;
    tuning them belongs to #2137's autoresearch harness.
    """

    name = "fiducial"

    def __init__(
        self,
        marker_map: dict[int, np.ndarray],
        *,
        camera_info: CameraInfo | None = None,
        marker_length_m: float = 0.10,
        ambiguity_ratio_min: float = 2.0,
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
        # marker_id -> (map_T_world 4x4, arrival ts in now_fn's timebase).
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
        return [
            Candidate(T=fix_T, source=self.name)
            for fix_T, fix_ts in self._fixes.values()
            if now - fix_ts <= self._age_max_s
        ]


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
