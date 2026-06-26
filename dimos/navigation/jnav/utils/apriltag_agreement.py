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

"""April-tag agreement: a drift-quality metric for a corrected trajectory.

A fixed April tag observed many times along a trajectory should map to ONE world
position. Odometry drift scatters those estimates (the same tag lands in a
different spot each lap); a good loop-closure / PGO correction pulls them back
together. So the spread of a tag's repeated world-position estimates is a
ground-truth-free measure of trajectory consistency — and the *drop* in spread
from raw odometry to PGO-corrected odometry is how much PGO helped.

Pure functions over plain arrays; no PGO, sim, or I/O here (the benchmark harness
in gsc_pgo/pgo_apriltag_benchmark.py feeds these from real hk_village data).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dimos.navigation.jnav.utils.trajectory_metrics import PoseLookup

# sightings further apart than this are treated as separate visits
VISIT_GAP_S = 20.0


@dataclass(frozen=True)
class TagAgreement:
    """Per-tag spread of repeated world-position estimates (metres)."""

    tag_id: int
    observations: int
    spread: float  # RMS distance of estimates to their centroid


@dataclass(frozen=True)
class AgreementReport:
    """Agreement across all multiply-observed tags."""

    per_tag: tuple[TagAgreement, ...]
    mean_spread: float  # mean per-tag spread (metres); lower = better agreement
    total_observations: int

    @property
    def tag_count(self) -> int:
        return len(self.per_tag)


def tag_spread(positions: np.ndarray) -> float:
    """RMS distance of a tag's world-position estimates to their centroid."""
    if len(positions) < 2:
        return 0.0
    centroid = positions.mean(axis=0)
    return float(np.sqrt(np.mean(np.sum((positions - centroid) ** 2, axis=1))))


def agreement_report(
    tag_positions: dict[int, np.ndarray], *, min_observations: int = 2
) -> AgreementReport:
    """Score per-tag agreement from ``tag_id -> (N, 3) world positions``.

    Tags seen fewer than ``min_observations`` times carry no agreement signal and
    are excluded from ``mean_spread``.
    """
    per_tag: list[TagAgreement] = []
    total = 0
    for tag_id in sorted(tag_positions):
        positions = np.asarray(tag_positions[tag_id], dtype=np.float64).reshape(-1, 3)
        total += len(positions)
        if len(positions) < min_observations:
            continue
        per_tag.append(TagAgreement(tag_id, len(positions), tag_spread(positions)))
    mean_spread = float(np.mean([t.spread for t in per_tag])) if per_tag else 0.0
    return AgreementReport(tuple(per_tag), mean_spread, total)


def agreement_improvement(raw: AgreementReport, corrected: AgreementReport) -> float:
    """Fractional drop in mean spread from ``raw`` to ``corrected`` (1.0 = perfect).

    Positive means the correction tightened tag agreement; negative means it made
    it worse. Returns 0.0 if there's no raw spread to improve on.
    """
    if raw.mean_spread <= 0.0:
        return 0.0
    return (raw.mean_spread - corrected.mean_spread) / raw.mean_spread


def tag_world_positions(
    sightings: dict[int, list[float]], pose_lookup: PoseLookup
) -> dict[int, np.ndarray]:
    """Map each tag's sighting times to robot world positions (the proxy estimate)."""
    positions: dict[int, np.ndarray] = {}
    for tag_id, times in sightings.items():
        located = [p for p in (pose_lookup(t) for t in times) if p is not None]
        if located:
            positions[tag_id] = np.vstack(located)
    return positions


def split_visits(times: list[float], *, gap_s: float) -> list[list[float]]:
    """Cluster sighting timestamps into visits separated by gaps > ``gap_s``."""
    visits: list[list[float]] = []
    for timestamp in sorted(times):
        if visits and timestamp - visits[-1][-1] <= gap_s:
            visits[-1].append(timestamp)
        else:
            visits.append([timestamp])
    return visits


def paired_tag_visit_positions(
    sightings: dict[int, list[float]],
    raw_lookup: PoseLookup,
    corrected_lookup: PoseLookup,
    *,
    gap_s: float,
) -> tuple[dict[int, np.ndarray], dict[int, np.ndarray]]:
    """One robot position per tag VISIT under both pose sources, visits paired.

    Outdoors a tag stays visible while the robot walks tens of metres, so
    per-sighting spread is dominated by viewing distance, not drift. Visits are
    clustered on timestamps only, and a visit is kept only when BOTH pose
    sources can place it — so raw and corrected reports always score the exact
    same visit set, and a visit outside the pose graph's coverage drops out
    instead of skewing one side.
    """
    raw_positions: dict[int, np.ndarray] = {}
    corrected_positions: dict[int, np.ndarray] = {}
    for tag_id, times in sightings.items():
        raw_medians: list[np.ndarray] = []
        corrected_medians: list[np.ndarray] = []
        for visit_times in split_visits(times, gap_s=gap_s):
            raw_located = [p for p in (raw_lookup(t) for t in visit_times) if p is not None]
            corrected_located = [
                p for p in (corrected_lookup(t) for t in visit_times) if p is not None
            ]
            if raw_located and corrected_located:
                raw_medians.append(np.median(np.vstack(raw_located), axis=0))
                corrected_medians.append(np.median(np.vstack(corrected_located), axis=0))
        if raw_medians:
            raw_positions[tag_id] = np.vstack(raw_medians)
            corrected_positions[tag_id] = np.vstack(corrected_medians)
    return raw_positions, corrected_positions
