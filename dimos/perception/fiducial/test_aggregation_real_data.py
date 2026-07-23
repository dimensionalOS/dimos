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

"""Integration test: robust aggregation on REAL sf_office AprilTag sightings.

Unlike test_apriltag_aggregation.py (seeded synthetic poses), this exercises
:func:`robust_cluster_pose` on ACTUAL detections replayed from the
sf_office_go2_20260718_survey1 recording -- the checked-in fixture
``testdata/sf_office_tag2_tag6_sightings.json`` holds every gated
``world_T_marker`` sighting of two tags (rotation as quaternion), extracted
from the recording's per-marker fix logs. It pins the two opposite
outcomes the flip-contamination study found on real data:

 - tag 2 (finding: raw orientation spread gated ~88 deg -> ~22 deg post-aggregation):
   the sightings are bimodal -- a CLEAN MAJORITY (8 of 13, tightly grouped) plus
   a mirror-flip minority ~160 deg away -- and the majority coincides with the
   most-trustworthy (lowest-reproj) sighting. Huber-IRLS aggregation locks onto that
   majority and RECOVERS a good pose: the aggregated orientation sits ~5 deg from the
   clean-majority consensus and ~10 deg from the best single sighting.

 - tag 6 (finding: aggregation stuck ~48 deg -- the documented limit): the sightings
   are a FLIP-MAJORITY tie -- two mirror modes ~160 deg apart of EQUAL weight
   (6 vs 6 of 14), so no clean majority exists. Aggregation has nothing to lock onto
   and resolves to the flip side, landing ~162 deg from the most-confident
   (lowest-reproj 0.12 px) sighting: it does NOT recover a tight estimate.

The mechanism both cases share: the flips PASS the reproj gate (mirror-ambiguity
PnP solutions reproject cleanly -- tag 6's best sighting and its 160-deg-away
flip both sit near 0.1 px), so gating cannot remove them. Only the robust aggregation
can, and only when one mode is a genuine majority (tag 2), not a tie (tag 6).

All poses are REAL detections loaded from the frozen fixture; the aggregation core is
deterministic (medoid + fixed-iteration IRLS, no RNG), so every number below is
reproducible without a seed.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from dimos.perception.fiducial.apriltag_aggregation import (
    TagObservation,
    robust_cluster_pose,
)

_FIXTURE = Path(__file__).parent / "testdata" / "sf_office_tag2_tag6_sightings.json"
_ROTATION_WEIGHT_M_PER_RAD = 0.5  # AggregationConfig default
_HUBER_DELTA_M = 0.05  # AggregationConfig default
_CLUSTER_DEG = 30.0  # a "mode": sightings whose orientations agree within this


def _angle_deg(qa: np.ndarray, qb: np.ndarray) -> float:
    """Geodesic angle (deg) between two unit quaternions, q/-q double-cover aware."""
    return 2.0 * math.degrees(math.acos(min(1.0, abs(float(np.dot(qa, qb))))))


def _load_tag(tag_id: str) -> tuple[list[TagObservation], list[np.ndarray], list[float]]:
    """Fixture sightings for one tag -> (observations, quaternions, reproj_px)."""
    sightings = json.loads(_FIXTURE.read_text())["sightings"][tag_id]
    observations, quaternions, reprojs = [], [], []
    for i, s in enumerate(sightings):
        pose = (*s["t"], *s["q"])
        observations.append(
            TagObservation(
                ts=float(i),  # order-only; robust_cluster_pose ignores ts
                marker_id=int(tag_id),
                pose=pose,  # type: ignore[arg-type]
                reproj_px=s["reproj_px"],
            )
        )
        quaternions.append(np.asarray(s["q"], dtype=np.float64))
        reprojs.append(float(s["reproj_px"]))
    return observations, quaternions, reprojs


def _raw_spread_deg(quaternions: list[np.ndarray]) -> float:
    """Median pairwise orientation angle across the raw sightings (bimodality)."""
    pairwise = [
        _angle_deg(quaternions[i], quaternions[j])
        for i in range(len(quaternions))
        for j in range(i + 1, len(quaternions))
    ]
    return float(np.median(pairwise))


def _largest_mode(quaternions: list[np.ndarray]) -> tuple[int, np.ndarray]:
    """Size and Markley-mean consensus of the biggest within-``_CLUSTER_DEG`` mode."""
    n = len(quaternions)
    counts = [
        sum(1 for j in range(n) if _angle_deg(quaternions[i], quaternions[j]) < _CLUSTER_DEG)
        for i in range(n)
    ]
    seed = int(np.argmax(counts))
    members = [
        quaternions[j]
        for j in range(n)
        if _angle_deg(quaternions[seed], quaternions[j]) < _CLUSTER_DEG
    ]
    stacked = np.asarray(members)
    stacked = stacked * np.sign(stacked @ quaternions[seed])[:, None]  # hemisphere-align
    scatter = np.einsum("ni,nj->nij", stacked, stacked).sum(0)
    consensus = np.linalg.eigh(scatter)[1][:, -1]  # Markley eigen-mean
    return len(members), consensus


def test_tag2_clean_majority_aggregation_recovers() -> None:
    """tag 2: real sightings are bimodal (large raw spread), a CLEAN MAJORITY of
    them coincides with the most-trustworthy sighting, and Huber aggregation recovers
    it -- the aggregated orientation sits with the clean majority, not the flips.
    Finding: raw ~88 deg spread gated to ~22 deg. Measured (frozen fixture):
    raw median-pairwise 91.3 deg, majority 8/13, aggregated 5.0 deg from the majority
    consensus and 10.4 deg from the best single sighting."""
    observations, quaternions, reprojs = _load_tag("2")
    assert len(observations) == 13

    # Raw input is bimodal: a mirror-flip minority ~160 deg off the majority.
    assert _raw_spread_deg(quaternions) > 60.0  # 91.3

    # A genuine majority exists (> half the sightings share one orientation)...
    majority_size, majority_q = _largest_mode(quaternions)
    assert majority_size == 8  # 8 of 13, > half

    # ...and it is the trustworthy mode: the lowest-reproj sighting is in it.
    best_q = quaternions[int(np.argmin(reprojs))]
    assert _angle_deg(best_q, majority_q) < 15.0  # 6.2

    # Aggregation recovers: aggregated orientation sits with the clean majority.
    aggregated = np.asarray(
        robust_cluster_pose(observations, _ROTATION_WEIGHT_M_PER_RAD, _HUBER_DELTA_M)[3:7]
    )
    assert _angle_deg(aggregated, majority_q) < 15.0  # 5.0 -- recovered signal
    assert _angle_deg(aggregated, best_q) < 20.0  # 10.4 -- agrees with the best sighting


def test_tag6_flip_majority_aggregation_does_not_recover() -> None:
    """tag 6: real sightings are ALSO bimodal (large raw spread), but the two
    mirror modes are EQUAL weight (6 vs 6 of 14) -- a flip-majority tie with no
    clean majority to lock onto. Aggregation resolves to the flip side and does NOT
    recover: the aggregated orientation lands ~162 deg from the most-confident
    (lowest-reproj 0.12 px) sighting. This is the documented limit (finding:
    stuck ~48 deg). Measured (frozen fixture): raw median-pairwise 103.0 deg,
    largest mode only 6/14 (< half), aggregated 162.4 deg from the best sighting."""
    observations, quaternions, reprojs = _load_tag("6")
    assert len(observations) == 14

    # Raw input is bimodal, like tag 2.
    assert _raw_spread_deg(quaternions) > 60.0  # 103.0

    # But NO clean majority: the biggest mode is at most half the sightings.
    majority_size, _ = _largest_mode(quaternions)
    assert majority_size <= 7  # 6/14 -- a tie between flip modes, not a majority

    # The flip is not a low-quality artifact: the most-confident sighting is
    # itself ~160 deg from the mode aggregation picks -- the gate cannot catch it.
    best_q = quaternions[int(np.argmin(reprojs))]
    assert min(reprojs) < 0.2  # 0.12 px -- a clean, trustworthy detection

    # Aggregation does NOT recover: aggregated orientation is flipped away from the best.
    aggregated = np.asarray(
        robust_cluster_pose(observations, _ROTATION_WEIGHT_M_PER_RAD, _HUBER_DELTA_M)[3:7]
    )
    assert _angle_deg(aggregated, best_q) > 90.0  # 162.4 -- locked onto the flip mode
