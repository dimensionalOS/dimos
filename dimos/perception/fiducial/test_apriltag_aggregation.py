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

"""Crucial-invariant test for the robust multi-sighting AprilTag aggregation core.

All poses are SIMULATED (numpy-seeded, deterministic) -- no hardware, no recording.
"""

from __future__ import annotations

import math

import pytest
from scipy.spatial.transform import Rotation

from dimos.perception.fiducial.apriltag_aggregation import TagObservation, robust_cluster_pose


@pytest.mark.parametrize(
    "n_clean, n_flip, lands_on",
    [
        pytest.param(5, 2, "clean", id="clean_majority_rejects_flips"),
        pytest.param(2, 5, "flip", id="follows_flip_majority"),
    ],
)
def test_robust_cluster_pose_is_a_majority_vote_over_mirror_flips(
    n_clean: int, n_flip: int, lands_on: str
) -> None:
    """Invariant 10: robust aggregation is a majority vote over the PnP mirror
    ambiguity, not an oracle. Flips are 180 deg about X (the mirror solution the gate
    can miss), a genuinely different rotation. A clean majority (5 vs 2) out-votes the
    flips and the IRLS aggregate lands on identity; once the flips dominate, IRLS
    seeds and converges inside the flipped cluster instead."""
    flip = Rotation.from_rotvec((math.pi, 0.0, 0.0))
    rots = [Rotation.identity()] * n_clean + [flip] * n_flip
    cluster = [
        TagObservation(ts=float(i), marker_id=1, pose=(0.0, 0.0, 1.0, *r.as_quat()))
        for i, r in enumerate(rots)
    ]
    aggregated = robust_cluster_pose(cluster, rotation_weight_m_per_rad=0.5, huber_delta_m=0.05)

    reference = Rotation.identity() if lands_on == "clean" else flip
    err_deg = (Rotation.from_quat(aggregated[3:7]) * reference.inv()).magnitude() * 180.0 / math.pi
    assert err_deg < 0.5
