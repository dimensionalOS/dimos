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

"""The floor estimator and the anchoring, against the rust twin's fixtures.

Every case here has a `floor.rs` counterpart with the same name and the same
numbers — the two are a port pair, and a behaviour change lands in both.
"""

import numpy as np

from dimos.navigation.motion.adapter.floor import anchor_to_floor, estimate_floor


def _slab(z: float, n: int = 400, at: tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """A flat floor at `z`, dense enough to be a floor sample."""
    a = np.arange(n) / n * 2 * np.pi
    return np.column_stack([at[0] + np.cos(a), at[1] + np.sin(a), np.full(n, z)]).astype(np.float32)


def test_a_flat_floor_is_read_off_the_cloud():
    assert abs(estimate_floor(_slab(-0.28), (0.0, 0.0)) - (-0.28)) < 1e-6


def test_a_sparse_neighbourhood_falls_back_to_the_prior():
    assert estimate_floor(_slab(-0.28, 10), (0.0, 0.0), prior=-0.24) == -0.24
    assert estimate_floor(_slab(-0.28, 10), (0.0, 0.0)) is None


def test_points_outside_the_radius_do_not_count():
    # a floor 10 m away is another room's floor
    pts = np.concatenate([_slab(-0.28), _slab(-2.0, at=(10.0, 0.0))])
    assert abs(estimate_floor(pts, (0.0, 0.0)) - (-0.28)) < 1e-6


def test_the_prior_bounds_a_wild_estimate():
    # a stairwell edge drags the low quantile down; tf's base height is the
    # sanity bound that keeps the band on the floor the robot stands on
    pts = np.concatenate([_slab(-0.28), _slab(-3.0)])
    assert estimate_floor(pts, (0.0, 0.0), prior=-0.24) == -0.24


def test_a_floor_the_prior_agrees_with_is_kept():
    # inside the tolerance the cloud wins: it is the measurement, the prior is
    # only the bound
    assert abs(estimate_floor(_slab(-0.28), (0.0, 0.0), prior=-0.24) - (-0.28)) < 1e-6


def test_anchoring_moves_the_band_onto_the_floor():
    # a 0.20 m obstacle over a floor at -0.28 reads as -0.08 absolute (under
    # the 0.05..0.45 band) and as 0.20 once anchored
    out = anchor_to_floor(np.array([[1.0, 0.0, -0.08]], dtype=np.float32), -0.28, 0.08)
    assert len(out) == 1
    assert abs(float(out[0][2]) - 0.2) < 1e-6


def test_the_ground_slab_is_dropped_rather_than_walled_into_the_band():
    # the +0.29 counterfactual in the diagnosis: quantisation puts the floor's
    # own layer just inside the band and every tick then refuses
    floor = _slab(-0.28, 40)
    upper = floor + np.array([0.0, 0.0, 0.08], dtype=np.float32)
    assert len(anchor_to_floor(np.concatenate([floor, upper]), -0.28, 0.08)) == 0


def test_a_zero_margin_keeps_everything():
    pts = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    assert len(anchor_to_floor(pts, 0.0, 0.0)) == 2


def test_a_floor_already_at_zero_is_a_no_op_shift():
    # the referee's sim worlds put the plan poses on the ground, and the
    # anchoring has to leave those exactly where they were
    pts = np.array([[1.0, 2.0, 0.3], [-1.0, 0.5, 0.44]], dtype=np.float32)
    assert np.array_equal(anchor_to_floor(pts, 0.0, 0.08), pts)


def test_the_percentile_interpolates_like_the_rust_twin():
    # np.percentile([0,1,2,3], 5) == 0.15, which floor.rs reproduces by hand
    pts = np.column_stack([np.zeros(4), np.zeros(4), np.arange(4.0)]).astype(np.float32)
    assert abs(estimate_floor(pts, (0.0, 0.0), min_points=4) - 0.15) < 1e-12
