# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

import numpy as np
import pytest

from dimos.manipulation.planning.kinematics.sew_retargeting import SewArmSolver, SewArmTarget


@pytest.fixture
def solver():
    return SewArmSolver(
        np.array([-4.4, -0.14, -2.32, -2.06, -2.32, -1.01, -1.54]),
        np.array([1.27, 3.1, 2.32, 0.31, 2.32, 1.01, 1.54]),
    )


def test_roundtrip_signed_axis_targets_and_hand(solver):
    rng = np.random.default_rng(120)
    for _ in range(100):
        q = rng.uniform(solver.lower + 0.02, solver.upper - 0.02)
        u, l, h = solver.features(q)
        target = SewArmTarget(np.zeros(3), u, u + l, h)
        np.testing.assert_allclose(solver.solve(target, q), q, atol=1e-7)


def test_target_scale_and_translation_do_not_change_solution(solver):
    q = np.array([0.2, 0.4, 0.2, -0.6, 0.1, 0.2, 0.3])
    u, l, h = solver.features(q)
    shift = np.array([2.0, -3.0, 5.0])
    target = SewArmTarget(shift, shift + u * 0.25, shift + u * 0.25 + l * 0.4, h)
    np.testing.assert_allclose(solver.solve(target, q), q, atol=1e-7)


def test_degenerate_input_rejected(solver):
    with pytest.raises(ValueError, match="degenerate"):
        solver.solve(SewArmTarget(np.zeros(3), np.zeros(3), np.ones(3), np.eye(3)), np.zeros(7))


def test_no_limit_compliant_wrist_is_not_clipped(solver):
    q = np.array([0.2, 0.4, 0.2, -0.6, 0.1, 1.3, 0.3])
    u, l, h = solver.features(q)
    with pytest.raises(ValueError, match="limit-compliant"):
        solver.solve(SewArmTarget(np.zeros(3), u, u + l, h), q)


def test_extended_elbow_is_finite_and_seed_preserving(solver):
    q = np.array([0.2, 0.4, 0.7, 0.0, 0.1, 0.2, 0.3])
    u, l, h = solver.features(q)
    result = solver.solve(SewArmTarget(np.zeros(3), u, u + l, h), q)
    np.testing.assert_allclose(solver.features(result)[2], h, atol=1e-7)
    assert np.isfinite(result).all()
