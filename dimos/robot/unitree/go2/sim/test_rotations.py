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

from __future__ import annotations

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.rotations import (
    mat_to_quat,
    quat_to_mat,
    rotation_angle,
    strip_yaw,
)


def _random_quats(n: int, seed: int = 0) -> np.ndarray:
    q = np.random.default_rng(seed).normal(size=(n, 4))
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def test_quat_to_mat_produces_rotations():
    m = quat_to_mat(_random_quats(50))
    eye = np.einsum("nij,nkj->nik", m, m)
    assert np.allclose(eye, np.eye(3)[None], atol=1e-12)
    assert np.allclose(np.linalg.det(m), 1.0)


def test_mat_to_quat_round_trips():
    for q in _random_quats(50, seed=1):
        back = mat_to_quat(quat_to_mat(q))
        assert np.allclose(back, q, atol=1e-9) or np.allclose(back, -q, atol=1e-9)


def test_strip_yaw_keeps_only_the_gravity_referenced_part():
    rng = np.random.default_rng(2)
    for _ in range(20):
        m = quat_to_mat(rng.normal(size=4))
        s = strip_yaw(m)
        # same gravity direction in body frame (roll/pitch preserved) ...
        assert np.allclose(s[2, :], m[2, :], atol=1e-12)
        # ... and the forward axis now has zero yaw
        assert abs(np.arctan2(s[1, 0], s[0, 0])) < 1e-12


def test_rotation_angle_recovers_a_known_rotation():
    th = 0.3
    rz = np.array([[np.cos(th), -np.sin(th), 0.0], [np.sin(th), np.cos(th), 0.0], [0.0, 0.0, 1.0]])
    got = rotation_angle(np.eye(3)[None], rz[None])
    assert got[0] == pytest.approx(th, abs=1e-12)
