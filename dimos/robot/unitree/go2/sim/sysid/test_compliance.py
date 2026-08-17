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

"""The compliance instrument: Jacobian, a rigid null, and a known spring."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.sysid.compliance import foot_forces, leg_jacobian, measure
from dimos.robot.unitree.go2.sim.sysid.gait import foot_base
from dimos.robot.unitree.go2.sim.sysid.test_gait import _synthetic_gait


def test_leg_jacobian_matches_the_fk() -> None:
    rng = np.random.default_rng(3)
    for leg in range(4):
        q3 = rng.uniform(-0.8, 0.8, 3)
        jac = leg_jacobian(q3, leg)
        dq = rng.uniform(-1e-4, 1e-4, 3)
        q12a = np.zeros(12)
        q12b = np.zeros(12)
        q12a[3 * leg : 3 * leg + 3] = q3
        q12b[3 * leg : 3 * leg + 3] = q3 + dq
        got = foot_base(q12b[None])[0, leg] - foot_base(q12a[None])[0, leg]
        assert np.allclose(jac @ dq, got, atol=1e-8)


def _rig(k: float | None) -> list:
    """The synthetic gait with a commanded FL force; base height obeys a
    series spring of stiffness ``k`` (None = rigid)."""
    t, q, quat, planar = _synthetic_gait(1.8, 0.5, seconds=20.0)
    fb = foot_base(q)
    f_z = 60.0 + 30.0 * np.sin(2 * np.pi * 3.1 * t)  # upward ground reaction, N
    tau = np.zeros((len(t), 12))
    for i in range(0, len(t)):
        f_des = np.array([0.0, 0.0, f_z[i]])
        tau[i, 0:3] = -leg_jacobian(q[i, 0:3], 0).T @ f_des
    base_z = -fb[:, 0, 2] - (f_z / k if k else 0.0)
    moving = np.ones(len(t), dtype=bool)
    return measure(t, q, quat, planar, base_z, tau, moving, stride=5)


def test_a_rigid_rig_reads_zero_compliance() -> None:
    res = _rig(None)[0]
    slope, lo, hi = res.dz_dfz
    assert abs(slope) < 1e-6  # under 1 um/N
    assert lo <= 0.0 <= hi or abs(hi - lo) < 2e-6


def test_a_known_spring_is_recovered() -> None:
    k = 20e3  # N/m -> slope -50 um/N
    res = _rig(k)[0]
    slope, lo, hi = res.dz_dfz
    assert slope == pytest.approx(-1.0 / k, rel=0.1)
    assert hi < 0.0  # the CI knows it is compliant


def test_forces_invert_the_jacobian_transpose() -> None:
    rng = np.random.default_rng(5)
    q = rng.uniform(-0.7, 0.7, (3, 12))
    rot = np.tile(np.eye(3), (3, 1, 1))
    f_des = np.array([5.0, -3.0, 40.0])
    tau = np.zeros((3, 12))
    for i in range(3):
        tau[i, 3:6] = -leg_jacobian(q[i, 3:6], 1).T @ f_des
    got = foot_forces(q, tau, rot, 1, np.arange(3))
    assert np.allclose(got, f_des[None], atol=1e-6)
