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

"""Ready-posture criteria and cargo-safe wrist heading changes."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.robot.galaxea.r1pro.classical_planning import (
    carry_ready_joints,
    carry_yaw,
)
from dimos.robot.galaxea.r1pro.posture_ik import NOMINAL_POSTURE, posture_is_valid


@pytest.mark.parametrize("loaded", [set(), {"left"}, {"right"}, {"left", "right"}])
def test_ready_accepts_bent_loaded_arms_without_requiring_home_wrist_attitude(loaded):
    home = NOMINAL_POSTURE.copy()
    joints = home.copy()
    for side, column in (("left", 8), ("right", 15)):
        if side in loaded:
            joints[column] = 0.9

    assert carry_ready_joints(joints, home, loaded)


@pytest.mark.parametrize("column,value", [(0, 0.2), (11, 0.2), (7, -np.deg2rad(15))])
def test_feasible_but_unfolded_posture_is_not_reported_as_ready(column, value):
    joints = NOMINAL_POSTURE.copy()
    joints[column] = value

    assert posture_is_valid(joints)
    assert not carry_ready_joints(joints, NOMINAL_POSTURE, {"left"})


def test_ready_never_waives_the_hard_posture_envelope_for_a_loaded_arm():
    joints = NOMINAL_POSTURE.copy()
    joints[5] = np.deg2rad(95)

    assert not carry_ready_joints(joints, NOMINAL_POSTURE, {"left"})


def test_gravity_yaw_returns_wrist_heading_without_tipping_side_grasped_cargo():
    wrist = Rotation.from_euler("xyz", [60, 0, 90], degrees=True).as_matrix()
    cargo = Rotation.from_euler("xyz", [3, 2, 20], degrees=True).as_matrix()
    attachment = wrist.T @ cargo
    yaw = carry_yaw(wrist, np.eye(3))

    assert yaw == pytest.approx(-np.pi / 2)
    for fraction in np.linspace(0, 1, 11):
        rotation = Rotation.from_rotvec([0, 0, fraction * yaw]).as_matrix() @ wrist
        assert (rotation @ attachment)[2, 2] == pytest.approx(cargo[2, 2], abs=1e-12)
    final = Rotation.from_rotvec([0, 0, yaw]).as_matrix() @ wrist
    np.testing.assert_allclose(
        final, Rotation.from_euler("x", 60, degrees=True).as_matrix(), atol=1e-12
    )
