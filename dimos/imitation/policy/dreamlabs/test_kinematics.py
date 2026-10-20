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


import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.imitation.policy.dreamlabs.kinematics import DreamlabsKinematics


@pytest.fixture(scope="module")
def reference() -> list[dict[str, Any]]:
    # Captured once from the official 1.0.1 SDK, never recomputed by the code under test.
    return json.loads(Path(__file__).with_name("fixtures").joinpath("sdk_yam_fk.json").read_text())[
        "poses"
    ]


@pytest.fixture
def kinematics() -> DreamlabsKinematics:
    return DreamlabsKinematics()


@pytest.mark.parametrize("index", range(38))
def test_pinocchio_matches_sdk_reference(
    kinematics: DreamlabsKinematics, reference: list[dict[str, Any]], index: int
) -> None:
    fixture = reference[index]
    pose = kinematics.pose(np.array(fixture["joints_rad"]))
    np.testing.assert_allclose(pose.translation, fixture["xyz_m"], atol=2e-6, rtol=0)
    np.testing.assert_allclose(
        np.concatenate((pose.rotation[:, 0], pose.rotation[:, 1])),
        fixture["rot6"],
        atol=2e-6,
        rtol=0,
    )


def test_distinct_left_right_poses_are_arm_local_and_column_major(
    kinematics: DreamlabsKinematics,
) -> None:
    vector = np.array([0.0] * 6 + [0.0, 0.5, 0.5, -0.35, 0.0, 0.0] + [0.2, 0.8])
    state = kinematics.observation(vector, "fixture")
    np.testing.assert_allclose(state["state.left_eef_xyz"], [0.244997, 0, 0.164002], atol=2e-5)
    np.testing.assert_allclose(state["state.right_eef_xyz"], [0.262523, 0, 0.206904], atol=2e-5)
    np.testing.assert_allclose(
        state["state.right_eef_rot6"], [0, 1, 0, 0.342891, 0, 0.939375], atol=2e-5
    )
    np.testing.assert_allclose(state["state.left_grip"], [0.2])
    np.testing.assert_allclose(state["state.right_grip"], [0.8])
    assert all(
        value.dtype == np.float32 for key, value in state.items() if key.startswith("state.")
    )


@pytest.mark.parametrize("q", [[0.0] * 5, [float("nan")] * 6, [float("inf")] * 6])
def test_invalid_pose_inputs_fail(kinematics: DreamlabsKinematics, q: list[float]) -> None:
    with pytest.raises(ValueError):
        kinematics.pose(np.array(q))
