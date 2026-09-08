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

import math

import numpy as np
import pytest

from dimos.evals.suites.sf_office_pose_preprocessing import preprocess_encoded_poses


def _encoded(timestamp: float, x: float, *, yaw_deg: float | None = None) -> dict[str, object]:
    encoded: dict[str, object] = {
        "timestamp_s": timestamp,
        "position_m": [x, 0.0, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    if yaw_deg is not None:
        encoded["yaw_deg"] = yaw_deg
    return encoded


def test_preprocess_encoded_poses_sorts_deduplicates_and_resamples() -> None:
    encoded = [_encoded(float(index), float(index)) for index in reversed(range(12))]
    encoded.append(_encoded(5.0, 50.0))

    trajectory = preprocess_encoded_poses(encoded)

    assert trajectory["time_s"][-1] == 11.0
    assert len(trajectory["time_s"]) == 111
    assert trajectory["position_m"].shape == (111, 3)
    assert trajectory["velocity_xy_m_s"].shape == (111, 2)
    assert trajectory["speed_m_s"].shape == (111,)
    assert np.all(np.isfinite(trajectory["position_m"]))


def test_preprocess_encoded_poses_keeps_fractional_tenth_endpoint() -> None:
    encoded = [_encoded(index * 0.12, float(index)) for index in range(11)]

    trajectory = preprocess_encoded_poses(encoded)

    assert len(trajectory["time_s"]) == 13
    assert trajectory["time_s"][-1] == pytest.approx(1.2)


def test_preprocess_encoded_poses_accepts_verified_yaw_alias() -> None:
    encoded = [_encoded(float(index), float(index), yaw_deg=90.0) for index in range(12)]
    for index, pose in enumerate(encoded):
        pose["planar_position_m"] = [float(index * 2), 1.0]

    trajectory = preprocess_encoded_poses(encoded)

    np.testing.assert_allclose(trajectory["yaw_rad"], math.pi / 2)
    np.testing.assert_allclose(trajectory["position_m"][-1, :2], [22.0, 1.0])
