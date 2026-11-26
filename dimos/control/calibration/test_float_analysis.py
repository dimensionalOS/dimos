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

from dimos.control.calibration._toy_arm import toy_arm
from dimos.control.calibration.float_analysis import analyze_float


def test_drift_is_the_fingertip_motion_from_first_to_last_sample() -> None:
    arm = toy_arm()
    t = np.arange(1001) / 100.0
    q = np.tile([0.0, 0.3, 0.6], (t.size, 1))
    q[:, 2] += np.linspace(0.0, math.radians(0.2), t.size)
    q[500, 1] += 0.01
    drift = analyze_float(arm, t, q, duration_s=10.0, drift_limit_mm=2.0)
    # The elbow turns 0.2 deg with the 0.2 m tip arm on it.
    assert drift.fingertip_drift_mm == pytest.approx(200.0 * math.radians(0.2), rel=1e-3)
    assert drift.fingertip_peak_mm > drift.fingertip_drift_mm
    assert drift.max_joint == "elbow" and drift.passed
    assert not analyze_float(arm, t, q, duration_s=10.0, drift_limit_mm=0.5).passed


def test_a_window_cut_short_or_not_finite_raises() -> None:
    arm = toy_arm()
    t = np.arange(500) / 100.0
    q = np.zeros((t.size, 3))
    with pytest.raises(ValueError, match="cover"):
        analyze_float(arm, t, q, duration_s=10.0, drift_limit_mm=2.0)
    q[3, 0] = math.inf
    with pytest.raises(ValueError, match="NaN"):
        analyze_float(arm, t, q, duration_s=5.0, drift_limit_mm=2.0)
