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

from dimos.robot.manipulators.calibration.float_analysis import (
    DRIFT_LIMIT_MM,
    FingertipModel,
    FloatEvent,
    analyze_window,
    segment_windows,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_HOME_JOINTS
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_ARM_JOINTS,
    DUAL_OPENYAM_LEFT_ARM_JOINTS,
    DUAL_OPENYAM_RIGHT_ARM_JOINTS,
)

HOME = dict(zip(DUAL_OPENYAM_ARM_JOINTS, DUAL_OPENYAM_HOME_JOINTS, strict=True))
LOOP_HZ = 200.0
LEFT = [DUAL_OPENYAM_ARM_JOINTS.index(n) for n in DUAL_OPENYAM_LEFT_ARM_JOINTS]
RIGHT = [DUAL_OPENYAM_ARM_JOINTS.index(n) for n in DUAL_OPENYAM_RIGHT_ARM_JOINTS]


@pytest.fixture(scope="module")
def left_model() -> FingertipModel:
    return FingertipModel("left")


# float_arm / hold_arm


# Analysis


def test_fingertip_model_moves_with_the_shoulder(left_model: FingertipModel) -> None:
    home = list(DUAL_OPENYAM_HOME_JOINTS[:6])
    tip = left_model.fingertip_m(home)
    assert np.isfinite(tip).all() and tip[2] > 0.0
    moved = list(home)
    moved[1] += math.radians(1.0)
    # One degree on joint 2 swings everything downstream: a few mm at least.
    assert 3.0 < np.linalg.norm(left_model.fingertip_m(moved) - tip) * 1000.0 < 15.0
    with pytest.raises(ValueError):
        left_model.fingertip_m(home[:5])
    with pytest.raises(ValueError):
        FingertipModel("middle")


def test_analyze_window_reports_drift_and_the_busiest_joint(left_model: FingertipModel) -> None:
    home = np.asarray(DUAL_OPENYAM_HOME_JOINTS[:6])
    t = np.linspace(0.0, 10.0, 201)
    q = np.tile(home, (t.size, 1))
    q[:, 2] += np.radians(0.02) * t  # 0.2 deg over 10 s on joint 3, nothing else
    event = FloatEvent(side="left", index=1, t0=0.0, duration_s=10.0)
    metrics = analyze_window(left_model, event, t, q)
    assert metrics.max_joint == "left_joint3"
    # Medians over the first and last 0.25 s shave a little off the full 0.2 deg.
    assert 0.17 < metrics.max_joint_drift_deg < 0.2
    assert 0.0 < metrics.fingertip_drift_mm < DRIFT_LIMIT_MM
    assert metrics.fingertip_peak_mm >= metrics.fingertip_drift_mm
    assert metrics.passed and metrics.samples == 201 and metrics.duration_s == 10.0
    still = analyze_window(left_model, event, t, np.tile(home, (t.size, 1)))
    assert still.fingertip_drift_mm == 0.0 and still.passed
    with pytest.raises(ValueError, match="sample"):
        analyze_window(left_model, event, t[:1], q[:1])


def test_segment_windows_cut_at_the_next_marker() -> None:
    a = FloatEvent(side="left", index=1, t0=100.0, duration_s=10.0)
    b = FloatEvent(side="left", index=2, t0=105.0, duration_s=10.0)
    assert segment_windows([b, a], t_end=112.0) == [(a, 100.0, 105.0), (b, 105.0, 112.0)]


# Pipeline: module with a drifting mock -> memory.db -> report
