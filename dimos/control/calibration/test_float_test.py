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

from dimos.control.calibration.float_analysis import (
    DRIFT_LIMIT_MM,
    FingertipModel,
    FloatEvent,
    analyze_window,
    format_metrics,
    segment_windows,
)
from dimos.control.calibration.float_report import FloatRecording, extract_windows
from dimos.control.calibration.spec import ArmCalibrationSpec
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_HOME_PER_ARM


@pytest.fixture(scope="module")
def left_model(left_spec: ArmCalibrationSpec) -> FingertipModel:
    return FingertipModel(left_spec)


# Analysis


def test_fingertip_model_moves_with_the_shoulder(left_model: FingertipModel) -> None:
    home = list(DUAL_OPENYAM_HOME_PER_ARM)
    tip = left_model.fingertip_m(home)
    assert np.isfinite(tip).all() and tip[2] > 0.0
    moved = list(home)
    moved[1] += math.radians(1.0)
    # One degree on joint 2 swings everything downstream: a few mm at least.
    assert 3.0 < np.linalg.norm(left_model.fingertip_m(moved) - tip) * 1000.0 < 15.0
    with pytest.raises(ValueError):
        left_model.fingertip_m(home[:5])


def test_analyze_window_reports_drift_and_the_busiest_joint(left_model: FingertipModel) -> None:
    home = np.asarray(DUAL_OPENYAM_HOME_PER_ARM)
    t = np.linspace(0.0, 10.0, 201)
    q = np.tile(home, (t.size, 1))
    q[:, 2] += np.radians(0.02) * t  # 0.2 deg over 10 s on joint 3, nothing else
    event = FloatEvent(side="left", index=1, t0=0.0, duration_s=10.0)
    metrics = analyze_window(left_model, event, t, q)
    assert metrics.max_joint == "left_joint3"
    # From the first sample to the median of the last 0.25 s: a hair under 0.2 deg.
    assert 0.195 < metrics.max_joint_drift_deg < 0.2
    assert 0.0 < metrics.fingertip_drift_mm < DRIFT_LIMIT_MM
    assert metrics.fingertip_peak_mm >= metrics.fingertip_drift_mm
    assert metrics.passed and metrics.samples == 201 and metrics.duration_s == 10.0
    still = analyze_window(left_model, event, t, np.tile(home, (t.size, 1)))
    assert still.fingertip_drift_mm == 0.0 and still.passed
    with pytest.raises(ValueError, match="sample"):
        analyze_window(left_model, event, t[:1], q[:1])


def test_a_fall_in_the_first_moments_counts_toward_the_drift(
    left_model: FingertipModel,
) -> None:
    home = np.asarray(DUAL_OPENYAM_HOME_PER_ARM)
    t = np.linspace(0.0, 10.0, 1001)
    q = np.tile(home, (t.size, 1))
    q[:, 3] -= np.radians(4.5) * np.clip(t / 0.4, 0.0, 1.0)
    event = FloatEvent(side="left", index=1, t0=0.0, duration_s=10.0)
    metrics = analyze_window(left_model, event, t, q)
    assert metrics.max_joint == "left_joint4"
    assert metrics.joint_drift_deg["left_joint4"] == pytest.approx(-4.5, abs=1e-9)
    assert metrics.fingertip_drift_mm > DRIFT_LIMIT_MM and not metrics.passed


def test_segment_windows_cut_at_the_next_marker() -> None:
    a = FloatEvent(side="left", index=1, t0=100.0, duration_s=10.0)
    b = FloatEvent(side="left", index=2, t0=105.0, duration_s=10.0)
    assert segment_windows([b, a], t_end=112.0) == [(a, 100.0, 105.0), (b, 105.0, 112.0)]


def test_extract_windows_reads_each_arm_by_joint_name(
    left_spec: ArmCalibrationSpec, right_spec: ArmCalibrationSpec
) -> None:
    t = np.linspace(0.0, 4.0, 401)
    home = np.asarray(DUAL_OPENYAM_HOME_PER_ARM)
    q_rad = {}
    for spec in (left_spec, right_spec):
        for index, name in enumerate(spec.joint_names):
            # Recorded under a robot prefix; the right arm's joint 3 creeps.
            values = np.full(t.size, home[index])
            if spec is right_spec and index == 2:
                values = values + np.radians(0.05) * t
            q_rad[f"arm/{name}"] = values
    events = [
        FloatEvent(side="left", index=1, t0=0.0, duration_s=2.0),
        FloatEvent(side="right", index=1, t0=2.0, duration_s=2.0),
    ]
    specs = {"left": left_spec, "right": right_spec}
    records = extract_windows(
        FloatRecording(t=t, q_rad=q_rad, events=events), resolve=specs.__getitem__
    )
    by_side = {record.event.side: record.metrics for record in records}
    assert by_side["left"].fingertip_drift_mm == 0.0
    assert by_side["right"].max_joint == "right_joint3"
    assert by_side["right"].fingertip_drift_mm > 0.0


def test_a_window_its_samples_do_not_span_fails_as_cut_short(
    left_spec: ArmCalibrationSpec, left_model: FingertipModel
) -> None:
    home = np.asarray(DUAL_OPENYAM_HOME_PER_ARM)
    t = np.linspace(0.0, 0.2, 21)
    q = np.tile(home, (t.size, 1))
    event = FloatEvent(side="left", index=1, t0=0.0, duration_s=10.0)
    short = analyze_window(left_model, event, t, q)
    assert short.fingertip_drift_mm == 0.0 and short.requested_s == 10.0
    assert not short.complete and not short.passed and short.verdict == "cut short"
    assert short.as_row()["complete"] is False
    assert "over 0.2 of 10 s" in format_metrics(short) and "[cut short," in format_metrics(short)
    q_rad = {name: q[:, index] for index, name in enumerate(left_spec.joint_names)}
    [record] = extract_windows(
        FloatRecording(t=t, q_rad=q_rad, events=[event]), {"left": left_model}
    )
    assert not record.metrics.complete and not record.metrics.passed
    full = np.linspace(0.0, 9.95, 200)
    complete = analyze_window(left_model, event, full, np.tile(home, (full.size, 1)))
    assert complete.complete and complete.passed and complete.verdict == "ok"
