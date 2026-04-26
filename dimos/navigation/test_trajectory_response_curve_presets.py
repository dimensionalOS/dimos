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

"""Response-curve preset contract tests for trajectory simulation artifacts."""

from __future__ import annotations

import pytest

from dimos.navigation.trajectory_response_curve_presets import (
    RESPONSE_CURVE_PRESETS,
    RESPONSE_CURVE_SCHEMA,
    RESPONSE_CURVE_VERSION,
    ResponseCurvePreset,
    get_response_curve_preset,
)


def test_required_synthetic_presets_exist_and_are_pr_gate_safe() -> None:
    required = {
        "ideal",
        "synthetic_nominal",
        "synthetic_sluggish",
        "synthetic_asymmetric",
        "synthetic_noisy",
        "synthetic_conservative_timing_stress",
    }
    assert required.issubset(RESPONSE_CURVE_PRESETS)
    for name in required:
        RESPONSE_CURVE_PRESETS[name].validate_for_pr_gate()


def test_plant_config_records_reproducible_curve_parameters() -> None:
    cfg = get_response_curve_preset("synthetic_nominal").to_plant_config()
    assert cfg["model"] == "actuated_holonomic"
    assert cfg["response_curve_source"] == "synthetic"
    assert cfg["response_curve_id"] == "synthetic_nominal_v1"
    assert cfg["response_curve_version"] == RESPONSE_CURVE_VERSION
    assert cfg["parameters"]["schema"] == RESPONSE_CURVE_SCHEMA
    assert cfg["parameters"]["version"] == RESPONSE_CURVE_VERSION
    curves = cfg["parameters"]["curves"]
    assert {curve["axis"] for curve in curves} == {"linear_x", "linear_y", "yaw"}
    assert all("tau_s" in curve and "max_accel" in curve for curve in curves)
    assert all("command_gain" in curve and "deadband" in curve for curve in curves)
    assert all("saturation" in curve and "noise_max" in curve for curve in curves)
    assert all("latency_s" in curve and "notes" in curve for curve in curves)


def test_unknown_response_curve_source_rejected_for_pr_gate() -> None:
    known = get_response_curve_preset("ideal")
    preset = ResponseCurvePreset(
        name="draft_unknown",
        response_curve_id="draft_unknown_v1",
        response_curve_version=RESPONSE_CURVE_VERSION,
        source="unknown",
        plant_model=known.plant_model,
        curves=known.curves,
        notes="Draft response curve that must not be used for PR gates.",
    )
    with pytest.raises(ValueError, match="not allowed for PR-gate"):
        preset.validate_for_pr_gate()


def test_ideal_preset_maps_to_current_actuated_plant_kwargs() -> None:
    kwargs = get_response_curve_preset("ideal").actuated_plant_kwargs()
    assert kwargs == {
        "linear_lag_time_constant_s": 0.0,
        "yaw_lag_time_constant_s": 0.0,
        "max_linear_accel_m_s2": 1.0e6,
        "max_yaw_accel_rad_s2": 1.0e6,
        "noise_linear_max_m_s": 0.0,
        "noise_yaw_max_rad_s": 0.0,
    }


def test_direction_specific_preset_requires_response_curve_runner() -> None:
    with pytest.raises(ValueError, match="not representable"):
        get_response_curve_preset("synthetic_asymmetric").actuated_plant_kwargs()
