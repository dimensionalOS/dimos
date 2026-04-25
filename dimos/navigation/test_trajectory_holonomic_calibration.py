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

"""Holonomic step-dwell-return plant calibration and v1 YAML (921 P5-1, P5-4)."""

from __future__ import annotations

import io
import math
from pathlib import Path

import pytest
import yaml

from dimos.navigation.trajectory_holonomic_calibration import (
    HOLONOMIC_TRAJECTORY_CALIBRATION_VERSION,
    HolonomicStepDwellReturnConfig,
    _estimate_first_order_tau_s,
    _suggested_gains_from_tau,
    plant_result_to_params_v1,
    read_holonomic_calibration_params_yaml,
    run_step_dwell_return_on_plant,
    write_holonomic_calibration_params_yaml,
)
from dimos.navigation.trajectory_holonomic_plant import ActuatedHolonomicPlant, IntegratedHolonomicPlant

_FIXTURE = Path(__file__).resolve().parent / "fixtures" / "holonomic_calibration_params_sample_v1.yaml"


def test_integrated_plant_completes_and_returns_near_origin() -> None:
    cfg = HolonomicStepDwellReturnConfig(
        max_displacement_m=0.5,
        step_displacement_m=0.1,
        peak_body_speed_m_s=0.3,
        dwell_s=0.0,
        dt_s=0.01,
    )
    plant = IntegratedHolonomicPlant()
    res = run_step_dwell_return_on_plant(plant, cfg)
    assert res.outbound_step_count > 0
    assert res.return_step_count == res.outbound_step_count
    assert res.dwell_step_count == 0
    assert res.return_plan_error_m == pytest.approx(0.0, abs=1e-5)


def test_lag_time_estimate_near_simulated_tau() -> None:
    dt = 0.01
    tau = 0.12
    plant = ActuatedHolonomicPlant(
        linear_lag_time_constant_s=tau,
        yaw_lag_time_constant_s=0.1,
        max_linear_accel_m_s2=50.0,
        max_yaw_accel_rad_s2=50.0,
    )
    cfg = HolonomicStepDwellReturnConfig(
        max_displacement_m=0.5,
        step_displacement_m=0.2,
        peak_body_speed_m_s=0.5,
        dwell_s=0.05,
        dt_s=dt,
    )
    res = run_step_dwell_return_on_plant(plant, cfg)
    assert res.estimated_linear_lag_s is not None
    assert res.estimated_linear_lag_s == pytest.approx(tau, rel=0.2, abs=0.04)


def test_suggested_gains_saturate() -> None:
    k_p, k_y = _suggested_gains_from_tau(0.05)
    assert k_p == pytest.approx(2.0)  # capped
    k_p0, k_y0 = _suggested_gains_from_tau(None)
    assert k_p0 == pytest.approx(1.0) and k_y0 == pytest.approx(0.75)


def test_tau_63_rise() -> None:
    dt = 0.02
    u = 0.4
    tau = 0.1
    t_samples: list[float] = [0.0]
    n = 80
    for k in range(1, n + 1):
        t = k * dt
        t_samples.append(u * (1.0 - math.exp(-t / tau)))
    est = _estimate_first_order_tau_s(dt, u, t_samples[1:])
    assert est is not None
    assert est == pytest.approx(tau, rel=0.1, abs=0.02)


def test_plant_to_yaml_round_trip() -> None:
    cfg = HolonomicStepDwellReturnConfig(
        max_displacement_m=0.3,
        step_displacement_m=0.1,
        peak_body_speed_m_s=0.25,
        dwell_s=0.1,
        dt_s=0.02,
    )
    p = IntegratedHolonomicPlant()
    r = run_step_dwell_return_on_plant(p, cfg)
    v1 = plant_result_to_params_v1(r)
    buf = io.StringIO()
    write_holonomic_calibration_params_yaml(v1, buf)
    back = read_holonomic_calibration_params_yaml(io.StringIO(buf.getvalue()))
    assert back.version == HOLONOMIC_TRAJECTORY_CALIBRATION_VERSION
    assert back.schema == "holonomic_trajectory_calibration"
    assert back.suggested_holonomic_gains["k_position_per_s"] == r.suggested_k_position_per_s


def test_reject_unsupported_version() -> None:
    t = (
        "schema: holonomic_trajectory_calibration\n"
        "version: 0\n"
        "kind: plant\n"
        'generated_utc: "2026-01-01T00:00:00+00:00"\n'
        "scenario: {}\n"
        "results: {}\n"
        "suggested_holonomic_gains: {}\n"
    )
    with pytest.raises(ValueError, match="unsupported calibration version"):
        read_holonomic_calibration_params_yaml(io.StringIO(t))


def test_load_sample_fixture() -> None:
    p = read_holonomic_calibration_params_yaml(_FIXTURE)
    assert p.kind == "plant"
    assert p.results.get("outbound_step_count") == 40
    data = yaml.safe_load(_FIXTURE.read_text(encoding="utf-8"))
    assert data["suggested_holonomic_gains"]["k_yaw_per_s"] == 0.9


def test_config_rejects_oversize_step() -> None:
    with pytest.raises(ValueError, match="step_displacement_m"):
        HolonomicStepDwellReturnConfig(
            max_displacement_m=0.1,
            step_displacement_m=0.2,
            peak_body_speed_m_s=0.1,
            dwell_s=0.0,
            dt_s=0.01,
        )
