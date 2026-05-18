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

"""Pure, fast unit tests for the DERIVE step and the tolerance->speed
inversion — no sim, no robot. The validation that the full pipeline runs
end to end lives in the README verification steps, not here.
"""

from __future__ import annotations

import pytest

from dimos.utils.benchmarking.go2_tuning import (
    SCHEMA_VERSION,
    Go2TuningConfig,
    OperatingPoint,
    Provenance,
    derive_config,
    invert_tolerance,
)
from dimos.utils.benchmarking.plant import FopdtChannelParams, Go2PlantParams
from dimos.utils.benchmarking.velocity_profile import GO2_VX_MAX, GO2_WZ_MAX


def _plant(kvx=0.9, kvy=0.5, kwz=2.4) -> Go2PlantParams:
    return Go2PlantParams(
        vx=FopdtChannelParams(K=kvx, tau=0.40, L=0.06),
        vy=FopdtChannelParams(K=kvy, tau=0.30, L=0.05),
        wz=FopdtChannelParams(K=kwz, tau=0.60, L=0.05),
    )


def _prov(**kw) -> Provenance:
    base = dict(robot_id="go2", surface="concrete", mode="default", sim_or_hw="hw")
    base.update(kw)
    return Provenance(**base)


# --- DERIVE ---------------------------------------------------------------


def test_feedforward_is_inverse_gain_per_axis_including_real_vy():
    cfg = derive_config(_plant(kvx=0.9, kvy=0.5, kwz=2.4), _prov())
    assert cfg.feedforward.K_vx == pytest.approx(1 / 0.9)
    # vy is a real, independently-fitted channel — NOT a copy of vx.
    assert cfg.feedforward.K_vy == pytest.approx(1 / 0.5)
    assert cfg.feedforward.K_vy != pytest.approx(cfg.feedforward.K_vx)
    assert cfg.feedforward.K_wz == pytest.approx(1 / 2.4)


def test_feedforward_guards_degenerate_zero_gain():
    cfg = derive_config(_plant(kvx=0.0), _prov())
    assert cfg.feedforward.K_vx == pytest.approx(1.0)  # guard, not div-by-zero


def test_wz_max_uses_measured_ceiling_minus_margin():
    # per_amplitude wz ceiling = max(K*|amp|) = 1.5*0.5 = 0.75; minus 15%.
    pa = {"wz": [{"amplitude": 0.5, "K": 1.5}, {"amplitude": 0.3, "K": 1.4}]}
    cfg = derive_config(_plant(), _prov(), per_amplitude=pa)
    assert cfg.velocity_profile.max_angular_speed == pytest.approx(0.75 * 0.85)


def test_wz_max_falls_back_to_envelope_without_per_amplitude():
    cfg = derive_config(_plant(), _prov(), per_amplitude=None)
    assert cfg.velocity_profile.max_angular_speed == pytest.approx(GO2_WZ_MAX * 0.85)


def test_ceiling_clamped_to_saturation_envelope():
    # Un-saturated fit extrapolates past the platform envelope -> clamp.
    pa = {"wz": [{"amplitude": 1.2, "K": 2.45}], "vx": [{"amplitude": 0.9, "K": 0.92}]}
    cfg = derive_config(_plant(), _prov(), per_amplitude=pa)
    assert cfg.velocity_profile.max_angular_speed == pytest.approx(GO2_WZ_MAX * 0.85)
    assert cfg.velocity_profile.max_linear_speed <= GO2_VX_MAX


def test_linear_accel_first_order_rise_and_asymmetric_decel():
    p = _plant()
    cfg = derive_config(p, _prov())  # no per_amplitude -> vx_ceiling=GO2_VX_MAX
    assert cfg.velocity_profile.max_linear_accel == pytest.approx(GO2_VX_MAX / p.vx.tau)
    assert cfg.velocity_profile.max_linear_decel == pytest.approx(
        2.0 * cfg.velocity_profile.max_linear_accel
    )


def test_recommended_controller_is_hardcoded_baseline_with_evidence():
    cfg = derive_config(_plant(), _prov())
    assert cfg.recommended_controller.name == "baseline"
    assert cfg.recommended_controller.params["k_angular"] == 0.5
    assert "plant floor" in cfg.recommended_controller.evidence
    assert len(cfg.recommended_controller.evidence) > 50


def test_caveats_reflect_provenance():
    cfg = derive_config(_plant(), _prov(surface="ice", mode="rage", sim_or_hw="hw"))
    blob = " ".join(cfg.caveats)
    assert "ice" in blob and "rage" in blob
    cfg_sim = derive_config(_plant(), _prov(sim_or_hw="sim"))
    assert any("sim plant" in c for c in cfg_sim.caveats)


def test_derive_leaves_operating_point_map_none():
    assert derive_config(_plant(), _prov()).operating_point_map is None


def test_valid_for_tuning_only_when_hw():
    hw = derive_config(_plant(), _prov(sim_or_hw="hw"))
    assert hw.valid_for_tuning is True
    assert not any("DO NOT TUNE" in c for c in hw.caveats)

    st = derive_config(_plant(), _prov(sim_or_hw="self-test"))
    assert st.valid_for_tuning is False
    assert any("DO NOT TUNE FROM THIS" in c for c in st.caveats)
    # the loud warning must be first so it can't be missed
    assert "PIPELINE CHECK ONLY" in st.caveats[0]


def test_valid_for_tuning_survives_round_trip(tmp_path):
    st = derive_config(_plant(), _prov(sim_or_hw="self-test"))
    back = Go2TuningConfig.from_json(st.to_json(tmp_path / "st.json"))
    assert back.valid_for_tuning is False
    hw = derive_config(_plant(), _prov(sim_or_hw="hw"))
    back_hw = Go2TuningConfig.from_json(hw.to_json(tmp_path / "hw.json"))
    assert back_hw.valid_for_tuning is True


# --- artifact round-trip --------------------------------------------------


def test_json_round_trip_identity(tmp_path):
    cfg = derive_config(_plant(), _prov())
    p = cfg.to_json(tmp_path / "c.json")
    back = Go2TuningConfig.from_json(p)
    assert back.feedforward == cfg.feedforward
    assert back.velocity_profile == cfg.velocity_profile
    assert back.plant == cfg.plant
    assert back.provenance == cfg.provenance
    assert back.schema_version == SCHEMA_VERSION


def test_loader_rejects_wrong_schema_version(tmp_path):
    cfg = derive_config(_plant(), _prov())
    p = cfg.to_json(tmp_path / "c.json")
    bad = p.read_text().replace(f'"schema_version": {SCHEMA_VERSION}', '"schema_version": 999')
    (tmp_path / "bad.json").write_text(bad)
    with pytest.raises(ValueError, match="schema_version"):
        Go2TuningConfig.from_json(tmp_path / "bad.json")


# --- tolerance -> max-safe-speed inversion --------------------------------


def _pts(rows) -> list[OperatingPoint]:
    return [
        OperatingPoint(path=p, speed=s, cte_max=c, cte_rms=c * 0.6, arrived=a)
        for (p, s, c, a) in rows
    ]


def test_inversion_binding_is_min_across_paths():
    # straight tolerates fast; circle is the slow/binding path.
    pts = _pts(
        [
            ("straight", 0.5, 0.02, True),
            ("straight", 0.9, 0.04, True),
            ("circle", 0.5, 0.06, True),
            ("circle", 0.9, 0.15, True),
        ]
    )
    (row,) = invert_tolerance(pts, [10.0])  # 10 cm = 0.10 m
    # straight ok @0.9 (0.04), circle ok only @0.5 (0.06) -> binding 0.5/circle
    assert row.max_speed == pytest.approx(0.5)
    assert row.binding_path == "circle"


def test_inversion_excludes_not_arrived():
    pts = _pts([("straight", 0.5, 0.01, False), ("straight", 0.3, 0.01, True)])
    (row,) = invert_tolerance(pts, [5.0])
    assert row.max_speed == pytest.approx(0.3)  # 0.5 not-arrived excluded


def test_inversion_none_when_no_speed_meets_tolerance():
    pts = _pts([("circle", 0.5, 0.20, True), ("circle", 0.9, 0.30, True)])
    (row,) = invert_tolerance(pts, [5.0])
    assert row.max_speed is None
    assert row.binding_path is None
