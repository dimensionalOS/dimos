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

"""Anchor what physics knows: a weighed robot re-pins three parameters."""

from __future__ import annotations

import pytest

from dimos.robot.unitree.go2.sim.anchors import (
    PAYLOAD_SITES,
    STOCK_MODEL_TOTAL_MASS,
    STOCK_TRUNK_MASS,
    RobotSpec,
    derive,
    torsional_friction,
)

# The 2026-08-16 weighed robot: 16.500 kg on a kitchen scale against the
# 15.206 kg model — a bigger battery no amount of fitting found.
WEIGHED = RobotSpec.from_dict(
    {
        "mass_kg": 16.500,
        "payload": {"mass_kg": 1.194, "where": "battery_bay"},
        "foot": {"material": "rubber", "radius_m": 0.022},
    }
)


def test_weighing_the_robot_reproduces_the_published_mass_anchor():
    """The one output that IS a derivation, exact at any mass."""
    got = derive(WEIGHED)
    # (6.921 + 16.500 - 15.206408) / 6.921 = 1.186908; the published 1.18693
    # was computed from the 3-decimal model total, hence the 2e-5 slack.
    assert got["trunk_mass_scale"] == pytest.approx(1.18693, abs=5e-5)


def test_the_parallel_axis_form_reconstructs_r8_fit2_by_construction():
    """ALGEBRA GUARD ONLY — this cannot fail for a physics reason. The payload
    position was SOLVED to hit these two numbers (two unknowns, two targets),
    so exact agreement is guaranteed; what this catches is someone breaking
    the parallel-axis arithmetic. The physics case for 1.118 is elsewhere:
    left free the inertia goes to 2.218 and buys 1.6 in-sample points, but
    1.118 generalises better (-10.4% vs -6.6% on held-out jumps), and the
    derived anchors reproduce the weighed total mass on the compiled model
    (test_backend)."""
    got = derive(WEIGHED)
    assert got["trunk_com_x"] == pytest.approx(-0.01204, abs=5e-5)
    assert got["trunk_inertia_scale"] == pytest.approx(1.118, abs=5e-3)


def test_the_effective_payload_point_is_outside_the_robot_and_says_so():
    """The solved z = -0.102 is 45 mm below the trunk's underside (the
    collision box ends at z = -0.057): an effective parameter, not a battery
    location. Pinned here so nobody re-reads it as a measurement — if a
    future analysis replaces it with a physical, in-body position, this test
    MUST flip and the interpolation labels in anchors.py come off with it."""
    x, y, z = PAYLOAD_SITES["battery_bay"]
    trunk_underside_z = -0.057  # menagerie go2 trunk collision box half-height
    assert z < trunk_underside_z, "payload point moved inside the trunk: relabel anchors.py"


def test_a_mass_away_from_the_calibration_point_is_refused_not_extrapolated():
    """The live risk: mass_kg = 17.2 for a bigger battery would project
    CoM/inertia anchors from the outside-the-robot point, calibrated at a
    surplus that no longer applies. Refuse, and say what remains valid."""
    with pytest.raises(ValueError, match="calibrated") as e:
        derive(RobotSpec(mass_kg=17.2))
    assert "trunk_mass_scale" in str(e.value)  # the exact part is named as still usable


def test_the_inertia_scale_stays_physical_across_plausible_bay_positions():
    """Robustness of the interpolation's neighbourhood, not a validation:
    R8-FIT2 quotes 1.07-1.21 across its bay candidates; a ±2 cm box around
    the solved point grazes 1.069, so the assertion carries a hair of slack.
    The point stands: the scale stays near 1.1, never near the 2.2-3.5 a
    free search runs to, and the CoM shift stays negative."""
    import dimos.robot.unitree.go2.sim.anchors as anchors

    base = PAYLOAD_SITES["battery_bay"]
    for dx in (-0.02, 0.0, 0.02):
        for dz in (-0.02, 0.0, 0.02):
            site = (base[0] + dx, base[1], base[2] + dz)
            old = anchors.PAYLOAD_SITES["battery_bay"]
            anchors.PAYLOAD_SITES["battery_bay"] = site
            try:
                got = derive(WEIGHED)
            finally:
                anchors.PAYLOAD_SITES["battery_bay"] = old
            assert 1.05 <= got["trunk_inertia_scale"] <= 1.22, site
            assert got["trunk_com_x"] < 0, site


def test_the_torsional_anchor_is_the_classical_patch_result():
    """(3*pi/16)*mu*a for an 11 mm patch — half the 22 mm foot — is 0.00583,
    where the fit lands independently. Menagerie's 0.02 default implies a
    37.7 mm patch, wider than the whole foot."""
    assert torsional_friction(0.9, 0.022) == pytest.approx(0.00583, abs=5e-6)
    got = derive(WEIGHED)
    assert got["foot_friction_torsional"] == pytest.approx(0.00583, abs=5e-6)


def test_a_mass_below_the_cad_model_is_the_wrong_robot_not_an_anchor():
    with pytest.raises(ValueError, match="not above"):
        derive(RobotSpec(mass_kg=14.0))


def test_an_unknown_payload_site_is_an_error():
    spec = RobotSpec.from_dict({"mass_kg": 16.5, "payload": {"mass_kg": 1.2, "where": "saddle"}})
    with pytest.raises(ValueError, match="payload site"):
        derive(spec)


def test_robot_json_round_trips_the_documented_schema(tmp_path):
    src = {
        "mass_kg": 16.5,
        "payload": {"mass_kg": 1.194, "where": "battery_bay"},
        "foot": {"material": "rubber", "radius_m": 0.022},
        "tracker": {
            "lever_arm_m": [0.0324, 0.0, 0.186],
            "mount_yaw_deg": 272.92,
            "flip": True,
        },
        "floors": {"wood": {"mu": 0.9}},
    }
    import json

    p = tmp_path / "robot.json"
    p.write_text(json.dumps(src))
    spec = RobotSpec.from_json(p)
    assert spec.mass_kg == 16.5
    assert spec.payload is not None and spec.payload.where == "battery_bay"
    assert spec.tracker is not None and spec.tracker.flip is True
    assert spec.tracker.mount_yaw_deg == pytest.approx(272.92)


def test_the_stock_constants_are_selfconsistent():
    # trunk is part of the model total; the payload site sits behind and
    # below the trunk CoM (battery bay + tracker, lumped)
    assert STOCK_TRUNK_MASS < STOCK_MODEL_TOTAL_MASS
    x, y, z = PAYLOAD_SITES["battery_bay"]
    assert x < 0 and y == 0 and z < 0
