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

"""Knobs judged in their own metric; presets immutable and reproducible."""

from __future__ import annotations

import pytest

from dimos.robot.unitree.go2.sim.ranges import (
    ACCEL,
    BUILTIN_PRESETS,
    CONTACT_DEFAULTS,
    KNOBS,
    MEASURED,
    PHYSICS_KEYS,
    Knob,
    Preset,
    load_preset,
)


def test_a_bound_is_judged_in_the_knobs_own_metric():
    """A linear test on a log range produced four false 'at bound' warnings,
    one of which was used to argue a fit had not converged."""
    k = Knob(0.001, 0.1, log=True)
    # 0.01 is the log-midpoint of [0.001, 0.1] but sits at 9% linearly:
    # the linear test cries 'near bound', the honest one says 'centre'.
    assert k.position(0.01) == pytest.approx(0.5)
    assert not k.at_bound(0.01)
    linear = (0.01 - k.lo) / (k.hi - k.lo)
    assert linear < 0.1  # what the false warnings were made of


def test_a_linear_knob_is_judged_linearly():
    k = Knob(0.0, 10.0)
    assert k.position(5.0) == pytest.approx(0.5)
    assert k.at_bound(0.1) and k.at_bound(9.95) and not k.at_bound(1.0)


def test_every_shipped_knob_names_its_provenance():
    for name, k in KNOBS.items():
        assert k.why, f"{name}: a range without provenance is a guess"
        assert k.lo < k.hi, name
        if k.log:
            assert k.lo > 0, f"{name}: a log range needs positive bounds"


def test_physics_keys_are_the_knobs_minus_the_actuator_lag():
    assert "actuator_tau" in KNOBS and "actuator_tau" not in PHYSICS_KEYS
    assert PHYSICS_KEYS == set(KNOBS) - {"actuator_tau"}
    assert set(CONTACT_DEFAULTS) <= PHYSICS_KEYS


def test_every_preset_key_is_a_known_knob():
    for p in BUILTIN_PRESETS.values():
        assert set(p.physics) <= PHYSICS_KEYS, p.name


def test_the_measured_preset_carries_the_anchors_not_refits():
    # The robot was weighed; these are measurements, and a changed value here
    # means someone refitted an anchor, which is exactly the mistake the
    # anchoring discipline exists to prevent.
    assert MEASURED.physics["trunk_mass_scale"] == 1.18693
    assert MEASURED.physics["trunk_com_x"] == -0.01204
    assert MEASURED.physics["trunk_inertia_scale"] == 1.118
    assert ACCEL.physics["trunk_mass_scale"] == 1.18693
    assert ACCEL.physics["foot_friction_torsional"] == 0.00583


def test_a_builtin_preset_refuses_to_be_overwritten(tmp_path):
    with pytest.raises(ValueError, match="built-in"):
        MEASURED.save(tmp_path / "measured.json")


def test_a_fit_result_round_trips_through_json(tmp_path):
    p = Preset(name="fit_20260816", physics={"armature": 0.006}, actuator_tau=0.003)
    path = p.save(tmp_path / "fit.json")
    got = load_preset(str(path))
    assert got == p


def test_an_unknown_preset_name_is_an_error_not_a_silent_default():
    with pytest.raises(ValueError, match="unknown preset"):
        load_preset("no-such-preset")
