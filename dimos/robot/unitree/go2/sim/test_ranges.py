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

from dimos.robot.unitree.go2.sim.anchors import torsional_friction
from dimos.robot.unitree.go2.sim.ranges import (
    BUILTIN_PRESETS,
    CONTACT_DEFAULTS,
    DR_FLOOR,
    ENGINE_DEFAULTS,
    FAST,
    FAST_REFIT,
    FLOOR_MU,
    GO2,
    KNOBS,
    MEASURED,
    PHYSICS_KEYS,
    SOLVER_DEFAULTS,
    SOLVER_KEYS,
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
    assert SOLVER_KEYS <= PHYSICS_KEYS
    assert ENGINE_DEFAULTS == {**CONTACT_DEFAULTS, **SOLVER_DEFAULTS}


def test_the_shipped_plant_records_its_solver_explicitly():
    """Once the solver is a choice (the batched engine made it one), a preset
    must fully determine it. `measured` records the scene's own values — the
    physical no-op is held by test_backend.py; this holds the SCHEMA: the
    keys are present, they are the declared scene defaults, and each names
    its provenance."""
    for k in SOLVER_KEYS:
        assert MEASURED.physics[k] == SOLVER_DEFAULTS[k], k
        assert MEASURED.provenance.get(k), f"measured.{k}: no provenance"
    # stock stays bare menagerie: recording the solver there would make
    # "untouched" a lie by one dict entry.
    assert not (SOLVER_KEYS & set(BUILTIN_PRESETS["stock"].physics))


def test_every_preset_key_is_a_known_knob():
    for p in BUILTIN_PRESETS.values():
        assert set(p.physics) <= PHYSICS_KEYS, p.name


def test_every_preset_value_lies_inside_the_range_that_admits_it():
    """Containment is the symptom check: the discarded early plant shipped
    foot_friction = 0.635 BELOW DR_FLOOR's declared (0.8, 1.0) — a value the
    fit had no gradient on, drifted to a resting place and frozen. A preset
    value outside its own declared range is residue, not a measurement."""
    for p in BUILTIN_PRESETS.values():
        for k, v in p.physics.items():
            pos = KNOBS[k].position(v)
            assert 0.0 <= pos <= 1.0, f"{p.name}.{k} = {v} outside its Knob range"
        if p.actuator_tau:
            assert 0.0 <= KNOBS["actuator_tau"].position(p.actuator_tau) <= 1.0, p.name
        if "foot_friction" in p.physics:
            lo, hi = DR_FLOOR["foot_friction"]
            assert lo <= p.physics["foot_friction"] <= hi, p.name


def test_every_derived_value_matches_its_own_derivation():
    """The cause check: a derived value stored beside its source can go
    stale — the discarded plant's torsional friction implied mu = 0.349
    against its own declared 0.635, two numbers in one dict that had stopped
    agreeing. Anything the codebase computes from something else must equal
    its formula where it ships."""
    for p in BUILTIN_PRESETS.values():
        if "foot_friction_torsional" in p.physics:
            assert p.physics["foot_friction_torsional"] == torsional_friction(
                p.physics["foot_friction"], GO2.foot.radius_m
            ), p.name


def test_the_shipped_plants_anchors_are_the_derivation_not_copies():
    from dimos.robot.unitree.go2.sim.anchors import derive

    anchors = derive(GO2, floor_mu=FLOOR_MU)
    for k, v in anchors.items():
        assert MEASURED.physics[k] == v, k


def test_the_shipped_plant_documents_every_values_provenance():
    """README 3a in the data: every value says whether it was fitted,
    derived, declared or measured. A value without provenance is a guess
    wearing a number's clothes — the same rule Knob.why already enforces."""
    for k in MEASURED.physics:
        assert MEASURED.provenance.get(k), f"measured.{k}: no provenance"
    assert MEASURED.provenance.get("actuator_tau")
    assert MEASURED.provenance.get("envelope")


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


def test_a_preset_carries_its_envelope_through_json(tmp_path):
    p = Preset(name="fit-env", physics={"armature": 0.006}, envelope="central")
    got = load_preset(str(p.save(tmp_path / "fit-env.json")))
    assert got == p and got.envelope == "central"


def test_an_envelope_free_preset_writes_no_envelope_key(tmp_path):
    import json

    p = Preset(name="fit-plain", physics={"armature": 0.006})
    d = json.loads(p.save(tmp_path / "fit-plain.json").read_text())
    assert "envelope" not in d  # older readers see the exact old shape


def test_three_builtins_with_distinct_jobs():
    """One plant to ship (measured knobs + measured envelope, README 9), one
    to compare against (stock = bare menagerie, the experimental control
    every comparative claim needs), and one to train in (fast = the contact
    re-identified for the cheap solver the batched engine wants). Nothing
    else: a built-in that exists because it used to be someone's answer is
    history, not architecture."""
    assert set(BUILTIN_PRESETS) == {"stock", "measured", "fast"}
    assert MEASURED.envelope == "central"
    stock = BUILTIN_PRESETS["stock"]
    assert stock.physics == {} and stock.envelope is None and stock.actuator_tau == 0.0


def test_fast_is_measured_with_a_reidentified_contact_and_nothing_else():
    """The isolation claim in the data: `fast` may differ from `measured`
    ONLY in the solver choice and the four contact knobs refit under it —
    anything else moving would confound 'what did the cheap solver cost'."""
    refit = set(FAST_REFIT)
    assert refit == SOLVER_KEYS | {
        "foot_solref_time",
        "foot_solref_damp",
        "foot_solimp_dmin",
        "foot_solimp_width",
    }
    for k, v in FAST.physics.items():
        if k not in refit:
            assert v == MEASURED.physics[k], f"fast.{k} drifted from measured"
    assert FAST.actuator_tau == MEASURED.actuator_tau
    assert FAST.envelope == MEASURED.envelope
    # the cheap solver, not the recorded scene default
    assert FAST.physics["solver_iterations"] == 1.0
    assert FAST.physics["solver_ls_iterations"] == 5.0
    assert FAST.physics["solver_cone"] == 1.0  # elliptic survived; pyramidal measured dead
    for k in FAST.physics:
        assert FAST.provenance.get(k), f"fast.{k}: no provenance"


def test_a_preset_round_trips_its_provenance(tmp_path):
    p = Preset(
        name="fit-prov",
        physics={"armature": 0.006},
        provenance={"armature": "fitted: test"},
    )
    got = load_preset(str(p.save(tmp_path / "fit-prov.json")))
    assert got == p and got.provenance["armature"] == "fitted: test"


def test_an_unknown_envelope_name_is_rejected_at_construction():
    with pytest.raises(ValueError, match="envelope"):
        Preset(name="typo", envelope="centrall")
