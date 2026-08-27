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

"""The G1 presets: the built-in is the fit's artifact, and every value has a why."""

import json
from pathlib import Path

import pytest

from dimos.robot.unitree.g1.sim.ranges import (
    BUILTIN_PRESETS,
    ENGINE_DEFAULTS,
    KNOBS,
    MEASURED,
    PHYSICS_KEYS,
    load_preset,
)

PRESETS = Path(__file__).parent / "presets"


def test_measured_is_the_fits_plant_json():
    """ranges.MEASURED restates presets/measured.plant.json; they must not drift."""
    d = json.loads((PRESETS / "measured.plant.json").read_text())
    assert d["physics"] == MEASURED.physics
    assert d["actuator_tau"] == MEASURED.actuator_tau


def test_every_builtin_value_sits_inside_its_range_with_a_why():
    for p in BUILTIN_PRESETS.values():
        assert set(p.physics) <= PHYSICS_KEYS
        for k, v in p.physics.items():
            assert KNOBS[k].lo <= v <= KNOBS[k].hi, (p.name, k, v)
        if p.physics:
            assert set(p.provenance) == set(p.physics), p.name
            assert p.provenance.get("actuator_tau") or p.actuator_tau == 0.0


def test_stock_is_empty_and_the_defaults_cover_every_physics_key():
    assert load_preset("stock").physics == {}
    assert set(ENGINE_DEFAULTS) == PHYSICS_KEYS


def test_an_envelope_is_refused_on_the_g1(tmp_path):
    from dimos.simulation.sysid.presets import Preset

    path = Preset(name="env", envelope="central").save(tmp_path / "env.json")
    with pytest.raises(ValueError, match="envelope"):
        load_preset(str(path))
