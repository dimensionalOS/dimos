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

"""Draws are JOINT rows over the fit's own base and pins — never resampled."""

from __future__ import annotations

import json

import pytest

from dimos.robot.unitree.go2.sim.ranges import MEASURED, Preset
from dimos.robot.unitree.go2.sim.sysid.select import load_draws


@pytest.fixture()
def fit_artifacts(tmp_path):
    """A tiny fit output: a cheap-solver base preset and a 3-row cloud."""
    base = Preset(
        name="fitbase-test",
        physics={**MEASURED.physics, "solver_iterations": 1.0, "solver_ls_iterations": 5.0},
        actuator_tau=MEASURED.actuator_tau,
        envelope="central",
    )
    base_path = tmp_path / "fitbase.json"
    base.save(base_path)
    ranges = {
        "cloud": {
            "foot_solref_time": [0.010, 0.012, 0.014],
            "foot_solimp_width": [0.030, 0.040, 0.050],
        },
        "pinned": {"foot_friction": {"value": 0.9, "why": "declared"}},
    }
    ranges_path = tmp_path / "fit.ranges.json"
    ranges_path.write_text(json.dumps(ranges))
    return ranges_path, str(base_path)


def test_a_draw_is_one_joint_row_over_base_and_pins(fit_artifacts):
    ranges_path, base_path = fit_artifacts
    draws = load_draws(ranges_path, base_path)
    assert [d.index for d in draws] == [0, 1, 2]
    # row 1 everywhere: joint, not per-knob resampled
    d = draws[1]
    assert d.values["foot_solref_time"] == 0.012
    assert d.values["foot_solimp_width"] == 0.040
    # pins and base ride along untouched
    assert d.values["foot_friction"] == 0.9
    assert d.values["solver_iterations"] == 1.0
    assert d.values["armature"] == MEASURED.physics["armature"]
    assert d.values["actuator_tau"] == MEASURED.actuator_tau


def test_a_draw_preset_carries_envelope_and_provenance(fit_artifacts):
    ranges_path, base_path = fit_artifacts
    d = load_draws(ranges_path, base_path)[2]
    p = d.preset("cand-draw002", envelope="central", why="selected: test")
    assert p.envelope == "central"
    assert p.actuator_tau == MEASURED.actuator_tau
    assert "actuator_tau" not in p.physics
    assert p.physics["foot_solimp_width"] == 0.050
    assert all(v == "selected: test" for v in p.provenance.values())


def test_misaligned_cloud_rows_are_an_error(tmp_path, fit_artifacts):
    _ranges_path, base_path = fit_artifacts
    bad = tmp_path / "bad.ranges.json"
    bad.write_text(json.dumps({"cloud": {"a_knob": [1.0, 2.0], "b_knob": [1.0]}}))
    with pytest.raises(ValueError, match="index-aligned"):
        load_draws(bad, base_path)


def test_a_cloudless_fit_cannot_be_selected_from(tmp_path, fit_artifacts):
    _ranges_path, base_path = fit_artifacts
    empty = tmp_path / "empty.ranges.json"
    empty.write_text(json.dumps({"point": {}}))
    with pytest.raises(ValueError, match="no cloud"):
        load_draws(empty, base_path)
