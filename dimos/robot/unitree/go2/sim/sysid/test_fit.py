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

"""The fit driver: pins vs searches, harvest, median, LOO-spread stopping."""

from __future__ import annotations

import json

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.ranges import KNOBS, Knob
from dimos.robot.unitree.go2.sim.sysid.fit import (
    DEFAULT_SEARCH,
    FitResult,
    KnobPlan,
    Pin,
    StudyOutcome,
    _out_file,
    _overlap_seconds,
    base_values,
    default_plan,
    fit,
    format_report,
    load_knob_plan,
    loo_drift,
    merged,
    point_and_spread,
    pooled_cloud,
)
from dimos.simulation.sysid.regimes import Segment
from dimos.simulation.sysid.score import Score

# ---------------------------------------------------------------- knob plan


def test_a_knob_cannot_be_both_pinned_and_searched():
    with pytest.raises(ValueError, match="both pinned and searched"):
        KnobPlan(pinned={"armature": Pin(0.01, "test")}, searched={"armature": KNOBS["armature"]})


def test_the_default_plan_pins_the_measured_anchors_and_searches_the_rest():
    plan = default_plan(KNOBS)
    assert set(plan.searched) == set(DEFAULT_SEARCH)
    # the anchors, at their published values
    assert plan.pinned["trunk_mass_scale"].value == pytest.approx(1.18693, abs=1e-4)
    assert plan.pinned["trunk_com_x"].value == pytest.approx(-0.01204, abs=1e-4)
    assert plan.pinned["trunk_inertia_scale"].value == pytest.approx(1.118, abs=1e-3)
    assert plan.pinned["foot_friction_torsional"].value == pytest.approx(0.00583, abs=1e-4)
    assert plan.pinned["damping"].value == pytest.approx(0.03808)
    # every pin says where it came from
    assert all(p.why for p in plan.pinned.values())


def test_an_explicit_search_overrides_a_default_pin():
    """Pinning does not delete a range: asking to search `damping` searches it,
    using the backend's declared range."""
    plan = default_plan(KNOBS, search=("damping", "armature"))
    assert "damping" in plan.searched and "damping" not in plan.pinned
    assert plan.searched["damping"] == KNOBS["damping"]


def test_knobs_json_expresses_pin_and_search(tmp_path):
    p = tmp_path / "knobs.json"
    p.write_text(
        json.dumps(
            {
                "armature": {"search": [0.002, 0.03], "log": True, "why": "narrowed for a test"},
                "damping": {"search": True},
                "trunk_mass_scale": {"pin": 1.2, "why": "scale"},
            }
        )
    )
    plan = load_knob_plan(p, KNOBS)
    assert plan.searched["armature"].lo == 0.002 and plan.searched["armature"].log
    assert plan.searched["damping"] == KNOBS["damping"]
    assert plan.pinned["trunk_mass_scale"] == Pin(1.2, "scale")
    p.write_text(json.dumps({"armature": {"frob": 1}}))
    with pytest.raises(ValueError, match="pin.*search|'pin' or 'search'"):
        load_knob_plan(p, KNOBS)
    p.write_text(json.dumps({"warp_drive": {"pin": 1}}))
    with pytest.raises(KeyError, match="does not expose"):
        load_knob_plan(p, KNOBS)


def test_merged_layers_base_then_pins_then_draws():
    plan = KnobPlan(pinned={"damping": Pin(0.5, "t")}, searched={"armature": KNOBS["armature"]})
    base = {"damping": 0.1, "armature": 0.01, "frictionloss": 1.0}
    got = merged(base, plan, {"armature": 0.02})
    assert got == {"damping": 0.5, "armature": 0.02, "frictionloss": 1.0}


def test_base_values_fill_the_contact_defaults():
    """Every searched knob needs a start value for the incumbent trial."""
    vals = base_values("measured")
    for name in DEFAULT_SEARCH:
        assert name in vals, name


# ------------------------------------------------------- pooling and spread


def _outcome(seed: int, values: dict[str, np.ndarray], best: float = 1.0) -> StudyOutcome:
    n = len(next(iter(values.values())))
    return StudyOutcome(
        seed=seed,
        n_trials=90,
        best_total=best,
        best_params={k: float(v[0]) for k, v in values.items()},
        tol=0.01,
        harvested=values,
        n_harvested=n,
    )


def test_the_point_is_the_median_of_the_pool_never_the_best_draw():
    """The best draw is an extreme of the region; picking it spends the
    held-out set to make the choice."""
    rng = np.random.default_rng(0)
    cloud = {"armature": np.concatenate([rng.uniform(0.004, 0.006, 50), [0.049]])}
    point, spread = point_and_spread(cloud)
    # 0.049 is the "best draw" here by construction; the point must ignore it
    assert 0.004 < point["armature"] < 0.006
    assert spread["armature"][0] < point["armature"] < spread["armature"][1]


def test_loo_drift_is_small_for_agreeing_studies_and_large_for_an_outlier():
    knob = Knob(0.001, 0.05, log=True)
    rng = np.random.default_rng(1)
    same = [_outcome(s, {"armature": rng.uniform(0.005, 0.015, 40)}) for s in range(5)]
    assert loo_drift(same, {"armature": knob}) < 0.10
    outlier = [*same[:3], _outcome(9, {"armature": np.full(40, 0.049)})]
    assert loo_drift(outlier, {"armature": knob}) > 0.10


def test_pooled_cloud_concatenates_every_studys_harvest():
    a = _outcome(0, {"k": np.array([1.0, 2.0])})
    b = _outcome(1, {"k": np.array([3.0])})
    assert list(pooled_cloud([a, b], ["k"])["k"]) == [1.0, 2.0, 3.0]


def test_overlap_is_measured_not_ignored():
    segs = [Segment(0.0, 5.0), Segment(4.0, 5.0), Segment(20.0, 2.0)]
    assert _overlap_seconds(segs) == pytest.approx(1.0)


# --------------------------------------------- the driver, physics stubbed out


class _Valley:
    """A deterministic objective with a degenerate valley: any (a, b) with
    a*b == 1e-4 scores the same, which is exactly the region phenomenon."""

    calls: int

    def __init__(self) -> None:
        self.calls = 0
        self.scales = None

    def calibrate(self, values):
        self.scales = {"accel": 1.0}
        return self.evaluate(values)

    def evaluate(self, values):
        import math

        self.calls += 1
        a, b = values["armature"], values["actuator_tau"]
        loss = abs(a * b / 1e-4 - 1.0) + 0.05
        # per-segment scatter NOT proportional to the loss, so the paired
        # harvest tolerance is non-degenerate, as with real segments
        per = (loss + 0.02 * math.sin(997 * a), loss, loss + 0.02 * math.cos(991 * b))
        return Score(total=float(np.mean(per)), terms={("accel", "floor"): loss}, per_segment=per)


def _valley_plan() -> KnobPlan:
    return KnobPlan(
        pinned={"damping": Pin(0.03808, "test")},
        searched={
            "armature": Knob(0.001, 0.05, log=True),
            "actuator_tau": Knob(0.0005, 0.02, log=True),
        },
    )


def test_the_fit_restarts_harvests_and_ships_a_median_with_a_spread():
    pytest.importorskip("optuna")
    plan = _valley_plan()
    base = {"armature": 0.02899, "actuator_tau": 0.00525, "damping": 0.1}
    res = fit(_Valley(), plan, base, trials=25, min_studies=3, max_studies=4, batch=1)
    assert res.stopped in ("stable", "cap")
    assert len(res.studies) >= 3
    for o in res.studies:
        assert o.n_trials == 25
        assert o.n_harvested >= 1
        assert o.tol > 0  # 1 SE of the loss across (stub) segments
    cloud = res.cloud["armature"]
    assert res.point["armature"] == pytest.approx(float(np.median(cloud)))
    lo, hi = res.spread["armature"]
    assert lo <= res.point["armature"] <= hi
    # the report renders and says what was pinned and why
    text = format_report(res)
    assert "damping" in text and "test" in text
    assert "median" in text


def test_the_fit_is_reproducible_from_its_seeds():
    """Studies are seeded and trials sequential; two runs of the same fit are
    the same fit — this is what the parallelism rules protect."""
    pytest.importorskip("optuna")
    base = {"armature": 0.02899, "actuator_tau": 0.00525, "damping": 0.1}
    a = fit(_Valley(), _valley_plan(), base, trials=12, min_studies=3, max_studies=3, batch=1)
    b = fit(_Valley(), _valley_plan(), base, trials=12, min_studies=3, max_studies=3, batch=1)
    assert a.point == b.point
    for k in a.cloud:
        assert np.array_equal(a.cloud[k], b.cloud[k])
    assert [o.best_total for o in a.studies] == [o.best_total for o in b.studies]


def test_a_diagnostic_single_study_is_labelled_as_such():
    pytest.importorskip("optuna")
    base = {"armature": 0.02899, "actuator_tau": 0.00525, "damping": 0.1}
    res = fit(_Valley(), _valley_plan(), base, trials=8, min_studies=1, max_studies=1, batch=1)
    assert res.stopped == "diagnostic"
    assert "DIAGNOSTIC" in format_report(res)


def test_unresolved_knobs_are_named_in_the_report():
    plan = _valley_plan()
    wide = {
        "armature": np.array([0.0011] * 10 + [0.049] * 10),
        "actuator_tau": np.array([0.001] * 10 + [0.0011] * 10),
    }
    point, spread = point_and_spread(wide)
    res = FitResult(
        plan=plan,
        point=point,
        spread=spread,
        cloud=wide,
        studies=[_outcome(0, wide)],
        drift_trace=[],
        stopped="cap",
        baseline=Score(1.0, {}, (1.0,)),
        point_score=Score(0.9, {}, (0.9,)),
    )
    assert res.unresolved() == ["armature"]
    text = format_report(res)
    assert "UNRESOLVED" in text
    assert "HIT THE STUDY CAP" in text  # the cap is a result, not a failure


def test_a_dotted_out_stem_keeps_its_dot():
    """`--out sweep_lam0.75` once wrote `sweep_lam0.plant.json` — with_suffix
    ate the `.75`, so all three sweep points collided onto ONE file and the
    last writer won. The output name must be a plain append."""
    from pathlib import Path

    assert _out_file(Path("/x/sweep_lam0.75"), ".plant.json") == Path("/x/sweep_lam0.75.plant.json")
    assert _out_file(Path("/x/plain"), ".ranges.json") == Path("/x/plain.ranges.json")
