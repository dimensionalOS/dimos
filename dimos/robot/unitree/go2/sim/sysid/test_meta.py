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

"""The outer space stays tiny: the weight vector's four live axes."""

from __future__ import annotations

import pytest

from dimos.robot.unitree.go2.sim.backend import CHANNELS
from dimos.robot.unitree.go2.sim.sysid.fit import Objective, PooledObjective
from dimos.robot.unitree.go2.sim.sysid.meta import (
    DEFAULT_POINT,
    INCUMBENT_POINT,
    MDD_N8,
    OuterPoint,
    second_dr_component,
)
from dimos.robot.unitree.go2.sim.sysid.regimes import Segment, Span
from dimos.robot.unitree.go2.sim.sysid.test_score import _result


def test_the_outer_point_is_four_decisions_and_no_more():
    fields = set(OuterPoint.__dataclass_fields__) - {"name"}
    assert fields == {"w_accel", "w_flight", "w_dq", "w_tau"}


def test_the_partition_point_carries_no_body_level_channel():
    p = OuterPoint("t", w_accel=0.0, w_flight=0.25, w_dq=1.0, w_tau=0.5)
    w = p.weights()
    assert w[("joint", "floor")] == pytest.approx(0.30)
    assert w[("dq", "flight")] == pytest.approx(0.10)
    assert w[("tau", "floor")] == pytest.approx(0.15)
    assert not any(c in ("accel", "gyro", "pos", "rot") for c, _r in w)


def test_the_incumbents_objective_is_reachable_exactly():
    """The search must contain the winner as a point, not an approximation:
    w_accel=1, w_flight=0.5 IS the accel scorer the shipped plant was
    fitted under (README 4's reachability argument)."""
    w = INCUMBENT_POINT.weights()
    assert w == {("accel", "floor"): pytest.approx(0.5), ("accel", "flight"): pytest.approx(0.5)}


def test_the_default_point_matches_the_shipped_default_weights():
    from dimos.robot.unitree.go2.sim.sysid.score import DEFAULT_WEIGHTS

    w = DEFAULT_POINT.weights()
    # same RATIOS as score.DEFAULT_WEIGHTS (the score renormalises, so only
    # ratios matter): joint/floor anchors the comparison
    k = DEFAULT_WEIGHTS[("joint", "floor")] / w[("joint", "floor")]
    for key, val in DEFAULT_WEIGHTS.items():
        assert w[key] * k == pytest.approx(val), key


def test_second_dr_component_keeps_ties_and_excludes_losers():
    log = [
        {"trial": 0, "ground_loss": 1.00, "values": {"armature": 0.010}},
        {"trial": 1, "ground_loss": 1.00 + MDD_N8 / 2, "values": {"armature": 0.020}},
        {"trial": 2, "ground_loss": 1.00 + 3 * MDD_N8, "values": {"armature": 0.500}},
    ]
    dr2 = second_dr_component(log)
    assert dr2["admissible_trials"] == [0, 1]  # the clear loser is not DR
    assert dr2["spread"] == {"armature": [0.010, 0.020]}
    assert "coarse" in str(dr2["caveat"])


class _StubRollouts:
    def run(self, specs):
        return [_result(accel_err=3.0) for _ in specs]


def _objective(weights=None) -> Objective:
    return Objective(
        _StubRollouts(),  # type: ignore[arg-type]
        segments=[Segment(0.0, 1.0)],
        spans=[Span("floor", 0.0, 1.0)],
        weights=weights or {("accel", "floor"): 1.0},
        backend_channels=frozenset(CHANNELS),
    )


def test_scoring_always_freezes_the_baseline_scale():
    obj = _objective()
    s = obj.calibrate({})
    assert obj.scales is not None and obj.scales["accel"] == pytest.approx(3.0)
    assert s.total == pytest.approx(1.0)  # the baseline normalises to itself


def test_zero_weight_channels_are_still_computed_and_reported():
    """README 4: the de-weighted residuals ARE the misspecification map."""
    obj = _objective(weights={("joint", "floor"): 1.0})
    s = obj.calibrate({})
    assert ("accel", "floor") in s.terms  # unscored, still reported
    assert s.terms[("accel", "floor")] == pytest.approx(3.0)


def test_pooled_objective_shares_one_scale_across_parts():
    a, b = _objective(), _objective()
    pooled = PooledObjective([a, b])
    s = pooled.calibrate({})
    assert pooled.scales is not None and pooled.scales["accel"] == pytest.approx(3.0)
    assert len(s.per_segment) == 2  # both parts' segments pool into the mean
    with pytest.raises(ValueError, match="one weight vector"):
        PooledObjective([_objective(), _objective(weights={("dq", "floor"): 1.0})])


def test_probe_builders_propagate_the_plants_envelope():
    # A fitted-with-envelope plant must be grounded WITH its envelope: every
    # probe the latency search and the mechanism table build carries it.
    from dimos.robot.unitree.go2.sim.sysid.ground import ObsNoise
    from dimos.robot.unitree.go2.sim.sysid.meta import (
        MeasuredLoop,
        latency_probes,
        mechanism_probes,
    )

    ml = MeasuredLoop(noise=ObsNoise(), intervals=(0.02, 0.025), transport_ms=1.3)
    for probes in (
        latency_probes({}, 0.005, (0.0, 0.006), ml, replicates=2, envelope="central"),
        mechanism_probes({}, 0.005, ml, replicates=2, envelope="central"),
    ):
        assert probes and all(p.envelope == "central" for p in probes)
