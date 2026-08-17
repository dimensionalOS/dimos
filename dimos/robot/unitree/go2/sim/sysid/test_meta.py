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

"""The outer space stays tiny, and both incumbent scorers live inside it."""

from __future__ import annotations

import pytest

from dimos.robot.unitree.go2.sim.backend import CHANNELS
from dimos.robot.unitree.go2.sim.sysid.fit import Objective
from dimos.robot.unitree.go2.sim.sysid.meta import FROZEN_STYLE, GO2SIM_STYLE, OuterPoint
from dimos.robot.unitree.go2.sim.sysid.regimes import Segment, Span
from dimos.robot.unitree.go2.sim.sysid.test_score import _result


def test_the_outer_point_is_three_decisions_and_no_more():
    fields = set(OuterPoint.__dataclass_fields__) - {"name"}
    assert fields == {"stratified", "normalised", "w_flight"}


def test_w_flight_maps_onto_the_weight_vector():
    p = OuterPoint("t", stratified=True, normalised=True, w_flight=0.3)
    assert p.weights() == {("accel", "floor"): 0.7, ("accel", "flight"): 0.3}


def test_the_two_incumbent_scorers_are_seed_points():
    """The disagreement is two points in one space, not two philosophies."""
    assert not FROZEN_STYLE.stratified and not FROZEN_STYLE.normalised
    assert GO2SIM_STYLE.stratified and GO2SIM_STYLE.normalised
    assert FROZEN_STYLE.w_flight == GO2SIM_STYLE.w_flight == 0.5


class _StubRollouts:
    def run(self, specs):
        return [_result(accel_err=3.0) for _ in specs]


def _objective(normalise: bool) -> Objective:
    return Objective(
        _StubRollouts(),  # type: ignore[arg-type]
        segments=[Segment(0.0, 1.0)],
        spans=[Span("floor", 0.0, 1.0)],
        weights={("accel", "floor"): 1.0},
        backend_channels=frozenset(CHANNELS),
        normalise=normalise,
    )


def test_normalised_scoring_freezes_the_baseline_scale():
    obj = _objective(normalise=True)
    s = obj.calibrate({})
    assert obj.scales == {"accel": pytest.approx(3.0)}
    assert s.total == pytest.approx(1.0)  # the baseline normalises to itself


def test_raw_scoring_is_the_frozen_conventions_shape():
    obj = _objective(normalise=False)
    s = obj.calibrate({})
    assert obj.scales == {"accel": 1.0}
    assert s.total == pytest.approx(3.0)  # natural units, unscaled


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
