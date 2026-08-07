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

"""The gold oracle must survive its own judge.

Regression for the densify_states truncation bug (fixed at d7c1b7c88):
the reference planner published yaw so coarsely that the judge's station
scoring swapped the body box for its circumscribing rotation cylinder and
vetoed the reference's own path on gen028. The invariant whose absence
produced that: for every scenario, `--planner gold` is never vetoed and
scores ~1.0 against itself.

Seeds: 28 is the historical regression world; 0 and 30 sample the mixed
embodiment roster.
"""

from __future__ import annotations

import pytest

from dimos.navigation.motion.planner.referee.geometry import AvoidanceConfig
from dimos.navigation.motion.planner.referee.scenarios import (
    EMBODIMENTS,
    SCENARIOS,
    Scenario,
    generate,
)
from dimos.navigation.motion.planner.referee.score import score_world
from dimos.navigation.motion.planner.referee.sim import Verdict, judge

GEN_SEEDS = [0, 28, 30]
WORLD_IDS = [sc.name for sc in SCENARIOS] + [f"gen{s:03d}" for s in GEN_SEEDS]


@pytest.fixture(scope="module")
def verdicts() -> dict[str, tuple[Scenario, Verdict]]:
    cfg = AvoidanceConfig()
    roster = list(EMBODIMENTS.values())
    worlds = list(SCENARIOS) + [generate(s, None, roster[s % len(roster)]) for s in GEN_SEEDS]
    return {sc.name: (sc, judge(sc, cfg, planner="gold")) for sc in worlds}


@pytest.mark.parametrize("name", WORLD_IDS)
def test_gold_survives_its_own_judge(
    verdicts: dict[str, tuple[Scenario, Verdict]], name: str
) -> None:
    sc, v = verdicts[name]
    w = score_world(v)
    assert w["dq"] is False, f"gold DQ'd on {name}"
    if sc.expect != "refuse":
        assert not v.veto, f"gold vetoed its own path on {name} (min_scored {v.min_scored:.3f})"
    if v.gold is not None:
        # Gold vs itself: only densify-vs-raw resampling separates them.
        assert w["gold"] > 0.9, f"gold scored {w['gold']} against itself on {name}"
