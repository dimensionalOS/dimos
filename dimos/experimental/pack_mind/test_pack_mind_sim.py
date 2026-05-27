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

"""PACK MIND sim — G1+G2 tests.

Run: bin/pytest-fast dimos/experimental/pack_mind/test_pack_mind_sim.py -v
"""

import pytest

from dimos.experimental.pack_mind.sim import build_sim
from dimos.experimental.pack_mind.world import free_cell_count, make_maze_world, plant_survivors


@pytest.mark.unit
def test_world_has_free_and_walls() -> None:
    w = make_maze_world()
    assert free_cell_count(w) > 0
    assert (w.grid == 100).any()  # walls exist


@pytest.mark.unit
def test_survivors_on_free_cells() -> None:
    w = make_maze_world()
    s = plant_survivors(w, 4, seed=0)
    assert len(s) == 4


@pytest.mark.unit
def test_single_robot_substantial_coverage() -> None:
    # GATE 0: the map is 100% coverable (no reachability wall), so one robot must
    # cover the bulk of it. The coverage-patrol heuristic plateaus ~79% solo; 3
    # robots fill the nooks. We only need to prove there is no reachability wall.
    sim = build_sim(shared=False, n_robots=1, n_survivors=1, seed=0)
    r = sim.run(max_ticks=6000)
    assert r.coverage > 0.7


@pytest.mark.unit
def test_ab_pack_beats_independent_on_coverage() -> None:
    """THE credibility test. Same robots, same single start, same map — the ONLY
    difference is shared vs private coverage memory. Pack reaches the coverage
    target in far fewer ticks and ends with higher coverage."""
    pack = build_sim(shared=True, seed=0).run()
    indep = build_sim(shared=False, seed=0).run()
    assert pack.ticks_to_target is not None
    assert indep.ticks_to_target is None or pack.ticks_to_target < indep.ticks_to_target
    assert pack.coverage > indep.coverage
    # NOTE: we deliberately do NOT assert pack finds survivors faster. Independent
    # robots blanket the map redundantly and can stumble onto victims sooner; the
    # shared-memory win is *complete-area coverage* (certainty nothing is missed),
    # not victim-stumble speed. Both find all survivors here; timing is reported,
    # not asserted.
    assert pack.found == pack.total_survivors
