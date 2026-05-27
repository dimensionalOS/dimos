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

"""PACK MIND — fog-of-war exploration engine tests (S1)."""

import pytest

from dimos.experimental.pack_mind.explore_sim import build_explore


@pytest.mark.unit
def test_reveal_grows() -> None:
    sim = build_explore(shared=True, n_dogs=1)
    for _ in range(50):
        sim.step()
    early = sim.revealed_frac()
    for _ in range(150):
        sim.step()
    late = sim.revealed_frac()
    assert late > early + 0.1 and late > 0.3


@pytest.mark.unit
def test_shared_reveals_faster_than_independent() -> None:
    s = build_explore(shared=True, seed=0).run()
    i = build_explore(shared=False, seed=0).run()
    assert s["ticks_to_target"] is not None
    assert i["ticks_to_target"] is None or s["ticks_to_target"] < i["ticks_to_target"]


@pytest.mark.unit
def test_independent_loses_offline_dog_knowledge() -> None:
    # INDEPENDENT: team fog = union of ONLINE dogs. Killing a dog that has
    # explored unique territory drops the team's knowledge.
    sim = build_explore(shared=False, n_dogs=3, seed=0)
    for _ in range(140):
        sim.step()
    before = sim.revealed_frac()
    sim.set_online(0, False)
    after = sim.revealed_frac()
    assert after < before - 0.1  # the dead dog's region leaves the team's knowledge


@pytest.mark.unit
def test_shared_keeps_offline_dog_knowledge() -> None:
    # SHARED: the one map persists regardless of who is online.
    sim = build_explore(shared=True, n_dogs=3, seed=0)
    for _ in range(140):
        sim.step()
    before = sim.revealed_frac()
    sim.set_online(0, False)
    after = sim.revealed_frac()
    assert after == before
