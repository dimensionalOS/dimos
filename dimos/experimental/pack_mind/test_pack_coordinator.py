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

"""PACK MIND coordinator tests — the no-overlap and stop-on-found guarantees."""

import pytest

from dimos.experimental.pack_mind.pack_coordinator import PackCoordinator

ZONES = ["north", "east", "south", "west"]


@pytest.mark.unit
def test_assign_never_double_assigns_a_zone() -> None:
    c = PackCoordinator(ZONES)
    c.start_search("red object")
    handed: list[str] = []
    for _ in range(len(ZONES)):
        for dog in ("alpha", "bravo"):
            z = c.assign_zone(dog)
            if z is not None:
                handed.append(z)
    assert sorted(handed) == sorted(ZONES)  # every zone handed exactly once
    assert len(handed) == len(set(handed))  # no zone given twice → no overlap


@pytest.mark.unit
def test_assign_returns_none_when_exhausted() -> None:
    c = PackCoordinator(["only"])
    c.start_search("red object")
    assert c.assign_zone("alpha") == "only"
    assert c.assign_zone("bravo") is None  # nothing left for the second dog


@pytest.mark.unit
def test_finding_stops_the_pack() -> None:
    c = PackCoordinator(ZONES)
    c.start_search("red object")
    c.assign_zone("alpha")  # claims a zone
    assert not c.found
    c.report_finding("alpha", "red object", "north")
    assert c.found
    assert c.should_stop("bravo")  # the other dog learns instantly
    assert c.assign_zone("bravo") is None  # no more assignments after a find


@pytest.mark.unit
def test_finding_is_idempotent_keeps_first() -> None:
    c = PackCoordinator(ZONES)
    c.start_search("red object")
    first = c.report_finding("alpha", "red object", "north")
    second = c.report_finding("bravo", "red object", "south")
    assert second is first  # first finding wins; pack already stopped


@pytest.mark.unit
def test_preferences_make_dogs_fan_out() -> None:
    # Alpha prefers north end, Bravo the south end → first picks diverge.
    prefs = {"alpha": ["north", "east"], "bravo": ["south", "west"]}
    c = PackCoordinator(ZONES, preferences=prefs)
    c.start_search("red object")
    assert c.assign_zone("alpha") == "north"
    assert c.assign_zone("bravo") == "south"


@pytest.mark.unit
def test_coverage_and_cleared() -> None:
    c = PackCoordinator(ZONES)
    c.start_search("red object")
    assert c.coverage() == 0.0
    c.report_cleared("alpha", "north")
    c.report_cleared("bravo", "south")
    assert c.coverage() == pytest.approx(0.5)


@pytest.mark.unit
def test_report_finding_falls_back_to_claimed_zone() -> None:
    # The agent may report a finding with a wrong/empty zone (LLM-filled). The
    # coordinator should fall back to the zone the dog actually holds claimed.
    c = PackCoordinator(ZONES)
    c.start_search("red object")
    claimed = c.assign_zone("alpha")
    f = c.report_finding("alpha", "red object", "")  # empty zone
    assert f.zone == claimed
    c2 = PackCoordinator(ZONES)
    c2.start_search("red object")
    claimed2 = c2.assign_zone("alpha")
    f2 = c2.report_finding("alpha", "red object", "not_a_real_zone")  # bogus zone
    assert f2.zone == claimed2


@pytest.mark.unit
def test_release_dog_lets_survivor_inherit_unfinished_zone() -> None:
    # The resilience climax: Alpha claims a zone, goes offline before clearing it;
    # its zone returns to the pool so Bravo inherits and finishes the mission.
    c = PackCoordinator(["north", "south"])
    c.start_search("red object")
    alpha_zone = c.assign_zone("alpha")  # alpha claims one
    bravo_zone = c.assign_zone("bravo")  # bravo claims the other
    assert {alpha_zone, bravo_zone} == {"north", "south"}
    # Bravo finishes its own zone, then would have nothing left...
    c.report_cleared("bravo", bravo_zone)
    assert c.assign_zone("bravo") is None  # alpha still holds the last zone
    # Alpha drops. Its claimed-but-uncleared zone must come back to the pack.
    c.release_dog("alpha")
    assert c.assign_zone("bravo") == alpha_zone  # Bravo inherits alpha's ground


@pytest.mark.unit
def test_release_dog_keeps_findings_and_cleared_zones() -> None:
    c = PackCoordinator(["north", "south", "east"])
    c.start_search("red object")
    c.report_cleared("alpha", "north")  # alpha already cleared one
    z = c.assign_zone("alpha")  # alpha claims another, unfinished
    c.release_dog("alpha")
    snap = c.snapshot()
    states = {zz["name"]: zz["state"] for zz in snap["zones"]}  # type: ignore[index]
    assert states["north"] == "cleared"  # cleared knowledge persists
    assert states[z] == "unsearched"  # unfinished zone reclaimed
    assert "alpha" in snap["offline"]  # type: ignore[operator]


@pytest.mark.unit
def test_finding_survives_finder_going_offline() -> None:
    c = PackCoordinator(["north", "south"])
    c.start_search("red object")
    c.report_finding("alpha", "red object", "north")
    c.release_dog("alpha")  # finder drops
    assert c.found  # the find outlives the robot
    assert c.finding is not None and c.finding.zone == "north"


@pytest.mark.unit
def test_start_search_resets_ledger() -> None:
    c = PackCoordinator(ZONES)
    c.start_search("red object")
    c.report_cleared("alpha", "north")
    c.report_finding("alpha", "red object", "east")
    c.start_search("blue box")  # new mission
    assert not c.found
    assert c.coverage() == 0.0
    assert c.assign_zone("alpha") is not None
