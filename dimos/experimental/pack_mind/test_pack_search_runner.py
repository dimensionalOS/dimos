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

"""PACK MIND runner tests — both magic beats end-to-end, hardware-free."""

import pytest

from dimos.experimental.pack_mind.pack_coordinator import PackCoordinator
from dimos.experimental.pack_mind.pack_search_runner import (
    MockDriver,
    act_on_finding,
    run_search,
)

ZONES = ["north", "east", "south", "west"]


@pytest.mark.unit
def test_single_dog_finds_target() -> None:
    c = PackCoordinator(ZONES)
    c.start_search("red kit")
    res = run_search(c, MockDriver(target_zone="south"), "alpha", "red kit")
    assert res.found and res.zone == "south"
    assert c.found


@pytest.mark.unit
def test_two_dogs_split_work_and_one_finds() -> None:
    # Preferences fan them out; only one zone holds the target.
    prefs = {"alpha": ["north", "east"], "bravo": ["west", "south"]}
    c = PackCoordinator(ZONES, preferences=prefs)
    c.start_search("red kit")
    # Interleave is unnecessary: whichever reaches "south" reports it and the other stops.
    a = run_search(c, MockDriver(target_zone="south"), "alpha", "red kit")
    b = run_search(c, MockDriver(target_zone="south"), "bravo", "red kit")
    assert c.found
    # No zone searched by both dogs (no overlap).
    assert not (set(a.visited) & set(b.visited))


@pytest.mark.unit
def test_handoff_acts_on_teammates_memory() -> None:
    c = PackCoordinator(ZONES)
    c.start_search("red kit")
    run_search(c, MockDriver(target_zone="north"), "alpha", "red kit")  # alpha finds
    bravo_driver = MockDriver(target_zone="north")
    msg = act_on_finding(c, bravo_driver, "bravo")  # bravo never searched
    assert "north" in msg and "alpha" in msg
    assert bravo_driver.visited[-1] == "north"  # bravo physically went on alpha's memory


@pytest.mark.unit
def test_inheritance_survivor_finishes_downed_dogs_mission() -> None:
    # Alpha claims the target's zone but goes offline before clearing it; Bravo,
    # already out of its own zones, inherits it and completes the mission.
    prefs = {"alpha": ["south"], "bravo": ["north", "east", "west"]}
    c = PackCoordinator(ZONES, preferences=prefs)
    c.start_search("red kit")
    alpha_zone = c.assign_zone("alpha")  # alpha claims "south" (where the kit is)
    assert alpha_zone == "south"
    # Bravo clears its three zones, none have the kit.
    bravo = MockDriver(target_zone="south")
    for _ in range(3):
        z = c.assign_zone("bravo")
        assert z is not None
        bravo.goto(z)
        c.report_cleared("bravo", z)
    assert c.assign_zone("bravo") is None  # alpha still holds "south"
    # Alpha drops. Bravo's loop resumes and inherits "south", finding the kit.
    c.release_dog("alpha")
    res = run_search(c, bravo, "bravo", "red kit")
    assert res.found and res.zone == "south"
    assert c.finding is not None and c.finding.by == "bravo"
