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

"""PACK MIND — hardware-free dry-run of the v4 demo's two magic beats.

Prints the dashboard-style causal chain the audience will see, driven entirely by
the real coordinator + runner (only the robot bodies are mocked). Use it to
rehearse narration and to smoke-test the brain without dogs.

    uv run python -m dimos.experimental.pack_mind.demo_pack_scene

`demo_` prefix keeps it out of pytest collection (it's a manual scene, not a test).
"""

from __future__ import annotations

from dimos.experimental.pack_mind.pack_coordinator import PackCoordinator
from dimos.experimental.pack_mind.pack_search_runner import MockDriver, act_on_finding, run_search


def _line(tag: str, msg: str) -> None:
    print(f"  {tag:<28} {msg}")


def beat_1_handoff() -> None:
    print("\n=== BEAT 1 — THE HANDOFF (a body acts on memory it never gathered) ===")
    c = PackCoordinator(["north", "east", "south", "west"])
    print(" ", c.start_search("red kit"))
    res = run_search(c, MockDriver(target_zone="east"), "alpha", "red kit")
    _line("ALPHA SEARCHED", f"{res.visited} → FOUND red kit in {res.zone}")
    _line("PACK MEMORY UPDATED", f"finding by alpha @ {res.zone}")
    bravo = MockDriver(target_zone="east")
    print(" ", act_on_finding(c, bravo, "bravo"))
    _line("BRAVO ACTED ON ALPHA'S MEMORY", f"went to {bravo.visited[-1]} — never searched it itself")


def beat_2_inheritance() -> None:
    print("\n=== BEAT 2 — INHERITANCE (the mission doesn't die with the robot) ===")
    prefs = {"alpha": ["south"], "bravo": ["north", "east", "west"]}
    c = PackCoordinator(["north", "east", "south", "west"], preferences=prefs)
    print(" ", c.start_search("red kit"))
    bravo = MockDriver(target_zone="south")  # the kit is in alpha's zone
    _line("ALPHA CLAIMS", c.assign_zone("alpha") or "-")  # "south", unfinished
    for _ in range(3):
        z = c.assign_zone("bravo")
        if not z:
            break
        bravo.goto(z)
        c.report_cleared("bravo", z)
        _line("BRAVO CLEARED", z)
    _line("ALPHA GOES OFFLINE", c.release_dog("alpha"))
    res = run_search(c, bravo, "bravo", "red kit")
    _line("BRAVO INHERITED + FINISHED", f"found red kit in {res.zone} (alpha started this zone)")
    snap = c.snapshot()
    _line("SHARED MEMORY", f"offline={snap['offline']}  found={snap['found']}  coverage={snap['coverage']}")


def main() -> None:
    print("PACK MIND — shared operational memory (hardware-free scene)")
    beat_1_handoff()
    beat_2_inheritance()
    print("\nAlpha saw it. Bravo remembered it. The pack acted. — PACK MIND\n")


if __name__ == "__main__":
    main()
