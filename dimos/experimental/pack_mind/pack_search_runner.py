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

"""PACK MIND search orchestration — the deterministic loop a dog runs.

The loop is decoupled from *how* a dog moves and sees via the ``RobotDriver``
Protocol, so the SAME orchestration runs:
  - in tests / the hardware-free scene, with ``MockDriver``;
  - on a real Go2, with a driver that wraps ``navigate_with_text`` + ``look_out_for``.

It talks to the coordinator through the ``SearchClient`` Protocol — satisfied
directly by ``PackCoordinator`` (in-process tests) or by an HTTP-backed adapter
(live). Determinism here is deliberate: a scripted loop is far more demo-reliable
than asking the LLM to remember to loop.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from dimos.experimental.pack_mind.pack_coordinator import Finding


class RobotDriver(Protocol):
    """How a body moves and perceives. Mock in tests; real skills on hardware."""

    def goto(self, zone: str) -> bool:
        """Travel to the named zone. Return True on arrival."""
        ...

    def look_for(self, target: str) -> bool:
        """Scan the current location. Return True iff the target is seen here."""
        ...


class SearchClient(Protocol):
    """The coordinator surface the loop needs. PackCoordinator satisfies this."""

    def assign_zone(self, dog: str) -> str | None: ...
    def report_cleared(self, dog: str, zone: str) -> str: ...
    def report_finding(self, dog: str, object: str, zone: str) -> Finding: ...
    def should_stop(self, dog: str) -> bool: ...
    @property
    def finding(self) -> Finding | None: ...


@dataclass
class SearchResult:
    dog: str
    found: bool
    zone: str | None
    visited: list[str] = field(default_factory=list)


def run_search(
    client: SearchClient, driver: RobotDriver, dog: str, target: str, max_steps: int = 100
) -> SearchResult:
    """Pull a distinct zone, go, look, report — until found, stopped, or empty.

    Inheriting another dog's reclaimed zone is automatic: ``assign_zone`` simply
    starts handing out the freed zones, so a survivor's loop continues seamlessly.
    """
    visited: list[str] = []
    for _ in range(max_steps):
        if client.should_stop(dog):
            break
        zone = client.assign_zone(dog)
        if not zone:
            break
        if not driver.goto(zone):
            continue  # couldn't reach it; ask for another
        visited.append(zone)
        if driver.look_for(target):
            client.report_finding(dog, target, zone)
            return SearchResult(dog, True, zone, visited)
        client.report_cleared(dog, zone)
    f = client.finding
    return SearchResult(dog, f is not None, f.zone if f else None, visited)


def act_on_finding(client: SearchClient, driver: RobotDriver, dog: str) -> str:
    """The handoff: go to a teammate's reported find — memory you never gathered."""
    f = client.finding
    if f is None:
        return f"{dog}: nothing in pack memory to act on yet."
    driver.goto(f.zone)
    return f"{dog} acted on {f.by}'s memory: went to {f.zone}, confirmed {f.object}."


@dataclass
class MockDriver:
    """Hardware-free driver: the target lives in ``target_zone``; travel always succeeds."""

    target_zone: str
    visited: list[str] = field(default_factory=list)

    def goto(self, zone: str) -> bool:
        self.visited.append(zone)
        return True

    def look_for(self, target: str) -> bool:
        return bool(self.visited) and self.visited[-1] == self.target_zone
