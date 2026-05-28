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

"""PACK MIND — laptop-side coordinator: the shared MEANING of the pack.

This is the brain that lets two (or more) dogs search without overlapping. It
holds a **zone ledger** (which named regions are unsearched / claimed / cleared)
and a **target blackboard** (what we're looking for, whether it's been found and
where). It deliberately stores **no coordinates** — only zone NAMES — so it works
across dogs that each run their own SLAM frame (the honest "share meaning, not
maps" design, same invariant as conductor.py).

The two guarantees that make the demo true:
  1. ``assign_zone`` never hands the same zone to two dogs (no overlap).
  2. once any dog reports a finding, ``found`` flips and assignment stops (the
     whole pack knows and halts).

Pure Python, thread-safe, no hardware deps — unit-testable in isolation and the
core of the HTTP coordinator (mockable like ``conductor.py --mock``).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Literal

ZoneState = Literal["unsearched", "claimed", "cleared"]


@dataclass(frozen=True)
class Finding:
    """An immutable record of an object sighting — by zone NAME, never coordinates."""

    object: str
    zone: str
    by: str
    ts: float


@dataclass
class Zone:
    name: str
    state: ZoneState = "unsearched"
    by: str | None = None  # the dog that claimed or cleared it


class PackCoordinator:
    """Shared search ledger + target blackboard for a pack of dogs.

    Args:
        zones: ordered zone names covering the search area.
        preferences: optional per-dog ordering of zones, so dogs fan out to
            different ends without any coordinate math (e.g. dog A prefers the
            north zones, dog B the south). Falls back to ``zones`` order.
    """

    def __init__(
        self, zones: list[str], preferences: dict[str, list[str]] | None = None
    ) -> None:
        if not zones:
            raise ValueError("PackCoordinator needs at least one zone")
        self._order = list(zones)
        self._zones: dict[str, Zone] = {z: Zone(z) for z in zones}
        self._preferences = preferences or {}
        self._lock = threading.RLock()
        self._target: str | None = None
        self._finding: Finding | None = None
        self._offline: set[str] = set()
        self._log: list[str] = []

    # -- mission ------------------------------------------------------------

    def start_search(self, target: str) -> str:
        """Begin a search for ``target``; resets the ledger so every zone is
        unsearched again. Returns a status line."""
        with self._lock:
            self._target = target
            self._finding = None
            self._offline = set()
            self._zones = {z: Zone(z) for z in self._order}
            self._log.append(f"search started: find '{target}'")
            return f"Searching for {target}. {len(self._zones)} zones to cover."

    # -- allocation (the no-overlap guarantee) ------------------------------

    def assign_zone(self, dog: str) -> str | None:
        """Hand ``dog`` a zone no other dog has claimed or cleared, following
        the dog's preference order. Returns None when the search is over (found
        or nothing left) — the dog should then stop / converge."""
        with self._lock:
            if self._finding is not None:
                return None  # object found → pack stops searching
            for name in self._iter_order(dog):
                zone = self._zones[name]
                if zone.state == "unsearched":
                    self._zones[name] = Zone(name, "claimed", dog)
                    self._log.append(f"{dog} -> {name} (claimed)")
                    return name
            return None

    def report_cleared(self, dog: str, zone: str) -> str:
        """Mark ``zone`` fully searched by ``dog`` (object not here)."""
        with self._lock:
            if zone in self._zones:
                self._zones[zone] = Zone(zone, "cleared", dog)
                self._log.append(f"{dog} cleared {zone}")
            return f"{zone} cleared."

    def report_finding(self, dog: str, object: str, zone: str = "") -> Finding:
        """Record that ``dog`` saw ``object``. Flips ``found`` so the whole pack
        stops. Idempotent: keeps the first finding.

        ``zone`` is best-effort: if it's empty or unknown (e.g. an LLM filled it in
        wrong via a ``then=`` continuation), we fall back to the zone this dog
        currently has claimed — the place it actually is."""
        with self._lock:
            if self._finding is None:
                if not zone or zone not in self._zones:
                    zone = self._claimed_zone_of(dog) or zone
                self._finding = Finding(object, zone, dog, time.time())
                if zone in self._zones:
                    self._zones[zone] = Zone(zone, "cleared", dog)
                self._log.append(f"{dog} FOUND {object} in {zone or '?'} — pack stop")
            return self._finding

    def _claimed_zone_of(self, dog: str) -> str | None:
        """The zone this dog currently holds claimed (where it is), if any."""
        for name, z in self._zones.items():
            if z.state == "claimed" and z.by == dog:
                return name
        return None

    def release_dog(self, dog: str) -> str:
        """Take ``dog`` offline and hand its unfinished ground back to the pack.

        Its *findings persist* (the shared memory outlives the robot), but every
        zone it had only **claimed** (not yet cleared) reverts to ``unsearched`` so
        a surviving teammate inherits and finishes the mission. Cleared zones stay
        cleared — that knowledge is not lost. This powers the resilience climax:
        "the mission doesn't die with the robot."
        """
        with self._lock:
            self._offline.add(dog)
            reclaimed = []
            for name, zone in self._zones.items():
                if zone.state == "claimed" and zone.by == dog:
                    self._zones[name] = Zone(name)  # back to unsearched
                    reclaimed.append(name)
            self._log.append(
                f"{dog} OFFLINE — findings kept; reclaimed {reclaimed or 'no'} zone(s)"
            )
            return f"{dog} offline. Reclaimed {len(reclaimed)} zone(s) for the pack."

    # -- queries ------------------------------------------------------------

    @property
    def found(self) -> bool:
        with self._lock:
            return self._finding is not None

    @property
    def finding(self) -> Finding | None:
        with self._lock:
            return self._finding

    def should_stop(self, dog: str) -> bool:
        """A dog polls this; True once the object is found by anyone."""
        return self.found

    def coverage(self) -> float:
        """Fraction of zones cleared — the 'how much of the area is searched' metric."""
        with self._lock:
            cleared = sum(1 for z in self._zones.values() if z.state == "cleared")
            return cleared / len(self._zones)

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return {
                "target": self._target,
                "found": self._finding is not None,
                "finding": (
                    {
                        "object": self._finding.object,
                        "zone": self._finding.zone,
                        "by": self._finding.by,
                    }
                    if self._finding
                    else None
                ),
                "coverage": round(self.coverage(), 3),
                "offline": sorted(self._offline),
                "zones": [
                    {"name": z.name, "state": z.state, "by": z.by}
                    for z in self._zones.values()
                ],
                "log": list(self._log),
            }

    # -- helpers ------------------------------------------------------------

    def _iter_order(self, dog: str) -> list[str]:
        """The dog's preferred zone order, then any remaining zones."""
        pref = self._preferences.get(dog, [])
        seen = set(pref)
        return [z for z in pref if z in self._zones] + [
            z for z in self._order if z not in seen
        ]
