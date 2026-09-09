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

"""Who is where, right now.

The watcher (`watch.py`) posts a sighting per person per second; this keeps the latest one and
answers "who is in front of FRANK". It is deliberately in-memory: a sighting is only interesting
while it is fresh, and the world state must never turn into history the agent has to page through.

Also holds the agent's watch list — people it is actively looking for — and decides which sightings
deserve to become `found` / `seen` events.
"""

from collections.abc import Callable
import threading
import time
from typing import Any

# A person is "in view" this long after their last sighting, unless the watcher says otherwise.
IN_VIEW_TTL_S = 5.0
# Watches are a short-term intent ("I am looking for Alice right now"), not a standing order.
WATCH_TTL_S = 600.0
# Default gap before a person coming into view is worth waking the agent for.
SEEN_GAP_S = 600.0


class World:
    def __init__(self, on_sighting: Callable[[str, dict[str, Any]], None] | None = None) -> None:
        self._lock = threading.Lock()
        self._people: dict[str, dict[str, Any]] = {}
        self._watches: dict[str, float] = {}
        # write-through hook, so the last sighting outlives the process. Watches are not persisted:
        # "I am looking for Alice right now" does not survive a restart, and should not.
        self._on_sighting = on_sighting

    def load(self, rows: dict[str, dict[str, Any]]) -> None:
        """Seed from storage at startup. Does not fire the write-through hook."""
        with self._lock:
            # Persist identity/last-seen time, never coordinates from a previous map.
            self._people = {
                pid: {"last_seen_ts": row.get("last_seen_ts"), "in_view": False}
                for pid, row in rows.items()
            }

    # --- sightings ------------------------------------------------------

    def record(self, person_id: str, fields: dict[str, Any], now: float | None = None) -> None:
        """Store the latest sighting. `fields` carries whatever the watcher measured."""
        now = now or time.time()
        with self._lock:
            row = self._people.setdefault(person_id, {})
            row.update({k: v for k, v in fields.items() if v is not None})
            row["in_view"] = bool(fields.get("in_view", True))
            if row["in_view"]:
                row["last_seen_ts"] = now
            saved = dict(row)
        if self._on_sighting:
            self._on_sighting(person_id, saved)

    def _in_view(self, row: dict[str, Any], now: float) -> bool:
        if not row.get("in_view"):
            return False
        return now - float(row.get("last_seen_ts") or 0.0) <= IN_VIEW_TTL_S

    def rows(self, now: float | None = None) -> dict[str, dict[str, Any]]:
        """A copy of the sighting state, with `in_view` aged out."""
        now = now or time.time()
        with self._lock:
            return {
                pid: {**row, "in_view": self._in_view(row, now)}
                for pid, row in self._people.items()
            }

    # --- watches --------------------------------------------------------

    def watch(self, person_id: str, now: float | None = None) -> float:
        now = now or time.time()
        with self._lock:
            self._watches[person_id] = now + WATCH_TTL_S
        return now + WATCH_TTL_S

    def unwatch(self, person_id: str) -> bool:
        with self._lock:
            return self._watches.pop(person_id, None) is not None

    def watches(self, now: float | None = None) -> dict[str, float]:
        now = now or time.time()
        with self._lock:
            self._watches = {p: t for p, t in self._watches.items() if t > now}
            return dict(self._watches)

    def is_watched(self, person_id: str, now: float | None = None) -> bool:
        return person_id in self.watches(now)

    # --- events ---------------------------------------------------------

    def event_for(
        self, person_id: str, seen_gap_s: float = SEEN_GAP_S, now: float | None = None
    ) -> str | None:
        """`found`, `seen`, or nothing, for a sighting that is about to be recorded.

        Call this *before* `record()`, so the stored `last_seen_ts` is still the previous one.
        `found` wins and clears the watch: the agent asked for this person and is about to be told.
        `seen` only fires when the person has been away long enough to be news.
        """
        now = now or time.time()
        if self.is_watched(person_id, now):
            self.unwatch(person_id)
            return "found"
        with self._lock:
            row = self._people.get(person_id) or {}
            last = float(row.get("last_seen_ts") or 0.0)
        return "seen" if now - last >= seen_gap_s else None

    def forget(self, person_id: str) -> None:
        with self._lock:
            self._people.pop(person_id, None)
            self._watches.pop(person_id, None)


def snapshot(
    world: World, people: list[dict[str, Any]], now: float | None = None
) -> dict[str, Any]:
    """`GET /agent/world`: one row per enrolled person, most recently seen first."""
    now = now or time.time()
    rows = world.rows(now)
    out = []
    for p in people:
        pid = p["person_id"]
        s = rows.get(pid, {})
        seen = s.get("last_seen_ts") or p.get("last_seen_ts")
        in_view = bool(s.get("in_view"))
        out.append(
            {
                "person_id": pid,
                "name": p.get("name"),
                "in_view": in_view,
                "last_seen_ts": seen,
                "x": s.get("x") if now - float(seen or 0) <= 30 else None,
                "y": s.get("y") if now - float(seen or 0) <= 30 else None,
                "bearing_deg": s.get("bearing_deg") if in_view else None,
                "range_m": s.get("range_m") if in_view else None,
                "last_chat_ts": p.get("last_chat_ts"),
            }
        )
    out.sort(key=lambda r: (r["in_view"], r["last_seen_ts"] or 0.0), reverse=True)
    return {"as_of": now, "people": out}
