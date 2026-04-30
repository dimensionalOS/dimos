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

"""Thread-safe typed state for temporal memory."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
import threading
from typing import Any


@dataclass
class TemporalState:
    """Typed, thread-safe state container for temporal memory.

    All public mutators acquire ``_lock``.  Callers that need a consistent
    snapshot should use :meth:`snapshot` which returns a deep-copy under
    the lock.
    """

    entity_roster: list[dict[str, Any]] = field(default_factory=list)
    rolling_summary: str = ""
    chunk_buffer: list[dict[str, Any]] = field(default_factory=list)
    next_summary_at_s: float = 0.0
    last_present: list[dict[str, Any]] = field(default_factory=list)

    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False, compare=False)

    def snapshot(self) -> TemporalState:
        """Return a deep-copy snapshot (safe to read outside the lock)."""
        with self._lock:
            return TemporalState(
                entity_roster=copy.deepcopy(self.entity_roster),
                rolling_summary=self.rolling_summary,
                chunk_buffer=copy.deepcopy(self.chunk_buffer),
                next_summary_at_s=self.next_summary_at_s,
                last_present=copy.deepcopy(self.last_present),
            )

    def to_dict(self) -> dict[str, Any]:
        """Convert to plain dict (for prompt building compatibility)."""
        with self._lock:
            return {
                "entity_roster": copy.deepcopy(self.entity_roster),
                "rolling_summary": self.rolling_summary,
                "chunk_buffer": copy.deepcopy(self.chunk_buffer),
                "next_summary_at_s": self.next_summary_at_s,
                "last_present": copy.deepcopy(self.last_present),
            }

    def update_from_window(
        self,
        parsed: dict[str, Any],
        w_end: float,
        summary_interval_s: float,
    ) -> bool:
        """Update state from parsed window result. Returns True if summary needed.

        Thread-safe: acquires ``_lock`` internally.
        """
        if "_error" in parsed:
            return False

        with self._lock:
            return self._update_from_window_unlocked(parsed, w_end, summary_interval_s)

    def _update_from_window_unlocked(
        self,
        parsed: dict[str, Any],
        w_end: float,
        summary_interval_s: float,
    ) -> bool:
        new_entities = parsed.get("new_entities", [])
        present = parsed.get("entities_present", [])

        # Handle new entities
        if new_entities:
            known = {e.get("id") for e in self.entity_roster if isinstance(e, dict)}
            for e in new_entities:
                if isinstance(e, dict) and e.get("id") not in known:
                    self.entity_roster.append(e)
                    known.add(e.get("id"))

        # Auto-add referenced entities not yet in roster
        known = {e.get("id") for e in self.entity_roster if isinstance(e, dict)}
        referenced: set[str] = set()
        for p in present or []:
            if isinstance(p, dict) and isinstance(p.get("id"), str):
                referenced.add(p["id"])
        for rel in parsed.get("relations") or []:
            if isinstance(rel, dict):
                for k in ("subject", "object"):
                    v = rel.get(k)
                    if isinstance(v, str) and v != "unknown":
                        referenced.add(v)
        for rid in sorted(referenced):
            if rid not in known:
                self.entity_roster.append(
                    {
                        "id": rid,
                        "type": "other",
                        "descriptor": "unknown (auto-added; rerun recommended)",
                    }
                )
                known.add(rid)

        self.last_present = present

        # Add to chunk buffer
        self.chunk_buffer.append(
            {
                "window": parsed.get("window"),
                "caption": parsed.get("caption", ""),
                "entities_present": parsed.get("entities_present", []),
                "new_entities": parsed.get("new_entities", []),
                "relations": parsed.get("relations", []),
                "on_screen_text": parsed.get("on_screen_text", []),
            }
        )

        # Check if summary update is needed
        if summary_interval_s > 0:
            if w_end + 1e-6 >= self.next_summary_at_s and self.chunk_buffer:
                return True

        return False

    def apply_summary(
        self,
        summary_text: str,
        w_end: float,
        summary_interval_s: float,
    ) -> None:
        """Apply a rolling-summary update."""
        with self._lock:
            if summary_text and summary_text.strip():
                self.rolling_summary = summary_text.strip()
            self.chunk_buffer = []
            while self.next_summary_at_s <= w_end + 1e-6:
                self.next_summary_at_s += summary_interval_s

    def clear(self, summary_interval_s: float = 0.0) -> None:
        """Reset to default state."""
        with self._lock:
            self.entity_roster = []
            self.rolling_summary = ""
            self.chunk_buffer = []
            self.next_summary_at_s = summary_interval_s
            self.last_present = []
