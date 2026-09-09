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

"""The agent's event queue: persisted, in order, delivered exactly once."""

import asyncio
from collections.abc import Awaitable, Callable
import json
import time
from typing import Any

from .store import Store

# Nobody polling for this long means the queue is stale; a late wake is worse than none.
STALE_S = 600.0

NONE_EVENT: dict[str, Any] = {"type": "none"}


class EventBus:
    def __init__(self, store: Store) -> None:
        self.store = store
        self._new = asyncio.Event()
        self._lock = asyncio.Lock()
        self.on_delivery: Callable[[dict[str, Any]], None] | None = None
        self.drop_stale()

    def publish(self, event: dict[str, Any]) -> None:
        event = {**event, "ts": event.get("ts", time.time())}
        self.store.write(
            "INSERT INTO events (type, payload, ts) VALUES (?, ?, ?)",
            (event["type"], json.dumps(event), event["ts"]),
        )
        self._new.set()

    def drop_stale(self) -> None:
        """Retire undelivered events nobody picked up in time (also runs at startup)."""
        self.store.write(
            "UPDATE events SET delivered = 1 WHERE delivered = 0 AND ts < ?",
            (time.time() - STALE_S,),
        )

    def _take(self) -> dict[str, Any] | None:
        self.drop_stale()
        # `found` is what the agent asked for, so it beats everything; then chat and enrolled;
        # then a person merely showing up; wakes last. Within a class, oldest first.
        row = self.store.one(
            "SELECT * FROM events WHERE delivered = 0 "
            "ORDER BY CASE type WHEN 'found' THEN 0 WHEN 'seen' THEN 2 "
            "WHEN 'wake' THEN 3 ELSE 1 END, id LIMIT 1"
        )
        if row is None:
            return None
        self.store.write("UPDATE events SET delivered = 1 WHERE id = ?", (row["id"],))
        return json.loads(row["payload"])

    async def poll(
        self, wait_s: float, disconnected: Callable[[], Awaitable[bool]] | None = None
    ) -> dict[str, Any]:
        """Return the next event, or `{"type": "none"}` if none arrives within wait_s."""
        deadline = time.monotonic() + wait_s
        while True:
            async with self._lock:
                event = self._take()
            if event is not None:
                if self.on_delivery:
                    self.on_delivery(event)
                return event
            if disconnected and await disconnected():
                return NONE_EVENT
            left = deadline - time.monotonic()
            if left <= 0:
                return NONE_EVENT
            self._new.clear()
            try:
                await asyncio.wait_for(self._new.wait(), timeout=min(left, 0.5))
            except asyncio.TimeoutError:
                pass


class Waiters:
    """Per-person wakeups for the phone's message long poll."""

    def __init__(self) -> None:
        self._events: dict[str, asyncio.Event] = {}

    def _for(self, person_id: str) -> asyncio.Event:
        return self._events.setdefault(person_id, asyncio.Event())

    def notify(self, person_id: str) -> None:
        ev = self._for(person_id)
        ev.set()
        ev.clear()

    async def wait(self, person_id: str, timeout: float) -> None:
        try:
            await asyncio.wait_for(self._for(person_id).wait(), timeout=timeout)
        except asyncio.TimeoutError:
            pass
