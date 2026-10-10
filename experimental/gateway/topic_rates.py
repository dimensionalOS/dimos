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

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
import re
import threading
import time
from typing import Any

WINDOW_S = 2.0
LIVELINESS_EVERY_S = 5.0
KIND = re.compile(r"[A-Za-z0-9_]+\.[A-Za-z0-9_]+")


def parse_key(key: str) -> tuple[str, str] | None:
    parts = key.split("/")
    if len(parts) < 3 or parts[0] != "dimos" or "*" in key or "/@" in key or parts[1] == "rpc":
        return None
    if not KIND.fullmatch(parts[-1]):
        return None
    return "/" + "/".join(parts[1:-1]), parts[-1]


@dataclass
class Topic:
    topic: str
    type: str
    messages: int = 0
    bytes: int = 0
    last: float | None = None
    declared: bool = False
    window: deque[tuple[float, int]] = field(default_factory=deque)


class TopicWatch:
    def __init__(
        self, open_session: Callable[[], Any], clock: Callable[[], float] = time.monotonic
    ) -> None:
        self.open_session = open_session
        self.clock = clock
        self.topics: dict[tuple[str, str], Topic] = {}
        self.lock = threading.Lock()
        self.session: Any = None
        self.subscriber: Any = None
        self.liveliness_at = 0.0
        self.error: str | None = None

    def heard(self, key: str, size: int) -> None:
        parsed = parse_key(key)
        if parsed is None:
            return
        now = self.clock()
        with self.lock:
            entry = self.topics.setdefault(parsed, Topic(*parsed))
            entry.messages += 1
            entry.bytes += size
            entry.last = now
            entry.window.append((now, size))

    def declared(self, key: str) -> None:
        parsed = parse_key(key.split("/@adv/", 1)[0])
        if parsed is not None:
            with self.lock:
                self.topics.setdefault(parsed, Topic(*parsed)).declared = True

    def start(self) -> None:
        if self.subscriber is not None:
            return
        try:
            self.session = self.session or self.open_session()
            self.subscriber = self.session.declare_subscriber(
                "dimos/**", lambda sample: self.heard(str(sample.key_expr), len(sample.payload))
            )
            self.error = None
        except Exception as error:
            self.error = f"can't listen to the bus: {error}"

    def _ask_liveliness(self) -> None:
        if self.session is None:
            return
        try:
            for reply in self.session.liveliness().get("dimos/**/@adv/pub/**", timeout=0.3):
                ok = getattr(reply, "ok", None)
                if ok is not None:
                    self.declared(str(ok.key_expr))
        except Exception:
            pass

    def snapshot(self) -> dict[str, Any]:
        now = self.clock()
        self.start()
        if now - self.liveliness_at > LIVELINESS_EVERY_S:
            self.liveliness_at = now
            self._ask_liveliness()
        rows: list[dict[str, Any]] = []
        with self.lock:
            for entry in self.topics.values():
                while entry.window and now - entry.window[0][0] > WINDOW_S:
                    entry.window.popleft()
                count = len(entry.window)
                rows.append(
                    {
                        "topic": entry.topic,
                        "type": entry.type,
                        "hz": round(count / WINDOW_S, 2),
                        "bps": round(sum(size for _, size in entry.window) / WINDOW_S, 1),
                        "messages": entry.messages,
                        "lastSeen": None if entry.last is None else round(now - entry.last, 1),
                        "declared": entry.declared,
                    }
                )
        rows.sort(key=lambda row: (-float(row["hz"]), str(row["topic"])))
        return {"up": self.error is None, "error": self.error, "topics": rows}
