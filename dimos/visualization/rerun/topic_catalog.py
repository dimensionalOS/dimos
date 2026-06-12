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
from dataclasses import asdict, dataclass, field
import threading
import time
from typing import Any, Literal, TypeAlias

from dimos.msgs.protocol import DimosMsg
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.pubsub.patterns import Glob, pattern_matches

Renderability: TypeAlias = Literal["renderable", "unsupported", "unknown"]


@dataclass(slots=True)
class LcmTopicCatalogEntry:
    channel: str
    name: str
    type_name: str | None
    renderability: Renderability
    render_reason: str
    live: bool
    selected: bool
    logging: bool
    last_seen_monotonic: float
    message_count: int
    rate_hz: float
    bandwidth_bps: float
    average_message_size_bytes: float
    last_error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class _MutableTopicStats:
    channel: str
    name: str
    type_name: str | None
    lcm_type: type[DimosMsg] | None
    renderability: Renderability
    render_reason: str
    last_seen_monotonic: float
    message_count: int = 0
    last_error: str | None = None
    history: deque[tuple[float, int]] = field(default_factory=deque)


def normalized_lcm_topic(channel: str | Topic) -> str:
    if isinstance(channel, Topic):
        return str(channel.topic)
    return channel.rsplit("#", 1)[0]


def lcm_channel(channel: str | Topic) -> str:
    return str(channel)


def topic_type_name(topic: Topic) -> str | None:
    if topic.lcm_type is not None:
        return topic.lcm_type.msg_name
    channel = lcm_channel(topic)
    if "#" not in channel:
        return None
    return channel.rsplit("#", 1)[1]


def is_selected_topic(topic: str | Topic, selected_topics: set[str]) -> bool:
    channel = lcm_channel(topic)
    name = normalized_lcm_topic(topic)
    return channel in selected_topics or name in selected_topics


def classify_lcm_topic_renderability(
    topic: Topic,
    *,
    entity_path: str,
    visual_override: dict[Glob | str, Callable[[Any], Any] | None] | None = None,
) -> tuple[Renderability, str]:
    if visual_override:
        matches = [
            fn for pattern, fn in visual_override.items() if pattern_matches(pattern, entity_path)
        ]
        if any(fn is None for fn in matches):
            return "unsupported", "suppressed by visual override"
        if matches:
            return "renderable", "visual converter"

    if topic.lcm_type is None:
        return "unknown", "unknown message type"

    if hasattr(topic.lcm_type, "to_rerun"):
        return "renderable", "native to_rerun()"

    return "unsupported", "no Rerun converter"


class LcmTopicCatalog:
    def __init__(self, *, freshness_window_s: float = 2.0, stats_window_s: float = 5.0) -> None:
        self.freshness_window_s = freshness_window_s
        self.stats_window_s = stats_window_s
        self._entries: dict[str, _MutableTopicStats] = {}
        self._lock = threading.Lock()

    def observe(
        self,
        topic: Topic,
        data: bytes | int,
        *,
        renderability: Renderability,
        render_reason: str,
        now: float | None = None,
    ) -> None:
        timestamp = now if now is not None else time.monotonic()
        channel = lcm_channel(topic)
        name = normalized_lcm_topic(topic)
        type_name = topic_type_name(topic)

        with self._lock:
            entry = self._entries.get(channel)
            if entry is None:
                entry = _MutableTopicStats(
                    channel=channel,
                    name=name,
                    type_name=type_name,
                    lcm_type=topic.lcm_type,
                    renderability=renderability,
                    render_reason=render_reason,
                    last_seen_monotonic=timestamp,
                )
                self._entries[channel] = entry
            else:
                entry.name = name
                entry.type_name = type_name
                entry.lcm_type = topic.lcm_type
                entry.renderability = renderability
                entry.render_reason = render_reason
                entry.last_seen_monotonic = timestamp

            size_bytes = len(data) if isinstance(data, bytes) else data
            entry.message_count += 1
            entry.history.append((timestamp, max(size_bytes, 0)))
            self._drop_old_history(entry, timestamp)

    def record_error(self, topic: str | Topic, error: BaseException | str) -> None:
        channel = lcm_channel(topic)
        message = str(error)
        with self._lock:
            entry = self._entries.get(channel)
            if entry is not None:
                entry.last_error = message

    def snapshot(
        self,
        *,
        staged_topics: set[str] | None = None,
        logging_topics: set[str] | None = None,
        now: float | None = None,
    ) -> list[LcmTopicCatalogEntry]:
        timestamp = now if now is not None else time.monotonic()
        staged = staged_topics or set()
        logging = logging_topics or set()

        with self._lock:
            entries = list(self._entries.values())
            return [self._snapshot_entry(entry, staged, logging, timestamp) for entry in entries]

    def to_dicts(
        self,
        *,
        staged_topics: set[str] | None = None,
        logging_topics: set[str] | None = None,
        now: float | None = None,
    ) -> list[dict[str, Any]]:
        return [
            entry.to_dict()
            for entry in self.snapshot(
                staged_topics=staged_topics,
                logging_topics=logging_topics,
                now=now,
            )
        ]

    def _snapshot_entry(
        self,
        entry: _MutableTopicStats,
        staged_topics: set[str],
        logging_topics: set[str],
        now: float,
    ) -> LcmTopicCatalogEntry:
        self._drop_old_history(entry, now)
        window = max(self.stats_window_s, 0.001)
        bytes_in_window = sum(size for _, size in entry.history)
        count_in_window = len(entry.history)
        return LcmTopicCatalogEntry(
            channel=entry.channel,
            name=entry.name,
            type_name=entry.type_name,
            renderability=entry.renderability,
            render_reason=entry.render_reason,
            live=now - entry.last_seen_monotonic <= self.freshness_window_s,
            selected=entry.channel in staged_topics or entry.name in staged_topics,
            logging=entry.channel in logging_topics or entry.name in logging_topics,
            last_seen_monotonic=entry.last_seen_monotonic,
            message_count=entry.message_count,
            rate_hz=count_in_window / window,
            bandwidth_bps=bytes_in_window / window,
            average_message_size_bytes=(
                bytes_in_window / count_in_window if count_in_window else 0.0
            ),
            last_error=entry.last_error,
        )

    def _drop_old_history(self, entry: _MutableTopicStats, now: float) -> None:
        cutoff = now - self.stats_window_s
        while entry.history and entry.history[0][0] < cutoff:
            entry.history.popleft()
