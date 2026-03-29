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

"""RecordReplay — record and replay pub/sub topics using memory2 stores.

Usage::

    from dimos.record import RecordReplay
    from dimos.protocol.pubsub.impl.lcmpubsub import LCM

    # Record from live LCM traffic
    rec = RecordReplay(Path("my_recording.db"))
    rec.start_recording([LCM()])
    # ... robot runs ...
    rec.stop_recording()

    # Replay into LCM (viewable via rerun-bridge)
    rec.play(pubsub=LCM(), speed=1.0)

    # Timeline editing
    rec.trim(start=2.0, end=30.0)
    rec.delete_range(start=10.0, end=12.0)

    # Query/filter via memory2 streams
    rec.stream("lidar").time_range(0, 5).count()
"""

import asyncio
from collections.abc import Callable, Collection, Container
from contextlib import suppress
import heapq
import logging
import math
import re
import sys
import time
from typing import Any, TypedDict

from dimos.memory2.codecs.base import _resolve_payload_type
from dimos.memory2.store.sqlite import SqliteStore
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic
from dimos.protocol.pubsub.spec import PubSub

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing import Any as Self

logger = logging.getLogger(__name__)

_SANITIZE_RE = re.compile(r"[^A-Za-z0-9_]")


def topic_to_stream_name(channel: str) -> str:
    """Convert a raw LCM channel/topic pattern to a safe stream name."""
    name = channel.split("#")[0].lstrip("/")
    name = _SANITIZE_RE.sub("_", name)
    if name and name[0].isdigit():
        name = f"_{name}"
    return name or "_unknown"


class StreamInfo(TypedDict):
    name: str
    count: int
    start: float
    end: float
    duration: float
    type: str


class RecordReplay:
    """Record and replay pub/sub topics using memory2 stores.

    A recording is a single SQLite file containing one stream per topic.
    Supports recording from any ``SubscribeAllCapable`` pubsub, playback
    with speed control, seeking, and timeline editing (trim/delete).

    The underlying :class:`SqliteStore` is fully accessible for advanced
    queries via :attr:`store`, :attr:`streams`, and :meth:`stream`.
    """

    def __init__(self, path: str) -> None:
        self._store = SqliteStore(path=path)

        self._recording = False
        self._unsubscribes: list[Callable[[], None]] = []
        self._topic_filter: Container[str] | None = None

        self._resume = asyncio.Event()
        self._play_task: asyncio.Task | None = None
        self._play_speed = 1.0
        self._position = 0.0
        self._pubsub = None

    @property
    def store(self) -> SqliteStore:
        """The underlying store."""
        return self._store

    @property
    def path(self) -> str:
        """Path to the recording file."""
        return self._store.config.path

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def is_playing(self) -> bool:
        return self._play_task is not None and not self._play_task.done()

    @property
    def is_paused(self) -> bool:
        return self.is_playing and not self._resume.is_set()

    def start_recording(
        self,
        pubsubs: Collection[LCMPubSubBase],
        topic_filter: Container[str] | None = None,
    ) -> None:
        """Start recording messages from the given pubsubs.

        Each pubsub is subscribed via ``subscribe_all()``. Messages are
        stored in per-topic streams with automatic codec selection.

        Args:
            pubsubs: List of pubsubs to subscribe to.
            topic_filter: If provided, only record topics whose sanitized
                stream name is in this set. If ``None``, record everything.
        """
        if self._recording:
            raise RuntimeError("Already recording")
        self._recording = True
        self._topic_filter = topic_filter

        for pubsub in pubsubs:
            pubsub.start()
            unsub = pubsub.subscribe_all(self._on_message)
            self._unsubscribes.append(unsub)

        logger.info("Recording started on %d pubsub(s)", len(pubsubs))

    def stop_recording(self) -> None:
        """Stop recording."""
        if not self._recording:
            return
        self._recording = False
        for unsub in self._unsubscribes:
            unsub()
        self._unsubscribes.clear()
        logger.info("Recording stopped")

    def _on_message(self, msg: bytes, topic: Topic) -> None:
        """Handle incoming message during recording."""
        stream_name = topic_to_stream_name(topic.pattern)

        if self._topic_filter is not None and stream_name not in self._topic_filter:
            return

        msg_type = type(msg)

        s = self._store.stream(stream_name, msg_type)
        s.append(msg, ts=time.time())

    @property
    def duration(self) -> float:
        """Total duration of the recording in seconds."""
        t_min, t_max = self.time_range
        return t_max - t_min

    @property
    def time_range(self) -> tuple[float, float]:
        """Absolute (min_ts, max_ts) across all streams."""
        streams = self._store.list_streams()
        if not streams:
            return (0.0, 0.0)
        t_min = math.inf
        t_max = -math.inf
        for name in streams:
            s = self._store.stream(name)
            if s.exists():
                t0, t1 = s.get_time_range()
                t_min = min(t_min, t0)
                t_max = max(t_max, t1)
        if t_min is math.inf:
            return (0.0, 0.0)
        return (t_min, t_max)

    @property
    def position(self) -> float:
        """Current playback position in seconds from recording start."""
        return self._position

    def stream_info(self) -> tuple[StreamInfo, ...]:
        """Return per-stream metadata: name, count, time range, type."""
        result = []
        for name in self._store.list_streams():
            s = self._store.stream(name)
            info: StreamInfo = {"name": name, "count": s.count()}
            if info["count"] > 0:
                t0, t1 = s.get_time_range()
                info["start"] = t0
                info["end"] = t1
                info["duration"] = t1 - t0
            # Get payload type from registry
            reg = self._store._registry.get(name)
            if reg:
                info["type"] = reg.get("payload_module", "unknown")
            result.append(info)
        return tuple(result)

    def play(
        self,
        pubsub: PubSub[Any, Any] | None = None,
        speed: float = 1.0,
    ) -> None:
        """Start playback in a background thread.

        Args:
            pubsub: If provided, messages are published to this pubsub.
                    Use LCM() to make them visible to rerun-bridge.
            speed: Playback speed multiplier (1.0 = realtime, 2.0 = 2x, etc).
        """
        if self.is_playing:
            raise RuntimeError("Already playing")

        self._play_speed = speed
        self._pubsub = pubsub
        # Set resume so playback starts, this is cleared to pause playback.
        self._resume.set()
        self._play_task = asyncio.create_task(self._playback_loop(pubsub))

    async def _playback_loop(self, pubsub: PubSub[Any, Any] | None) -> None:
        """Main playback loop — merges all streams by timestamp."""
        t_min, t_max = self.time_range
        if t_min >= t_max:
            return

        # Build iterators for each stream, starting from seek position
        start_ts = t_min + self._position
        heap: list[tuple[float, int, str, Any]] = []
        counter = 0  # tiebreaker for heapq

        for name in self._store.list_streams():
            s = self._store.stream(name)
            it = iter(s.after(start_ts - 0.001))
            try:
                obs = next(it)
            except StopIteration:
                continue
            heapq.heappush(heap, (obs.ts, counter, name, (obs, it)))
            counter += 1

        if not heap:
            return

        topic_map = {} if pubsub is None else self._build_topic_map(pubsub)
        wall_start = time.monotonic()
        rec_start = heap[0][0]  # earliest observation timestamp

        while heap:
            if not self._resume.is_set():
                pause_start = time.monotonic()
                await self._resume.wait()
                wall_start += time.monotonic() - pause_start

            ts, _, stream_name, (obs, it) = heapq.heappop(heap)

            # Wait for correct playback time
            elapsed_rec = ts - rec_start
            target_wall = wall_start + (elapsed_rec / self._play_speed)
            sleep_time = target_wall - time.monotonic()
            await asyncio.sleep(sleep_time)

            self._position = ts - t_min
            if pubsub is not None and stream_name in topic_map:
                pubsub.publish(topic_map[stream_name], obs.data)

            try:
                next_obs = next(it)
            except StopIteration:
                continue
            heapq.heappush(heap, (next_obs.ts, counter, stream_name, (next_obs, it)))
            counter += 1

    def _build_topic_map(self, pubsub: PubSub[Any, Any]) -> dict[str, Topic]:
        """Build stream_name -> Topic mapping for publishing."""
        topic_map: dict[str, Topic] = {}

        for name in self._store.list_streams():
            reg = self._store._registry.get(name)
            if reg is None:
                continue

            payload_module = reg.get("payload_module")
            lcm_type = None
            if payload_module:
                lcm_type = _resolve_payload_type(payload_module)

            topic_map[name] = Topic(f"/{name}", lcm_type=lcm_type)

        return topic_map

    def pause(self) -> None:
        """Pause playback. Resume with :meth:`resume`."""
        self._resume.clear()

    def resume(self) -> None:
        """Resume paused playback."""
        self._resume.set()

    async def stop_playback(self) -> None:
        """Stop playback."""
        self._resume.set()
        if self._play_task is not None:
            self._play_task.cancel()
            with suppress(asyncio.CancelledError):
                await self._play_task
            self._play_task = None

    def seek(self, seconds: float) -> None:
        """Set playback position in seconds from recording start.

        Takes effect immediately if playing (restarts playback loop).
        If not playing, sets the position for the next :meth:`play`.
        """
        self._position = max(0.0, min(seconds, self.duration))
        if self.is_playing:
            self.stop_playback()
            assert self._pubsub is not None
            self.play(pubsub=self._pubsub, speed=self._play_speed)

    def delete_range(self, start: float, end: float) -> int:
        """Delete observations in [start, end] seconds from recording start.

        Returns total count of deleted observations across all streams.
        """
        t_min, _ = self.time_range
        abs_start = t_min + start
        abs_end = t_min + end

        total = 0
        for name in self._store.list_streams():
            s = self._store.stream(name)
            total += s.delete_range(abs_start, abs_end)
        return total

    def trim(self, start: float, end: float) -> int:
        """Keep only [start, end] seconds, delete everything else.

        Returns total count of deleted observations.
        """
        t_min, t_max = self.time_range
        total = 0
        if start > 0:
            total += self.delete_range(0, start - 0.0001)
        if end < (t_max - t_min):
            total += self.delete_range(end + 0.0001, t_max - t_min + 1)
        return total

    async def close(self) -> None:
        """Stop recording/playback and close the store."""
        self.stop_recording()
        await self.stop_playback()
        self._store.stop()

    async def __aenter__(self) -> Self:
        return self

    async def __aexit__(self, *exc: Any) -> None:
        await self.close()

    def __repr__(self) -> str:
        streams = self._store.list_streams()
        dur = self.duration
        return f"RecordReplay({self._store.config.path!r}, streams={len(streams)}, duration={dur:.1f}s)"
