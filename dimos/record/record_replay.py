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
from collections.abc import Callable, Collection
from contextlib import suppress
import heapq
import logging
import math
import re
import sys
import time
from typing import Any, NotRequired, TypedDict

import rerun as rr

from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service.spec import Service
from dimos.types.timestamped import Timestamped
from dimos.visualization.rerun.bridge import RerunConvertible, is_rerun_multi

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
    start: NotRequired[float]
    end: NotRequired[float]
    duration: NotRequired[float]
    type: NotRequired[str]


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
        self._ts_offsets: dict[str, float] = {}

        self._resume = asyncio.Event()
        self._play_task: asyncio.Task[None] | None = None
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
        pubsubs: Collection[PubSub[Topic, object]],
        topics: Collection[Topic],
    ) -> None:
        """Start recording messages from the given pubsubs."""
        if self._recording:
            raise RuntimeError("Already recording")
        self._recording = True

        for pubsub in pubsubs:
            if isinstance(pubsub, Service):
                pubsub.start()
            for topic in topics:
                unsub = pubsub.subscribe(topic, self._on_message)
                self._unsubscribes.append(unsub)

        logger.info("Recording started on %d pubsub(s)", len(pubsubs))

    def stop_recording(self) -> None:
        if not self._recording:
            return
        self._recording = False
        for unsub in self._unsubscribes:
            unsub()
        self._unsubscribes.clear()
        self._ts_offsets.clear()
        logger.info("Recording stopped")

    def _on_message(self, msg: object, topic: Topic) -> None:
        stream_name = topic_to_stream_name(topic.pattern)

        ts: float | None = None
        if isinstance(msg, Timestamped) and msg.ts:
            # Record the time offset on first message and then offset all messages
            # by the same amount. This allows recording live channels and replayed
            # channels together with consistent timestamps.
            if stream_name not in self._ts_offsets:
                self._ts_offsets[stream_name] = time.time() - msg.ts
            ts = msg.ts + self._ts_offsets[stream_name]

        s = self._store.stream(stream_name, type(msg))
        s.append(msg, ts=ts)

        reg = self._store._registry.get(stream_name)
        if reg and "channel" not in reg:
            reg["channel"] = str(topic)
            self._store._registry.put(stream_name, reg)

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
            s: Stream[object] = self._store.stream(name)
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
            s: Stream[object] = self._store.stream(name)
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

    def play(self, speed: float = 1.0) -> None:
        """Start playback as a separate Rerun recording.

        Connects to the running Rerun viewer and logs decoded messages
        under a recording called ``"playback"``, so it appears alongside
        (but separate from) any live data.
        """
        if self.is_playing:
            raise RuntimeError("Already playing")

        self._play_speed = speed
        # Set resume so playback starts, this is cleared to pause playback.
        self._resume.set()
        self._play_task = asyncio.create_task(self._playback_loop())

    async def _playback_loop(self) -> None:
        t_min, t_max = self.time_range
        if t_min >= t_max:
            return

        # Separate Rerun recording so playback appears as its own source
        rec = rr.RecordingStream("playback", make_default=False)
        rec.connect_grpc()

        # Build topic map for decoding raw bytes -> DimosMsg
        topic_map: dict[str, Topic] = {}
        for name in self._store.list_streams():
            reg = self._store._registry.get(name)
            if reg:
                channel = reg.get("channel")
                if channel:
                    topic_map[name] = Topic.from_channel_str(channel)

        # Merge-sort all streams by timestamp
        start_ts = t_min + self._position
        heap: list[tuple[float, int, str, Any]] = []
        counter = 0  # tiebreaker for heapq

        for name in self._store.list_streams():
            s: Stream[object] = self._store.stream(name)
            it = iter(s.after(start_ts - 0.001))
            try:
                obs = next(it)
            except StopIteration:
                continue
            heapq.heappush(heap, (obs.ts, counter, name, (obs, it)))
            counter += 1

        if not heap:
            return

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
            await asyncio.sleep(target_wall - time.monotonic())

            self._position = ts - t_min

            topic = topic_map.get(stream_name)
            if topic is not None and topic.lcm_type is not None:
                msg = (
                    obs.data
                    if not isinstance(obs.data, bytes)
                    else topic.lcm_type.lcm_decode(obs.data)
                )
                if isinstance(msg, RerunConvertible):
                    entity_path = f"world/{stream_name}"
                    rerun_data = msg.to_rerun()
                    if is_rerun_multi(rerun_data):
                        for path, archetype in rerun_data:
                            rec.log(path, archetype)
                    else:
                        rec.log(entity_path, rerun_data)  # type: ignore[arg-type]

            try:
                next_obs = next(it)
            except StopIteration:
                continue
            heapq.heappush(heap, (next_obs.ts, counter, stream_name, (next_obs, it)))
            counter += 1

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

    async def seek(self, seconds: float) -> None:
        """Set playback position in seconds from recording start.

        Takes effect immediately if playing (restarts playback loop).
        If not playing, sets the position for the next :meth:`play`.
        """
        self._position = max(0.0, min(seconds, self.duration))
        if self.is_playing:
            await self.stop_playback()
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
            s: Stream[object] = self._store.stream(name)
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
