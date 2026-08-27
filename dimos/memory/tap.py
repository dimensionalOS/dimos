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

"""``--record``: subscribe to every stream transport in the blueprint and write one store.

Runs in the ``dimos run`` process. Each stream is tapped on its own transport (LCM,
Zenoh, SHM, ...), so whatever carries it is what gets recorded. One ``memory.db`` per
run at ``recordings/<run-id>/``, streams named by their blueprint stream name.
Poses are not resolved here; ``tf`` is recorded like any other stream and
``dimos map pose-fill`` derives poses on read.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
import fnmatch
import os
from pathlib import Path
import queue
import threading
import time
from typing import TYPE_CHECKING, Any, Protocol

from dimos.constants import RECORDINGS_DIR
from dimos.core.global_config import global_config
from dimos.memory.store.sqlite import SqliteStore
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.stream import Stream

logger = setup_logger()


class Subscribable(Protocol):
    """What a dimos ``Transport`` offers a tap: ``subscribe(callback) -> unsubscribe``."""

    def subscribe(
        self, callback: Callable[[Any], None], selfstream: Any = None
    ) -> Callable[[], None] | None: ...


def recording_dir() -> Path:
    """``recordings/<run-id>`` for this ``dimos run``; a timestamp when run outside it."""
    run_id = os.environ.get("DIMOS_RUN_ID") or time.strftime("%Y%m%d-%H%M%S")
    return RECORDINGS_DIR / run_id


class TransportRecorder:
    """Appends every message seen on a tapped transport to ``store``, one stream per name.

    Transport callbacks only enqueue; one writer thread does the encoding and SQLite
    work, so a slow append never stalls a transport's handler thread. When the queue
    is full the message is dropped and counted.
    """

    def __init__(self, store: SqliteStore, topics: str = "*", queue_size: int = 1000) -> None:
        self.store = store
        self._globs = topics.split(",")
        self._queue: queue.Queue[tuple[Stream[Any], Any, float] | None] = queue.Queue(queue_size)
        self.dropped = 0
        self._writer = threading.Thread(target=self._drain, name="record-writer", daemon=True)
        self._writer.start()

    def _drain(self) -> None:
        while (item := self._queue.get()) is not None:
            stream, msg, ts = item
            stream.append(msg, ts=ts)

    def close(self) -> None:
        """Flush queued messages and stop the writer."""
        self._queue.put(None)
        self._writer.join()
        if self.dropped:
            logger.warning("--record: dropped %d messages (writer queue full)", self.dropped)

    def tap(
        self, name: str, stream_type: type, transport: Subscribable
    ) -> Callable[[], None] | None:
        """Subscribe *transport* and record into stream *name*; returns the unsubscribe."""
        if not any(fnmatch.fnmatch(name, g) for g in self._globs):
            return None
        if not hasattr(stream_type, "lcm_encode"):
            logger.info("--record: %s (%s) is not a dimos message type, skipped", name, stream_type)
            return None
        stream: Stream[Any] = self.store.stream(name, stream_type)

        def on_msg(msg: Any) -> None:
            try:
                self._queue.put_nowait((stream, msg, getattr(msg, "ts", None) or time.time()))
            except queue.Full:
                self.dropped += 1

        logger.info("Recording %s (%s) via %s", name, stream_type.__name__, transport)
        return transport.subscribe(on_msg)


@contextmanager
def recording(transports: Mapping[tuple[str, type], Subscribable]) -> Iterator[None]:
    """Record every stream in *transports* for the duration of the block when ``--record`` is set."""
    if not global_config.record or global_config.replay:
        yield
        return
    path = recording_dir() / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    recorder = TransportRecorder(store, global_config.record_topics)
    unsubscribes = [
        u for (name, t), tr in transports.items() if (u := recorder.tap(name, t, tr)) is not None
    ]
    logger.info("Recording to %s", path)
    try:
        yield
    finally:
        for unsubscribe in unsubscribes:
            unsubscribe()
        recorder.close()
        store.stop()
