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

"""``--record`` stream selection and recording-engine lifecycle.

The stable Python SQLite engine runs in the ``dimos run`` process and taps each
selected transport directly. The experimental Rust engine is owned as a
standalone subprocess. Both name artifact streams by their blueprint stream
name. Poses are not resolved here; ``tf`` is recorded like any other stream and
``dimos map pose-fill`` derives poses on read.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Iterator, Mapping
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
    from dimos.core.stream import Transport
    from dimos.memory.stream import Stream

logger = setup_logger()


def recording_dir() -> Path:
    """``recordings/<run-id>`` for this ``dimos run``; a timestamp when run outside it."""
    run_id = os.environ.get("DIMOS_RUN_ID") or time.strftime("%Y%m%d-%H%M%S")
    return RECORDINGS_DIR / run_id


def _globs(topics: str) -> list[str]:
    return [g.strip().strip("/") for g in topics.split(",")]


def matching(topics: str, names: Iterable[str]) -> set[str]:
    """Stream *names* selected by the comma-separated ``--record-topics`` globs."""
    globs = _globs(topics)
    return {n for n in names if any(fnmatch.fnmatch(n, g) for g in globs)}


def check_topics(topics: str, names: Iterable[str]) -> None:
    """Raise ``ValueError`` when *topics* selects none of *names*."""
    names = sorted(names)
    if not matching(topics, names):
        raise ValueError(f"--record-topics {topics!r} matched none of: {', '.join(names)}")


class TransportRecorder:
    """Appends every message seen on a tapped transport to ``store``, one stream per name.

    Transport callbacks only enqueue; one writer thread does the encoding and SQLite
    work, so a slow append never stalls a transport's handler thread. When the queue
    is full the message is dropped and counted.
    """

    def __init__(self, store: SqliteStore, topics: str = "*", queue_size: int = 1000) -> None:
        self.store = store
        self._topics = topics
        self._queue: queue.Queue[tuple[Stream[Any], Any, float] | None] = queue.Queue(queue_size)
        self.dropped = 0
        self._writer = threading.Thread(target=self._drain, name="record-writer", daemon=True)
        self._writer.start()

    def _drain(self) -> None:
        while (item := self._queue.get()) is not None:
            stream, msg, ts = item
            try:
                stream.append(msg, ts=ts)
            except Exception:
                logger.exception("--record: failed to append to %s", stream)

    def close(self) -> None:
        """Flush queued messages and stop the writer."""
        self._queue.put(None)
        self._writer.join()
        if self.dropped:
            logger.warning("--record: dropped %d messages (writer queue full)", self.dropped)

    def tap(
        self, name: str, stream_type: type, transport: Transport[Any]
    ) -> Callable[[], None] | None:
        """Subscribe *transport* and record into stream *name*; returns the unsubscribe."""
        if not matching(self._topics, [name]):
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
                if self.dropped % 1000 == 1:
                    logger.warning("--record: writer queue full, %d dropped so far", self.dropped)

        logger.info("Recording %s (%s) via %s", name, stream_type.__name__, transport)
        return transport.subscribe(on_msg)


class RecordingSession(Protocol):
    """Lifecycle signal shared by the Python and experimental Rust engines."""

    failure_event: threading.Event

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def raise_if_failed(self) -> None: ...


class _InactiveRecordingSession:
    failure_event = threading.Event()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def raise_if_failed(self) -> None:
        pass


class _PythonRecordingSession:
    """Own one stable in-process SQLite recorder."""

    def __init__(self, transports: Mapping[tuple[str, type], Transport[Any]]) -> None:
        self.failure_event = threading.Event()
        self._transports = transports
        self._store = SqliteStore(path=str(recording_dir() / "memory.db"))
        self._recorder: TransportRecorder | None = None
        self._unsubscribes: list[Callable[[], None]] = []
        self._stopped = False

    def start(self) -> None:
        self._store.start()
        self._recorder = TransportRecorder(self._store, global_config.record_topics)
        self._unsubscribes = [
            unsubscribe
            for (name, payload_type), transport in self._transports.items()
            if (unsubscribe := self._recorder.tap(name, payload_type, transport)) is not None
        ]
        logger.info("Recording to %s", recording_dir() / "memory.db")

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        for unsubscribe in self._unsubscribes:
            unsubscribe()
        self._unsubscribes.clear()
        if self._recorder is not None:
            self._recorder.close()
            self._recorder = None
        self._store.stop()

    def raise_if_failed(self) -> None:
        pass


@contextmanager
def recording(
    transports: Mapping[tuple[str, type], Transport[Any]],
) -> Iterator[RecordingSession]:
    """Record every stream in *transports* for the duration of the block when ``--record`` is set."""
    if not global_config.record or global_config.replay:
        yield _InactiveRecordingSession()
        return
    check_topics(global_config.record_topics, {n for n, _ in transports})
    if global_config.record_engine == "rust":
        from dimos.experimental.memory.rust_cli_recorder import (
            RustRecordingSession,
            make_plan,
        )

        session: RecordingSession = RustRecordingSession(make_plan(dict(transports)))
    else:
        if global_config.record != "sqlite":
            raise ValueError("MCAP recording requires --record mcap --record-engine rust")
        session = _PythonRecordingSession(transports)

    try:
        session.start()
        yield session
    finally:
        session.stop()
