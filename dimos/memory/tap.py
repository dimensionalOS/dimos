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

"""``--record``: subscribe to every topic on the transport bus and write one store.

Runs in the ``dimos run`` process, like the Rerun bridge: one ``memory.db`` per run
at ``recordings/<run-id>/``. Streams are named by topic (``/lidar`` -> ``lidar``).
Poses are not resolved here; ``tf`` is recorded like any other topic and
``dimos map pose-fill`` derives poses on read.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
import fnmatch
import os
from pathlib import Path
import re
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.constants import RECORDINGS_DIR
from dimos.core.global_config import global_config
from dimos.core.transport_factory import pubsub_backend
from dimos.memory.store.sqlite import SqliteStore
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.stream import Stream
    from dimos.msgs.protocol import DimosMsg

logger = setup_logger()


def recording_dir() -> Path:
    """``recordings/<run-id>`` for this ``dimos run``; a timestamp when run outside it."""
    run_id = os.environ.get("DIMOS_RUN_ID") or time.strftime("%Y%m%d-%H%M%S")
    return RECORDINGS_DIR / run_id


class BusRecorder:
    """Appends every decoded bus message to ``store``, one stream per topic."""

    def __init__(self, store: SqliteStore, topics: str = "*") -> None:
        self.store = store
        self._globs = topics.split(",")
        self._lock = threading.Lock()
        # topic -> stream, or None when filtered out / not a dimos message type.
        self._streams: dict[str, Stream[DimosMsg] | None] = {}

    def _stream(self, topic: Any) -> Stream[DimosMsg] | None:
        key = topic.topic
        if key in self._streams:
            return self._streams[key]
        name = re.sub(r"\W", "_", key.strip("/"))
        wanted = any(fnmatch.fnmatch(name, g) for g in self._globs)
        if not wanted or topic.lcm_type is None:
            self._streams[key] = None
            return None
        self._streams[key] = self.store.stream(name, topic.lcm_type)
        logger.info("Recording %s -> %s (%s)", key, name, topic.lcm_type.__name__)
        return self._streams[key]

    def on_message(self, msg: Any, topic: Any) -> None:
        with self._lock:
            stream = self._stream(topic)
            if stream is not None:
                stream.append(msg, ts=getattr(msg, "ts", None) or time.time())


@contextmanager
def recording() -> Iterator[None]:
    """Record the bus for the duration of the block when ``--record`` is set."""
    if not global_config.record or global_config.replay:
        yield
        return
    path = recording_dir() / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    bus = pubsub_backend()
    if hasattr(bus, "start"):
        bus.start()
    unsubscribe: Callable[[], None] = bus.subscribe_all(
        BusRecorder(store, global_config.record_topics).on_message
    )
    logger.info("Recording to %s", path)
    try:
        yield
    finally:
        unsubscribe()
        if hasattr(bus, "stop"):
            bus.stop()
        store.stop()
