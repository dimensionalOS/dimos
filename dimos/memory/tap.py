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

"""``--record``: every ``Out.publish`` in this process is appended to a memory store.

One store per process: ``recordings/<run-id>/memory-<pid>.db``. Streams are named by
topic (``/lidar`` -> ``lidar``). Poses are not resolved here; ``tf`` is recorded like
any other topic and ``dimos map pose-fill`` derives poses on read.
"""

from __future__ import annotations

import fnmatch
import os
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.constants import RECORDINGS_DIR
from dimos.core.global_config import global_config

if TYPE_CHECKING:
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.memory.stream import Stream

_lock = threading.Lock()
_store: SqliteStore | None = None
_streams: dict[str, Stream[Any] | None] = {}


def recording_dir() -> Path:
    """``recordings/<run-id>`` for this ``dimos run``; a timestamp when run outside it."""
    run_id = os.environ.get("DIMOS_RUN_ID") or time.strftime("%Y%m%d-%H%M%S")
    return RECORDINGS_DIR / run_id


def _stream(topic: str, payload_type: type) -> Stream[Any] | None:
    global _store
    if topic in _streams:
        return _streams[topic]
    name = topic.strip("/").replace("/", "_")
    if not any(fnmatch.fnmatch(name, p) for p in global_config.record_topics.split(",")):
        _streams[topic] = None
        return None
    if _store is None:
        from dimos.memory.store.sqlite import SqliteStore

        path = recording_dir() / f"memory-{os.getpid()}.db"
        _store = SqliteStore(path=str(path))
        _store.start()
    _streams[topic] = _store.stream(name, payload_type)
    return _streams[topic]


def record(topic: str, payload_type: type, msg: Any) -> None:
    """Append *msg* under *topic* when ``--record`` is on. No-op otherwise."""
    if not global_config.record or global_config.replay:
        return
    # ponytail: one lock per process; per-stream locks if publish contention shows up.
    with _lock:
        stream = _stream(topic, payload_type)
        if stream is not None:
            stream.append(msg, ts=getattr(msg, "ts", None) or time.time())
