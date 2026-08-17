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

"""CollectionRecorder — captures teleop collection streams to a memory DB.

A `Recorder` (memory) subscribes each declared `In` port and appends every
message to a SQLite store, flushing durably on stop(). Only *connected*
streams are recorded, so the same recorder works for any arm whose
coordinator publishes `coordinator_joint_state`.

The recorded stream names match what DataPrep reads: `color_image`
and `coordinator_joint_state` (observation), `status` (episode segmentation).
"""

from __future__ import annotations

from dataclasses import dataclass
import queue
import sqlite3
import threading
import time
from typing import Any, Literal

from pydantic import Field
from reactivex.abc import DisposableBase

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.imitation.collection.episode_monitor import EpisodeStatus
from dimos.memory.module import Recorder, RecorderConfig
from dimos.memory.stream import Stream
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class _RecorderMetrics:
    received: int = 0
    written: int = 0
    queued: int = 0
    dropped: int = 0
    failed: int = 0


@dataclass
class _RecordItem:
    name: str
    stream: Stream[Any]
    msg: Any
    recv_ts: float


_STOP = object()


class CollectionRecorderConfig(RecorderConfig):
    record_tf: bool = False
    queue_maxsize: int = Field(default=1024, ge=1)
    queue_overflow: Literal["drop_new"] = "drop_new"
    drain_timeout: float = Field(default=30.0, gt=0)
    drop_warning_interval: float = Field(default=5.0, ge=0)


class CollectionRecorder(Recorder):
    """Records the streams DataPrep consumes from a teleop session."""

    config: CollectionRecorderConfig

    color_image: In[Image]  # observation (camera)
    coordinator_joint_state: In[JointState]  # observation + action (measured/next state)
    status: In[EpisodeStatus]  # episode start/save/discard segmentation

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._record_queue: queue.Queue[_RecordItem | object] | None = None
        self._writer_thread: threading.Thread | None = None
        self._record_subscriptions: list[DisposableBase] = []
        self._record_metrics: dict[str, _RecorderMetrics] = {}
        self._writer_failures = 0
        self._metrics_lock = threading.Lock()
        self._accepting_records = False
        self._last_drop_warning: dict[str, float] = {}

    @rpc
    def start(self) -> None:
        if not self.config.g.replay:
            if self.config.record_tf:
                raise ValueError("CollectionRecorder does not support TF recording")
            missing = set(self._data_ports()) - set(self.config.poseless_streams)
            if missing:
                raise ValueError(
                    "CollectionRecorder requires every data stream to be poseless; "
                    f"missing from poseless_streams: {sorted(missing)}"
                )
            self._start_record_queue()
        try:
            super().start()
        except BaseException:
            self._stop_record_queue()
            raise

    def _start_record_queue(self) -> None:
        self._record_queue = queue.Queue(maxsize=self.config.queue_maxsize)
        with self._metrics_lock:
            self._accepting_records = True
        self._writer_thread = threading.Thread(
            target=self._record_writer,
            daemon=True,
            name="CollectionRecorderWriter",
        )
        self._writer_thread.start()

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        def on_msg(msg: Any) -> None:
            self._submit_record(_RecordItem(name, stream, msg, time.time()))

        self._record_subscriptions.append(input_topic.pure_observable().subscribe(on_msg))

    def _submit_record(self, item: _RecordItem) -> None:
        record_queue = self._record_queue
        with self._metrics_lock:
            metrics = self._record_metrics.setdefault(item.name, _RecorderMetrics())
            metrics.received += 1
            if not self._accepting_records or record_queue is None:
                metrics.dropped += 1
                self._warn_record_drop(item.name, metrics.dropped)
                return
            try:
                record_queue.put_nowait(item)
            except queue.Full:
                metrics.dropped += 1
                self._warn_record_drop(item.name, metrics.dropped)
                return
            metrics.queued += 1

    def _warn_record_drop(self, name: str, dropped: int) -> None:
        now = time.monotonic()
        last = self._last_drop_warning.get(name, float("-inf"))
        if now - last < self.config.drop_warning_interval:
            return
        self._last_drop_warning[name] = now
        logger.warning(
            "Collection recorder queue saturated; dropping new %s message (dropped=%d, maxsize=%d)",
            name,
            dropped,
            self.config.queue_maxsize,
        )

    def _record_writer(self) -> None:
        record_queue = self._record_queue
        assert record_queue is not None
        while True:
            item = record_queue.get()
            if item is _STOP:
                record_queue.task_done()
                return
            assert isinstance(item, _RecordItem)
            with self._metrics_lock:
                self._record_metrics[item.name].queued -= 1
            try:
                item.stream.append(
                    item.msg,
                    ts=self._resolve_ts(item.name, item.msg),
                    pose=None,
                    tags={"reception_ts": item.recv_ts},
                )
                with self._metrics_lock:
                    self._record_metrics[item.name].written += 1
            except Exception:
                with self._metrics_lock:
                    self._record_metrics[item.name].failed += 1
                    self._writer_failures += 1
                logger.exception("Failed to record collection stream %s", item.name)
            finally:
                record_queue.task_done()

    @rpc
    def recording_metrics(self) -> dict[str, dict[str, int]]:
        """Return per-stream collection queue and write counters."""
        with self._metrics_lock:
            return {
                name: {
                    "received": metrics.received,
                    "written": metrics.written,
                    "queued": metrics.queued,
                    "dropped": metrics.dropped,
                    "failed": metrics.failed,
                }
                for name, metrics in self._record_metrics.items()
            }

    def _stop_record_queue(self) -> None:
        for subscription in self._record_subscriptions:
            subscription.dispose()
        self._record_subscriptions.clear()
        with self._metrics_lock:
            self._accepting_records = False

        record_queue = self._record_queue
        writer_thread = self._writer_thread
        if record_queue is None or writer_thread is None:
            return

        deadline = time.monotonic() + self.config.drain_timeout
        try:
            record_queue.put(_STOP, timeout=self.config.drain_timeout)
        except queue.Full as exc:
            raise RuntimeError(
                f"Collection recorder queue did not accept its stop marker within "
                f"{self.config.drain_timeout}s"
            ) from exc
        writer_thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if writer_thread.is_alive():
            pending = sum(
                metrics.received - metrics.written - metrics.dropped - metrics.failed
                for metrics in self._record_metrics.values()
            )
            raise RuntimeError(
                f"Collection recorder failed to drain {pending} accepted messages within "
                f"{self.config.drain_timeout}s; store left open"
            )
        self._record_queue = None
        self._writer_thread = None

    def _checkpoint(self) -> None:
        with sqlite3.connect(self.config.db_path) as connection:
            connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")

    @rpc
    def stop(self) -> None:
        if self.config.g.replay or self._record_queue is None:
            super().stop()
            return
        self._stop_record_queue()
        self._checkpoint()
        failures = self._writer_failures
        super().stop()
        if failures:
            raise RuntimeError(f"Collection recorder failed to write {failures} accepted messages")
