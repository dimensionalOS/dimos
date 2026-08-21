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

from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
import threading

import pytest

from dimos.memory.backend import Backend, PreparedWrite, TransactionFactory
from dimos.memory.codecs.pickle import PickleCodec
from dimos.memory.notifier.subject import SubjectNotifier
from dimos.memory.observationstore.memory import ListObservationStore
from dimos.memory.recording import RecordingFailedError, RecordingPipeline
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation


def _transaction(events: list[str], name: str) -> TransactionFactory:
    @contextmanager
    def transaction() -> Iterator[None]:
        try:
            yield
        except BaseException:
            events.append(f"rollback-{name}")
            raise
        else:
            events.append(f"commit-{name}")

    return transaction


def _backend(name: str, transaction: TransactionFactory | None = None) -> Backend[float]:
    return Backend[float](
        metadata_store=ListObservationStore[float](name=name),
        codec=PickleCodec(),
        notifier=SubjectNotifier[float](),
        transaction=transaction,
    )


def _write(backend: Backend[float], value: float) -> tuple[PreparedWrite[float]]:
    observation = Observation(ts=value, _data=value)
    return (PreparedWrite(backend, backend.prepare_append(observation)),)


def test_pipeline_batches_backends_and_preserves_global_notification_fifo(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []
    camera = _backend("camera", _transaction(events, "camera"))
    imu = _backend("imu", _transaction(events, "imu"))
    monkeypatch.setattr(
        camera.notifier, "notify", lambda observation: events.append(f"notify-{observation.ts}")
    )
    monkeypatch.setattr(
        imu.notifier, "notify", lambda observation: events.append(f"notify-{observation.ts}")
    )
    pipeline = RecordingPipeline(
        {
            "camera": lambda value: _write(camera, value),
            "imu": lambda value: _write(imu, value),
        },
        batch_rows=4,
        batch_delay_s=1.0,
    )

    pipeline.start()
    try:
        pipeline.submit("camera", 1.0)
        pipeline.submit("imu", 2.0)
        pipeline.submit("camera", 3.0)
        pipeline.submit("imu", 4.0)
    finally:
        pipeline.close(timeout_s=1.0)

    assert events == [
        "commit-camera",
        "commit-imu",
        "notify-1.0",
        "notify-2.0",
        "notify-3.0",
        "notify-4.0",
    ]


def test_pipeline_fails_instead_of_dropping_when_ingress_is_full() -> None:
    backend = _backend("depth")
    started = threading.Event()
    release = threading.Event()

    def process(value: float) -> tuple[PreparedWrite[float]]:
        started.set()
        assert release.wait(timeout=1.0)
        return _write(backend, value)

    pipeline = RecordingPipeline({"depth": process}, ingress_size=1)
    pipeline.start()
    pipeline.submit("depth", 1.0)
    assert started.wait(timeout=1.0)
    pipeline.submit("depth", 2.0)

    with pytest.raises(RecordingFailedError, match="reached capacity"):
        pipeline.submit("depth", 3.0)

    release.set()
    with pytest.raises(RecordingFailedError, match="failed"):
        pipeline.close(timeout_s=1.0)


def test_pipeline_reports_preparation_failure() -> None:
    def process(_value: float) -> tuple[PreparedWrite[float]]:
        raise OSError("encoder failed")

    pipeline = RecordingPipeline({"depth": process})
    pipeline.start()
    pipeline.submit("depth", 1.0)

    with pytest.raises(RecordingFailedError, match="failed") as exc:
        pipeline.close(timeout_s=1.0)
    assert isinstance(exc.value.__cause__, OSError)


def test_pipeline_does_not_notify_when_a_backend_group_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []
    camera = _backend("camera", _transaction(events, "camera"))
    depth = _backend("depth", _transaction(events, "depth"))

    def fail_insert(_observation: Observation[float]) -> int:
        raise OSError("locked")

    monkeypatch.setattr(depth.metadata_store, "insert", fail_insert)
    monkeypatch.setattr(camera.notifier, "notify", lambda _observation: events.append("notify"))
    monkeypatch.setattr(depth.notifier, "notify", lambda _observation: events.append("notify"))
    pipeline = RecordingPipeline(
        {
            "camera": lambda value: _write(camera, value),
            "depth": lambda value: _write(depth, value),
        },
        batch_rows=2,
        batch_delay_s=1.0,
    )

    pipeline.start()
    pipeline.submit("camera", 1.0)
    pipeline.submit("depth", 2.0)

    with pytest.raises(RecordingFailedError, match="failed"):
        pipeline.close(timeout_s=1.0)

    assert events == ["commit-camera", "rollback-depth"]


def test_pipeline_reports_notification_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    backend = _backend("camera")

    def fail_notify(_observation: Observation[float]) -> None:
        raise OSError("consumer failed")

    monkeypatch.setattr(backend.notifier, "notify", fail_notify)
    pipeline = RecordingPipeline(
        {"camera": lambda value: _write(backend, value)},
        batch_rows=1,
    )
    pipeline.start()
    pipeline.submit("camera", 1.0)

    with pytest.raises(RecordingFailedError, match="failed") as exc:
        pipeline.close(timeout_s=1.0)
    assert isinstance(exc.value.__cause__, OSError)


def test_backend_commits_batch_before_notifying(monkeypatch: pytest.MonkeyPatch) -> None:
    events: list[str] = []
    store = ListObservationStore[float](name="camera")
    notifier = SubjectNotifier[float]()
    monkeypatch.setattr(notifier, "notify", lambda obs: events.append(f"notify-{obs.ts}"))
    backend = Backend[float](
        metadata_store=store,
        codec=PickleCodec(),
        notifier=notifier,
        transaction=_transaction(events, "batch"),
    )
    appends = [backend.prepare_append(Observation(ts=ts, _data=ts)) for ts in (1.0, 2.0)]

    backend.append_prepared(appends)

    assert events == ["commit-batch", "notify-1.0", "notify-2.0"]


def test_backend_rolls_back_without_notifying(monkeypatch: pytest.MonkeyPatch) -> None:
    events: list[str] = []
    store = ListObservationStore[float](name="camera")
    notifier = SubjectNotifier[float]()

    def fail_insert(_observation: Observation[float]) -> int:
        raise OSError("locked")

    monkeypatch.setattr(store, "insert", fail_insert)
    monkeypatch.setattr(notifier, "notify", lambda _observation: events.append("notify"))
    backend = Backend[float](
        metadata_store=store,
        codec=PickleCodec(),
        notifier=notifier,
        transaction=_transaction(events, "batch"),
    )
    append = backend.prepare_append(Observation(ts=1.0, _data=1.0))

    with pytest.raises(OSError, match="locked"):
        backend.append_prepared((append,))

    assert events == ["rollback-batch"]


def test_sqlite_backend_rolls_back_metadata_when_blob_write_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    with SqliteStore(path=tmp_path / "recording.db") as store:
        stream = store.stream("camera", bytes)
        write = stream.prepare_write(Observation(ts=1.0, _data=b"frame"))
        blob_store = write.backend.blob_store
        assert blob_store is not None

        def fail_put(_stream_name: str, _key: int, _data: bytes) -> None:
            raise OSError("blob write failed")

        monkeypatch.setattr(blob_store, "put", fail_put)

        with pytest.raises(OSError, match="blob write failed"):
            write.backend.append_prepared((write.append,))

        assert stream.count() == 0
