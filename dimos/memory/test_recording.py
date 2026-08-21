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

import threading

import pytest

from dimos.memory.backend import Backend, PreparedWrite
from dimos.memory.codecs.pickle import PickleCodec
from dimos.memory.notifier.subject import SubjectNotifier
from dimos.memory.observationstore.memory import ListObservationStore
from dimos.memory.recording import RecordingFailedError, RecordingPipeline
from dimos.memory.type.filter import StreamQuery
from dimos.memory.type.observation import Observation


def _backend(name: str) -> Backend[float]:
    return Backend[float](
        metadata_store=ListObservationStore[float](name=name),
        codec=PickleCodec(),
        notifier=SubjectNotifier[float](),
    )


def _write(backend: Backend[float], value: float) -> tuple[PreparedWrite[float]]:
    observation = Observation(ts=value, _data=value)
    return (PreparedWrite(backend, backend.prepare_append(observation)),)


def test_pipeline_batches_backends_and_preserves_global_notification_fifo(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []
    camera = _backend("camera")
    imu = _backend("imu")
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

    assert events == ["notify-1.0", "notify-2.0", "notify-3.0", "notify-4.0"]


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
    camera = _backend("camera")
    depth = _backend("depth")

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

    assert events == []
    assert camera.count(StreamQuery()) == 1
    assert depth.count(StreamQuery()) == 0


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


def test_backend_persists_batch_before_notifying(monkeypatch: pytest.MonkeyPatch) -> None:
    events: list[str] = []
    store = ListObservationStore[float](name="camera")
    notifier = SubjectNotifier[float]()
    insert = store.insert

    def record_insert(observation: Observation[float]) -> int:
        events.append(f"persist-{observation.ts}")
        return insert(observation)

    monkeypatch.setattr(store, "insert", record_insert)
    monkeypatch.setattr(notifier, "notify", lambda obs: events.append(f"notify-{obs.ts}"))
    backend = Backend[float](
        metadata_store=store,
        codec=PickleCodec(),
        notifier=notifier,
    )
    appends = [backend.prepare_append(Observation(ts=ts, _data=ts)) for ts in (1.0, 2.0)]

    backend.append_prepared(appends)

    assert events == ["persist-1.0", "persist-2.0", "notify-1.0", "notify-2.0"]


def test_backend_does_not_notify_when_persistence_fails(monkeypatch: pytest.MonkeyPatch) -> None:
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
    )
    append = backend.prepare_append(Observation(ts=1.0, _data=1.0))

    with pytest.raises(OSError, match="locked"):
        backend.append_prepared((append,))

    assert events == []
