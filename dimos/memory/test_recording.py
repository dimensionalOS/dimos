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
from typing import Any, cast

import pytest

from dimos.memory.backend import Backend, PreparedAppend
from dimos.memory.codecs.pickle import PickleCodec
from dimos.memory.notifier.subject import SubjectNotifier
from dimos.memory.observationstore.memory import ListObservationStore
from dimos.memory.recording import PreparedWrite, RecordingFailedError, RecordingPipeline
from dimos.memory.type.observation import Observation


class _Backend:
    def __init__(self) -> None:
        self.batches: list[list[float]] = []

    def append_prepared(self, appends: list[PreparedAppend[Any]]) -> list[Observation[Any]]:
        observations = [append.observation for append in appends]
        self.batches.append([observation.ts for observation in observations])
        return observations


def _write(backend: Backend[Any], value: float) -> tuple[PreparedWrite]:
    observation = Observation(ts=value, _data=value)
    return (PreparedWrite(backend, PreparedAppend(observation, None)),)


def test_pipeline_preserves_global_fifo_and_batches_writes() -> None:
    backend = cast("Backend[Any]", _Backend())
    pipeline = RecordingPipeline(
        {
            "camera": lambda value: _write(backend, value),
            "imu": lambda value: _write(backend, value),
        },
        batch_rows=3,
        batch_delay_s=1.0,
    )
    pipeline.start()
    try:
        pipeline.submit("camera", 1.0)
        pipeline.submit("imu", 2.0)
        pipeline.submit("camera", 3.0)
    finally:
        pipeline.close(timeout_s=1.0)

    assert cast("_Backend", backend).batches == [[1.0, 2.0, 3.0]]


def test_pipeline_fails_instead_of_dropping_when_ingress_is_full() -> None:
    backend = cast("Backend[Any]", _Backend())
    started = threading.Event()
    release = threading.Event()

    def process(value: float) -> tuple[PreparedWrite]:
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
    def process(_: float) -> tuple[PreparedWrite]:
        raise OSError("encoder failed")

    pipeline = RecordingPipeline({"depth": process})
    pipeline.start()
    pipeline.submit("depth", 1.0)

    with pytest.raises(RecordingFailedError, match="failed") as exc:
        pipeline.close(timeout_s=1.0)
    assert isinstance(exc.value.__cause__, OSError)


def test_backend_commits_batch_before_notifying(monkeypatch: pytest.MonkeyPatch) -> None:
    events: list[str] = []
    store = ListObservationStore[float](name="camera")
    notifier = SubjectNotifier[float]()
    monkeypatch.setattr(store, "commit", lambda: events.append("commit"), raising=False)
    monkeypatch.setattr(notifier, "notify", lambda obs: events.append(f"notify-{obs.ts}"))
    backend = Backend[float](metadata_store=store, codec=PickleCodec(), notifier=notifier)
    appends = [backend.prepare_append(Observation(ts=ts, _data=ts)) for ts in (1.0, 2.0)]

    backend.append_prepared(appends)

    assert events == ["commit", "notify-1.0", "notify-2.0"]


def test_backend_rolls_back_without_notifying(monkeypatch: pytest.MonkeyPatch) -> None:
    events: list[str] = []
    store = ListObservationStore[float](name="camera")
    notifier = SubjectNotifier[float]()

    def fail_insert(_: Observation[float]) -> int:
        raise OSError("locked")

    monkeypatch.setattr(store, "insert", fail_insert)
    monkeypatch.setattr(store, "rollback", lambda: events.append("rollback"), raising=False)
    monkeypatch.setattr(notifier, "notify", lambda obs: events.append("notify"))
    backend = Backend[float](metadata_store=store, codec=PickleCodec(), notifier=notifier)
    append = backend.prepare_append(Observation(ts=1.0, _data=1.0))

    with pytest.raises(OSError, match="locked"):
        backend.append_prepared((append,))

    assert events == ["rollback"]
