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

import asyncio
from collections.abc import Callable
from concurrent.futures import Future, ThreadPoolExecutor, TimeoutError as FutureTimeoutError
from pathlib import Path
import threading
from types import SimpleNamespace
from typing import Any, overload
from unittest.mock import MagicMock

import pytest
from reactivex import operators as ops
from reactivex.subject import Subject

from dimos.core.stream import In
from dimos.memory2 import module as memory_module
from dimos.memory2.module import Recorder
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.rpc.spec import Args, RPCSpec

SYNC_TIMEOUT: float = 2.0


class _TestRPC(RPCSpec):
    def __init__(self, **_kwargs: Any) -> None:
        pass

    def serve_rpc(self, _f: Callable[..., Any], _name: str) -> Callable[[], None]:
        return lambda: None

    @overload
    def call(self, _name: str, _arguments: Args, _cb: None) -> None: ...

    @overload
    def call(
        self,
        _name: str,
        _arguments: Args,
        _cb: Callable[[Any], None],
    ) -> Callable[[], Any]: ...

    def call(
        self,
        _name: str,
        _arguments: Args,
        _cb: Callable[[Any], None] | None,
    ) -> Callable[[], Any] | None:
        return None if _cb is None else lambda: None

    def call_nowait(self, _name: str, _arguments: Args) -> None:
        pass


def _recorder(tmp_path: Path, store: MagicMock) -> Recorder:
    module = Recorder(
        db_path=tmp_path / "recording.db",
        record_tf=False,
        rpc_transport=_TestRPC,
    )
    module._store = store
    module.register_disposable(store)
    return module


def _store() -> tuple[MagicMock, threading.Event]:
    store = MagicMock(spec=SqliteStore)
    stopped = threading.Event()

    def dispose() -> None:
        store.stop()
        stopped.set()

    store.dispose.side_effect = dispose
    return store, stopped


def test_stop_waits_for_active_input_before_closing_store(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    store, store_stopped = _store()
    stream = MagicMock(spec=Stream)
    input_topic = MagicMock(spec=In)
    subject: Subject[Any] = Subject()
    unsubscribed = threading.Event()
    input_topic.pure_observable.return_value = subject.pipe(ops.finally_action(unsubscribed.set))
    module = _recorder(tmp_path, store)
    append_started = threading.Event()
    append_release = threading.Event()
    append_finished = threading.Event()

    async def resolve_pose(_name: str, _msg: Any, _ts: float) -> None:
        return None

    def append(*_args: Any, **_kwargs: Any) -> None:
        append_started.set()
        assert append_release.wait(timeout=SYNC_TIMEOUT)
        append_finished.set()

    def stop_store() -> None:
        assert append_finished.is_set()

    monkeypatch.setattr(module, "_resolve_pose", resolve_pose)
    stream.append.side_effect = append
    store.stop.side_effect = stop_store
    module._port_to_stream("color_image", input_topic, stream)

    try:
        subject.on_next(SimpleNamespace(ts=1.0))
        assert append_started.wait(timeout=SYNC_TIMEOUT)
        with ThreadPoolExecutor(max_workers=1) as pool:
            stop_future = pool.submit(module.stop)
            try:
                assert unsubscribed.wait(timeout=SYNC_TIMEOUT)
                assert not store_stopped.is_set()
                assert not stop_future.done()
            finally:
                append_release.set()
            stop_future.result(timeout=SYNC_TIMEOUT)
    finally:
        append_release.set()
        module.stop()

    assert store_stopped.is_set()
    store.stop.assert_called_once_with()


def test_stop_waits_for_active_tf_before_closing_store(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    store, store_stopped = _store()
    tf_stream = MagicMock(spec=Stream)
    store.stream.return_value = tf_stream
    module = _recorder(tmp_path, store)
    callback: list[Callable[[TFMessage], None]] = []
    unsubscribed = threading.Event()
    tf_port = MagicMock(spec=In)
    tf_port._transport = MagicMock()

    def subscribe(cb: Callable[[TFMessage], None]) -> Callable[[], None]:
        callback.append(cb)
        return unsubscribed.set

    tf_port.subscribe.side_effect = subscribe
    module.tf = tf_port
    append_started = threading.Event()
    append_release = threading.Event()
    append_finished = threading.Event()

    def append(*_args: Any, **_kwargs: Any) -> None:
        append_started.set()
        assert append_release.wait(timeout=SYNC_TIMEOUT)
        append_finished.set()

    def stop_store() -> None:
        assert append_finished.is_set()

    tf_stream.append.side_effect = append
    store.stop.side_effect = stop_store
    module._record_tf()

    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            callback_future = pool.submit(
                callback[0], TFMessage(Transform(ts=1.0, frame_id="world"))
            )
            assert append_started.wait(timeout=SYNC_TIMEOUT)
            stop_future = pool.submit(module.stop)
            try:
                assert unsubscribed.wait(timeout=SYNC_TIMEOUT)
                assert not store_stopped.is_set()
                assert not stop_future.done()
            finally:
                append_release.set()
            callback_future.result(timeout=SYNC_TIMEOUT)
            stop_future.result(timeout=SYNC_TIMEOUT)
    finally:
        append_release.set()
        module.stop()

    assert store_stopped.is_set()
    store.stop.assert_called_once_with()


def test_late_tf_callback_is_ignored_after_stop(tmp_path: Path) -> None:
    store, _ = _store()
    tf_stream = MagicMock(spec=Stream)
    store.stream.return_value = tf_stream
    module = _recorder(tmp_path, store)
    callback: list[Callable[[TFMessage], None]] = []
    tf_port = MagicMock(spec=In)
    tf_port._transport = MagicMock()

    def subscribe(cb: Callable[[TFMessage], None]) -> Callable[[], None]:
        callback.append(cb)
        return lambda: None

    tf_port.subscribe.side_effect = subscribe
    module.tf = tf_port
    module._record_tf()
    module.stop()

    callback[0](TFMessage(Transform(ts=1.0, frame_id="world")))

    tf_stream.append.assert_not_called()


def test_concurrent_stop_waits_for_active_teardown(tmp_path: Path) -> None:
    store, _ = _store()
    module = _recorder(tmp_path, store)
    store_stop_started = threading.Event()
    store_stop_release = threading.Event()
    second_stop_started = threading.Event()

    def stop_store() -> None:
        store_stop_started.set()
        assert store_stop_release.wait(timeout=SYNC_TIMEOUT)

    def stop_again() -> None:
        second_stop_started.set()
        module.stop()

    store.stop.side_effect = stop_store
    first_stop: Future[None] | None = None
    second_stop: Future[None] | None = None
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            first_stop = pool.submit(module.stop)
            assert store_stop_started.wait(timeout=SYNC_TIMEOUT)
            second_stop = pool.submit(stop_again)
            assert second_stop_started.wait(timeout=SYNC_TIMEOUT)
            assert not module._module_closed
            with pytest.raises(FutureTimeoutError):
                second_stop.result(timeout=0.05)
            store_stop_release.set()
            first_stop.result(timeout=SYNC_TIMEOUT)
            second_stop.result(timeout=SYNC_TIMEOUT)
    finally:
        store_stop_release.set()
        if first_stop is not None:
            first_stop.result(timeout=SYNC_TIMEOUT)
        if second_stop is not None:
            second_stop.result(timeout=SYNC_TIMEOUT)

    store.stop.assert_called_once_with()


def test_stop_can_retry_after_drain_timeout(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    store, store_stopped = _store()
    stream = MagicMock(spec=Stream)
    input_topic = MagicMock(spec=In)
    subject: Subject[Any] = Subject()
    input_topic.pure_observable.return_value = subject
    module = _recorder(tmp_path, store)
    append_started = threading.Event()
    append_release = threading.Event()
    append_finished = threading.Event()
    callback_threads: list[threading.Thread] = []

    async def resolve_pose(_name: str, _msg: Any, _ts: float) -> None:
        return None

    def append(*_args: Any, **_kwargs: Any) -> None:
        append_started.set()
        assert append_release.wait(timeout=SYNC_TIMEOUT)
        append_finished.set()

    monkeypatch.setattr(module, "_resolve_pose", resolve_pose)
    monkeypatch.setattr(
        memory_module,
        "_RECORDER_DRAIN_TIMEOUT_SECONDS",
        0.01,
        raising=False,
    )
    monkeypatch.setattr(
        memory_module,
        "_RECORDER_DRAIN_LOG_INTERVAL_SECONDS",
        0.005,
        raising=False,
    )

    def make_dispatch(
        async_callback: Callable[[Any], Any],
    ) -> tuple[Callable[[Any], None], MagicMock]:
        dispatcher = MagicMock()

        def on_next(msg: Any) -> None:
            thread = threading.Thread(target=lambda: asyncio.run(async_callback(msg)))
            callback_threads.append(thread)
            thread.start()

        return on_next, dispatcher

    monkeypatch.setattr(module, "_make_async_dispatch", make_dispatch)
    stream.append.side_effect = append
    module._port_to_stream("color_image", input_topic, stream)

    try:
        subject.on_next(SimpleNamespace(ts=1.0))
        assert append_started.wait(timeout=SYNC_TIMEOUT)
        with pytest.raises(RuntimeError, match="Timed out waiting for 1 recorder callback"):
            module.stop()
        assert not store_stopped.is_set()
        assert not module._module_closed

        append_release.set()
        assert append_finished.wait(timeout=SYNC_TIMEOUT)
        for thread in callback_threads:
            thread.join(timeout=SYNC_TIMEOUT)

        module.stop()
    finally:
        append_release.set()
        for thread in callback_threads:
            thread.join(timeout=SYNC_TIMEOUT)
        if not module._module_closed:
            module.stop()

    assert store_stopped.is_set()
    assert module._module_closed
    store.stop.assert_called_once_with()
