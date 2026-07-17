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

"""Grid tests for StreamModule — same e2e logic across all pipeline styles."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading
from typing import Any, ClassVar
from unittest.mock import MagicMock

import pytest
from reactivex.disposable import Disposable

from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.memory2 import module as memory_module
from dimos.memory2.module import MemoryModule, Recorder, StreamModule
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import Transformer
from dimos.memory2.type.observation import Observation
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.rpc.spec import Args, RPCSpec
from dimos.protocol.tf.tf import TFSpec


class _TestRPC(RPCSpec):
    def __init__(self, **_kwargs: Any) -> None:
        pass

    def serve_rpc(self, _f: Any, _name: str) -> Any:
        return lambda: None

    def call(self, _name: str, _arguments: Args, _cb: Any) -> Any:
        return None

    def call_nowait(self, _name: str, _arguments: Args) -> None:
        pass


class _CountingTF(TFSpec):
    instances: ClassVar[int] = 0

    def __init__(self, **kwargs: Any) -> None:
        type(self).instances += 1
        super().__init__(**kwargs)

    def publish(self, *args: Transform) -> None:
        pass

    def publish_static(self, *args: Transform) -> None:
        pass

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
        *,
        forward_tolerance: float = 0.0,
    ) -> Transform | None:
        return None


# -- Shared transformer ---------------------------------------------------


class Double(Transformer[int, int]):
    def __init__(self, factor: int = 2) -> None:
        self.factor = factor

    def __call__(self, upstream: Iterator[Observation[int]]) -> Iterator[Observation[int]]:
        for obs in upstream:
            yield obs.derive(data=obs.data * self.factor)


# -- Pipeline styles -------------------------------------------------------


class StaticStreamModule(StreamModule[int, int]):
    """Pipeline as a static Stream chain on the class."""

    pipeline = Stream().transform(Double())
    numbers: In[int]
    doubled: Out[int]


class StaticTransformerModule(StreamModule[int, int]):
    """Pipeline as a bare Transformer on the class."""

    pipeline = Double()
    numbers: In[int]
    doubled: Out[int]


class MethodPipelineConfig(ModuleConfig):
    factor: int = 2


class MethodPipelineModule(StreamModule[int, int]):
    """Pipeline as a method with access to self.config."""

    config: MethodPipelineConfig

    def pipeline(self, stream: Stream[int]) -> Stream[int]:
        return stream.transform(Double(factor=self.config.factor))

    numbers: In[int]
    doubled: Out[int]


# -- Grid ------------------------------------------------------------------

module_cases = [
    pytest.param(StaticStreamModule, id="static-stream"),
    pytest.param(StaticTransformerModule, id="static-transformer"),
    pytest.param(MethodPipelineModule, id="method-pipeline"),
]

TFRecorderFixture = tuple[
    Recorder,
    MagicMock,
    MagicMock,
    Callable[[TFMessage, Any], None],
    MagicMock,
]
SYNC_TIMEOUT: float = 2.0


@pytest.fixture
def tf_recorder(tmp_path: Path) -> Iterator[TFRecorderFixture]:
    store = MagicMock(spec=SqliteStore)
    tf_stream = MagicMock(spec=Stream)
    store.stream.return_value = tf_stream
    unsubscribe = MagicMock()
    pubsub = MagicMock()
    pubsub.subscribe.return_value = unsubscribe
    tf = MagicMock()
    tf.config.topic = "/tf"
    tf.pubsub = pubsub
    module = Recorder(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )

    try:
        module._store = store
        module._tf = tf
        module._record_tf()
        callback = pubsub.subscribe.call_args.args[1]
        yield module, store, tf_stream, callback, unsubscribe
    finally:
        module.stop()


def test_recorder_stop_waits_for_active_tf_callback(
    monkeypatch: pytest.MonkeyPatch,
    tf_recorder: TFRecorderFixture,
) -> None:
    module, store, tf_stream, callback, unsubscribe = tf_recorder
    append_started = threading.Event()
    append_release = threading.Event()
    append_finished = threading.Event()
    unsubscribed = threading.Event()
    store_stopped = threading.Event()
    warning_logged = threading.Event()
    test_logger = MagicMock()
    test_logger.warning.side_effect = lambda *_args, **_kwargs: warning_logged.set()
    monkeypatch.setattr(memory_module, "_TF_DRAIN_LOG_INTERVAL_SECONDS", 0.01)
    monkeypatch.setattr(memory_module, "logger", test_logger)

    def append(*_args: Any, **_kwargs: Any) -> None:
        append_started.set()
        assert append_release.wait(timeout=SYNC_TIMEOUT)
        append_finished.set()

    def stop_store() -> None:
        assert append_finished.is_set()
        store_stopped.set()

    tf_stream.append.side_effect = append
    unsubscribe.side_effect = unsubscribed.set
    store.stop.side_effect = stop_store
    transform = Transform(ts=1.0)
    message = TFMessage(transform)

    with ThreadPoolExecutor(max_workers=2) as pool:
        callback_future = pool.submit(callback, message, "/tf")
        assert append_started.wait(timeout=SYNC_TIMEOUT)
        stop_future = pool.submit(module.stop)
        try:
            assert unsubscribed.wait(timeout=SYNC_TIMEOUT)
            assert warning_logged.wait(timeout=SYNC_TIMEOUT)
            assert not store_stopped.is_set()
        finally:
            append_release.set()

        callback_future.result(timeout=SYNC_TIMEOUT)
        stop_future.result(timeout=SYNC_TIMEOUT)

    recorded_message = tf_stream.append.call_args.args[0]
    assert recorded_message.transforms == [transform]
    assert tf_stream.append.call_args.kwargs == {"ts": 1.0, "pose": None}
    unsubscribe.assert_called_once_with()
    store.stop.assert_called_once_with()
    assert store_stopped.is_set()
    test_logger.warning.assert_called()
    assert test_logger.warning.call_args.args == ("Still waiting for tf callbacks",)
    assert test_logger.warning.call_args.kwargs["active_callbacks"] == 1


def test_recorder_rejects_tf_callback_when_stop_races_setup(
    tmp_path: Path,
) -> None:
    store = MagicMock(spec=SqliteStore)
    tf_stream = MagicMock(spec=Stream)
    stream_started = threading.Event()
    stream_release = threading.Event()
    stop_started = threading.Event()
    store_stopped = threading.Event()
    store.stop.side_effect = store_stopped.set

    def open_stream(*_args: Any, **_kwargs: Any) -> MagicMock:
        stream_started.set()
        assert stream_release.wait(timeout=SYNC_TIMEOUT)
        return tf_stream

    store.stream.side_effect = open_stream
    unsubscribe = MagicMock()
    retained_callback: list[Callable[[TFMessage, Any], None]] = []
    pubsub = MagicMock()

    def subscribe(_topic: str, callback: Callable[[TFMessage, Any], None]) -> MagicMock:
        retained_callback.append(callback)
        return unsubscribe

    pubsub.subscribe.side_effect = subscribe
    tf = MagicMock()
    tf.config.topic = "/tf"
    tf.pubsub = pubsub
    module = Recorder(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )
    module._store = store
    module._tf = tf

    def stop_module() -> None:
        stop_started.set()
        module.stop()

    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            record_future = pool.submit(module._record_tf)
            assert stream_started.wait(timeout=SYNC_TIMEOUT)
            stop_future = pool.submit(stop_module)
            try:
                assert stop_started.wait(timeout=SYNC_TIMEOUT)
                assert not store_stopped.is_set()
            finally:
                stream_release.set()
            record_future.result(timeout=SYNC_TIMEOUT)
            stop_future.result(timeout=SYNC_TIMEOUT)

        assert len(retained_callback) == 1
        retained_callback[0](TFMessage(Transform(ts=1.0)), "/tf")
    finally:
        stream_release.set()
        module.stop()

    unsubscribe.assert_called_once_with()
    store.stop.assert_called_once_with()
    assert store_stopped.is_set()
    tf_stream.append.assert_not_called()


def test_recorder_unsubscribes_when_stop_races_subscribe(
    tmp_path: Path,
) -> None:
    store = MagicMock(spec=SqliteStore)
    tf_stream = MagicMock(spec=Stream)
    store.stream.return_value = tf_stream
    subscribe_started = threading.Event()
    subscribe_release = threading.Event()
    store_stopped = threading.Event()
    store.stop.side_effect = store_stopped.set
    unsubscribe = MagicMock()
    retained_callback: list[Callable[[TFMessage, Any], None]] = []
    pubsub = MagicMock()

    def subscribe(_topic: str, callback: Callable[[TFMessage, Any], None]) -> MagicMock:
        retained_callback.append(callback)
        subscribe_started.set()
        assert subscribe_release.wait(timeout=SYNC_TIMEOUT)
        return unsubscribe

    pubsub.subscribe.side_effect = subscribe
    tf = MagicMock()
    tf.config.topic = "/tf"
    tf.pubsub = pubsub
    module = Recorder(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )
    module._store = store
    module._tf = tf

    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            record_future = pool.submit(module._record_tf)
            assert subscribe_started.wait(timeout=SYNC_TIMEOUT)
            stop_future = pool.submit(module.stop)
            try:
                assert store_stopped.wait(timeout=SYNC_TIMEOUT)
                unsubscribe.assert_not_called()
            finally:
                subscribe_release.set()

            record_future.result(timeout=SYNC_TIMEOUT)
            stop_future.result(timeout=SYNC_TIMEOUT)

        assert len(retained_callback) == 1
        retained_callback[0](TFMessage(Transform(ts=1.0)), "/tf")
    finally:
        subscribe_release.set()
        module.stop()

    unsubscribe.assert_called_once_with()
    store.stop.assert_called_once_with()
    tf_stream.append.assert_not_called()


def test_recorder_tf_gate_runs_before_other_disposables(
    tmp_path: Path,
) -> None:
    store = MagicMock(spec=SqliteStore)
    tf_stream = MagicMock(spec=Stream)
    store.stream.return_value = tf_stream
    unsubscribe = MagicMock()
    pubsub = MagicMock()
    pubsub.subscribe.return_value = unsubscribe
    tf = MagicMock()
    tf.config.topic = "/tf"
    tf.pubsub = pubsub
    module = Recorder(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )

    def fail_cleanup() -> None:
        raise RuntimeError("cleanup failed")

    try:
        module._store = store
        module._tf = tf
        module.register_disposable(Disposable(fail_cleanup))
        module._record_tf()
        callback = pubsub.subscribe.call_args.args[1]

        with pytest.raises(RuntimeError, match="cleanup failed"):
            module.stop()

        unsubscribe.assert_called_once_with()
        store.stop.assert_not_called()

        module.stop()
        callback(TFMessage(Transform(ts=1.0)), "/tf")
    finally:
        module.stop()

    store.stop.assert_called_once_with()
    tf_stream.append.assert_not_called()


def test_recorder_rejects_tf_setup_after_stop(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    monkeypatch.setattr(_CountingTF, "instances", 0)
    module = Recorder(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
        tf_transport=_CountingTF,
    )

    try:
        module.stop()
        with pytest.raises(RuntimeError, match="stopping or stopped"):
            module._record_tf()
    finally:
        module.stop()

    assert _CountingTF.instances == 0


@pytest.mark.parametrize("module_cls", module_cases)
def test_blueprint_ports(module_cls: type[StreamModule[Any, Any]]) -> None:
    """All pipeline styles produce a blueprint with the correct In/Out ports."""
    bp = module_cls.blueprint()

    assert len(bp.blueprints) == 1
    atom = bp.blueprints[0]
    stream_names = {s.name for s in atom.streams}
    assert "numbers" in stream_names
    assert "doubled" in stream_names


def test_memory_module_stops_subscriptions_before_store(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    events: list[str] = []
    store = MagicMock(spec=SqliteStore)
    store.start.side_effect = lambda: events.append("store-started")
    store.stop.side_effect = lambda: events.append("store-stopped")
    store.dispose.side_effect = lambda: events.append("store-disposed")
    store_factory = MagicMock(return_value=store)
    monkeypatch.setattr(memory_module, "SqliteStore", store_factory)
    module = MemoryModule(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )
    stopped = False

    try:
        assert module.store is store
        module.register_disposable(Disposable(lambda: events.append("subscription")))

        module.stop()
        stopped = True
    finally:
        if not stopped:
            module.stop()

    assert events == ["store-started", "subscription", "store-stopped"]
    store_factory.assert_called_once_with(path=str(tmp_path / "recording.db"))
    store.start.assert_called_once_with()
    store.stop.assert_called_once_with()
    store.dispose.assert_not_called()


def test_memory_module_serializes_concurrent_stop(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    events: list[str] = []
    close_started = threading.Event()
    close_release = threading.Event()
    store_stopped = threading.Event()
    second_started = threading.Event()
    store = MagicMock(spec=SqliteStore)

    def stop_store() -> None:
        events.append("store-stopped")
        store_stopped.set()

    store.stop.side_effect = stop_store
    monkeypatch.setattr(memory_module, "SqliteStore", MagicMock(return_value=store))
    module = MemoryModule(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )
    assert module.store is store
    close_rpc = module._close_rpc

    def blocking_close_rpc() -> None:
        events.append("close-started")
        close_started.set()
        assert close_release.wait(timeout=2)
        events.append("close-finished")
        close_rpc()

    monkeypatch.setattr(module, "_close_rpc", blocking_close_rpc)

    def stop_again() -> None:
        second_started.set()
        module.stop()

    with ThreadPoolExecutor(max_workers=2) as pool:
        first_stop = pool.submit(module.stop)
        second_stop = None
        try:
            assert close_started.wait(timeout=1)
            second_stop = pool.submit(stop_again)
            assert second_started.wait(timeout=1)
            assert not store_stopped.wait(timeout=0.1)
            assert not second_stop.done()
        finally:
            close_release.set()

        first_stop.result(timeout=2)
        assert second_stop is not None
        second_stop.result(timeout=2)

    assert events == ["close-started", "close-finished", "store-stopped"]
    store.stop.assert_called_once_with()


def test_memory_module_serializes_store_initialization(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    factory_entered = threading.Event()
    factory_release = threading.Event()
    second_started = threading.Event()
    store = MagicMock(spec=SqliteStore)

    def make_store(**_kwargs: Any) -> SqliteStore:
        factory_entered.set()
        assert factory_release.wait(timeout=2)
        return store

    store_factory = MagicMock(side_effect=make_store)
    monkeypatch.setattr(memory_module, "SqliteStore", store_factory)
    module = MemoryModule(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )

    def get_store_again() -> SqliteStore:
        second_started.set()
        return module.store

    with ThreadPoolExecutor(max_workers=2) as pool:
        first_store = pool.submit(lambda: module.store)
        second_store = None
        try:
            assert factory_entered.wait(timeout=1)
            second_store = pool.submit(get_store_again)
            assert second_started.wait(timeout=1)
            assert not second_store.done()
        finally:
            factory_release.set()

        assert first_store.result(timeout=2) is store
        assert second_store is not None
        assert second_store.result(timeout=2) is store

    module.stop()

    store_factory.assert_called_once_with(path=str(tmp_path / "recording.db"))
    store.start.assert_called_once_with()
    store.stop.assert_called_once_with()


def test_memory_module_stop_waits_for_store_initialization(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    start_entered = threading.Event()
    start_release = threading.Event()
    start_finished = threading.Event()
    stop_started = threading.Event()
    store_stop_called = threading.Event()
    store = MagicMock(spec=SqliteStore)

    def start_store() -> None:
        start_entered.set()
        assert start_release.wait(timeout=2)
        start_finished.set()

    def stop_store() -> None:
        store_stop_called.set()
        assert start_finished.is_set()

    store.start.side_effect = start_store
    store.stop.side_effect = stop_store
    monkeypatch.setattr(memory_module, "SqliteStore", MagicMock(return_value=store))
    module = MemoryModule(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )

    def stop_module() -> None:
        stop_started.set()
        module.stop()

    with ThreadPoolExecutor(max_workers=2) as pool:
        getter = pool.submit(lambda: module.store)
        assert start_entered.wait(timeout=1)

        stopper = pool.submit(stop_module)
        assert stop_started.wait(timeout=1)

        try:
            assert not store_stop_called.wait(timeout=0.1)
            assert not stopper.done()
        finally:
            start_release.set()

        assert getter.result(timeout=2) is store
        stopper.result(timeout=2)

    store.start.assert_called_once_with()
    store.stop.assert_called_once_with()


def test_memory_module_refuses_store_creation_after_stop_begins(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    close_started = threading.Event()
    close_release = threading.Event()
    getter_started = threading.Event()
    store_factory = MagicMock()
    monkeypatch.setattr(memory_module, "SqliteStore", store_factory)
    module = MemoryModule(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )
    close_rpc = module._close_rpc

    def blocking_close_rpc() -> None:
        close_started.set()
        assert close_release.wait(timeout=2)
        close_rpc()

    monkeypatch.setattr(module, "_close_rpc", blocking_close_rpc)

    def get_store() -> SqliteStore:
        getter_started.set()
        return module.store

    with ThreadPoolExecutor(max_workers=2) as pool:
        stop = pool.submit(module.stop)
        getter = None
        try:
            assert close_started.wait(timeout=1)
            getter = pool.submit(get_store)
            assert getter_started.wait(timeout=1)
            assert not getter.done()
        finally:
            close_release.set()

        stop.result(timeout=2)
        assert getter is not None
        with pytest.raises(RuntimeError, match="stopping or stopped"):
            getter.result(timeout=2)

    store_factory.assert_not_called()


def test_memory_module_retries_failed_store_shutdown(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    store = MagicMock(spec=SqliteStore)
    store.stop.side_effect = [RuntimeError("close failed"), None]
    monkeypatch.setattr(memory_module, "SqliteStore", MagicMock(return_value=store))
    module = MemoryModule(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )
    assert module.store is store

    with pytest.raises(RuntimeError, match="close failed"):
        module.stop()

    assert module._memory_stopping
    assert not module._memory_stopped.is_set()
    assert module._store is store
    with pytest.raises(RuntimeError, match="stopping or stopped"):
        assert module.store is not None

    module.stop()

    assert store.stop.call_count == 2
    store.stop.assert_called_with()
    assert module._store is None
    assert module._memory_stopped.is_set()


def test_memory_module_restores_fresh_runtime_store_state(tmp_path: Path) -> None:
    module = MemoryModule(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )
    module._store = MagicMock(spec=SqliteStore)

    state = module.__getstate__()
    restored = MemoryModule.__new__(MemoryModule)
    restored.__setstate__(state)

    module._store = None
    module.stop()

    assert "_memory_stop_lock" not in state
    assert "_memory_stopping" not in state
    assert "_memory_stopped" not in state
    assert "_store" not in state
    assert restored._store is None
    assert not restored._memory_stopping
    assert not restored._memory_stopped.is_set()


def test_memory_module_preserves_stopped_state_when_restored(tmp_path: Path) -> None:
    module = MemoryModule(
        db_path=tmp_path / "recording.db",
        rpc_transport=_TestRPC,
    )
    module.stop()

    state = module.__getstate__()
    restored = MemoryModule.__new__(MemoryModule)
    restored.__setstate__(state)

    assert restored._module_closed
    assert restored._memory_stopping
    assert restored._memory_stopped.is_set()
    with pytest.raises(RuntimeError, match="stopping or stopped"):
        assert restored.store is not None
