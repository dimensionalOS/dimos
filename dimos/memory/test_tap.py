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

from collections.abc import Callable
from contextlib import ExitStack
from pathlib import Path
import threading
from typing import Any, cast

import pytest
from pytest_mock import MockerFixture

from dimos.core.global_config import global_config
from dimos.core.stream import Transport
from dimos.core.transport import ROSTransport, ZenohTransport, pZenohTransport
from dimos.memory import tap
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.pubsub.impl import rospubsub


class _Transport(Transport[Any]):
    def __init__(self) -> None:
        self.callbacks: list[Callable[[Any], None]] = []

    def subscribe(
        self, callback: Callable[[Any], None], selfstream: Any = None
    ) -> Callable[[], None]:
        self.callbacks.append(callback)
        return lambda: self.callbacks.remove(callback)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def broadcast(self, selfstream: Any, msg: Any) -> None:
        for cb in self.callbacks:
            cb(msg)


class _FailingTransport(_Transport):
    def subscribe(
        self, callback: Callable[[Any], None], selfstream: Any = None
    ) -> Callable[[], None]:
        raise RuntimeError("subscription failed")


def test_taps_matching_dimos_streams(tmp_path: Path) -> None:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    rec = TransportRecorder(store, topics="/odom, lidar")
    odom, goal, raw = _Transport(), _Transport(), _Transport()
    unsub = rec.tap("odom", PoseStamped, odom)
    assert rec.tap("goal", PoseStamped, goal) is None  # filtered out
    assert rec.tap("lidar", dict, raw) is None  # not a dimos message type
    odom.publish(PoseStamped(ts=1.0))
    odom.publish(PoseStamped(ts=2.0))
    goal.publish(PoseStamped(ts=3.0))
    assert unsub is not None
    unsub()
    odom.publish(PoseStamped(ts=4.0))
    rec.close()
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    assert store.list_streams() == ["odom"]
    assert [o.ts for o in store.stream("odom", PoseStamped)] == [1.0, 2.0]
    store.stop()


def test_full_queue_drops_and_counts(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    warnings: list[str] = []
    monkeypatch.setattr(tap.logger, "warning", lambda msg, *args: warnings.append(msg % args))
    store = SqliteStore(path=str(tmp_path / "memory.db"))
    store.start()
    rec = TransportRecorder(store, queue_size=1)
    rec._queue.put(None)  # stop the writer so nothing drains
    rec._writer.join()
    odom = _Transport()
    rec.tap("odom", PoseStamped, odom)
    odom.publish(PoseStamped(ts=1.0))
    odom.publish(PoseStamped(ts=2.0))
    odom.publish(PoseStamped(ts=3.0))
    assert rec.dropped == 2
    assert len(warnings) == 1
    store.stop()


def test_append_failure_does_not_kill_writer(tmp_path: Path) -> None:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    rec = TransportRecorder(store)
    odom = _Transport()
    rec.tap("odom", PoseStamped, odom)
    odom.publish("not a PoseStamped")  # append raises TypeError inside the writer
    odom.publish(PoseStamped(ts=2.0))
    rec.close()  # returns: the writer is still draining
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    assert [o.ts for o in store.stream("odom", PoseStamped)] == [2.0]
    store.stop()


def test_recording_fails_loudly_when_no_stream_matches(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_topics", "color_image2")
    with pytest.raises(ValueError, match="matched none of: lidar, odom"):
        with tap.recording(
            {("odom", PoseStamped): _Transport(), ("lidar", PoseStamped): _Transport()}
        ):
            pass
    assert not any(tmp_path.rglob("memory.db"))


@pytest.mark.parametrize("transport_cls", [ZenohTransport, pZenohTransport])
def test_recording_starts_zenoh_subscriptions_concurrently(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
    transport_cls: type[ZenohTransport[Any]] | type[pZenohTransport[Any]],
) -> None:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_topics", "*")
    barrier = threading.Barrier(4)
    transports: dict[tuple[str, type], ZenohTransport[Any] | pZenohTransport[Any]] = {
        (f"stream_{i}", PoseStamped): transport_cls(f"stream_{i}") for i in range(4)
    }
    unsubscribe = mocker.Mock()

    def subscribe(*args: Any) -> Callable[[], None]:
        barrier.wait(timeout=2.0)
        return cast("Callable[[], None]", unsubscribe)

    subscriptions = []
    for transport in transports.values():
        mocker.patch.object(transport.zenoh, "start")
        subscriptions.append(
            mocker.patch.object(transport.zenoh, "subscribe", side_effect=subscribe)
        )

    with tap.recording(transports):
        assert [subscription.call_count for subscription in subscriptions] == [1, 1, 1, 1]

    assert unsubscribe.call_count == 4


def test_recording_serializes_subscriptions_on_shared_transport(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_topics", "*")
    calls = threading.Barrier(2)
    active = threading.Lock()
    unsubscribe = mocker.Mock()

    def subscribe(*args: Any) -> Callable[[], None]:
        assert active.acquire(blocking=False), "overlapping subscriptions"
        try:
            try:
                calls.wait(timeout=1.0)
            except threading.BrokenBarrierError:
                pass
            return cast("Callable[[], None]", unsubscribe)
        finally:
            active.release()

    shared = ZenohTransport[PoseStamped]("odom", PoseStamped)
    start = mocker.patch.object(shared.zenoh, "start")
    subscription = mocker.patch.object(shared.zenoh, "subscribe", side_effect=subscribe)
    with tap.recording({("odom", PoseStamped): shared, ("pose", PoseStamped): shared}):
        assert subscription.call_count == 2

    start.assert_called_once_with()
    assert unsubscribe.call_count == 2


@pytest.mark.parametrize("fail", [False, True])
def test_recording_bounds_zenoh_batches(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture, fail: bool
) -> None:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_topics", "*")
    monkeypatch.setattr(tap, "MAX_PARALLEL_TAPS", 2)
    transports = [ZenohTransport[PoseStamped](f"stream_{i}", PoseStamped) for i in range(5)]
    unsubscribe = mocker.Mock()
    subscriptions = []
    for transport in transports:
        mocker.patch.object(transport.zenoh, "start")
        subscriptions.append(
            mocker.patch.object(transport.zenoh, "subscribe", return_value=unsubscribe)
        )
    batches = mocker.spy(tap, "safe_thread_map")
    streams: dict[tuple[str, type], ZenohTransport[PoseStamped]] = {
        (f"stream_{i}", PoseStamped): tr for i, tr in enumerate(transports)
    }

    if fail:
        subscriptions[3].side_effect = RuntimeError("subscription failed")
        with pytest.raises(RuntimeError, match="subscription failed"):
            with tap.recording(streams):
                pytest.fail("startup must fail")
        assert [len(call.args[0]) for call in batches.call_args_list] == [2, 2]
        assert unsubscribe.call_count == 3
        subscriptions[4].assert_not_called()
    else:
        with tap.recording(streams):
            assert [subscription.call_count for subscription in subscriptions] == [1] * 5
        assert [len(call.args[0]) for call in batches.call_args_list] == [2, 2, 1]
        assert unsubscribe.call_count == 5


@pytest.mark.parametrize("already_started", [False, True])
@pytest.mark.parametrize("zenoh_failure", [False, True])
def test_failed_subscription_preserves_ros_lifecycle(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
    already_started: bool,
    zenoh_failure: bool,
) -> None:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_topics", "*")
    mocker.patch.object(rospubsub, "ROS_AVAILABLE", True)
    mocker.patch.object(rospubsub, "rclpy").ok.return_value = True
    mocker.patch.object(rospubsub, "Node")
    mocker.patch.object(
        rospubsub.DimosROS, "_to_raw_topic", return_value=rospubsub.RawROSTopic("/odom", object)
    )
    halted = threading.Event()
    executor = mocker.patch.object(rospubsub, "SingleThreadedExecutor").return_value
    executor.spin_once.side_effect = lambda timeout_sec: halted.wait(timeout_sec)
    executor.shutdown.side_effect = halted.set
    ros = ROSTransport("/odom", PoseStamped, qos=mocker.Mock())
    failing: Transport[Any] = _FailingTransport()
    if zenoh_failure:
        failing = ZenohTransport("bad", PoseStamped)
        mocker.patch.object(failing.zenoh, "start", side_effect=RuntimeError("subscription failed"))

    with ExitStack() as cleanup:
        cleanup.callback(ros.stop)
        if already_started:
            ros.start()
        threads_before = {t for t in threading.enumerate() if t.name == "ros_pubsub_spin"}
        with pytest.raises(RuntimeError, match="subscription failed"):
            with tap.recording({("bad", PoseStamped): failing, ("odom", PoseStamped): ros}):
                pytest.fail("startup must fail")
        assert {t for t in threading.enumerate() if t.name == "ros_pubsub_spin"} == threads_before
        executor.shutdown.assert_not_called()


def test_recording_cleans_up_after_partial_subscription_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_topics", "*")
    successful = _Transport()

    with pytest.raises(RuntimeError, match="subscription failed"):
        with tap.recording(
            {
                ("successful", PoseStamped): successful,
                ("failing", PoseStamped): _FailingTransport(),
            }
        ):
            pass

    assert not successful.callbacks
    assert not any(t.name == "record-writer" for t in threading.enumerate())


def test_failed_zenoh_batch_unsubscribes_and_does_not_start_next_backend(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_topics", "*")
    barrier = threading.Barrier(2)
    good, bad = [ZenohTransport[PoseStamped](name, PoseStamped) for name in ("good", "bad")]
    unsubscribe = mocker.Mock()

    def subscribe(*args: Any) -> Callable[[], None]:
        barrier.wait(timeout=2.0)
        return cast("Callable[[], None]", unsubscribe)

    def fail(*args: Any) -> Callable[[], None]:
        barrier.wait(timeout=2.0)
        raise RuntimeError("subscription failed")

    mocker.patch.object(good.zenoh, "start")
    mocker.patch.object(bad.zenoh, "start")
    mocker.patch.object(good.zenoh, "subscribe", side_effect=subscribe)
    mocker.patch.object(bad.zenoh, "subscribe", side_effect=fail)
    later = _Transport()
    later_subscribe = mocker.spy(later, "subscribe")

    with pytest.raises(RuntimeError, match="subscription failed"):
        with tap.recording(
            {("good", PoseStamped): good, ("bad", PoseStamped): bad, ("later", PoseStamped): later}
        ):
            pytest.fail("startup must fail")

    unsubscribe.assert_called_once_with()
    later_subscribe.assert_not_called()
    assert not any(t.name == "record-writer" for t in threading.enumerate())


def test_recording_cleans_up_after_tap_preparation_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_topics", "*")
    mocker.patch.object(
        TransportRecorder,
        "prepare_tap",
        side_effect=RuntimeError("prepare failed"),
    )

    with pytest.raises(RuntimeError, match="prepare failed"):
        with tap.recording({("odom", PoseStamped): _Transport()}):
            pass

    assert not any(t.name == "record-writer" for t in threading.enumerate())


def test_recording_delegates_to_rust_session(
    monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    from dimos.experimental.memory import rust_cli_recorder

    transport = _Transport()
    transports: dict[tuple[str, type], Transport[Any]] = {("odom", PoseStamped): transport}
    session = mocker.Mock()
    make_plan = mocker.patch.object(rust_cli_recorder, "make_plan", return_value="plan")
    create_session = mocker.patch.object(
        rust_cli_recorder, "RustRecordingSession", return_value=session
    )
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_engine", "rust")
    monkeypatch.setattr(global_config, "record_topics", "*")

    with tap.recording(transports):
        session.start.assert_called_once_with()

    make_plan.assert_called_once_with(transports)
    create_session.assert_called_once_with("plan")
    session.stop.assert_called_once_with()
