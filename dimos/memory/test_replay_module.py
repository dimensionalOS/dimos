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

from pathlib import Path
import threading
import time
from typing import Any
from unittest import mock

import pytest
from pytest_mock import MockerFixture
from reactivex.disposable import Disposable
from reactivex.testing import TestScheduler

from dimos.core.stream import Out
from dimos.memory.replay import ReplayStream
from dimos.memory.replay_module import replay_module
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


@pytest.fixture
def recording(tmp_path: Path) -> str:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    odom = store.stream("odom", PoseStamped)
    goal = store.stream("goal", PoseStamped)
    for ts in (1.0, 1.1, 1.2):
        odom.append(PoseStamped(ts=ts), ts=ts)
    goal.append(PoseStamped(ts=1.05), ts=1.05)
    store.stop()
    return str(path)


def test_ports_come_from_the_recording(recording: str) -> None:
    cls = replay_module(recording)
    assert cls.__annotations__ == {"goal": Out[PoseStamped], "odom": Out[PoseStamped]}
    assert replay_module(recording, topics="odom").__annotations__ == {"odom": Out[PoseStamped]}
    assert cls.stream_types == {"goal": PoseStamped, "odom": PoseStamped}
    assert replay_module("").__annotations__ == {}
    with pytest.raises(ValueError, match="matched none of: goal, odom"):
        replay_module(recording, topics="nope")


def test_module_publishes_recorded_messages(recording: str) -> None:
    cls = replay_module(recording)
    module = cls(dataset=recording)
    got: dict[str, list[float]] = {"odom": [], "goal": []}
    module.outputs["odom"].subscribe(lambda m: got["odom"].append(m.ts))
    module.outputs["goal"].subscribe(lambda m: got["goal"].append(m.ts))
    module.start()
    deadline = time.time() + 5
    while time.time() < deadline and len(got["odom"]) < 3:
        time.sleep(0.01)
    module.stop()
    assert [round(t, 3) for t in got["odom"]] == [1.0, 1.1, 1.2]
    assert [round(t, 3) for t in got["goal"]] == [1.05]


def test_stop_waits_for_in_flight_decode(recording: str, monkeypatch: pytest.MonkeyPatch) -> None:
    entered = threading.Event()
    release = threading.Event()
    decoded = threading.Event()
    stopping = threading.Event()
    subscriptions_disposed = threading.Event()
    stopped = threading.Event()
    errors: list[Exception] = []
    decode = ReplayStream._decode

    def held_decode(self: ReplayStream[Any], obs: Any) -> Any:
        entered.set()
        try:
            assert release.wait(5), "decode was not released"
            data = decode(self, obs)
            decoded.set()
            return data
        except Exception as exc:
            errors.append(exc)
            raise

    monkeypatch.setattr(ReplayStream, "_decode", held_decode)
    module = replay_module(recording, topics="odom")(dataset=recording, topics="odom")
    module.register_disposable(Disposable(stopping.set))

    def stop() -> None:
        try:
            module.stop()
        except Exception as exc:
            errors.append(exc)
        finally:
            stopped.set()

    thread = threading.Thread(target=stop)
    try:
        module.start()
        module.register_disposable(Disposable(subscriptions_disposed.set))
        assert entered.wait(5), "no emission reached decode"
        thread.start()
        assert stopping.wait(5), "stop did not reach subscription disposal"
        assert not subscriptions_disposed.wait(0.1), "disposal did not wait for the active decode"
    finally:
        release.set()
        if thread.ident is not None:
            thread.join(timeout=5)
        module.stop()

    assert stopped.is_set()
    assert subscriptions_disposed.is_set()
    assert decoded.is_set()
    assert errors == []
    with SqliteStore(path=recording) as store:
        assert [obs.data.ts for obs in store.stream("odom")] == pytest.approx([1.0, 1.1, 1.2])


def test_portless_class_gets_ports_from_dataset(recording: str) -> None:
    """A worker imports the class without a recording; the instance still has the ports."""
    module = replay_module("")(dataset=recording)
    assert sorted(module.outputs) == ["goal", "odom"]
    module.stop()


def test_topic_filter_reaches_the_instance(recording: str) -> None:
    """Workers rebuild ports from config, so the filter must travel in the blueprint kwargs."""
    cls = replay_module(recording, topics="odom")
    assert cls.blueprint(dataset=recording).blueprints[0].kwargs["topics"] == "odom"
    module = replay_module("")(dataset=recording, topics="odom")
    assert sorted(module.outputs) == ["odom"]
    module.stop()


def test_stream_named_like_a_module_attribute_is_skipped(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from unittest.mock import MagicMock

    from dimos.memory import replay_module as rm

    log = MagicMock()
    monkeypatch.setattr(rm, "logger", log)
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    store.stream("start", PoseStamped).append(PoseStamped(ts=1.0), ts=1.0)
    store.stream("odom", PoseStamped).append(PoseStamped(ts=1.0), ts=1.0)
    store.stop()
    assert list(replay_module(str(path)).__annotations__) == ["odom"]
    module = replay_module("")(dataset=str(path))
    assert sorted(module.outputs) == ["odom"]
    module.stop()
    assert [c.args[1] for c in log.warning.call_args_list] == ["start", "start"]


def test_subclass_payload_gets_the_base_port_type(tmp_path: Path) -> None:
    """A recorded unitree Odometry is a PoseStamped on the wire; the port must say so or a
    PoseStamped consumer of the same name lands on a different topic."""
    from dimos.robot.unitree.type.odometry import Odometry

    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    store.stream("odom", Odometry).append(Odometry(ts=1.0), ts=1.0)
    store.stop()
    assert replay_module(str(path)).__annotations__ == {"odom": Out[PoseStamped]}


def _publications(module: Any) -> list[tuple[str, Any]]:
    """Every message the module publishes, in order, with its port name."""
    published: list[tuple[str, Any]] = []
    for name, port in module.outputs.items():
        port.subscribe(lambda msg, name=name: published.append((name, msg)))
    return published


def _replay_to_the_end(module: Any, mocker: MockerFixture, wait_until: Any) -> None:
    log = mocker.patch("dimos.memory.replay_progress.logger")
    try:
        module.start()
        wait_until(
            lambda: any(c.args[:1] == ("Replay finished",) for c in log.info.mock_calls), timeout=5
        )
    finally:
        module.stop()


def _tf_recording(path: Path) -> None:
    """A moving world -> base edge on tf, and base -> camera / lidar mounts on tf_static."""
    with SqliteStore(path=str(path)) as store:
        tf = store.stream("tf", TFMessage)
        for ts in (1000.0, 1001.0):
            tf.append(TFMessage(Transform(frame_id="world", child_frame_id="base", ts=ts)), ts=ts)
        tf_static = store.stream("tf_static", TFMessage)
        for child, ts in (("camera", 1000.25), ("lidar", 1000.5)):
            transform = Transform(frame_id="base", child_frame_id=child, ts=ts)
            tf_static.append(TFMessage(transform), ts=ts)


def test_static_tf_replays_as_recorded(tmp_path: Path, wait_until: Any) -> None:
    path = tmp_path / "memory.db"
    _tf_recording(path)
    # Slow enough that the 0.25 s gaps between streams keep their order.
    module = replay_module(str(path))(dataset=str(path), loop=True, speed=1.0)
    published = _publications(module)
    try:
        module.start()
        wait_until(lambda: len(published) >= 8, timeout=5)
    finally:
        module.stop()
    edges = [(name, [t.child_frame_id for t in msg.transforms]) for name, msg in published]
    one_pass = [
        ("tf", ["base"]),
        ("tf_static", ["camera"]),
        ("tf_static", ["lidar"]),
        ("tf", ["base"]),
    ]
    assert edges[:8] == one_pass * 2


@pytest.mark.parametrize(
    ("seek", "duration", "expected_odom", "expected_tf"),
    [
        (None, 1.0, [999.5, 1000.0], [1000.0]),
        (0.5, None, [1000.0, 1001.0], [1000.0, 1001.0]),
        (990.0, 1.0, [], []),
    ],
)
def test_window_is_measured_from_the_first_timed_row(
    tmp_path: Path,
    mocker: MockerFixture,
    wait_until: Any,
    seek: float | None,
    duration: float | None,
    expected_odom: list[float],
    expected_tf: list[float],
) -> None:
    # Odometry starts half a second before the moving tf.
    path = tmp_path / "memory.db"
    _tf_recording(path)
    with SqliteStore(path=str(path)) as store:
        for ts in (999.5, 1000.0, 1001.0):
            store.stream("odom", PoseStamped).append(PoseStamped(ts=ts), ts=ts)
    module = replay_module(str(path))(dataset=str(path), seek=seek, duration=duration, speed=20.0)
    published = _publications(module)
    _replay_to_the_end(module, mocker, wait_until)
    assert [msg.ts for name, msg in published if name == "odom"] == expected_odom
    tf = [t for name, msg in published if name == "tf" for t in msg.transforms]
    assert [t.ts for t in tf] == expected_tf


def test_seek_on_late_topic_uses_recording_origin(
    tmp_path: Path, mocker: MockerFixture, wait_until: Any
) -> None:
    path = tmp_path / "memory.db"
    _tf_recording(path)
    with SqliteStore(path=str(path)) as store:
        for ts in (1010.0, 1010.25):
            store.stream("late", PoseStamped).append(PoseStamped(ts=ts), ts=ts)
    cls = replay_module(str(path), topics="late")
    module = cls(dataset=str(path), topics="late", seek=10.0, duration=0.5, speed=20.0)
    published = _publications(module)
    _replay_to_the_end(module, mocker, wait_until)
    assert [msg.ts for _, msg in published] == [1010.0, 1010.25]


def test_slow_first_decode_does_not_skip_other_streams(
    recording: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Anchor is pinned after setup; a stream that is slow to prime must not make later streams late."""
    from dimos.memory import replay as replay_mod

    real = replay_mod.ReplayStream._decode

    def slow(self: Any, obs: Any) -> Any:
        if obs.ts == 1.05:  # the goal frame, subscribed first (sorted order)
            time.sleep(0.2)
        return real(self, obs)

    monkeypatch.setattr(replay_mod.ReplayStream, "_decode", slow)
    module = replay_module(recording)(dataset=recording)
    got: dict[str, list[float]] = {"odom": [], "goal": []}
    module.outputs["odom"].subscribe(lambda m: got["odom"].append(m.ts))
    module.outputs["goal"].subscribe(lambda m: got["goal"].append(m.ts))
    module.start()
    deadline = time.time() + 5
    while time.time() < deadline and (len(got["odom"]) < 3 or not got["goal"]):
        time.sleep(0.01)
    module.stop()
    assert [round(t, 3) for t in got["odom"]] == [1.0, 1.1, 1.2]


def test_slow_stream_setup_does_not_skip_the_first_frames(recording: str) -> None:
    """Slow setup should not lose initial frames."""
    real = ReplayStream._base_stream

    def slow(self: Any) -> Any:
        time.sleep(0.1)  # twice the late tolerance, on every query a subscribe makes
        return real(self)

    got: dict[str, list[float]] = {"odom": [], "goal": []}
    with mock.patch("dimos.memory.replay.ReplayStream._base_stream", slow):
        module = replay_module(recording)(dataset=recording)
        module.outputs["odom"].subscribe(lambda m: got["odom"].append(m.ts))
        module.outputs["goal"].subscribe(lambda m: got["goal"].append(m.ts))
        module.start()
    deadline = time.time() + 5
    while time.time() < deadline and (len(got["odom"]) < 3 or not got["goal"]):
        time.sleep(0.01)
    module.stop()
    assert [round(t, 3) for t in got["odom"]] == [1.0, 1.1, 1.2]
    assert [round(t, 3) for t in got["goal"]] == [1.05]


@pytest.mark.parametrize(
    ("topics", "expected"),
    [
        ("odom", [(0.0, "odom", 1.0), (0.1, "odom", 1.1), (0.2, "odom", 1.2)]),
        (
            "*",
            [
                (0.0, "odom", 1.0),
                (0.05, "goal", 1.05),
                (0.1, "odom", 1.1),
                (0.2, "odom", 1.2),
            ],
        ),
    ],
)
def test_slow_cursor_open_anchors_after_all_streams_are_ready(
    recording: str,
    mocker: MockerFixture,
    topics: str,
    expected: list[tuple[float, str, float]],
) -> None:
    scheduler = TestScheduler()
    mocker.patch("dimos.memory.replay.TimeoutScheduler", return_value=scheduler)
    mocker.patch("dimos.memory.replay.time.time", side_effect=lambda: scheduler.clock)
    base_stream = ReplayStream._base_stream

    def slow_open(self: ReplayStream[Any]) -> Any:
        scheduler.advance_by(0.15)
        return base_stream(self)

    mocker.patch.object(ReplayStream, "_base_stream", slow_open)
    module = replay_module(recording, topics=topics)(dataset=recording, topics=topics)
    events: list[tuple[float, str, float]] = []

    def record(name: str) -> Any:
        return lambda msg: events.append((scheduler.clock, name, msg.ts))

    for name, port in module.outputs.items():
        port.subscribe(record(name))
    try:
        module.start()
        start = scheduler.clock
        assert events == []
        scheduler.advance_by(1.0)
    finally:
        module.stop()
    assert [(round(t - start, 3), name, round(ts, 3)) for t, name, ts in events] == expected


def test_recorded_rerun_config_from_run_dir(tmp_path: Path) -> None:
    from dimos.memory.replay_module import recorded_rerun_config

    db = tmp_path / "20260910-135235-unitree-go2" / "memory.db"
    cfg = recorded_rerun_config(str(db))
    assert "world/robot_body" in cfg["static"]
    assert callable(cfg["blueprint"])  # the live layout, so replay looks like the live run
    assert recorded_rerun_config(str(tmp_path / "downloads" / "x.db")) == {}
    assert recorded_rerun_config(str(tmp_path / "20260910-135235-no-such-bp" / "m.db")) == {}


def test_dataset_path_resolves_names_and_skips_the_default(recording: str) -> None:
    from dimos.memory.replay_module import dataset_path

    assert dataset_path(recording) == recording
    assert dataset_path(recording, explicit=False) == recording  # a real file always counts
    assert dataset_path("go2_short", explicit=False) == ""  # registry import: no LFS
    assert dataset_path("no-such-dataset-for-this-test") == ""


def test_replay_logs_source_and_finishes_after_all_timed_streams(recording, mocker):
    scheduler = TestScheduler()
    mocker.patch("dimos.memory.replay.TimeoutScheduler", return_value=scheduler)
    log = mocker.patch("dimos.memory.replay_progress.logger")
    module = replay_module(recording)(dataset=recording)
    try:
        module.start()
        log.info.assert_called_once_with(
            "Replay started",
            source=recording,
            path=recording,
            duration_s=pytest.approx(0.2),
            speed=1.0,
            loop=False,
        )
        scheduler.advance_by(0.1)  # goal has ended; odom still has a frame to publish
        assert log.info.call_count == 1
        scheduler.advance_by(1)
        assert log.info.call_args_list[1:] == [mocker.call("Replay finished", source=recording)]
    finally:
        module.stop()
