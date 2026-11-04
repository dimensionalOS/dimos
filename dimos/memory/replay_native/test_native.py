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

import pytest

from dimos.core.stream import Out
from dimos.memory.replay_native.native import ReplayNative, replay_native
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry


@pytest.fixture
def recording(tmp_path: Path) -> str:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    for ts in (1.0, 1.1, 1.2):
        store.stream("odom", PoseStamped).append(PoseStamped(ts=ts), ts=ts)
    store.stream("goal", PoseStamped).append(PoseStamped(ts=1.05), ts=1.05)
    store.stream("build", PoseStamped).append(PoseStamped(ts=1.0), ts=1.0)
    store.stop()
    return str(path)


def test_ports_come_from_the_recording(recording: str) -> None:
    # "build" clashes with a NativeModule method, so it is skipped like ReplayModule does.
    cls = replay_native(recording)
    assert cls.__annotations__ == {"goal": Out[PoseStamped], "odom": Out[PoseStamped]}
    assert issubclass(cls, ReplayNative)
    assert replay_native(recording, topics="odom").__annotations__ == {"odom": Out[PoseStamped]}
    assert replay_native("").__annotations__ == {}


def test_portless_class_gets_ports_and_filter_from_config(recording: str) -> None:
    assert replay_native(recording, topics="odom").blueprint().blueprints[0].kwargs == {
        "topics": "odom"
    }
    module = replay_native("")(db=recording, topics="odom")
    try:
        assert sorted(module.outputs) == ["odom"]
    finally:
        module.stop()


def test_stdin_config_names_the_ports_and_fills_unset_window(recording: str) -> None:
    module = replay_native(recording)(db=recording, speed=2.0)
    try:
        module.config.ports = sorted(module.outputs)
        config = module.config.to_config_dict()
    finally:
        module.stop()
    assert config == {
        "db": str(Path(recording).resolve()),
        "speed": 2.0,
        "loop": False,
        "exit_on_complete": False,
        "seek": 0.0,
        "duration": 0.0,
        "loop_gap": pytest.approx(0.05),
        "static_period": 1.0,
        "ports": ["goal", "odom"],
    }


class StaticReplay(ReplayNative):
    odom: Out[PoseStamped]
    wheel_odom: Out[Odometry]


def test_without_a_db_start_is_a_no_op_that_keeps_static_ports() -> None:
    module = StaticReplay()
    try:
        module.start()
        assert sorted(module.outputs) == ["odom", "wheel_odom"]
        assert module._process is None
    finally:
        module.stop()


def test_static_ports_sit_beside_the_recordings_streams(recording: str) -> None:
    # A declared port the recording lacks (wheel_odom) stays; the binary leaves it silent.
    module = StaticReplay(db=recording)
    try:
        assert sorted(module.outputs) == ["goal", "odom", "wheel_odom"]
        assert module.outputs["odom"].type is PoseStamped
    finally:
        module.stop()


def test_exit_on_complete_turns_a_clean_exit_into_shutdown(
    recording: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    import threading

    from dimos.memory.replay_native import native

    requested = threading.Event()
    monkeypatch.setattr(native, "_request_shutdown", requested.set)
    module = replay_native(recording)(db=recording, exit_on_complete=True)
    try:
        assert not module._exit_is_expected(1)
        assert not requested.is_set()
        assert module._exit_is_expected(0)
        assert requested.wait(5)
        module.config.loop = True
        assert not module._exit_is_expected(0)
    finally:
        module.stop()


def test_shutdown_signals_only_this_runs_coordinator(monkeypatch: pytest.MonkeyPatch) -> None:
    """Two runs can share a bus; the shutdown must reach this run's coordinator, by run id."""
    import signal
    from types import SimpleNamespace

    from dimos.core import run_registry
    from dimos.memory.replay_native import native

    runs = [SimpleNamespace(run_id="other", pid=111), SimpleNamespace(run_id="mine", pid=222)]
    monkeypatch.setattr(run_registry, "list_runs", lambda: runs)
    killed: list[tuple[int, int]] = []
    monkeypatch.setattr(native.os, "kill", lambda pid, sig: killed.append((pid, sig)))

    monkeypatch.setenv("DIMOS_RUN_ID", "mine")
    native._request_shutdown()
    assert killed == [(222, signal.SIGTERM)]

    monkeypatch.delenv("DIMOS_RUN_ID")
    native._request_shutdown()
    assert killed == [(222, signal.SIGTERM)], "no run id: nothing is signalled"
