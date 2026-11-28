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

from collections.abc import Iterator
from pathlib import Path
import threading
import time
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.constants import HUNG_TEST_TIMEOUT
from dimos.memory.replay_module import ReplayModule, replay_module
from dimos.memory.store.sqlite import SqliteStore

pytestmark = [
    pytest.mark.filterwarnings("error::pytest.PytestUnhandledThreadExceptionWarning"),
    pytest.mark.filterwarnings("error::pytest.PytestUnraisableExceptionWarning"),
]

# Recorded seconds; odom and lidar interleave, camera starts first.
_RECORDING = {
    "odom": [1.00, 1.02, 1.04],
    "lidar": [1.01, 1.03],
    "camera": [0.99, 1.05],
}


@pytest.fixture
def recording(tmp_path: Path) -> str:
    path = tmp_path / "replay.db"
    # A page per row forces each advance to read SQLite again.
    with SqliteStore(path=str(path), page_size=1) as store:
        for name, timestamps in _RECORDING.items():
            stream = store.stream(name, float)
            for ts in timestamps:
                stream.append(ts, ts=ts)
    return str(path)


@pytest.fixture
def play(recording: str) -> Iterator[Any]:
    """Start a replay of *recording*; every port's frames go to one (port, frame) list."""
    modules: list[ReplayModule] = []

    def start(topics: str = "*", **config: Any) -> tuple[ReplayModule, list[tuple[str, float]]]:
        module = replay_module(recording, topics)(dataset=recording, topics=topics, **config)
        modules.append(module)
        published: list[tuple[str, float]] = []
        for name, port in module.outputs.items():
            port.subscribe(lambda frame, name=name: published.append((name, frame)))
        module.start()
        return module, published

    yield start
    for module in modules:
        module.stop()


def test_streams_publish_in_recorded_order(play: Any) -> None:
    module, published = play(speed=10.0)
    module.on_system_modules_ready([])
    assert module.replay_done.wait(HUNG_TEST_TIMEOUT)

    expected = sorted((ts, name) for name, stamps in _RECORDING.items() for ts in stamps)
    assert published == [(name, ts) for ts, name in expected]


def test_nothing_publishes_before_the_clock_starts(play: Any) -> None:
    module, published = play()
    assert published == []
    module.on_system_modules_ready([])
    assert module.replay_done.wait(HUNG_TEST_TIMEOUT)
    assert len(published) == sum(map(len, _RECORDING.values()))


def test_no_frame_publishes_before_its_recorded_offset(play: Any) -> None:
    module, published = play(topics="odom", speed=0.5)
    arrivals: list[float] = []
    module.outputs["odom"].subscribe(lambda _frame: arrivals.append(time.monotonic()))
    started = time.monotonic()
    module.on_system_modules_ready([])
    assert module.replay_done.wait(HUNG_TEST_TIMEOUT)

    # At half speed the 20 ms recorded gaps take 40 ms. Timers only ever wake late.
    assert arrivals[1] - started >= 0.04
    assert arrivals[2] - started >= 0.08


def test_slow_subscriber_drops_nothing_and_warns_once(play: Any, mocker: MockerFixture) -> None:
    log = mocker.patch("dimos.memory.replay_module.logger")
    module, published = play(topics="odom", lag_warning=0.05)
    module.outputs["odom"].subscribe(lambda _frame: time.sleep(0.1))  # twice the lag warning
    module.on_system_modules_ready([])
    assert module.replay_done.wait(HUNG_TEST_TIMEOUT)

    assert [frame for _, frame in published] == _RECORDING["odom"]
    assert log.warning.call_count == 1
    assert log.warning.call_args.args == ("Replay is behind schedule",)


def test_loop_replays_in_order_after_the_gap(play: Any) -> None:
    module, published = play(topics="odom", loop=True, loop_gap=0.3)
    arrivals: list[float] = []
    second_pass = threading.Event()

    def on_frame(_frame: float) -> None:
        arrivals.append(time.monotonic())
        if len(arrivals) == 2 * len(_RECORDING["odom"]):
            second_pass.set()

    module.outputs["odom"].subscribe(on_frame)
    started = time.monotonic()
    module.on_system_modules_ready([])
    assert second_pass.wait(HUNG_TEST_TIMEOUT)

    assert [frame for _, frame in published[:6]] == [*_RECORDING["odom"], *_RECORDING["odom"]]
    # The pass's last frame is due 0.04 s in and the next pass 0.3 s after it. Timers only wake late.
    assert arrivals[3] - started >= 0.04 + 0.3


def test_stop_from_a_callback_closes_the_recording(play: Any) -> None:
    module, published = play(topics="odom")
    stopped = threading.Event()

    def stop_on_frame(_frame: float) -> None:
        module.stop()
        stopped.set()

    module.outputs["odom"].subscribe(stop_on_frame)
    module.on_system_modules_ready([])
    assert stopped.wait(HUNG_TEST_TIMEOUT)
    assert published == [("odom", _RECORDING["odom"][0])]
    assert module._store is None


def test_failing_subscriber_leaves_the_other_streams_playing(
    play: Any, mocker: MockerFixture
) -> None:
    log = mocker.patch("dimos.memory.replay_module.logger")
    module, published = play(speed=10.0)

    def broken(_frame: float) -> None:
        raise RuntimeError("subscriber bug")

    module.outputs["lidar"].subscribe(broken)
    module.on_system_modules_ready([])
    assert module.replay_done.wait(HUNG_TEST_TIMEOUT)

    assert [frame for name, frame in published if name == "odom"] == _RECORDING["odom"]
    assert log.exception.call_count == len(_RECORDING["lidar"])
