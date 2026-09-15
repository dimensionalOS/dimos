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
from contextlib import ExitStack
from pathlib import Path
import sqlite3
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.evals.environments.sim import Sim
from dimos.memory.store.sqlite import SqliteStore


@pytest.fixture
def recording(tmp_path: Path) -> Iterator[SqliteStore]:
    with SqliteStore(path=str(tmp_path / "memory.db")) as store:
        yield store


@pytest.fixture
def launch(mocker: MockerFixture, recording: SqliteStore) -> MagicMock:
    process = mocker.patch("dimos.evals.environments.sim.DimosCliCall").return_value
    adapter = mocker.patch("dimos.evals.environments.sim.McpAdapter")
    adapter.return_value.wait_for_ready.return_value = True
    mocker.patch.object(Sim, "_wait_recording", return_value=Path(recording.config.path))
    return process


@pytest.fixture
def environment(launch: MagicMock) -> Iterator[Sim]:
    env = Sim(
        blueprint=["unitree-go2"],
        required_recording_streams=("lidar", "odom"),
        launch_timeout_s=1.0,
    )
    with ExitStack() as cleanup:
        cleanup.callback(env.stop)
        yield env


def test_start_waits_for_observations_in_every_required_stream(
    environment: Sim, recording: SqliteStore, mocker: MockerFixture
) -> None:
    elapsed = 0.0
    stages = []

    def advance(seconds: float) -> None:
        nonlocal elapsed
        elapsed += seconds
        stages.append(len(stages))
        if len(stages) == 1:
            recording.stream("lidar", int)
            recording.stream("odom", int)
        elif len(stages) == 2:
            recording.streams.lidar.append(1, ts=1.0)
        elif len(stages) == 3:
            recording.streams.odom.append(2, ts=1.0)
        else:
            pytest.fail("ready observations were not visible to the separate reader")

    mocker.patch(
        "dimos.evals.environments.sim.time",
        SimpleNamespace(monotonic=lambda: elapsed, sleep=advance),
    )

    running = environment.start(())

    assert stages == [0, 1, 2], "missing, empty, and partly populated stores must all wait"
    assert running.artifacts["recording"] == Path(recording.config.path)
    assert environment._recording is not None
    assert environment._recording.streams.lidar.last().data == 1
    assert environment._recording.streams.odom.last().data == 2


def test_start_returns_without_waiting_when_required_observations_exist(
    environment: Sim, recording: SqliteStore, mocker: MockerFixture
) -> None:
    recording.stream("lidar", int).append(1, ts=1.0)
    recording.stream("odom", int).append(2, ts=1.0)
    sleep = mocker.patch("dimos.evals.environments.sim.time.sleep")

    running = environment.start(())

    assert running.artifacts["recording"] == Path(recording.config.path)
    sleep.assert_not_called()


def test_start_without_required_streams_accepts_an_empty_recording(
    launch: MagicMock, recording: SqliteStore, mocker: MockerFixture
) -> None:
    sleep = mocker.patch("dimos.evals.environments.sim.time.sleep")
    env = Sim(blueprint=["unitree-go2"])
    with ExitStack() as cleanup:
        cleanup.callback(env.stop)
        running = env.start(())

    assert running.artifacts["recording"] == Path(recording.config.path)
    sleep.assert_not_called()


def test_missing_observations_use_the_remaining_launch_budget_and_allow_cleanup(
    launch: MagicMock, recording: SqliteStore, mocker: MockerFixture
) -> None:
    recording.stream("odom", int).append(2, ts=1.0)
    recording.stream("lidar", int)
    elapsed = 0.0
    sleeps = []

    def become_mcp_ready(**kwargs: float) -> bool:
        nonlocal elapsed
        elapsed = 0.73
        return True

    def advance(seconds: float) -> None:
        nonlocal elapsed
        sleeps.append(seconds)
        elapsed += seconds

    adapter = mocker.patch("dimos.evals.environments.sim.McpAdapter")
    adapter.return_value.wait_for_ready.side_effect = become_mcp_ready
    mocker.patch(
        "dimos.evals.environments.sim.time",
        SimpleNamespace(monotonic=lambda: elapsed, sleep=advance),
    )
    env = Sim(
        blueprint=["unitree-go2"],
        required_recording_streams=("lidar", "odom"),
        launch_timeout_s=1.0,
    )
    with ExitStack() as cleanup:
        cleanup.callback(env.stop)
        with pytest.raises(TimeoutError, match=r"required recording streams \['lidar'\]"):
            env.start(())
        reader = env._recording
        assert reader is not None

    assert sleeps == pytest.approx([0.1, 0.1, 0.07])
    assert elapsed == pytest.approx(1.0)
    assert env._recording is None
    launch.stop.assert_called_once_with()
    with pytest.raises(sqlite3.ProgrammingError, match="closed"):
        reader.list_streams()
