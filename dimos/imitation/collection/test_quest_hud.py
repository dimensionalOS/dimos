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

import asyncio
from collections.abc import Iterator
import json

import pytest
import pytest_mock

from dimos.imitation.collection.episode_monitor import EpisodeStatus
from dimos.imitation.collection.quest_hud import QuestCollectionTeleopModule


@pytest.fixture
def module() -> Iterator[QuestCollectionTeleopModule]:
    module = QuestCollectionTeleopModule()
    try:
        yield module
    finally:
        module.stop()


def _status() -> EpisodeStatus:
    return EpisodeStatus(
        ts=123.0,
        state="recording",
        episodes_saved=12,
        episodes_discarded=1,
        last_event="start",
        task_label="Pick up red mug",
    )


def test_episode_status_is_cached_and_broadcast(
    module: QuestCollectionTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    broadcast = mocker.patch.object(module, "_broadcast_text")
    mocker.patch("dimos.imitation.collection.quest_hud.time.time", return_value=165.5)

    assert module._loop is not None
    asyncio.run_coroutine_threadsafe(module.handle_status(_status()), module._loop).result(
        timeout=5
    )

    assert module._latest_episode_status == _status()
    payload = json.loads(broadcast.call_args.args[0])
    assert payload == {
        "type": "episode_status",
        "elapsed_s": 42.5,
        "ts": 123.0,
        "state": "recording",
        "episodes_saved": 12,
        "episodes_discarded": 1,
        "last_event": "start",
        "task_label": "Pick up red mug",
    }


def test_connected_client_receives_latest_episode_status(
    module: QuestCollectionTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    module._latest_episode_status = _status()
    broadcast = mocker.patch.object(module, "_broadcast_text")

    assert module._client_connected(mocker.MagicMock()) is True

    payload = json.loads(broadcast.call_args.args[0])
    assert payload["type"] == "episode_status"
    assert payload["episodes_saved"] == 12


def test_connected_client_receives_ready_status_before_monitor_update(
    module: QuestCollectionTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    broadcast = mocker.patch.object(module, "_broadcast_text")

    assert module._client_connected(mocker.MagicMock()) is True

    payload = json.loads(broadcast.call_args.args[0])
    assert payload["state"] == "idle"
    assert payload["episodes_saved"] == 0
    assert payload["episodes_discarded"] == 0
    assert payload["last_event"] == "init"
