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

from collections.abc import Callable, Coroutine, Iterator
from pathlib import Path
from typing import Any

import pytest
import pytest_mock
import reactivex as rx

from dimos.memory2.module import Recorder
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.pubsubrpc import LCMRPC

RecordHandler = Callable[[tuple[float, Any]], Coroutine[Any, Any, None]]


@pytest.fixture
def recorder(
    mocker: pytest_mock.MockerFixture,
    tmp_path: Path,
) -> Iterator[Recorder]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    mocker.patch.object(LCMRPC, "__init__", return_value=None)
    mocker.patch.object(LCMRPC, "serve_module_rpc", return_value=None)
    mocker.patch.object(LCMRPC, "start", return_value=None)
    mocker.patch.object(LCMRPC, "stop", return_value=None)
    module = Recorder(db_path=tmp_path / "recording.db")
    yield module
    module.stop()


def _capture_handler(
    recorder: Recorder,
    mocker: pytest_mock.MockerFixture,
) -> tuple[RecordHandler, Any]:
    handlers: list[RecordHandler] = []

    def capture(_observable: object, handler: RecordHandler) -> None:
        handlers.append(handler)

    mocker.patch.object(recorder, "process_observable", side_effect=capture)
    input_topic = mocker.MagicMock()
    input_topic.pure_observable.return_value = rx.never()
    stream = mocker.MagicMock()

    recorder._port_to_stream("joint_state", input_topic, stream)

    assert len(handlers) == 1
    return handlers[0], stream


@pytest.mark.asyncio
async def test_record_callback_drops_message_after_shutdown_begins(
    recorder: Recorder,
    mocker: pytest_mock.MockerFixture,
) -> None:
    handler, stream = _capture_handler(recorder, mocker)
    recorder._closing.set()

    await handler((1.0, JointState(ts=1.0)))

    stream.append.assert_not_called()


@pytest.mark.asyncio
async def test_record_callback_rate_limits_missing_pose_warning(
    recorder: Recorder,
    mocker: pytest_mock.MockerFixture,
) -> None:
    handler, stream = _capture_handler(recorder, mocker)
    mocker.patch.object(recorder, "_resolve_pose", new=mocker.AsyncMock(return_value=None))
    warning = mocker.patch("dimos.memory2.module.logger.warning")

    for timestamp in (1.0, 2.0, 3.0):
        await handler((timestamp, JointState(ts=timestamp)))

    assert stream.append.call_count == 3
    warning.assert_called_once()
