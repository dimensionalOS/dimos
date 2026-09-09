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

"""Service-owned robot operation contracts, without hardware or network."""

import asyncio
import importlib.util
import json
from pathlib import Path
from unittest.mock import Mock

import pytest
import pytest_asyncio

_SPEC = importlib.util.spec_from_file_location(
    "frank_runtime", Path(__file__).with_name("runtime.py")
)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
Runtime = _MODULE.Runtime


def handle(result):
    return json.loads(result["content"][0]["text"])["operation_id"]


@pytest_asyncio.fixture
async def runtime(mocker):
    rt = Runtime(Mock())
    rt.connected = True
    mocker.patch.object(rt, "command", return_value={"content": [{"type": "text", "text": "ok"}]})
    try:
        yield rt
    finally:
        await rt.stop()


@pytest.mark.asyncio
async def test_handle_returns_before_completion_and_callback_has_same_id(runtime, mocker):
    released = asyncio.Event()
    entered = asyncio.Event()

    async def command(*args):
        entered.set()
        await released.wait()
        return {"content": [{"type": "text", "text": "Navigation goal reached"}]}

    mocker.patch.object(runtime, "command", side_effect=command)
    token = handle(await runtime.call("move_to", {"degrees": -20, "relative": True}))
    await asyncio.wait_for(entered.wait(), 1)
    assert runtime.status(token)["status"] == "running"
    runtime.bus.publish.assert_not_called()
    task = runtime.command_tasks[token]
    released.set()
    await asyncio.wait_for(task, 1)
    assert runtime.status(token)["status"] == "succeeded"
    event = runtime.bus.publish.call_args.args[0]
    assert (event["operation_id"], event["status"]) == (token, "succeeded")


@pytest.mark.asyncio
async def test_cancellation_stops_navigation_and_ignores_late_result(runtime, mocker):
    entered = asyncio.Event()

    async def command(name, *args):
        if name == "move_to":
            entered.set()
            await asyncio.Event().wait()
        return {"content": []}

    command_mock = mocker.patch.object(runtime, "command", side_effect=command)
    token = handle(await runtime.call("move_to", {"x": 1}))
    await asyncio.wait_for(entered.wait(), 1)
    assert (await runtime.cancel_operation(token))["status"] == "cancelled"
    command_mock.assert_any_await("stop_navigation", {}, token)
    runtime.finish(token, False, '{"content":[]}')
    assert runtime.status(token)["status"] == "cancelled"
    assert runtime.bus.publish.call_count == 1


@pytest.mark.asyncio
async def test_failure_is_a_terminal_event(runtime, mocker):
    error = {"isError": True, "content": [{"type": "text", "text": "Blocked"}]}
    mocker.patch.object(runtime, "command", return_value=error)
    token = handle(await runtime.call("navigate_with_text", {"query": "door"}))
    await asyncio.wait_for(runtime.command_tasks[token], 1)
    event = runtime.bus.publish.call_args.args[0]
    assert event["status"] == "failed"
    assert event["result"] == error


@pytest.mark.asyncio
async def test_observe_returns_image_without_spurious_callback(runtime, mocker):
    image = {"content": [{"type": "image", "data": "YWJj", "mimeType": "image/jpeg"}]}
    mocker.patch.object(runtime, "command", return_value=image)
    assert await runtime.call("observe", {}) == image
    runtime.bus.publish.assert_not_called()


@pytest.mark.asyncio
async def test_background_survives_call_and_another_chat_tool(runtime):
    token = handle(await runtime.call("begin_exploration", {}))
    await asyncio.wait_for(runtime.command_tasks[token], 1)
    await runtime.call("observe", {})
    assert runtime.status(token)["status"] == "running"
    runtime.notification({"method": "dimos/tool_stopped", "params": {"progressToken": token}})
    assert runtime.status(token)["status"] == "stopped"


@pytest.mark.asyncio
async def test_second_movement_is_rejected_until_cancelled(runtime):
    token = handle(await runtime.call("begin_exploration", {}))
    with pytest.raises(RuntimeError, match="active"):
        await runtime.call("move_to", {"x": 2})
    await runtime.cancel_operation(token)
    new_token = handle(await runtime.call("move_to", {"x": 2}))
    assert new_token != token


@pytest.mark.asyncio
async def test_reset_clears_handles_and_rejects_old_callbacks(runtime, mocker):
    mocker.patch.object(runtime, "start")
    token = handle(await runtime.call("begin_exploration", {}))
    await asyncio.wait_for(runtime.command_tasks[token], 1)
    await runtime.reset()
    with pytest.raises(KeyError):
        runtime.status(token)
    runtime.bus.publish.reset_mock()
    runtime.notification(
        {"method": "notifications/progress", "params": {"progressToken": token, "message": "old"}}
    )
    runtime.bus.publish.assert_not_called()
