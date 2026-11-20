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
from datetime import datetime, timedelta
from typing import Any

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
import pytest
from pytest_mock import MockerFixture
from textual.widgets import Static

from dimos.agents.mcp import tool_stream
from dimos.agents.mcp.constants import TOOL_ORIGIN_KEY
from dimos.cli.human import humancli
from dimos.cli.human.humancli import HumanCLIApp
from dimos.utils.asyncio_timeout import timeout

# What the OpenAI Responses API path produces: blocks, never a plain string.
RESPONSES_CONTENT: list[Any] = [
    {"type": "reasoning", "id": "rs_1", "summary": [{"type": "summary_text", "text": "hmm"}]},
    {"type": "text", "text": "On my way.", "annotations": [], "id": "msg_1"},
    {"type": "function_call", "call_id": "c1", "name": "move_to", "arguments": "{}"},
]


class FakeTransport:
    def __init__(self) -> None:
        self.callback: Any = None

    def subscribe(self, callback: Any) -> Any:
        self.callback = callback
        return lambda: None

    def publish(self, msg: Any) -> None:
        pass


@pytest.fixture
def transports(monkeypatch: pytest.MonkeyPatch) -> dict[str, FakeTransport]:
    transports: dict[str, FakeTransport] = {}

    def fake_agent_transports() -> dict[str, FakeTransport]:
        for topic in ("/human_input", "/agent", "/agent_idle"):
            transports[topic] = FakeTransport()
        return {
            "agent_input": transports["/human_input"],
            "agent": transports["/agent"],
            "agent_idle": transports["/agent_idle"],
        }

    monkeypatch.setattr(humancli, "agent_transports", fake_agent_transports)
    transports["tool_stream"] = FakeTransport()
    monkeypatch.setattr(tool_stream, "subscribe", transports["tool_stream"].subscribe)
    return transports


async def test_block_content_messages_render(transports: dict[str, FakeTransport]) -> None:
    app = HumanCLIApp()
    async with app.run_test(size=(120, 40)) as pilot:
        while transports["/agent"].callback is None:
            await pilot.pause(0.05)
        receive = transports["/agent"].callback
        loop = asyncio.get_running_loop()

        messages = [
            HumanMessage(content="go to the kitchen"),
            AIMessage(
                content=RESPONSES_CONTENT, tool_calls=[{"name": "move_to", "args": {}, "id": "c1"}]
            ),
            ToolMessage(content=[{"type": "text", "text": "arrived"}], tool_call_id="c1"),
            AIMessage(content=[{"type": "text", "text": "Done."}]),
            # A tool's image artefact: a user turn for the model, a tool line here.
            HumanMessage(
                content=[
                    {"type": "text", "text": "This is the artefact for the 'observe' tool."},
                    {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,AAAA"}},
                ],
                additional_kwargs={TOOL_ORIGIN_KEY: "observe"},
            ),
        ]
        for msg in messages:
            # The real callback runs on the transport thread, never the app's.
            await loop.run_in_executor(None, receive, msg)
        await pilot.pause(0.1)

        assert app.chat_log is not None
        text = "\n".join(strip.text for strip in app.chat_log.lines)
        assert "go to the kitchen" in text
        assert "On my way." in text
        assert "▶ move_to({})" in text
        assert "↳ arrived" in text
        assert "Done." in text
        assert "tool │ ↳ This is the artefact for the 'observe' tool." in text
        assert "human │ This is the artefact" not in text
        assert "could not render" not in text


async def test_tool_panel_elapsed_ticks_without_messages(
    transports: dict[str, FakeTransport], mocker: MockerFixture
) -> None:
    start = datetime(2026, 11, 16, 10, 48, 0)
    clock = mocker.patch.object(humancli, "datetime")
    clock.now.return_value = start
    app = HumanCLIApp()
    async with app.run_test(size=(160, 40)) as pilot:
        async with timeout(5):
            while transports["/agent"].callback is None:
                await pilot.pause(0.01)
        for name in ("start_patrol", "lookout"):
            await asyncio.to_thread(
                transports["/agent"].callback,
                HumanMessage(content=f"[tool:{name}] Started"),
            )
        await pilot.pause()
        panels = list(app.query("#tool-panels Static"))
        assert len(panels) == 2

        for seconds in (1, 2):
            clock.now.return_value = start + timedelta(seconds=seconds)
            async with timeout(2):
                while not all(
                    f"running  00:0{seconds}" in panel.render_line(0).text for panel in panels
                ):
                    await pilot.pause(0.01)
            for panel in panels:
                assert f"running  00:0{seconds}" in panel.render_line(0).text

        assert app.chat_log is not None
        app.chat_log.clear()
        await asyncio.to_thread(
            transports["tool_stream"].callback,
            {
                "method": tool_stream.TOOL_STREAM_STOPPED_METHOD,
                "params": {"tool_name": "start_patrol"},
            },
        )
        async with timeout(5):
            while len(app.query("#tool-panels Static")) != 1:
                await pilot.pause(0.01)
        finalized = [line.text for line in app.chat_log.lines]
        assert "start_patrol  1 updates  00:02" in finalized[0]

        clock.now.return_value = start + timedelta(seconds=4)
        async with timeout(2):
            while "lookout  running  00:04" not in panels[1].render_line(0).text:
                await pilot.pause(0.01)
        assert [line.text for line in app.chat_log.lines] == finalized


@pytest.mark.parametrize("terminal_width", [60, 160])
async def test_finalized_tool_panel_uses_current_terminal_width(
    transports: dict[str, FakeTransport], terminal_width: int
) -> None:
    app = HumanCLIApp()
    async with app.run_test(size=(120, 40)) as pilot:
        async with timeout(5):
            while transports["/agent"].callback is None:
                await pilot.pause(0.01)
        await asyncio.to_thread(
            transports["/agent"].callback,
            HumanMessage(content="[tool:start_patrol] Started"),
        )
        await pilot.resize_terminal(terminal_width, 40)
        panel = app.query_one("#tool-panels Static", Static)
        assert panel.render_line(0).text.endswith("╮")
        assert panel.size.width == terminal_width
        assert app.chat_log is not None
        app.chat_log.clear()

        await asyncio.to_thread(
            transports["tool_stream"].callback,
            {
                "method": tool_stream.TOOL_STREAM_STOPPED_METHOD,
                "params": {"tool_name": "start_patrol"},
            },
        )
        async with timeout(5):
            while app.query("#tool-panels Static"):
                await pilot.pause(0.01)

        lines = app.chat_log.lines
        assert "start_patrol  1 updates" in lines[0].text
        assert "Started" in lines[1].text
        assert {line.cell_length for line in lines} == {
            app.chat_log.scrollable_content_region.width
        }
        assert lines[0].text.endswith("╮")
        assert lines[-1].text.endswith("╯")
