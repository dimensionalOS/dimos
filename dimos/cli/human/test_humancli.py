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
from typing import Any

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
import pytest

from dimos.agents.mcp import tool_stream
from dimos.cli.human import humancli
from dimos.cli.human.humancli import HumanCLIApp

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


async def test_block_content_messages_render(monkeypatch: pytest.MonkeyPatch) -> None:
    transports: dict[str, FakeTransport] = {}

    def fake_make_transport(name: str, *args: Any, **kwargs: Any) -> FakeTransport:
        transports[name] = FakeTransport()
        return transports[name]

    monkeypatch.setattr(humancli, "make_transport", fake_make_transport)
    monkeypatch.setattr(tool_stream, "subscribe", lambda cb: (lambda: None))

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
        assert "could not render" not in text
