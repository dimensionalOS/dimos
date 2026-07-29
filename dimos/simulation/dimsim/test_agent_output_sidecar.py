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

from __future__ import annotations

from io import StringIO
import json
from threading import Event
from typing import Any

from langchain_core.messages import AIMessage, HumanMessage

from dimos.simulation.dimsim.agent_output_sidecar import (
    run_sidecar,
    serialize_agent_idle,
    serialize_agent_message,
)


class FakeTransport:
    def __init__(self, messages: list[Any]) -> None:
        self.messages = messages
        self.started = False
        self.unsubscribed = False
        self.stopped = False

    def start(self) -> None:
        self.started = True

    def subscribe(self, callback: Any) -> Any:
        for message in self.messages:
            callback(message)

        def unsubscribe() -> None:
            self.unsubscribed = True

        return unsubscribe

    def stop(self) -> None:
        self.stopped = True


def test_serialize_agent_message_ignores_non_ai_messages() -> None:
    result = serialize_agent_message(HumanMessage(content="FOUND_BATHTUB"))

    assert result is None


def test_serialize_agent_message_extracts_responses_text_and_tool_metadata() -> None:
    message = AIMessage(
        content=[
            {"type": "reasoning", "summary": "private"},
            {"type": "output_text", "text": "FOUND_"},
            {"type": "text", "text": "BATHTUB"},
        ],
        tool_calls=[
            {
                "name": "speak",
                "args": {"text": "FOUND_BATHTUB"},
                "id": "tool-1",
                "type": "tool_call",
            }
        ],
    )

    result = serialize_agent_message(message, timestamp=12.345)

    assert result == {
        "type": "agent_output",
        "text": "FOUND_BATHTUB",
        "hasToolCalls": True,
        "timestampMs": 12345,
    }


def test_serialize_agent_idle_accepts_only_boolean_state() -> None:
    assert serialize_agent_idle(True, timestamp=12.345) == {
        "type": "agent_idle",
        "idle": True,
        "timestampMs": 12345,
    }
    assert serialize_agent_idle("true", timestamp=12.345) is None


def test_run_sidecar_signals_ready_before_buffered_output_and_cleans_up() -> None:
    transport = FakeTransport(
        [
            HumanMessage(content="ignored"),
            AIMessage(content="FOUND_BATHTUB"),
        ]
    )
    output = StringIO()
    stop_event = Event()
    stop_event.set()

    run_sidecar(transport, output, stop_event)

    lines = [json.loads(line) for line in output.getvalue().splitlines()]
    assert lines == [
        {"type": "ready"},
        {
            "type": "agent_output",
            "text": "FOUND_BATHTUB",
            "hasToolCalls": False,
            "timestampMs": lines[1]["timestampMs"],
        },
    ]
    assert transport.started
    assert transport.unsubscribed
    assert transport.stopped


def test_run_sidecar_streams_idle_state_and_cleans_up_both_transports() -> None:
    agent_transport = FakeTransport([])
    idle_transport = FakeTransport([False, True])
    output = StringIO()
    stop_event = Event()
    stop_event.set()

    run_sidecar(
        agent_transport,
        output,
        stop_event,
        idle_transport=idle_transport,
    )

    lines = [json.loads(line) for line in output.getvalue().splitlines()]
    assert lines == [
        {"type": "ready"},
        {
            "type": "agent_idle",
            "idle": False,
            "timestampMs": lines[1]["timestampMs"],
        },
        {
            "type": "agent_idle",
            "idle": True,
            "timestampMs": lines[2]["timestampMs"],
        },
    ]
    assert agent_transport.unsubscribed
    assert agent_transport.stopped
    assert idle_transport.unsubscribed
    assert idle_transport.stopped
