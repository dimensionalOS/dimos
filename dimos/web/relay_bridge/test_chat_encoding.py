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

"""chat.json.v1: LangChain messages -> chat lines (humancli's rendering as data)."""

import json

from langchain_core.messages import (
    AIMessage,
    BaseMessage,
    ChatMessage,
    HumanMessage,
    SystemMessage,
    ToolMessage,
)
import pytest

from dimos.web.codecs import resolve_encoder
from dimos.web.relay_bridge.chat_codec import _CHAT_TEXT_MAX_CHARS, chat_lines, encode_chat

_IMAGE = {"type": "image", "data": "AAAA", "mime_type": "image/png"}
_TS = 1752576000.5


@pytest.mark.parametrize(
    ("msg", "lines"),
    [
        (HumanMessage("walk forward"), [{"role": "human", "text": "walk forward"}]),
        (
            HumanMessage("[tool:nav] 40% there"),
            [{"role": "tool", "tool": "nav", "text": "40% there"}],
        ),
        (HumanMessage("[tool:nav]"), []),
        (AIMessage("on my way"), [{"role": "ai", "text": "on my way"}]),
        (
            AIMessage("sure", tool_calls=[{"name": "nav", "args": {"x": 1.5, "y": 0}, "id": "c1"}]),
            [
                {"role": "ai", "text": "sure"},
                {"role": "ai", "tool": "nav", "text": 'nav({"x":1.5,"y":0})'},
            ],
        ),
        (AIMessage(""), []),
        (
            ToolMessage("arrived", tool_call_id="c1", name="nav"),
            [{"role": "tool", "tool": "nav", "text": "arrived"}],
        ),
        (
            ToolMessage("arrived", tool_call_id="c1"),
            [{"role": "tool", "tool": "tool", "text": "arrived"}],
        ),
        (SystemMessage("be brief"), [{"role": "system", "text": "be brief"}]),
        (
            HumanMessage(content=[{"type": "text", "text": "artefact"}, _IMAGE]),
            [{"role": "human", "text": "artefact"}],
        ),
        (ChatMessage("hm", role="critic"), [{"role": "chat", "text": "hm"}]),
    ],
    ids=[
        "human",
        "tool_progress",
        "tool_progress_empty",
        "ai",
        "ai_tool_calls",
        "ai_empty",
        "tool",
        "tool_unnamed",
        "system",
        "multimodal",
        "unknown_kind",
    ],
)
def test_chat_lines(msg: BaseMessage, lines: list[dict[str, str]]) -> None:
    assert chat_lines(msg, _TS) == [{**line, "ts": _TS} for line in lines]


def test_long_text_is_capped() -> None:
    (line,) = chat_lines(ToolMessage("x" * (_CHAT_TEXT_MAX_CHARS + 1), tool_call_id="c1"), _TS)
    assert line["text"] == "x" * _CHAT_TEXT_MAX_CHARS + "..."


def test_encoder_emits_compact_json_or_skips() -> None:
    assert resolve_encoder("chat.json.v1", BaseMessage).encode is encode_chat
    payload = encode_chat(HumanMessage(content=[{"type": "text", "text": "artefact"}, _IMAGE]))
    assert payload is not None and b"image" not in payload
    (line,) = json.loads(payload)
    assert line["role"] == "human" and line["text"] == "artefact" and line["ts"] > 0
    assert encode_chat(AIMessage("")) is None
