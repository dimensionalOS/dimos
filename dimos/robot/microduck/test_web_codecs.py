# Copyright 2025-2026 Dimensional Inc.
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

"""Microduck cockpit encoders: message in, compact JSON frame bytes out.

No bridge here - these are the plain registry functions the cockpit blueprint
resolves by encoding id. The transcript entry number the viewer dedupes on is
deliberately absent from every payload: it belongs to the bridge's replay log
and rides the frame meta instead (RelayBridgeModule._with_n).
"""

from __future__ import annotations

import json
import time
from types import SimpleNamespace
from typing import Any

from langchain_core.messages import (
    AIMessage,
    AIMessageChunk,
    ChatMessage,
    FunctionMessage,
    HumanMessage,
    HumanMessageChunk,
    SystemMessage,
    SystemMessageChunk,
    ToolMessage,
    ToolMessageChunk,
)
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.robot.microduck.web_codecs import (
    _CHAT_CONTENT_MAX_CHARS,
    _PATH_MAX_POINTS,
    encode_chat,
    encode_flag,
    encode_path,
    encode_state_json,
)
from dimos.web.codecs import encoder_definition

_STATE_ENCODINGS = ["navstate.json.v1", "mode.json.v1", "places.json.v1", "policy.json.v1"]


def chat(msg: Any) -> dict[str, Any]:
    payload = encode_chat(msg)
    assert b": " not in payload and b", " not in payload  # compact
    return json.loads(payload)


def test_encode_chat_shapes() -> None:
    t0 = time.time()
    human = chat(HumanMessage(content="hi there"))
    assert human == {
        "role": "human",
        "content": "hi there",
        "name": None,
        "tool_calls": [],
        "tool_call_id": None,
        "id": None,
        "t": human["t"],
    }
    assert t0 <= human["t"] <= time.time()

    ai = chat(
        AIMessage(
            content=[
                {"type": "text", "text": "Looking."},
                {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}},
                {"type": "text", "text": "Found it."},
            ],
            tool_calls=[{"name": "navigate_to", "args": {"place": "kitchen"}, "id": "c1"}],
            id="run-1",
        ),
    )
    assert ai["role"] == "ai"
    assert ai["content"] == "Looking.\n[image]\nFound it."
    assert ai["tool_calls"] == [{"id": "c1", "name": "navigate_to", "args": {"place": "kitchen"}}]
    assert ai["tool_call_id"] is None and ai["id"] == "run-1"

    tool = chat(ToolMessage(content="ok", tool_call_id="c1", name="navigate_to"))
    assert (tool["role"], tool["content"], tool["tool_call_id"], tool["name"]) == (
        "tool",
        "ok",
        "c1",
        "navigate_to",
    )
    assert tool["tool_calls"] == []

    system = chat(SystemMessage(content="You are a duck."))
    assert (system["role"], system["content"]) == ("system", "You are a duck.")


def test_encode_chat_caps_content() -> None:
    cap = _CHAT_CONTENT_MAX_CHARS
    entry = chat(ToolMessage(content="x" * (cap + 5000), tool_call_id="c1"))
    assert entry["content"] == "x" * cap + "\n[truncated]"
    exact = chat(HumanMessage(content="y" * cap))
    assert exact["content"] == "y" * cap


@pytest.mark.parametrize(
    ("msg", "role"),
    [
        (HumanMessageChunk(content="h"), "human"),
        (AIMessageChunk(content="a"), "ai"),
        (ToolMessageChunk(content="t", tool_call_id="c1"), "tool"),
        (SystemMessageChunk(content="s"), "system"),
        (ChatMessage(content="c", role="narrator"), "system"),
        (FunctionMessage(content="f", name="fn"), "system"),
        # The encoder reads the message's own `type` tag, nothing langchain
        # specific: any object shaped like a message encodes.
        (SimpleNamespace(type="human", content="duck"), "human"),
        (SimpleNamespace(type="mystery", content=["a", "b"]), "system"),
    ],
    ids=lambda v: v if isinstance(v, str) else type(v).__name__,
)
def test_encode_chat_role_from_message_type(msg: Any, role: str) -> None:
    entry = chat(msg)
    assert entry["role"] == role
    assert entry["tool_calls"] == [] and entry["tool_call_id"] == getattr(msg, "tool_call_id", None)
    assert isinstance(entry["t"], float)


def test_encode_flag_and_path() -> None:
    flag = json.loads(encode_flag(True))
    assert flag["value"] is True and isinstance(flag["t"], float)

    def pose(i: int) -> PoseStamped:
        return PoseStamped(ts=1.0, frame_id="map", position=Vector3(float(i), -0.5 * i, 0.0))

    short = NavPath(ts=1.0, frame_id="map", poses=[pose(i) for i in range(3)])
    path = json.loads(encode_path(short))
    assert path["frame"] == "map"
    assert path["points"] == [[0.0, 0.0], [1.0, -0.5], [2.0, -1.0]]
    assert isinstance(path["t"], float)

    long = NavPath(ts=1.0, frame_id="map", poses=[pose(i) for i in range(1000)])
    points = json.loads(encode_path(long))["points"]
    # Uniform decimation keeps both endpoints and the order.
    assert len(points) == _PATH_MAX_POINTS
    assert points[0] == [0.0, 0.0] and points[-1] == [999.0, -499.5]
    assert points == sorted(points)


@pytest.mark.parametrize("encoding", _STATE_ENCODINGS)
def test_passthrough_encoders_validate_json(encoding: str) -> None:
    # One behaviour behind four encoding ids; the id is only what tells the
    # cockpit which panel slot the channel fills.
    definition = encoder_definition(encoding)
    assert definition is not None and definition.encode is encode_state_json
    # Re-dumped compactly, the producer's own "t" kept.
    payload = definition.encode('{"state": "idle", "goal": null, "t": 12.5}')
    assert payload == b'{"state":"idle","goal":null,"t":12.5}'
    # A producer that forgot "t" gets the bridge clock.
    record = json.loads(definition.encode('{"mode": "agent"}'))
    assert record["mode"] == "agent" and isinstance(record["t"], float)


@pytest.mark.parametrize("payload", ["not json", "[1, 2]", '"a string"', ""])
def test_passthrough_rejects_non_objects(payload: str) -> None:
    # Raising is how a bad sample is dropped now: the bridge's _run_encoder
    # catches it, logs at most once per channel per window, and sends no
    # frame - rather than forwarding garbage to every viewer.
    with pytest.raises(ValueError):
        encode_state_json(payload)
