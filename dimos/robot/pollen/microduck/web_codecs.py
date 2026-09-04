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

"""Web encoders for the microduck cockpit's own channels.

The cockpit blueprint declares these streams with `Channel(...)` and the
bridge resolves the encoder by encoding id (dimos/web/codecs.py), so none of
this lives in the generic relay bridge: the transcript encoder can import
langchain outright, and no microduck stream welds a port onto
RelayBridgeModule.

Encoders are module level because the registry ships them into the worker by
pickle reference. Each returns compact JSON bytes; every record carries "t",
the encode-time wall clock (a passthrough keeps the producer's own).
"""

from __future__ import annotations

import json
import time
from typing import Any

from langchain_core.messages.base import BaseMessage

from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.web.codecs import web_encoder

_JSON_SEPARATORS = (",", ":")

# Transcript content cap (characters): a pathological tool result must not
# turn one reliable frame into a megabyte for every viewer.
_CHAT_CONTENT_MAX_CHARS = 16 * 1024

# Path points per frame: enough for any room-scale plan; longer plans are
# decimated uniformly (endpoints kept) so the viewer's polyline stays cheap.
_PATH_MAX_POINTS = 256


def _json_frame(record: dict[str, Any]) -> bytes:
    record.setdefault("t", time.time())
    return json.dumps(record, separators=_JSON_SEPARATORS, default=str).encode()


def _chat_text(content: Any) -> str:
    """Flatten langchain message content - a string or a list of provider
    content blocks - into transcript text (images become "[image]")."""
    if isinstance(content, str):
        return content
    if not isinstance(content, list):
        return str(content)
    parts: list[str] = []
    for block in content:
        if isinstance(block, str):
            parts.append(block)
        elif isinstance(block, dict):
            kind = block.get("type")
            if kind == "text" and isinstance(block.get("text"), str):
                parts.append(block["text"])
            elif kind in ("image", "image_url"):
                parts.append("[image]")
            elif kind:
                parts.append(f"[{kind}]")
        else:
            parts.append(str(block))
    return "\n".join(parts)


def _chat_role(msg: BaseMessage) -> str:
    """Transcript role from the message's own `type` tag ("human", "ai",
    "tool", "system"; the streaming variants say "AIMessageChunk" and so on).
    The exotic kinds the agent loop never emits (chat, function) read as
    system notes."""
    kind = str(getattr(msg, "type", "")).lower().removesuffix("messagechunk")
    return kind if kind in ("human", "ai", "tool") else "system"


@web_encoder("chat.json.v1")
def encode_chat(msg: BaseMessage) -> bytes:
    """One transcript entry per langchain message.

    The entry number the viewer dedupes on is NOT here: it belongs to the
    bridge's replay log, which ships it as the frame's `n` meta so a replayed
    frame and its live original agree (see RelayBridgeModule._with_n).
    """
    tool_calls = [
        {"id": call.get("id"), "name": call.get("name"), "args": call.get("args")}
        for call in (getattr(msg, "tool_calls", None) or [])
    ]
    content = _chat_text(getattr(msg, "content", ""))
    if len(content) > _CHAT_CONTENT_MAX_CHARS:
        content = content[:_CHAT_CONTENT_MAX_CHARS] + "\n[truncated]"
    return _json_frame(
        {
            "role": _chat_role(msg),
            "content": content,
            "name": getattr(msg, "name", None),
            "tool_calls": tool_calls,
            "tool_call_id": getattr(msg, "tool_call_id", None),
            "id": getattr(msg, "id", None),
        }
    )


@web_encoder("flag.json.v1")
def encode_flag(msg: bool) -> bytes:
    return _json_frame({"value": bool(msg)})


@web_encoder("path.json.v1")
def encode_path(msg: NavPath) -> bytes:
    poses = msg.poses
    if len(poses) > _PATH_MAX_POINTS:
        step = (len(poses) - 1) / (_PATH_MAX_POINTS - 1)
        poses = [poses[round(i * step)] for i in range(_PATH_MAX_POINTS)]
    return _json_frame(
        {
            "frame": msg.frame_id,
            "points": [[p.position.x, p.position.y] for p in poses],
        }
    )


# One behaviour, four encoding ids: DuckControl and PlacesMemory all publish
# JSON strings, and the id is what tells the cockpit which panel slot a
# channel fills (dimos/web/relay_bridge/manifest.py, _SLOT_KINDS).
@web_encoder("navstate.json.v1")
@web_encoder("mode.json.v1")
@web_encoder("places.json.v1")
@web_encoder("policy.json.v1")
def encode_state_json(msg: str) -> bytes:
    """Re-dump a JSON-string producer's payload compactly. A payload that is
    not a JSON object raises, so the bridge drops the frame (throttled log)
    rather than forwarding garbage to every viewer."""
    record = json.loads(msg)
    if not isinstance(record, dict):
        raise ValueError(f"expected a JSON object, got {type(record).__name__}")
    return _json_frame(record)
