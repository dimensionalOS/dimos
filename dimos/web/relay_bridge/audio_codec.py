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

"""audio.json.v1: browser push-to-talk audio chunks (the chat panel's mic).

Wire contract (browser -> robot, publish tx): one JSON object per frame,
`{"sid": str, "seq": int, "mime": str, "data": str, "final": bool}`.
`sid` names one utterance (one per button hold), `seq` counts contiguously
from 0 within it, `mime` is the recorder's container type (e.g.
"audio/webm;codecs=opus") and `data` is standard base64 of at most 16 KiB of
container bytes ("" allowed). The sender always ends an utterance with a
`final` frame of empty data; a cancelled hold simply stops sending and the
consumer reaps the sid. Concatenating the decoded `data` of seq 0..N yields
the recorder's exact byte stream - slices need not align with recorder
chunks, so the container survives re-framing untouched.
"""

import base64
from dataclasses import dataclass
from typing import Any

from dimos.web.codecs import web_decoder

# Utterance/mime ids are short tags, not data carriers.
_MAX_ID_CHARS = 64
# The serialized frame is capped at 32 KiB upstream (MAX_PUB_DATA_BYTES, both
# SDK- and relay-enforced); senders slice at 16 KiB raw. Defense in depth.
_MAX_DATA_BYTES = 24_000
_MAX_SEQ = 1_000_000


@dataclass(frozen=True)
class AudioChunk:
    """One decoded slice of an utterance's container byte stream."""

    sid: str
    seq: int
    mime: str
    data: bytes
    final: bool


def _checked_id(value: dict[str, Any], key: str) -> str:
    tag = value.get(key)
    if not isinstance(tag, str) or not 1 <= len(tag) <= _MAX_ID_CHARS:
        raise ValueError(f"{key} must be a 1..{_MAX_ID_CHARS} char string")
    return tag


@web_decoder("audio.json.v1")
def decode_audio_chunk(value: dict[str, Any]) -> AudioChunk:
    if not isinstance(value, dict):
        raise ValueError("audio chunk must be a JSON object")
    sid = _checked_id(value, "sid")
    mime = _checked_id(value, "mime")
    seq = value.get("seq")
    if isinstance(seq, bool) or not isinstance(seq, int) or not 0 <= seq <= _MAX_SEQ:
        raise ValueError(f"seq must be an int in 0..{_MAX_SEQ}")
    final = value.get("final")
    if not isinstance(final, bool):
        raise ValueError("final must be a bool")
    encoded = value.get("data")
    if not isinstance(encoded, str):
        raise ValueError("data must be a base64 string")
    try:
        data = base64.b64decode(encoded, validate=True)
    except ValueError as e:
        raise ValueError(f"bad base64 data: {e}") from e
    if len(data) > _MAX_DATA_BYTES:
        raise ValueError(f"data is {len(data)} B decoded (limit {_MAX_DATA_BYTES})")
    return AudioChunk(sid=sid, seq=seq, mime=mime, data=data, final=final)
