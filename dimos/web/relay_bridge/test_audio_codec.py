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

"""audio.json.v1: browser push-to-talk frames -> AudioChunk."""

import base64
from typing import Any

import pytest

from dimos.web.codecs import resolve_decoder
from dimos.web.relay_bridge.audio_codec import _MAX_DATA_BYTES, AudioChunk, decode_audio_chunk


def _frame(**overrides: Any) -> dict[str, Any]:
    frame: dict[str, Any] = {
        "sid": "u1",
        "seq": 0,
        "mime": "audio/webm;codecs=opus",
        "data": base64.b64encode(b"\x1aE\xdf\xa3 opus bytes").decode(),
        "final": False,
    }
    frame.update(overrides)
    return frame


def test_decode_valid_chunk_roundtrips_bytes() -> None:
    chunk = decode_audio_chunk(_frame(seq=3))
    assert chunk == AudioChunk(
        sid="u1",
        seq=3,
        mime="audio/webm;codecs=opus",
        data=b"\x1aE\xdf\xa3 opus bytes",
        final=False,
    )


def test_decode_final_empty_data() -> None:
    chunk = decode_audio_chunk(_frame(seq=7, data="", final=True))
    assert chunk.data == b"" and chunk.final


def test_extra_keys_are_tolerated() -> None:
    # Forward compatibility: a newer panel may add fields.
    assert decode_audio_chunk(_frame(later=1.5)).sid == "u1"


def test_bad_b64_raises() -> None:
    with pytest.raises(ValueError, match="bad base64"):
        decode_audio_chunk(_frame(data="not base64!"))


@pytest.mark.parametrize(
    "frame",
    [
        ["not", "an", "object"],
        _frame(sid=None),
        _frame(sid=""),
        _frame(sid="x" * 65),
        _frame(mime=""),
        _frame(seq=True),
        _frame(seq=-1),
        _frame(seq=1.5),
        _frame(seq=1_000_001),
        _frame(data=None),
        _frame(final="yes"),
        {"sid": "u1"},
    ],
    ids=[
        "not_an_object",
        "sid_none",
        "sid_empty",
        "sid_long",
        "mime_empty",
        "seq_bool",
        "seq_negative",
        "seq_float",
        "seq_huge",
        "data_none",
        "final_string",
        "fields_missing",
    ],
)
def test_wrong_shape_raises(frame: Any) -> None:
    with pytest.raises(ValueError):
        decode_audio_chunk(frame)


def test_oversize_data_raises() -> None:
    over = base64.b64encode(b"x" * (_MAX_DATA_BYTES + 1)).decode()
    with pytest.raises(ValueError, match="decoded"):
        decode_audio_chunk(_frame(data=over))


def test_resolve_decoder_binds_audio_chunk() -> None:
    definition = resolve_decoder("audio.json.v1", AudioChunk)
    assert definition.decode is decode_audio_chunk
    assert definition.takes_context is False
