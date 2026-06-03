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

"""Tests for the MiniMax TTS node."""

import json
from unittest.mock import MagicMock, patch

import pytest

from dimos.stream.audio.tts.node_minimax import (
    DEFAULT_MINIMAX_BASE_URL,
    DEFAULT_MINIMAX_TTS_MODEL,
    DEFAULT_MINIMAX_VOICE,
    MiniMaxTTSNode,
    MiniMaxTTSVoice,
)


def test_default_constants() -> None:
    """Default model and voice should match the MiniMax TTS docs."""
    assert DEFAULT_MINIMAX_TTS_MODEL == "speech-2.8-hd"
    assert DEFAULT_MINIMAX_VOICE == "English_Graceful_Lady"
    assert DEFAULT_MINIMAX_BASE_URL == "https://api.minimax.io"


def test_missing_api_key_raises() -> None:
    """Calling t2a without an API key must fail loudly."""
    with patch.dict("os.environ", {}, clear=True):
        node = MiniMaxTTSNode()
        with pytest.raises(ValueError, match="MINIMAX_API_KEY"):
            node._resolve_api_key()


def test_call_t2a_decodes_hex_audio() -> None:
    """The t2a endpoint returns hex-encoded audio; the node must decode it."""
    fake_hex_audio = "FFFF"  # 0xFF 0xFF

    fake_response_body = json.dumps(
        {
            "data": {"audio": fake_hex_audio},
            "base_resp": {"status_code": 0, "status_msg": "ok"},
        }
    ).encode("utf-8")

    fake_response = MagicMock()
    fake_response.read.return_value = fake_response_body
    fake_response.__enter__.return_value = fake_response
    fake_response.__exit__.return_value = False

    with patch(
        "dimos.stream.audio.tts.node_minimax.urllib.request.urlopen",
        return_value=fake_response,
    ):
        node = MiniMaxTTSNode(api_key="test-key")
        result = node._call_t2a("hello")

    assert result == bytes.fromhex(fake_hex_audio)


def test_call_t2a_raises_on_error_status() -> None:
    """A non-zero base_resp.status_code should raise a clear RuntimeError."""
    fake_response_body = json.dumps(
        {
            "data": {"audio": ""},
            "base_resp": {"status_code": 2013, "status_msg": "bad params"},
        }
    ).encode("utf-8")

    fake_response = MagicMock()
    fake_response.read.return_value = fake_response_body
    fake_response.__enter__.return_value = fake_response
    fake_response.__exit__.return_value = False

    with patch(
        "dimos.stream.audio.tts.node_minimax.urllib.request.urlopen",
        return_value=fake_response,
    ):
        node = MiniMaxTTSNode(api_key="test-key")
        with pytest.raises(RuntimeError, match="2013"):
            node._call_t2a("hello")


def test_call_t2a_sends_correct_request_shape() -> None:
    """The outbound request must include model, voice_setting, audio_setting."""
    captured: dict = {}

    def fake_urlopen(req, timeout=30):  # type: ignore[no-untyped-def]
        captured["url"] = req.full_url
        captured["headers"] = dict(req.headers)
        captured["body"] = json.loads(req.data.decode("utf-8"))

        body = json.dumps(
            {
                "data": {"audio": "00"},
                "base_resp": {"status_code": 0, "status_msg": "ok"},
            }
        ).encode("utf-8")
        resp = MagicMock()
        resp.read.return_value = body
        resp.__enter__.return_value = resp
        resp.__exit__.return_value = False
        return resp

    with patch(
        "dimos.stream.audio.tts.node_minimax.urllib.request.urlopen",
        side_effect=fake_urlopen,
    ):
        node = MiniMaxTTSNode(
            api_key="test-key",
            voice=MiniMaxTTSVoice.INSIGHTFUL_SPEAKER,
            speed=1.2,
        )
        node._call_t2a("hi there")

    assert captured["url"] == "https://api.minimax.io/v1/t2a_v2"
    assert captured["headers"]["Authorization"] == "Bearer test-key"
    body = captured["body"]
    assert body["model"] == "speech-2.8-hd"
    assert body["text"] == "hi there"
    assert body["stream"] is False
    assert body["voice_setting"]["voice_id"] == "English_Insightful_Speaker"
    assert body["voice_setting"]["speed"] == 1.2
    assert body["audio_setting"]["format"] == "mp3"


def test_voice_enum_values_match_documented_ids() -> None:
    """Voice IDs must be exactly what MiniMax documents in its FAQ."""
    assert MiniMaxTTSVoice.GRACEFUL_LADY.value == "English_Graceful_Lady"
    assert MiniMaxTTSVoice.LUCKY_ROBOT.value == "English_Lucky_Robot"
