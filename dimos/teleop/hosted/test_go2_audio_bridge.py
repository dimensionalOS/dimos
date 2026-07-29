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

from io import BytesIO
import json
from types import SimpleNamespace
from unittest.mock import MagicMock
import wave

import numpy as np
import pytest

from dimos.core.module import Module
from dimos.stream.audio.base import AudioEvent
from dimos.teleop.hosted.blueprints.cloudflare import teleop_hosted_go2_transport
from dimos.teleop.hosted.go2_audio_bridge import (
    ENTER_MEGAPHONE,
    GET_AUDIO_LIST,
    TARGET_SAMPLE_RATE,
    UPLOAD_MEGAPHONE,
    Go2AudioBridgeModule,
)


@pytest.fixture
def bridge(monkeypatch: pytest.MonkeyPatch) -> Go2AudioBridgeModule:
    config = SimpleNamespace(
        speaker="auto",
        batch_ms=100,
        idle_timeout_sec=1.0,
        queue_frames=10,
        chunk_interval_sec=0.0,
        megaphone_enter_delay_sec=0.0,
        target_peak=12000,
        max_gain=128.0,
        noise_gate_peak=32,
    )

    def init(module: Module, **kwargs: object) -> None:
        module.config = config  # type: ignore[assignment]

    monkeypatch.setattr(Module, "__init__", init)
    result = Go2AudioBridgeModule()
    result.go2 = MagicMock()
    return result


def test_stereo_audio_is_mixed_and_resampled_for_go2() -> None:
    left = np.full(4800, 1000, dtype=np.int16)
    right = np.full(4800, 3000, dtype=np.int16)
    interleaved = np.column_stack((left, right)).reshape(-1)
    frame = AudioEvent(interleaved, sample_rate=48000, timestamp=1.0, channels=2)

    result = Go2AudioBridgeModule._to_mono_target_rate(frame)

    assert result.dtype == np.int16
    assert result.shape == (4410,)
    assert np.all(result == 2000)


def test_hosted_blueprint_accepts_speaker_override() -> None:
    config = teleop_hosted_go2_transport.config()

    resolved = config(**{"go2audiobridgemodule": {"speaker": "enabled"}})

    assert resolved.go2audiobridgemodule.speaker == "enabled"


def test_auto_mode_disables_speaker_when_audio_hub_probe_fails(
    bridge: Go2AudioBridgeModule,
) -> None:
    bridge.go2.publish_request.side_effect = RuntimeError("unsupported")  # type: ignore[attr-defined]

    assert bridge._ensure_speaker() is False
    assert bridge._ensure_speaker() is False

    bridge.go2.publish_request.assert_called_once()  # type: ignore[attr-defined]


def test_nonzero_firmware_status_disables_auto_speaker(bridge: Go2AudioBridgeModule) -> None:
    bridge.go2.publish_request.return_value = {  # type: ignore[attr-defined]
        "data": {"header": {"status": {"code": 3102}}}
    }

    assert bridge._ensure_speaker() is False


def test_quiet_audio_is_amplified_to_target_peak(bridge: Go2AudioBridgeModule) -> None:
    pcm = np.array([-100, 0, 100], dtype=np.int16)

    result = bridge._normalize_level(pcm)

    np.testing.assert_array_equal(result, np.array([-12000, 0, 12000], dtype=np.int16))


def test_near_silence_is_removed_by_noise_gate(bridge: Go2AudioBridgeModule) -> None:
    pcm = np.array([-20, 0, 20], dtype=np.int16)

    result = bridge._normalize_level(pcm)

    assert result.size == 0


def test_supported_speaker_uploads_wav_through_megaphone(
    bridge: Go2AudioBridgeModule,
) -> None:
    bridge.go2.publish_request.return_value = {  # type: ignore[attr-defined]
        "data": {"header": {"status": {"code": 0}}}
    }
    pcm = np.arange(5000, dtype=np.int16)

    bridge._flush([pcm])

    requests = [call.args[1] for call in bridge.go2.publish_request.call_args_list]  # type: ignore[attr-defined]
    assert [request["api_id"] for request in requests[:2]] == [GET_AUDIO_LIST, ENTER_MEGAPHONE]
    uploads = [request for request in requests if request["api_id"] == UPLOAD_MEGAPHONE]
    assert uploads
    first_upload = json.loads(uploads[0]["parameter"])
    assert first_upload["current_block_index"] == 1
    assert first_upload["total_block_number"] == len(uploads)

    wav_data = Go2AudioBridgeModule._wav_bytes(pcm)
    with wave.open(BytesIO(wav_data), "rb") as wav:
        assert (wav.getnchannels(), wav.getframerate(), wav.getsampwidth()) == (
            1,
            TARGET_SAMPLE_RATE,
            2,
        )
