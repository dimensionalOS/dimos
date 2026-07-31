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

from collections.abc import Iterator
from io import BytesIO
import json
import queue
from typing import Any
from unittest.mock import MagicMock
import wave

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.module import Module
from dimos.stream.audio.base import AudioEvent
from dimos.teleop.hosted.blueprints.cloudflare import teleop_hosted_go2_transport
from dimos.teleop.hosted.go2_audio_bridge import (
    ENTER_MEGAPHONE,
    EXIT_MEGAPHONE,
    GET_AUDIO_LIST,
    TARGET_SAMPLE_RATE,
    UPLOAD_MEGAPHONE,
    Go2AudioBridgeModule,
)


class AudioBridgeTestModule(Go2AudioBridgeModule):
    go2: MagicMock


@pytest.fixture
def bridge() -> Iterator[AudioBridgeTestModule]:
    result = AudioBridgeTestModule(
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
    result.go2 = MagicMock()
    try:
        yield result
    finally:
        result.stop()


def audio_frame(samples: int = 4410, value: int = 100) -> AudioEvent:
    return AudioEvent(
        np.full(samples, value, dtype=np.int16),
        sample_rate=TARGET_SAMPLE_RATE,
        timestamp=1.0,
        channels=1,
    )


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
    parser = BlueprintConfigParser(teleop_hosted_go2_transport)

    parsed = parser.parse(environ={}, overrides={"go2audiobridgemodule": {"speaker": "enabled"}})

    assert parsed.module_kwargs("go2audiobridgemodule")["speaker"] == "enabled"


def test_auto_mode_disables_speaker_when_audio_hub_probe_fails(
    bridge: AudioBridgeTestModule,
) -> None:
    bridge.go2.publish_request.side_effect = RuntimeError("unsupported")

    assert bridge._ensure_speaker() is False
    assert bridge._ensure_speaker() is False

    bridge.go2.publish_request.assert_called_once()


def test_nonzero_firmware_status_disables_auto_speaker(bridge: AudioBridgeTestModule) -> None:
    bridge.go2.publish_request.return_value = {"data": {"header": {"status": {"code": 3102}}}}

    assert bridge._ensure_speaker() is False


def test_quiet_audio_is_amplified_to_target_peak(bridge: AudioBridgeTestModule) -> None:
    pcm = np.array([-100, 0, 100], dtype=np.int16)

    result = bridge._normalize_level(pcm)

    np.testing.assert_array_equal(result, np.array([-12000, 0, 12000], dtype=np.int16))


def test_near_silence_is_removed_by_noise_gate(bridge: AudioBridgeTestModule) -> None:
    pcm = np.array([-20, 0, 20], dtype=np.int16)

    result = bridge._normalize_level(pcm)

    assert result.size == 0


def test_supported_speaker_uploads_wav_through_megaphone(
    bridge: AudioBridgeTestModule,
) -> None:
    bridge.go2.publish_request.return_value = {"data": {"header": {"status": {"code": 0}}}}
    pcm = np.arange(5000, dtype=np.int16)

    bridge._flush([pcm])

    requests = [call.args[1] for call in bridge.go2.publish_request.call_args_list]
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


def test_start_and_stop_manage_audio_subscription_and_worker(
    bridge: AudioBridgeTestModule, mocker
) -> None:
    bridge.config.speaker = "enabled"
    operator_audio = MagicMock()
    bridge.operator_audio = operator_audio
    subscription = MagicMock()
    operator_audio.subscribe.return_value = subscription
    worker = mocker.patch("dimos.teleop.hosted.go2_audio_bridge.threading.Thread")
    worker.return_value.is_alive.return_value = False
    mocker.patch.object(Module, "start")
    stop = mocker.patch.object(Module, "stop")
    register = mocker.patch.object(bridge, "register_disposable")
    exit_megaphone = mocker.patch.object(bridge, "_exit_megaphone")

    bridge.start()
    bridge.stop()

    operator_audio.subscribe.assert_called_once_with(bridge._on_audio)
    register.assert_called_once()
    worker.return_value.start.assert_called_once_with()
    worker.return_value.join.assert_called_once_with(timeout=2.0)
    exit_megaphone.assert_called_once_with()
    stop.assert_called_once_with()


def test_disabled_speaker_does_not_subscribe_to_operator_audio(
    bridge: AudioBridgeTestModule, mocker
) -> None:
    bridge.config.speaker = "disabled"
    operator_audio = MagicMock()
    bridge.operator_audio = operator_audio
    worker = mocker.patch("dimos.teleop.hosted.go2_audio_bridge.threading.Thread")
    mocker.patch.object(Module, "start")

    bridge.start()

    assert bridge._speaker_available is False
    operator_audio.subscribe.assert_not_called()
    worker.return_value.start.assert_not_called()


def test_full_audio_queue_drops_oldest_frame(bridge: AudioBridgeTestModule) -> None:
    frames = [audio_frame(value=value) for value in range(11)]
    for frame in frames[:10]:
        bridge._frames.put_nowait(frame)

    bridge._on_audio(frames[10])

    queued = [bridge._frames.get_nowait() for _ in range(10)]
    assert queued == frames[1:]


def test_worker_batches_audio_and_flushes_remaining_frames(
    bridge: AudioBridgeTestModule, mocker
) -> None:
    bridge.config.batch_ms = 100
    frame = audio_frame(samples=2205)
    bridge._frames.put_nowait(frame)
    bridge._frames.put_nowait(frame)
    bridge._frames.put_nowait(frame)
    bridge._frames.put_nowait(None)
    batches: list[list[NDArray[np.int16]]] = []
    flush = mocker.patch.object(
        bridge, "_flush", side_effect=lambda frames: batches.append(list(frames))
    )

    bridge._run()

    assert flush.call_count == 2
    assert [len(batch) for batch in batches] == [2, 1]


def test_worker_exits_megaphone_after_idle_timeout(bridge: AudioBridgeTestModule, mocker) -> None:
    get = mocker.patch.object(bridge._frames, "get", side_effect=[queue.Empty, None])
    flush = mocker.patch.object(bridge, "_flush")
    exit_megaphone = mocker.patch.object(bridge, "_exit_megaphone")

    bridge._run()

    assert get.call_count == 2
    assert flush.call_count == 2
    exit_megaphone.assert_called_once_with()


def test_worker_treats_continuous_silent_frames_as_idle(
    bridge: AudioBridgeTestModule, mocker
) -> None:
    audible = audio_frame(value=100)
    silent = audio_frame(value=0)
    mocker.patch.object(bridge._frames, "get", side_effect=[audible, silent, silent, None])
    mocker.patch(
        "dimos.teleop.hosted.go2_audio_bridge.time.monotonic",
        side_effect=[0.0, 0.5, 1.1],
    )

    def flush(frames: list[NDArray[np.int16]]) -> None:
        if frames and np.max(np.abs(frames[0])) > bridge.config.noise_gate_peak:
            bridge._megaphone_active = True

    mocker.patch.object(bridge, "_flush", side_effect=flush)
    exit_megaphone = mocker.patch.object(
        bridge, "_exit_megaphone", side_effect=lambda: setattr(bridge, "_megaphone_active", False)
    )

    bridge._run()

    exit_megaphone.assert_called_once_with()


def test_stop_does_not_race_cleanup_with_live_worker(bridge: AudioBridgeTestModule, mocker) -> None:
    worker = MagicMock()
    worker.is_alive.return_value = True
    bridge._worker = worker
    exit_megaphone = mocker.patch.object(bridge, "_exit_megaphone")
    stop = mocker.patch.object(Module, "stop")

    bridge.stop()

    assert bridge._worker is worker
    exit_megaphone.assert_not_called()
    stop.assert_called_once_with()


def test_failed_upload_disables_auto_speaker(bridge: AudioBridgeTestModule) -> None:
    bridge._speaker_available = True

    def respond(_topic: str, request: dict[str, Any]) -> dict[str, Any]:
        if request["api_id"] == UPLOAD_MEGAPHONE:
            raise RuntimeError("upload failed")
        return {"code": 0}

    bridge.go2.publish_request.side_effect = respond

    bridge._flush([np.array([100], dtype=np.int16)])

    requests = [call.args[1]["api_id"] for call in bridge.go2.publish_request.call_args_list]
    assert requests == [ENTER_MEGAPHONE, UPLOAD_MEGAPHONE, EXIT_MEGAPHONE]
    assert bridge._speaker_available is False
    assert bridge._megaphone_active is False


def test_exit_megaphone_preserves_state_for_retry_when_request_fails(
    bridge: AudioBridgeTestModule,
) -> None:
    bridge._megaphone_active = True
    bridge.go2.publish_request.side_effect = RuntimeError("request failed")

    bridge._exit_megaphone()

    assert bridge._megaphone_active is True

    bridge.go2.publish_request.side_effect = None
    bridge.go2.publish_request.return_value = {"code": 0}
    bridge._exit_megaphone()

    assert bridge._megaphone_active is False


@pytest.mark.parametrize(
    ("response", "expected"),
    [
        ({"code": 1}, 1),
        ({"data": {"code": 2}}, 2),
        ({"data": {"header": {"status": {"code": 3}}}}, 3),
        ("unexpected", None),
        ({}, None),
    ],
)
def test_response_code_extracts_supported_firmware_shapes(
    response: Any, expected: int | None
) -> None:
    assert Go2AudioBridgeModule._response_code(response) == expected


def test_request_rejects_empty_response(bridge: AudioBridgeTestModule) -> None:
    bridge.go2.publish_request.return_value = None

    with pytest.raises(RuntimeError, match="returned no response"):
        bridge._request(GET_AUDIO_LIST)


def test_audio_conversion_rejects_invalid_metadata() -> None:
    frame = AudioEvent(np.array([1], dtype=np.int16), sample_rate=0, timestamp=1.0, channels=1)

    assert Go2AudioBridgeModule._to_mono_target_rate(frame).size == 0
