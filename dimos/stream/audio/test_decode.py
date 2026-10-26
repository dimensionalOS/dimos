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

"""Decoding browser recordings in-process: a real container in, 16-kHz PCM out."""

import io

import av
import numpy as np

from dimos.stream.audio.decode import decode_audio_bytes


def _webm_opus(seconds: float) -> bytes:
    """What Chromium's MediaRecorder produces: a 48-kHz opus stream in webm."""
    buffer = io.BytesIO()
    with av.open(buffer, "w", format="webm") as container:
        stream = container.add_stream("libopus", rate=48_000, layout="mono")
        t = np.arange(int(48_000 * seconds)) / 48_000
        tone = (0.3 * np.sin(2 * np.pi * 440 * t)).astype(np.float32)
        frame = av.AudioFrame.from_ndarray(tone[None, :], format="flt", layout="mono")
        frame.sample_rate = 48_000
        container.mux(stream.encode(frame))
        container.mux(stream.encode(None))
    return buffer.getvalue()


def test_webm_opus_decodes_to_16k_mono_float32() -> None:
    event = decode_audio_bytes(_webm_opus(0.5))
    assert event is not None
    assert event.sample_rate == 16_000 and event.channels == 1
    assert event.data.dtype == np.float32 and event.data.ndim == 1
    assert 7_500 < event.data.shape[0] < 8_500  # ~0.5 s at 16 kHz
    assert 0.25 < np.abs(event.data).max() < 0.35  # the tone's amplitude survived


def test_garbage_returns_none() -> None:
    assert decode_audio_bytes(b"definitely not an audio container") is None
