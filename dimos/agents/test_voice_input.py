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

"""VoiceInput: audio_in chunks -> decode -> STT pipeline -> human_input text.

The real normalizer and WhisperNode wiring run against a fake Whisper model;
the ffmpeg boundary runs for real in the container test and is otherwise
replaced with a byte-mirroring stub.
"""

from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
import io
import math
import struct
from threading import Event
import time
from typing import Any
import wave

import numpy as np
import pytest

from dimos.agents import voice_input as voice_input_module
from dimos.agents.voice_input import VoiceInput
from dimos.stream.audio.base import AudioEvent
from dimos.stream.audio.node_normalizer import AudioNormalizer
from dimos.stream.audio.stt import node_whisper as whisper_module
from dimos.web.relay_bridge.audio_codec import AudioChunk
from dimos.web.relay_bridge.module_test_support import FakeTransport


def _chunk(sid: str = "u1", seq: int = 0, data: bytes = b"", final: bool = False) -> AudioChunk:
    return AudioChunk(sid=sid, seq=seq, mime="audio/webm", data=data, final=final)


def _wav_bytes(seconds: float = 0.25) -> bytes:
    """A real container: 48-kHz stereo 16-bit WAV of a 440 Hz tone."""
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as writer:
        writer.setnchannels(2)
        writer.setsampwidth(2)
        writer.setframerate(48_000)
        samples = (
            int(12_000 * math.sin(2 * math.pi * 440 * i / 48_000))
            for i in range(int(48_000 * seconds))
        )
        writer.writeframes(b"".join(struct.pack("<hh", v, v) for v in samples))
    return buffer.getvalue()


@dataclass
class _Harness:
    module: VoiceInput
    texts: list[str] = field(default_factory=list)
    events: list[AudioEvent] = field(default_factory=list)
    decoded: list[bytes] = field(default_factory=list)
    # What the fake STT pipeline "hears" for one PCM event.
    transcribe: Callable[[AudioEvent], str] = lambda event: f"heard {event.data.shape[0]} samples"

    def send(self, chunk: AudioChunk) -> None:
        self.module.audio_in.transport.publish(chunk)

    def assembly_ids(self) -> set[str]:
        with self.module._lock:
            return set(self.module._assemblies)


_MakeVoice = Callable[..., _Harness]


@pytest.fixture
def make_voice(monkeypatch: pytest.MonkeyPatch) -> Iterator[_MakeVoice]:
    modules: list[VoiceInput] = []
    normalize_audio = AudioNormalizer._normalize_audio

    def make(*, real_decode: bool = False, **config: Any) -> _Harness:
        module = VoiceInput(**config)
        modules.append(module)
        harness = _Harness(module)

        def record_normalized(normalizer: AudioNormalizer, event: AudioEvent) -> AudioEvent:
            normalized = normalize_audio(normalizer, event)
            harness.events.append(normalized)
            return normalized

        class FakeModel:
            def transcribe(self, _data: np.ndarray, **_modelopts: Any) -> dict[str, str]:
                return {"text": harness.transcribe(harness.events[-1])}

        class FakeWhisper:
            @staticmethod
            def load_model(_model: str) -> FakeModel:
                return FakeModel()

        monkeypatch.setattr(AudioNormalizer, "_normalize_audio", record_normalized)
        monkeypatch.setattr(whisper_module, "_USE_FASTER_WHISPER", False)
        monkeypatch.setattr(whisper_module, "whisper", FakeWhisper(), raising=False)
        if not real_decode:

            def fake_decode(raw: bytes) -> AudioEvent | None:
                harness.decoded.append(raw)
                if raw == b"undecodable":
                    return None
                return AudioEvent(
                    data=np.frombuffer(raw, dtype=np.uint8), sample_rate=16_000, timestamp=0.0
                )

            monkeypatch.setattr(voice_input_module, "decode_audio_bytes", fake_decode)
        # In ports deliver through a transport; the test thread plays LCM.
        module.audio_in.transport = FakeTransport()
        module.human_input.subscribe(harness.texts.append)
        module.start()
        return harness

    yield make
    for module in modules:
        if not module._module_closed:
            module.stop()


def test_chunks_reassemble_through_decode_to_human_input(
    make_voice: _MakeVoice, wait_until: Any
) -> None:
    h = make_voice()
    h.send(_chunk(seq=0, data=b"come to"))
    h.send(_chunk(seq=1, data=b" the kitchen"))
    h.send(_chunk(seq=2, final=True))
    wait_until(lambda: h.texts == ["heard 19 samples"], timeout=5.0)
    # The decoder saw the exact reassembled container byte stream, once.
    assert h.decoded == [b"come to the kitchen"]


def test_real_container_bytes_reach_pcm_and_human_input(
    make_voice: _MakeVoice, wait_until: Any
) -> None:
    # A real WAV through the real ffmpeg decode: the pipeline receives the
    # 16-kHz mono float32 PCM WhisperNode assumes, and text reaches the port.
    h = make_voice(real_decode=True)
    raw = _wav_bytes()
    for seq, offset in enumerate(range(0, len(raw), 16_384)):
        h.send(_chunk(seq=seq, data=raw[offset : offset + 16_384]))
    h.send(_chunk(seq=seq + 1, final=True))
    wait_until(lambda: len(h.texts) == 1, timeout=15.0)
    (event,) = h.events
    assert event.sample_rate == 16_000
    assert event.data.ndim == 1 and event.data.dtype == np.float32
    # 0.25 s resampled from 48 kHz: ~4000 samples.
    assert 3_000 < event.data.shape[0] < 5_000


def test_seq_gap_discards_utterance_but_not_the_next(
    make_voice: _MakeVoice, wait_until: Any
) -> None:
    h = make_voice()
    h.send(_chunk(sid="gap", seq=0, data=b"lost"))
    h.send(_chunk(sid="gap", seq=2, data=b"lost", final=True))
    # The gapped sid is gone; its stragglers no longer start an utterance.
    h.send(_chunk(sid="gap", seq=3, final=True))
    h.send(_chunk(sid="ok", seq=0, data=b"kept"))
    h.send(_chunk(sid="ok", seq=1, final=True))
    wait_until(lambda: h.texts == ["heard 4 samples"], timeout=5.0)
    assert h.decoded == [b"kept"]


def test_interleaved_sids_assemble_independently(make_voice: _MakeVoice, wait_until: Any) -> None:
    h = make_voice()
    h.send(_chunk(sid="a", seq=0, data=b"one"))
    h.send(_chunk(sid="b", seq=0, data=b"seven"))
    h.send(_chunk(sid="a", seq=1, final=True))
    h.send(_chunk(sid="b", seq=1, final=True))
    wait_until(lambda: sorted(h.texts) == ["heard 3 samples", "heard 5 samples"], timeout=5.0)
    assert sorted(h.decoded) == [b"one", b"seven"]


def test_blank_transcript_not_published(make_voice: _MakeVoice, wait_until: Any) -> None:
    h = make_voice()
    h.transcribe = lambda event: "   " if event.data.shape[0] == 5 else "talk"
    h.send(_chunk(sid="blank", seq=0, data=b"quiet"))
    h.send(_chunk(sid="blank", seq=1, final=True))
    h.send(_chunk(sid="real", seq=0, data=b"speech!"))
    h.send(_chunk(sid="real", seq=1, final=True))
    wait_until(lambda: h.texts == ["talk"], timeout=5.0)
    assert len(h.events) == 2


def test_empty_utterance_never_reaches_the_decoder(make_voice: _MakeVoice, wait_until: Any) -> None:
    h = make_voice()
    # A hold released before any audio arrived: just the final frame.
    h.send(_chunk(sid="empty", seq=0, final=True))
    h.send(_chunk(sid="real", seq=0, data=b"talk"))
    h.send(_chunk(sid="real", seq=1, final=True))
    wait_until(lambda: h.texts == ["heard 4 samples"], timeout=5.0)
    assert h.decoded == [b"talk"]


def test_decode_failure_does_not_kill_the_worker(make_voice: _MakeVoice, wait_until: Any) -> None:
    h = make_voice()
    h.send(_chunk(sid="bad", seq=0, data=b"undecodable"))
    h.send(_chunk(sid="bad", seq=1, final=True))
    h.send(_chunk(sid="next", seq=0, data=b"still alive"))
    h.send(_chunk(sid="next", seq=1, final=True))
    wait_until(lambda: h.texts == ["heard 11 samples"], timeout=5.0)
    assert h.decoded == [b"undecodable", b"still alive"]


def test_transcription_error_does_not_kill_the_pipeline(
    make_voice: _MakeVoice, wait_until: Any
) -> None:
    h = make_voice()
    attempts = 0

    def transcribe(event: AudioEvent) -> str:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise RuntimeError("model failed once")
        return "recovered"

    h.transcribe = transcribe
    h.send(_chunk(sid="bad", seq=0, data=b"first"))
    h.send(_chunk(sid="bad", seq=1, final=True))
    h.send(_chunk(sid="next", seq=0, data=b"second"))
    h.send(_chunk(sid="next", seq=1, final=True))
    wait_until(lambda: h.texts == ["recovered"], timeout=5.0)
    assert attempts == 2 and len(h.events) == 2


def test_cancelled_utterance_reaped_without_new_traffic(
    make_voice: _MakeVoice, wait_until: Any
) -> None:
    # A cancelled hold sends no final frame and may be the browser's last
    # traffic ever; the worker's idle tick must still expire it.
    h = make_voice(stale_utterance_s=0.02)
    h.send(_chunk(sid="cancelled", seq=0, data=b"never sent"))
    assert h.assembly_ids() == {"cancelled"}
    wait_until(lambda: h.assembly_ids() == set(), timeout=5.0)
    assert h.texts == [] and h.decoded == []


def test_oversized_utterance_dropped(make_voice: _MakeVoice, wait_until: Any) -> None:
    h = make_voice(max_utterance_bytes=8)
    h.send(_chunk(sid="big", seq=0, data=b"123456789"))
    h.send(_chunk(sid="big", seq=1, final=True))
    h.send(_chunk(sid="ok", seq=0, data=b"tiny"))
    h.send(_chunk(sid="ok", seq=1, final=True))
    wait_until(lambda: h.texts == ["heard 4 samples"], timeout=5.0)
    assert h.decoded == [b"tiny"]


def test_stale_straggler_cannot_resume_a_reaped_utterance(
    make_voice: _MakeVoice, wait_until: Any
) -> None:
    h = make_voice(stale_utterance_s=0.02)
    h.send(_chunk(sid="cancelled", seq=0, data=b"never sent"))
    wait_until(lambda: h.assembly_ids() == set(), timeout=5.0)
    h.send(_chunk(sid="cancelled", seq=1, final=True))
    h.send(_chunk(sid="fresh", seq=0, data=b"kept"))
    h.send(_chunk(sid="fresh", seq=1, final=True))
    wait_until(lambda: h.texts == ["heard 4 samples"], timeout=5.0)
    assert h.decoded == [b"kept"]


def test_incoming_traffic_reaps_stale_assemblies_while_stt_is_busy(
    make_voice: _MakeVoice, wait_until: Any
) -> None:
    entered = Event()
    release = Event()
    h = make_voice(stale_utterance_s=0.02)

    def blocking_transcribe(event: AudioEvent) -> str:
        entered.set()
        release.wait(timeout=5.0)
        return "done"

    h.transcribe = blocking_transcribe
    h.send(_chunk(sid="busy", seq=0, data=b"work"))
    h.send(_chunk(sid="busy", seq=1, final=True))
    assert entered.wait(timeout=5.0)
    h.send(_chunk(sid="cancelled", seq=0, data=b"partial"))
    with h.module._lock:
        h.module._assemblies["cancelled"].last_at = time.monotonic() - 1.0
    try:
        h.send(_chunk(sid="trigger", seq=0, data=b"new"))
        remaining = h.assembly_ids()
    finally:
        release.set()
    assert remaining == {"trigger"}
    wait_until(lambda: h.texts == ["done"], timeout=5.0)


def test_stop_joins_worker(make_voice: _MakeVoice) -> None:
    h = make_voice()
    assert h.module._thread.is_alive()
    h.module.stop()
    assert not h.module._thread.is_alive()
