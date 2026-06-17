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

"""AudioModule — captures mic audio and publishes AudioStamped on an Out stream.

Mirrors CameraModule shape.  Supports:
  - Real capture via sounddevice / PortAudio (synthetic=False, default)
  - Sine-tone fallback with no mic required (synthetic=True)

macOS setup:
  brew install portaudio
  pip install sounddevice numpy
  Grant microphone permission on first real run.
"""

from __future__ import annotations

from collections.abc import Callable
import asyncio
import math
import queue
import threading
import time

import numpy as np
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.audio_msgs.AudioStamped import AudioStamped
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class AudioConfig(ModuleConfig):
    sample_rate: int = 16000
    channels: int = 1
    frame_ms: int = 20
    sample_format: str = "S16LE"
    device: int | None = Field(default=None)
    synthetic: bool = False


class AudioModule(Module):
    """Publishes chunked PCM audio as AudioStamped messages.

    Streams:
        audio (Out[AudioStamped]): one message per frame_ms chunk.

    RPC:
        start() / stop()

    Skills:
        record_clip(seconds) -> bytes  raw S16LE PCM bytes

    io() matches CameraModule shape: single typed Out stream, @rpc start/stop.
    """

    config: AudioConfig
    audio: Out[AudioStamped]

    # ------------------------------------------------------------------
    # Lifecycle  (async def main = one yield: open before, close after)
    # ------------------------------------------------------------------

    async def main(self) -> None:  # type: ignore[override]
        """Open audio device (or start synthetic generator) before yield; close after."""
        frame_size = int(self.config.sample_rate * self.config.frame_ms / 1000)

        if self.config.synthetic:
            task = asyncio.create_task(self._synth_loop(frame_size))
            logger.info("AudioModule: synthetic sine-tone source started")
            yield
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            logger.info("AudioModule: synthetic source stopped")
        else:
            import sounddevice as sd  # type: ignore[import-untyped]

            stream = sd.InputStream(
                device=self.config.device,
                samplerate=self.config.sample_rate,
                channels=self.config.channels,
                blocksize=frame_size,
                dtype="int16",
                callback=self._sd_callback,
            )
            stream.start()
            logger.info(
                "AudioModule: sounddevice stream started "
                f"({self.config.sample_rate}Hz, {self.config.channels}ch, "
                f"{self.config.frame_ms}ms frames)"
            )
            yield
            stream.stop()
            stream.close()
            logger.info("AudioModule: sounddevice stream closed")

    # ------------------------------------------------------------------
    # RPC
    # ------------------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _sd_callback(
        self,
        indata: np.ndarray,
        frames: int,
        time_info: object,
        status: object,
    ) -> None:
        if status:
            logger.warning(f"AudioModule sounddevice status: {status}")
        msg = AudioStamped.from_pcm(
            pcm_bytes=indata.tobytes(),
            sample_rate=self.config.sample_rate,
            channels=self.config.channels,
            sample_format=self.config.sample_format,
            coding_format="pcm",
            ts=time.monotonic(),
        )
        self.audio.publish(msg)

    async def _synth_loop(self, frame_size: int) -> None:
        """Generates a 440 Hz sine tone at ~frame_ms cadence."""
        interval = self.config.frame_ms / 1000.0
        freq = 440.0
        phase = 0
        amplitude = 0.3 * 32767  # ~-10 dBFS in int16

        while True:
            t_start = time.monotonic()

            t = (np.arange(frame_size) + phase) / self.config.sample_rate
            tone = (np.sin(2 * np.pi * freq * t) * amplitude).astype(np.int16)
            phase = (phase + frame_size) % self.config.sample_rate

            if self.config.channels > 1:
                pcm = np.column_stack([tone] * self.config.channels)
            else:
                pcm = tone

            msg = AudioStamped.from_pcm(
                pcm_bytes=pcm.tobytes(),
                sample_rate=self.config.sample_rate,
                channels=self.config.channels,
                sample_format=self.config.sample_format,
                coding_format="pcm",
                ts=time.monotonic(),
            )
            self.audio.publish(msg)

            elapsed = time.monotonic() - t_start
            await asyncio.sleep(max(0.0, interval - elapsed))

    # ------------------------------------------------------------------
    # Skills
    # ------------------------------------------------------------------

    @skill
    def record_clip(self, seconds: float = 1.0) -> bytes:
        """Record and return a clip of raw PCM audio.

        Collects frames from the live audio stream for `seconds` seconds and
        returns them concatenated as raw S16LE PCM bytes.
        """
        import threading

        buf: list[bytes] = []
        done = threading.Event()
        collected = [0.0]

        def on_frame(msg: AudioStamped) -> None:
            buf.append(msg.data)
            collected[0] += self.config.frame_ms / 1000.0
            if collected[0] >= seconds:
                done.set()

        unsub = self.audio.subscribe(on_frame)
        done.wait(timeout=seconds + 2.0)
        unsub()
        return b"".join(buf)


class SpeakerConfig(ModuleConfig):
    device: int | None = Field(default=None)
    queue_max_chunks: int = 256


class SpeakerModule(Module):
    config: SpeakerConfig
    audio: In[AudioStamped]

    _queue: queue.Queue[AudioStamped] | None = None
    _running: threading.Event | None = None
    _writer_thread: threading.Thread | None = None
    _unsub: Callable[[], None] | None = None
    _stream: object | None = None
    _stream_lock: threading.Lock = threading.Lock()
    _stream_rate: int | None = None
    _stream_channels: int | None = None
    _stream_dtype: str | None = None

    async def main(self) -> None:  # type: ignore[override]
        self._queue = queue.Queue(maxsize=self.config.queue_max_chunks)
        self._running = threading.Event()
        self._running.set()
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer_thread.start()
        yield
        if self._unsub is not None:
            try:
                self._unsub()
            except Exception:
                logger.exception("SpeakerModule failed to unsubscribe")
            self._unsub = None
        if self._running is not None:
            self._running.clear()
        if self._writer_thread is not None:
            self._writer_thread.join(timeout=2.0)
            self._writer_thread = None
        with self._stream_lock:
            if self._stream is not None:
                try:
                    self._stream.stop()
                except Exception:
                    logger.exception("SpeakerModule failed to stop output stream")
                try:
                    self._stream.close()
                except Exception:
                    logger.exception("SpeakerModule failed to close output stream")
                self._stream = None
        self._queue = None
        self._running = None
        self._stream_rate = None
        self._stream_channels = None
        self._stream_dtype = None

    @rpc
    def start(self) -> None:
        super().start()
        self._unsub = self.audio.subscribe(self._on_audio)

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_audio(self, msg: AudioStamped) -> None:
        q = self._queue
        if q is None:
            return
        try:
            q.put_nowait(msg)
        except queue.Full:
            try:
                _ = q.get_nowait()
            except queue.Empty:
                return
            try:
                q.put_nowait(msg)
            except queue.Full:
                return

    def _writer_loop(self) -> None:
        running = self._running
        q = self._queue
        if running is None or q is None:
            return

        try:
            import sounddevice as sd  # type: ignore[import-untyped]
        except Exception as e:
            logger.warning(f"SpeakerModule: sounddevice unavailable ({e}); dropping audio")
            while running.is_set():
                try:
                    _ = q.get(timeout=0.25)
                except queue.Empty:
                    continue
            return

        while running.is_set():
            try:
                msg = q.get(timeout=0.25)
            except queue.Empty:
                continue

            if msg.coding_format != "pcm":
                continue

            if msg.sample_format == "S16LE":
                dtype = "<i2"
                sd_dtype = "int16"
            elif msg.sample_format in ("F32LE", "F32"):
                dtype = "<f4"
                sd_dtype = "float32"
            else:
                continue

            if (
                self._stream is None
                or self._stream_rate != msg.sample_rate
                or self._stream_channels != msg.channels
                or self._stream_dtype != sd_dtype
            ):
                with self._stream_lock:
                    if self._stream is not None:
                        try:
                            self._stream.stop()
                        except Exception:
                            logger.exception("SpeakerModule failed to stop output stream")
                        try:
                            self._stream.close()
                        except Exception:
                            logger.exception("SpeakerModule failed to close output stream")
                        self._stream = None
                    try:
                        self._stream = sd.OutputStream(
                            device=self.config.device,
                            samplerate=msg.sample_rate,
                            channels=msg.channels,
                            dtype=sd_dtype,
                        )
                        self._stream.start()
                        self._stream_rate = msg.sample_rate
                        self._stream_channels = msg.channels
                        self._stream_dtype = sd_dtype
                        logger.info(
                            "SpeakerModule: output stream started "
                            f"({msg.sample_rate}Hz, {msg.channels}ch, {sd_dtype})"
                        )
                    except Exception as e:
                        self._stream = None
                        self._stream_rate = None
                        self._stream_channels = None
                        self._stream_dtype = None
                        logger.warning(
                            f"SpeakerModule: no audio output device available ({e}); dropping audio"
                        )

            data = np.frombuffer(msg.data, dtype=np.dtype(dtype))
            if msg.channels > 1:
                data = data.reshape(-1, msg.channels)
            with self._stream_lock:
                if self._stream is not None:
                    try:
                        self._stream.write(data)
                    except Exception:
                        logger.exception("SpeakerModule failed to write audio")


class SpeechToTextConfig(ModuleConfig):
    model: str = "base"
    language: str = "en"
    fp16: bool = False
    segment_seconds: float = 3.0
    queue_max_segments: int = 8


class SpeechToTextModule(Module):
    config: SpeechToTextConfig
    audio: In[AudioStamped]
    text: Out[str]

    _segment_queue: queue.Queue[tuple[np.ndarray, int]] | None = None
    _running: threading.Event | None = None
    _thread: threading.Thread | None = None
    _unsub: Callable[[], None] | None = None
    _buf: np.ndarray | None = None
    _buf_rate: int | None = None

    async def main(self) -> None:  # type: ignore[override]
        self._segment_queue = queue.Queue(maxsize=self.config.queue_max_segments)
        self._running = threading.Event()
        self._running.set()
        self._buf = np.zeros((0,), dtype=np.float32)
        self._buf_rate = None
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()
        yield
        if self._unsub is not None:
            try:
                self._unsub()
            except Exception:
                logger.exception("SpeechToTextModule failed to unsubscribe")
            self._unsub = None
        if self._running is not None:
            self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._segment_queue = None
        self._running = None
        self._buf = None
        self._buf_rate = None

    @rpc
    def start(self) -> None:
        super().start()
        self._unsub = self.audio.subscribe(self._on_audio)

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_audio(self, msg: AudioStamped) -> None:
        if msg.coding_format != "pcm":
            return
        if msg.sample_format == "S16LE":
            x = msg.to_numpy().astype(np.float32)
            if x.ndim > 1:
                x = x.mean(axis=1)
            x = x / 32768.0
        elif msg.sample_format in ("F32LE", "F32"):
            x = msg.to_numpy().astype(np.float32)
            if x.ndim > 1:
                x = x.mean(axis=1)
        else:
            return

        if self._buf is None:
            return

        if self._buf_rate is None:
            self._buf_rate = msg.sample_rate
        elif self._buf_rate != msg.sample_rate:
            self._buf = np.zeros((0,), dtype=np.float32)
            self._buf_rate = msg.sample_rate

        self._buf = np.concatenate([self._buf, x], axis=0)
        rate = self._buf_rate
        if rate is None:
            return
        target_samples = max(1, int(self.config.segment_seconds * rate))

        while self._buf.shape[0] >= target_samples:
            segment = self._buf[:target_samples]
            self._buf = self._buf[target_samples:]
            q = self._segment_queue
            if q is None:
                return
            try:
                q.put_nowait((segment, rate))
            except queue.Full:
                try:
                    _ = q.get_nowait()
                except queue.Empty:
                    break
                try:
                    q.put_nowait((segment, rate))
                except queue.Full:
                    break

    def _worker_loop(self) -> None:
        running = self._running
        q = self._segment_queue
        if running is None or q is None:
            return

        model: object | None = None
        use_faster = False
        try:
            import whisper  # type: ignore[import-untyped]

            model = whisper.load_model(self.config.model)
            use_faster = False
            logger.info(f"SpeechToTextModule: ready (backend=openai-whisper, model={self.config.model})")
        except Exception:
            try:
                from faster_whisper import WhisperModel  # type: ignore[import-untyped]

                compute_type = "float16" if self.config.fp16 else "int8"
                model = WhisperModel(self.config.model, device="auto", compute_type=compute_type)
                use_faster = True
                logger.info(
                    "SpeechToTextModule: ready "
                    f"(backend=faster-whisper, model={self.config.model}, fp16={self.config.fp16})"
                )
            except Exception as e:
                logger.warning(f"SpeechToTextModule: no whisper backend available ({e}); dropping audio")
                while running.is_set():
                    try:
                        _ = q.get(timeout=0.25)
                    except queue.Empty:
                        continue
                return

        while running.is_set():
            try:
                segment, _rate = q.get(timeout=0.25)
            except queue.Empty:
                continue

            try:
                if use_faster:
                    segments, _info = model.transcribe(segment, language=self.config.language)  # type: ignore[union-attr]
                    text = " ".join(seg.text.strip() for seg in segments).strip()
                else:
                    result = model.transcribe(  # type: ignore[union-attr]
                        segment,
                        language=self.config.language,
                        fp16=self.config.fp16,
                    )
                    text = str(result.get("text", "")).strip()
            except Exception:
                logger.exception("SpeechToTextModule transcription failed")
                continue

            if text:
                preview = text if len(text) <= 120 else (text[:120] + "…")
                logger.info(f"SpeechToTextModule: transcribed: {preview}")
                self.text.publish(text)


class TextToSpeechConfig(ModuleConfig):
    voice: str = "echo"
    model: str = "tts-1"
    speed: float = 1.0
    sample_rate: int = 24000
    frame_ms: int = 20
    api_key: str | None = Field(default=None)
    queue_max_texts: int = 64


class TextToSpeechModule(Module):
    config: TextToSpeechConfig
    text: In[str]
    audio: Out[AudioStamped]

    _queue: queue.Queue[str] | None = None
    _running: threading.Event | None = None
    _thread: threading.Thread | None = None
    _unsub: Callable[[], None] | None = None

    async def main(self) -> None:  # type: ignore[override]
        self._queue = queue.Queue(maxsize=self.config.queue_max_texts)
        self._running = threading.Event()
        self._running.set()
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()
        yield
        if self._unsub is not None:
            try:
                self._unsub()
            except Exception:
                logger.exception("TextToSpeechModule failed to unsubscribe")
            self._unsub = None
        if self._running is not None:
            self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._queue = None
        self._running = None

    @rpc
    def start(self) -> None:
        super().start()
        self._unsub = self.text.subscribe(self._on_text)

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_text(self, text: str) -> None:
        if not text.strip():
            return
        q = self._queue
        if q is None:
            return
        try:
            q.put_nowait(text)
        except queue.Full:
            try:
                _ = q.get_nowait()
            except queue.Empty:
                return
            try:
                q.put_nowait(text)
            except queue.Full:
                return

    def _worker_loop(self) -> None:
        running = self._running
        q = self._queue
        if running is None or q is None:
            return

        try:
            import io

            from openai import OpenAI
            import soundfile as sf  # type: ignore[import-untyped]
        except Exception as e:
            logger.warning(f"TextToSpeechModule: missing dependencies ({e}); dropping text")
            while running.is_set():
                try:
                    _ = q.get(timeout=0.25)
                except queue.Empty:
                    continue
            return

        client = OpenAI(api_key=self.config.api_key)
        frame_size = max(1, int(self.config.sample_rate * self.config.frame_ms / 1000))
        logger.info(
            "TextToSpeechModule: ready "
            f"(model={self.config.model}, voice={self.config.voice}, sample_rate={self.config.sample_rate})"
        )

        while running.is_set():
            try:
                text = q.get(timeout=0.25)
            except queue.Empty:
                continue

            preview = text if len(text) <= 120 else (text[:120] + "…")
            logger.info(f"TextToSpeechModule: synthesizing: {preview}")
            try:
                response = client.audio.speech.create(
                    model=self.config.model,
                    voice=self.config.voice,
                    input=text,
                    speed=self.config.speed,
                )
                audio_data = io.BytesIO(response.content)
                with sf.SoundFile(audio_data, "r") as sound_file:
                    src_rate = int(sound_file.samplerate)
                    samples = sound_file.read(dtype="float32")
            except Exception:
                logger.exception("TextToSpeechModule synthesis failed")
                continue

            if samples.ndim > 1:
                samples = samples.mean(axis=1)

            if src_rate != self.config.sample_rate:
                x = np.asarray(samples, dtype=np.float32)
                old_n = x.shape[0]
                if old_n <= 1:
                    continue
                new_n = int(old_n * self.config.sample_rate / src_rate)
                old_idx = np.arange(old_n)
                new_idx = np.linspace(0, old_n - 1, new_n)
                samples = np.interp(new_idx, old_idx, x).astype(np.float32)

            pcm = np.clip(samples, -1.0, 1.0)
            pcm_i16 = (pcm * 32767.0).astype(np.int16)
            pcm_bytes = pcm_i16.tobytes()

            n_bytes_per_sample = 2
            chunk_bytes = frame_size * n_bytes_per_sample
            for offset in range(0, len(pcm_bytes), chunk_bytes):
                chunk = pcm_bytes[offset : offset + chunk_bytes]
                self.audio.publish(
                    AudioStamped.from_pcm(
                        pcm_bytes=chunk,
                        sample_rate=self.config.sample_rate,
                        channels=1,
                        sample_format="S16LE",
                        coding_format="pcm",
                        ts=time.monotonic(),
                    )
                )
            logger.info(
                "TextToSpeechModule: published audio "
                f"({len(pcm_bytes) / (n_bytes_per_sample * self.config.sample_rate):.2f}s)"
            )


def _phase_vocoder(D: np.ndarray, rate: float, hop_length: int, n_fft: int) -> np.ndarray:
    if D.shape[1] <= 1:
        return D
    time_steps = np.arange(0, D.shape[1] - 1, rate, dtype=np.float32)
    out = np.empty((D.shape[0], len(time_steps)), dtype=np.complex64)

    phase_acc = np.angle(D[:, 0]).astype(np.float32)
    phase_advance = (2.0 * np.pi * hop_length * np.arange(D.shape[0]) / n_fft).astype(
        np.float32
    )

    for t, step in enumerate(time_steps):
        i = int(step)
        frac = float(step - i)
        mag = (1.0 - frac) * np.abs(D[:, i]) + frac * np.abs(D[:, i + 1])

        phase = np.angle(D[:, i + 1]) - np.angle(D[:, i])
        phase = phase - phase_advance
        phase = (phase + np.pi) % (2.0 * np.pi) - np.pi

        phase_acc = phase_acc + phase_advance + phase
        out[:, t] = mag * np.exp(1j * phase_acc)

    return out


def _stft(y: np.ndarray, n_fft: int, hop_length: int, window: np.ndarray) -> np.ndarray:
    if y.shape[0] < n_fft:
        y = np.pad(y, (0, n_fft - y.shape[0]))
    n_frames = 1 + (y.shape[0] - n_fft) // hop_length
    frames = np.empty((n_fft // 2 + 1, n_frames), dtype=np.complex64)
    for i in range(n_frames):
        start = i * hop_length
        frame = y[start : start + n_fft] * window
        frames[:, i] = np.fft.rfft(frame, n=n_fft).astype(np.complex64)
    return frames


def _istft(D: np.ndarray, n_fft: int, hop_length: int, window: np.ndarray, length: int) -> np.ndarray:
    y = np.zeros((hop_length * (D.shape[1] - 1) + n_fft,), dtype=np.float32)
    wsum = np.zeros_like(y)
    for i in range(D.shape[1]):
        start = i * hop_length
        frame = np.fft.irfft(D[:, i], n=n_fft).astype(np.float32)
        y[start : start + n_fft] += frame * window
        wsum[start : start + n_fft] += window * window
    nonzero = wsum > 1e-8
    y[nonzero] /= wsum[nonzero]
    if y.shape[0] < length:
        y = np.pad(y, (0, length - y.shape[0]))
    return y[:length]


def _resample_linear(y: np.ndarray, n: int) -> np.ndarray:
    if n <= 0:
        return np.zeros((0,), dtype=np.float32)
    if y.shape[0] == 0:
        return np.zeros((n,), dtype=np.float32)
    if y.shape[0] == 1:
        return np.full((n,), float(y[0]), dtype=np.float32)
    xp = np.arange(y.shape[0], dtype=np.float32)
    x = np.linspace(0.0, float(y.shape[0] - 1), n, dtype=np.float32)
    return np.interp(x, xp, y).astype(np.float32)


def _pitch_shift_block(y: np.ndarray, sample_rate: int, semitones: float) -> np.ndarray:
    if semitones == 0.0:
        return y
    n_fft = 1024
    hop = 256
    p = 2.0 ** (semitones / 12.0)
    rate = 1.0 / p
    window = np.hanning(n_fft).astype(np.float32)

    D = _stft(y, n_fft=n_fft, hop_length=hop, window=window)
    D_stretch = _phase_vocoder(D, rate=rate, hop_length=hop, n_fft=n_fft)
    stretched_len = max(1, int(round(y.shape[0] * rate)))
    y_stretch = _istft(D_stretch, n_fft=n_fft, hop_length=hop, window=window, length=stretched_len)
    y_shift = _resample_linear(y_stretch, y.shape[0])
    return y_shift


class FunVoiceEffectsConfig(ModuleConfig):
    enabled: bool = True
    input_gain: float = 1.0

    noise_gate_enabled: bool = True
    noise_gate_threshold_db: float = -35.0
    noise_gate_attack_ms: float = 10.0
    noise_gate_release_ms: float = 120.0

    pitch_shift_enabled: bool = True
    pitch_semitones: float = 7.0

    robotize_enabled: bool = True
    ringmod_hz: float = 45.0
    bitcrush_bits: int = 8
    bitcrush_downsample: int = 4

    echo_enabled: bool = True
    echo_delay_ms: float = 160.0
    echo_feedback: float = 0.35
    echo_mix: float = 0.25

    block_ms: float = 160.0
    vu_log_interval_s: float = 0.75


class _FunVoiceProcessor:
    def __init__(self) -> None:
        self.sample_rate: int | None = None
        self.input_gain: float = 1.0
        self.gate_gain: float = 1.0
        self.gate_attack: float = 0.0
        self.gate_release: float = 0.0
        self.gate_threshold: float = 0.0
        self.pitch_semitones: float = 0.0
        self.ringmod_hz: float = 0.0
        self.ring_phase: float = 0.0
        self.bit_bits: int = 8
        self.bit_downsample: int = 1
        self.bit_hold: float = 0.0
        self.bit_count: int = 0
        self.echo_delay: int = 0
        self.echo_feedback: float = 0.0
        self.echo_mix: float = 0.0
        self.echo_buf: np.ndarray | None = None
        self.echo_idx: int = 0
        self.block_size: int = 0
        self.hop_size: int = 0
        self.window: np.ndarray | None = None
        self.prev_overlap: np.ndarray | None = None
        self.inbuf = np.zeros((0,), dtype=np.float32)
        self.outbuf = np.zeros((0,), dtype=np.float32)
        self.last_vu_log = 0.0

    def reconfigure(self, cfg: FunVoiceEffectsConfig, sample_rate: int) -> None:
        self.sample_rate = sample_rate
        self.input_gain = float(cfg.input_gain)

        th = 10.0 ** (float(cfg.noise_gate_threshold_db) / 20.0)
        self.gate_threshold = float(th)
        self.gate_attack = max(1e-6, float(cfg.noise_gate_attack_ms) / 1000.0)
        self.gate_release = max(1e-6, float(cfg.noise_gate_release_ms) / 1000.0)

        self.pitch_semitones = float(cfg.pitch_semitones) if cfg.pitch_shift_enabled else 0.0
        self.ringmod_hz = float(cfg.ringmod_hz) if cfg.robotize_enabled else 0.0
        self.bit_bits = int(cfg.bitcrush_bits)
        self.bit_downsample = max(1, int(cfg.bitcrush_downsample))

        self.echo_feedback = float(cfg.echo_feedback) if cfg.echo_enabled else 0.0
        self.echo_mix = float(cfg.echo_mix) if cfg.echo_enabled else 0.0
        self.echo_delay = int(round(float(cfg.echo_delay_ms) * sample_rate / 1000.0))
        if self.echo_delay > 0 and self.echo_mix > 0.0:
            self.echo_buf = np.zeros((self.echo_delay,), dtype=np.float32)
            self.echo_idx = 0
        else:
            self.echo_buf = None
            self.echo_idx = 0

        self.block_size = max(1024, int(round(float(cfg.block_ms) * sample_rate / 1000.0)))
        self.block_size = int(2 ** math.ceil(math.log2(self.block_size)))
        self.hop_size = self.block_size // 2
        self.window = np.sqrt(np.hanning(self.block_size).astype(np.float32))
        self.prev_overlap = np.zeros((self.hop_size,), dtype=np.float32)

        self.inbuf = np.zeros((0,), dtype=np.float32)
        self.outbuf = np.zeros((0,), dtype=np.float32)
        self.ring_phase = 0.0
        self.bit_hold = 0.0
        self.bit_count = 0
        self.gate_gain = 1.0
        self.last_vu_log = 0.0

    def push(self, x: np.ndarray) -> None:
        self.inbuf = np.concatenate([self.inbuf, x.astype(np.float32, copy=False)], axis=0)
        while self.inbuf.shape[0] >= self.block_size:
            block = self.inbuf[: self.block_size]
            self.inbuf = self.inbuf[self.hop_size :]
            y = self._process_block(block)
            w = self.window
            prev = self.prev_overlap
            if w is None or prev is None:
                self.outbuf = np.concatenate([self.outbuf, y[: self.hop_size]], axis=0)
            else:
                yw = y * w
                out = yw[: self.hop_size] + prev
                self.prev_overlap = yw[self.hop_size :]
                self.outbuf = np.concatenate([self.outbuf, out], axis=0)

    def pop(self, n: int) -> np.ndarray:
        if n <= 0:
            return np.zeros((0,), dtype=np.float32)
        if self.outbuf.shape[0] < n:
            out = np.zeros((n,), dtype=np.float32)
            if self.outbuf.shape[0] > 0:
                out[: self.outbuf.shape[0]] = self.outbuf
            self.outbuf = np.zeros((0,), dtype=np.float32)
            return out
        out = self.outbuf[:n]
        self.outbuf = self.outbuf[n:]
        return out

    def _apply_gate(self, y: np.ndarray) -> np.ndarray:
        rms = float(np.sqrt(np.mean(y * y) + 1e-12))
        target = 0.0 if rms < self.gate_threshold else 1.0
        sr = float(self.sample_rate or 1)
        if target > self.gate_gain:
            step = 1.0 / max(1.0, self.gate_attack * sr)
            self.gate_gain = min(1.0, self.gate_gain + step * y.shape[0])
        else:
            step = 1.0 / max(1.0, self.gate_release * sr)
            self.gate_gain = max(0.0, self.gate_gain - step * y.shape[0])
        return y * self.gate_gain

    def _apply_ringmod(self, y: np.ndarray) -> np.ndarray:
        if self.ringmod_hz <= 0.0:
            return y
        sr = float(self.sample_rate or 1)
        phase_inc = 2.0 * np.pi * self.ringmod_hz / sr
        idx = np.arange(y.shape[0], dtype=np.float32)
        ph = self.ring_phase + phase_inc * idx
        mod = np.sin(ph).astype(np.float32)
        self.ring_phase = float((ph[-1] + phase_inc) % (2.0 * np.pi))
        return y * mod

    def _apply_bitcrush(self, y: np.ndarray) -> np.ndarray:
        bits = int(self.bit_bits)
        if bits >= 16 and self.bit_downsample <= 1:
            return y
        levels = float(2 ** max(1, bits - 1))
        out = np.empty_like(y)
        for i, s in enumerate(y):
            if self.bit_count <= 0:
                q = np.round(s * levels) / levels
                self.bit_hold = float(q)
                self.bit_count = self.bit_downsample
            out[i] = self.bit_hold
            self.bit_count -= 1
        return out

    def _apply_echo(self, y: np.ndarray) -> np.ndarray:
        buf = self.echo_buf
        if buf is None or self.echo_delay <= 0 or self.echo_mix <= 0.0:
            return y
        out = np.empty_like(y)
        idx = self.echo_idx
        fb = float(self.echo_feedback)
        mix = float(self.echo_mix)
        for i, s in enumerate(y):
            d = float(buf[idx])
            wet = s + d
            buf[idx] = s + d * fb
            idx += 1
            if idx >= buf.shape[0]:
                idx = 0
            out[i] = (1.0 - mix) * s + mix * wet
        self.echo_idx = idx
        return out

    def _process_block(self, x: np.ndarray) -> np.ndarray:
        y = x.astype(np.float32, copy=False) * self.input_gain
        y = self._apply_gate(y)
        if self.pitch_semitones != 0.0 and self.sample_rate is not None:
            y = _pitch_shift_block(y, sample_rate=self.sample_rate, semitones=self.pitch_semitones)
        y = self._apply_ringmod(y)
        y = self._apply_bitcrush(y)
        y = self._apply_echo(y)
        return np.clip(y, -1.0, 1.0)


class FunVoiceEffectsModule(Module):
    config: FunVoiceEffectsConfig
    audio_in: In[AudioStamped]
    audio_out: Out[AudioStamped]

    _queue: queue.Queue[AudioStamped] | None = None
    _running: threading.Event | None = None
    _thread: threading.Thread | None = None
    _unsub: Callable[[], None] | None = None
    _processor: _FunVoiceProcessor = _FunVoiceProcessor()
    _out_frame_size: int | None = None

    async def main(self) -> None:  # type: ignore[override]
        self._queue = queue.Queue(maxsize=256)
        self._running = threading.Event()
        self._running.set()
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()
        yield
        if self._unsub is not None:
            try:
                self._unsub()
            except Exception:
                logger.exception("FunVoiceEffectsModule failed to unsubscribe")
            self._unsub = None
        if self._running is not None:
            self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._queue = None
        self._running = None

    @rpc
    def start(self) -> None:
        super().start()
        self._unsub = self.audio_in.subscribe(self._on_audio)

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_audio(self, msg: AudioStamped) -> None:
        q = self._queue
        if q is None or not self.config.enabled:
            if self.config.enabled:
                self.audio_out.publish(msg)
            return
        try:
            q.put_nowait(msg)
        except queue.Full:
            try:
                _ = q.get_nowait()
            except queue.Empty:
                return
            try:
                q.put_nowait(msg)
            except queue.Full:
                return

    def _decode_to_float(self, msg: AudioStamped) -> np.ndarray | None:
        if msg.coding_format != "pcm":
            return None
        if msg.sample_format == "S16LE":
            x = msg.to_numpy().astype(np.float32)
            if x.ndim > 1:
                x = x.mean(axis=1)
            return x / 32768.0
        if msg.sample_format in ("F32LE", "F32"):
            x = msg.to_numpy().astype(np.float32)
            if x.ndim > 1:
                x = x.mean(axis=1)
            return x
        return None

    def _encode_from_float(self, y: np.ndarray, sample_rate: int) -> bytes:
        y = np.clip(y, -1.0, 1.0)
        pcm_i16 = (y * 32767.0).astype(np.int16)
        return pcm_i16.tobytes()

    def _worker_loop(self) -> None:
        running = self._running
        q = self._queue
        if running is None or q is None:
            return
        while running.is_set():
            try:
                msg = q.get(timeout=0.25)
            except queue.Empty:
                continue
            x = self._decode_to_float(msg)
            if x is None:
                continue
            if self._processor.sample_rate != msg.sample_rate:
                self._processor.reconfigure(self.config, sample_rate=msg.sample_rate)
                self._out_frame_size = x.shape[0]
                logger.info(
                    "FunVoiceEffectsModule: ready "
                    f"(sr={msg.sample_rate}, block_ms={self.config.block_ms}, pitch={self.config.pitch_semitones})"
                )
            frame_size = self._out_frame_size or x.shape[0]
            self._processor.push(x)
            y = self._processor.pop(frame_size)

            now = time.monotonic()
            if now - self._processor.last_vu_log >= float(self.config.vu_log_interval_s):
                peak = float(np.max(np.abs(y))) if y.shape[0] else 0.0
                rms = float(np.sqrt(np.mean(y * y) + 1e-12)) if y.shape[0] else 0.0
                self._processor.last_vu_log = now
                logger.info(f"FunVoiceEffectsModule: vu peak={peak:.3f} rms={rms:.3f}")

            self.audio_out.publish(
                AudioStamped.from_pcm(
                    pcm_bytes=self._encode_from_float(y, sample_rate=msg.sample_rate),
                    sample_rate=msg.sample_rate,
                    channels=1,
                    sample_format="S16LE",
                    coding_format="pcm",
                    ts=time.monotonic(),
                )
            )


audio_speech_loopback = autoconnect(
    AudioModule.blueprint(),
    SpeechToTextModule.blueprint(),
    TextToSpeechModule.blueprint(),
    FunVoiceEffectsModule.blueprint(),
    SpeakerModule.blueprint(),
).remappings(
    [
        (AudioModule, "audio", "mic_audio"),
        (SpeechToTextModule, "audio", "mic_audio"),
        (SpeechToTextModule, "text", "speech_text"),
        (TextToSpeechModule, "text", "speech_text"),
        (TextToSpeechModule, "audio", "tts_audio_raw"),
        (FunVoiceEffectsModule, "audio_in", "tts_audio_raw"),
        (FunVoiceEffectsModule, "audio_out", "tts_audio"),
        (SpeakerModule, "audio", "tts_audio"),
    ]
)


demo_audio = autoconnect(
    AudioModule.blueprint(),
    SpeakerModule.blueprint(),
)
