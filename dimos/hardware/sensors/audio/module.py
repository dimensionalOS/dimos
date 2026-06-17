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


audio_speech_loopback = autoconnect(
    AudioModule.blueprint(),
    SpeechToTextModule.blueprint(),
    TextToSpeechModule.blueprint(),
    SpeakerModule.blueprint(),
).remappings(
    [
        (AudioModule, "audio", "mic_audio"),
        (SpeechToTextModule, "audio", "mic_audio"),
        (SpeechToTextModule, "text", "speech_text"),
        (TextToSpeechModule, "text", "speech_text"),
        (TextToSpeechModule, "audio", "tts_audio"),
        (SpeakerModule, "audio", "tts_audio"),
    ]
)


demo_audio = autoconnect(
    AudioModule.blueprint(),
    SpeakerModule.blueprint(),
)
