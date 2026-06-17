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

import asyncio
import time

import numpy as np
from pydantic import Field

from dimos.agents.annotation import skill
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
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


demo_audio = autoconnect(AudioModule.blueprint())
