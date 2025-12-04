#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

import time
from typing import Any

import numpy as np
from reactivex import Observable, create, disposable
import sounddevice as sd  # type: ignore[import-untyped]

from dimos.stream.audio.base import (
    AbstractAudioEmitter,
    AudioEvent,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SounddeviceAudioSource(AbstractAudioEmitter):
    """Audio source implementation using the sounddevice library."""

    def __init__(
        self,
        device_index: int | None = None,
        sample_rate: int = 16000,
        channels: int = 1,
        block_size: int = 1024,
        dtype: np.dtype = np.float32,  # type: ignore[assignment, type-arg]
    ) -> None:
        """
        Initialize SounddeviceAudioSource.

        Args:
            device_index: Audio device index (None for default)
            sample_rate: Audio sample rate in Hz
            channels: Number of audio channels (1=mono, 2=stereo)
            block_size: Number of samples per audio frame
            dtype: Data type for audio samples (np.float32 or np.int16)
        """
        self.device_index = device_index
        self.sample_rate = sample_rate
        self.channels = channels
        self.block_size = block_size
        self.dtype = dtype

        self._stream = None
        self._running = False

    def emit_audio(self) -> Observable:  # type: ignore[type-arg]
        """
        Create an observable that emits audio frames.

        Returns:
            Observable emitting AudioEvent objects
        """

        def on_subscribe(observer, scheduler):  # type: ignore[no-untyped-def]
            # Callback function to process audio data
            def audio_callback(indata, frames, time_info, status) -> None:  # type: ignore[no-untyped-def]
                if status:
                    logger.warning(f"Audio callback status: {status}")

                # Create audio event
                audio_event = AudioEvent(
                    data=indata.copy(),
                    sample_rate=self.sample_rate,
                    timestamp=time.time(),
                    channels=self.channels,
                )

                observer.on_next(audio_event)

            # Start the audio stream
            try:
                self._stream = sd.InputStream(
                    device=self.device_index,
                    samplerate=self.sample_rate,
                    channels=self.channels,
                    blocksize=self.block_size,
                    dtype=self.dtype,
                    callback=audio_callback,
                )
                self._stream.start()  # type: ignore[attr-defined]
                self._running = True

                logger.info(
                    f"Started audio capture: {self.sample_rate}Hz, "
                    f"{self.channels} channels, {self.block_size} samples per frame"
                )

            except Exception as e:
                logger.error(f"Error starting audio stream: {e}")
                observer.on_error(e)

            # Return a disposable to clean up resources
            def dispose() -> None:
                logger.info("Stopping audio capture")
                self._running = False
                if self._stream:
                    self._stream.stop()
                    self._stream.close()
                    self._stream = None

            return disposable.Disposable(dispose)

        return create(on_subscribe)

    def get_available_devices(self) -> list[dict[str, Any]]:
        """Get a list of available audio input devices."""
        return sd.query_devices()  # type: ignore[no-any-return]


if __name__ == "__main__":
    from dimos.stream.audio.node_volume_monitor import monitor
    from dimos.stream.audio.utils import keepalive

    monitor(SounddeviceAudioSource().emit_audio())
    keepalive()
