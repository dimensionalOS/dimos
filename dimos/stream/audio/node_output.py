#!/usr/bin/env python3
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

import threading
from time import monotonic

import numpy as np
from reactivex import Observable

from dimos.stream.audio.base import AbstractAudioConsumer, AudioEvent
from dimos.stream.audio.sounddevice_loader import sounddevice as sd
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
_DEVICE_RETRY_SECONDS = 1.0


class SounddeviceAudioOutput(AbstractAudioConsumer):
    """Play audio events using the sounddevice library."""

    def __init__(
        self,
        device_index: int | None = None,
        sample_rate: int = 16000,
        channels: int = 1,
        block_size: int = 1024,
        dtype: np.dtype = np.float32,  # type: ignore[assignment]
    ) -> None:
        """
        Initialize SounddeviceAudioOutput.

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

        self._stream: sd.OutputStream | None = None
        self._retry_after = 0.0
        self._stream_lock = threading.Lock()
        self._running = threading.Event()
        self._subscription = None

    def consume_audio(self, audio_observable: Observable) -> "SounddeviceAudioOutput":  # type: ignore[type-arg]
        """
        Subscribe to an audio observable and play the audio through the speakers.

        Args:
            audio_observable: Observable emitting AudioEvent objects

        Returns:
            Self for method chaining
        """
        self._retry_after = 0.0
        self._running.set()
        self._subscription = audio_observable.subscribe(  # type: ignore[assignment]
            on_next=self._play_audio_event,
            on_error=self._handle_error,
            on_completed=self._handle_completion,
        )

        return self

    def stop(self) -> None:
        """Stop audio output and clean up resources."""
        self._running.clear()

        if self._subscription:
            self._subscription.dispose()
            self._subscription = None

        with self._stream_lock:
            if self._stream:
                self._stream.stop()
                self._stream.close()
                self._stream = None

    def _open_stream(self) -> None:
        """Open on demand and limit device probes while the server is down."""
        if monotonic() < self._retry_after:
            return

        stream = None
        try:
            stream = sd.OutputStream(
                device=self.device_index,
                samplerate=self.sample_rate,
                channels=self.channels,
                blocksize=self.block_size,
                dtype=self.dtype,
            )
            stream.start()
        except Exception as e:
            if stream is not None:
                stream.close()
            if not self._retry_after:
                logger.warning(
                    "Audio output unavailable, dropping audio until it recovers", error=str(e)
                )
            self._retry_after = monotonic() + _DEVICE_RETRY_SECONDS
        else:
            self._stream = stream
            self._retry_after = 0.0

    def _play_audio_event(self, audio_event: AudioEvent) -> None:
        """Play audio from an AudioEvent."""
        try:
            # Ensure data type matches our stream
            if audio_event.dtype != self.dtype:
                if self.dtype == np.float32:
                    audio_event = audio_event.to_float32()
                elif self.dtype == np.int16:
                    audio_event = audio_event.to_int16()

            with self._stream_lock:
                if not self._running.is_set():
                    return
                if self._stream is None:
                    self._open_stream()
                if self._stream is not None:
                    self._stream.write(audio_event.data)
        except Exception as e:
            logger.error(f"Error playing audio: {e}")

    def _handle_error(self, error) -> None:  # type: ignore[no-untyped-def]
        """Handle errors from the observable."""
        logger.error(f"Error in audio observable: {error}")

    def _handle_completion(self) -> None:
        """Handle completion of the observable."""
        logger.info("Audio observable completed")
        self._running.clear()
        with self._stream_lock:
            if self._stream:
                self._stream.stop()
                self._stream.close()
                self._stream = None
