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

"""Audio resampling operator for changing sample rate and channel count.

This operator only works with raw audio (RawAudioEvent).
"""

from typing import Optional

import numpy as np
from reactivex import create
from reactivex.abc import ObservableBase
from scipy import signal

from dimos.stream.audio2.types import AudioEvent, RawAudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.operators.resample")


def resample(
    target_sample_rate: Optional[int] = None,
    target_channels: Optional[int] = None,
):
    """Create an audio resampling operator.

    This operator resamples raw audio to a different sample rate and/or channel count.
    Useful for converting audio to the format required by downstream components.

    NOTE: This operator only works with RawAudioEvent. Compressed audio is passed
    through unchanged with a warning.

    Args:
        target_sample_rate: Target sample rate in Hz (None = no resampling)
        target_channels: Target number of channels (None = no channel conversion)
            - 1: Convert to mono (average all channels)
            - 2: Convert to stereo (duplicate mono, or pass through stereo)

    Returns:
        An operator function that can be used with pipe()

    Examples:
        # Resample to 16kHz mono for WhisperLive
        file_input("audio.wav").pipe(
            resample(target_sample_rate=16000, target_channels=1),
            whisperlive_stt()
        ).run()

        # Convert to stereo at 48kHz
        microphone().pipe(
            resample(target_sample_rate=48000, target_channels=2),
            speaker()
        ).run()
    """

    def resample_audio(event: AudioEvent) -> AudioEvent:
        """Resample a single audio event."""
        # Only process raw audio events
        if not isinstance(event, RawAudioEvent):
            logger.warning("Resample received compressed audio - passing through unchanged")
            return event

        audio_data = event.data
        sample_rate = event.sample_rate
        channels = event.channels

        # Handle channel conversion first
        if target_channels is not None and channels != target_channels:
            if target_channels == 1 and channels > 1:
                # Convert to mono by averaging channels
                # Reshape to (samples, channels) for easier processing
                audio_data = audio_data.reshape(-1, channels)
                audio_data = np.mean(audio_data, axis=1)
                channels = 1
            elif target_channels == 2 and channels == 1:
                # Convert mono to stereo by duplicating
                audio_data = np.stack([audio_data, audio_data], axis=1).flatten()
                channels = 2
            elif target_channels == 2 and channels > 2:
                # Downmix to stereo (keep first 2 channels)
                audio_data = audio_data.reshape(-1, event.channels)[:, :2].flatten()
                channels = 2
            else:
                logger.warning(
                    f"Unsupported channel conversion: {event.channels} -> {target_channels}"
                )

        # Handle sample rate conversion
        if target_sample_rate is not None and sample_rate != target_sample_rate:
            # Calculate number of samples after resampling
            num_samples = len(audio_data) // channels
            target_num_samples = int(num_samples * target_sample_rate / sample_rate)

            if channels == 1:
                # Mono: direct resample
                audio_data = signal.resample(audio_data, target_num_samples)
            else:
                # Multi-channel: resample each channel separately
                audio_data = audio_data.reshape(-1, channels)
                resampled_channels = []
                for ch in range(channels):
                    resampled_ch = signal.resample(audio_data[:, ch], target_num_samples)
                    resampled_channels.append(resampled_ch)
                audio_data = np.stack(resampled_channels, axis=1).flatten()

            sample_rate = target_sample_rate

        # Ensure float32 dtype
        if audio_data.dtype != np.float32:
            audio_data = audio_data.astype(np.float32)

        # Create new audio event with resampled data
        return RawAudioEvent(
            data=audio_data,
            sample_rate=sample_rate,
            channels=channels,
            timestamp=event.timestamp,
        )

    def _resample(source: ObservableBase) -> ObservableBase:
        """The actual operator function."""

        def subscribe(observer, scheduler=None):
            logger.info(
                f"Started audio resampler (target_rate={target_sample_rate}Hz, "
                f"target_channels={target_channels})"
            )

            def on_next(event):
                try:
                    resampled = resample_audio(event)
                    observer.on_next(resampled)
                except Exception as e:
                    logger.error(f"Error resampling audio: {e}")
                    observer.on_error(e)

            def on_completed():
                logger.info("Audio resampler completed")
                observer.on_completed()

            return source.subscribe(on_next, observer.on_error, on_completed, scheduler=scheduler)

        return create(subscribe)

    return _resample
