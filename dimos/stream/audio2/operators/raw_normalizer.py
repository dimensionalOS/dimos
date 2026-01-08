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

"""Raw audio normalizer operator for dynamic volume normalization.

This operator only works with raw audio (RawAudioEvent). For compressed audio,
use the GStreamer-based normalizer operator instead.
"""

from typing import Callable

import numpy as np
from reactivex import create
from reactivex.abc import ObservableBase

from dimos.stream.audio2.types import AudioEvent, RawAudioEvent
from dimos.stream.audio2.operators.utils import calculate_peak_volume
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.operators.normalizer")


def raw_normalizer(
    target_level: float = 1.0,
    min_volume_threshold: float = 0.01,
    max_gain: float = 10.0,
    decay_factor: float = 0.999,
    adapt_speed: float = 0.05,
    volume_func: Callable[[np.ndarray], float] = calculate_peak_volume,
):
    """Create a raw audio normalizer operator.

    This operator applies dynamic normalization to raw audio events. It tracks
    the maximum volume encountered and normalizes audio to a target level.

    NOTE: This operator only works with RawAudioEvent. Compressed audio is passed
    through unchanged with a warning. For a version that handles any format, use
    the GStreamer-based normalizer operator.

    Args:
        target_level: Target normalization level (0.0 to 1.0, default: 1.0)
        min_volume_threshold: Minimum volume to apply normalization (default: 0.01)
        max_gain: Maximum allowed gain to prevent excessive amplification (default: 10.0)
        decay_factor: Decay factor for max volume (0.0-1.0, higher = slower decay, default: 0.999)
        adapt_speed: How quickly to adapt to new volume levels (0.0-1.0, default: 0.05)
        volume_func: Function to calculate volume (default: calculate_peak_volume)

    Returns:
        An operator function that can be used with pipe()

    Examples:
        # Normalize raw audio from a file
        file_input("audio.wav").pipe(
            raw_normalizer(target_level=0.8),
            speaker()
        ).run()

        # Use RMS volume for normalization
        from dimos.stream.audio2.operators.utils import calculate_rms_volume

        signal(frequency=440).pipe(
            raw_normalizer(volume_func=calculate_rms_volume),
            speaker()
        ).run()
    """

    # State variables (captured in closure)
    state = {"max_volume": 0.0, "current_gain": 1.0}

    def normalize_audio(event: AudioEvent) -> AudioEvent:
        """Normalize a single audio event."""
        # Only process raw audio events
        if not isinstance(event, RawAudioEvent):
            logger.warning("Normalizer received compressed audio - passing through unchanged")
            return event

        # Convert to float32 for processing if needed
        audio_data = event.data
        if audio_data.dtype != np.float32:
            audio_data = audio_data.astype(np.float32)
            if audio_data.dtype == np.int16:
                audio_data = audio_data / 32768.0
            elif audio_data.dtype == np.int32:
                audio_data = audio_data / 2147483648.0

        # Calculate current volume
        current_volume = volume_func(audio_data)

        # Update max volume with decay
        state["max_volume"] = max(current_volume, state["max_volume"] * decay_factor)

        # Calculate ideal gain
        if state["max_volume"] > min_volume_threshold:
            ideal_gain = target_level / state["max_volume"]
        else:
            ideal_gain = 1.0  # No normalization for very quiet audio

        # Limit gain to max_gain
        ideal_gain = min(ideal_gain, max_gain)

        # Smoothly adapt current gain towards ideal gain
        state["current_gain"] = (1 - adapt_speed) * state["current_gain"] + adapt_speed * ideal_gain

        # Apply gain to audio data
        normalized_data = audio_data * state["current_gain"]

        # Clip to prevent distortion
        normalized_data = np.clip(normalized_data, -1.0, 1.0)

        # Create new audio event with normalized data
        return RawAudioEvent(
            data=normalized_data,
            sample_rate=event.sample_rate,
            channels=event.channels,
            timestamp=event.timestamp,
        )

    def _normalizer(source: ObservableBase) -> ObservableBase:
        """The actual operator function."""

        def subscribe(observer, scheduler=None):
            logger.info(f"Started audio normalizer (target={target_level}, max_gain={max_gain})")

            def on_next(event):
                try:
                    normalized = normalize_audio(event)
                    observer.on_next(normalized)
                except Exception as e:
                    logger.error(f"Error normalizing audio: {e}")
                    observer.on_error(e)

            def on_completed():
                logger.info("Audio normalizer completed")
                observer.on_completed()

            return source.subscribe(on_next, observer.on_error, on_completed, scheduler=scheduler)

        return create(subscribe)

    return _normalizer
