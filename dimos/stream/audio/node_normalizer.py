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

from collections.abc import Callable

import numpy as np
from reactivex import Observable, create, disposable

from dimos.stream.audio.base import (
    AbstractAudioTransform,
    AudioEvent,
)
from dimos.stream.audio.volume import (
    calculate_peak_volume,
    calculate_rms_volume,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class AudioNormalizer(AbstractAudioTransform):
    """
    Audio normalizer that remembers max volume and rescales audio to normalize it.

    This class applies dynamic normalization to audio frames. It keeps track of
    the max volume encountered and uses that to normalize the audio to a target level.
    """

    def __init__(
        self,
        target_level: float = 1.0,
        min_volume_threshold: float = 0.01,
        max_gain: float = 10.0,
        decay_factor: float = 0.999,
        adapt_speed: float = 0.05,
        volume_func: Callable[[np.ndarray], float] = calculate_peak_volume,  # type: ignore[type-arg]
    ) -> None:
        """
        Initialize AudioNormalizer.

        Args:
            target_level: Target normalization level (0.0 to 1.0)
            min_volume_threshold: Minimum volume to apply normalization
            max_gain: Maximum allowed gain to prevent excessive amplification
            decay_factor: Decay factor for max volume (0.0-1.0, higher = slower decay)
            adapt_speed: How quickly to adapt to new volume levels (0.0-1.0)
            volume_func: Function to calculate volume (default: peak volume)
        """
        self.target_level = target_level
        self.min_volume_threshold = min_volume_threshold
        self.max_gain = max_gain
        self.decay_factor = decay_factor
        self.adapt_speed = adapt_speed
        self.volume_func = volume_func

        # Internal state
        self.max_volume = 0.0
        self.current_gain = 1.0
        self.audio_observable = None

    def _normalize_audio(self, audio_event: AudioEvent) -> AudioEvent:
        """
        Normalize audio data based on tracked max volume.

        Args:
            audio_event: Input audio event

        Returns:
            Normalized audio event
        """
        # Convert to float32 for processing if needed
        if audio_event.data.dtype != np.float32:
            audio_event = audio_event.to_float32()

        # Calculate current volume using provided function
        current_volume = self.volume_func(audio_event.data)

        # Update max volume with decay
        self.max_volume = max(current_volume, self.max_volume * self.decay_factor)

        # Calculate ideal gain
        if self.max_volume > self.min_volume_threshold:
            ideal_gain = self.target_level / self.max_volume
        else:
            ideal_gain = 1.0  # No normalization needed for very quiet audio

        # Limit gain to max_gain
        ideal_gain = min(ideal_gain, self.max_gain)

        # Smoothly adapt current gain towards ideal gain
        self.current_gain = (
            1 - self.adapt_speed
        ) * self.current_gain + self.adapt_speed * ideal_gain

        # Apply gain to audio data
        normalized_data = audio_event.data * self.current_gain

        # Clip to prevent distortion (values should stay within -1.0 to 1.0)
        normalized_data = np.clip(normalized_data, -1.0, 1.0)

        # Create new audio event with normalized data
        return AudioEvent(
            data=normalized_data,
            sample_rate=audio_event.sample_rate,
            timestamp=audio_event.timestamp,
            channels=audio_event.channels,
        )

    def consume_audio(self, audio_observable: Observable) -> "AudioNormalizer":  # type: ignore[type-arg]
        """
        Set the audio source observable to consume.

        Args:
            audio_observable: Observable emitting AudioEvent objects

        Returns:
            Self for method chaining
        """
        self.audio_observable = audio_observable  # type: ignore[assignment]
        return self

    def emit_audio(self) -> Observable:  # type: ignore[type-arg]
        """
        Create an observable that emits normalized audio frames.

        Returns:
            Observable emitting normalized AudioEvent objects
        """
        if self.audio_observable is None:
            raise ValueError("No audio source provided. Call consume_audio() first.")

        def on_subscribe(observer, scheduler):
            # Subscribe to the audio observable
            audio_subscription = self.audio_observable.subscribe(
                on_next=lambda event: observer.on_next(self._normalize_audio(event)),
                on_error=lambda error: observer.on_error(error),
                on_completed=lambda: observer.on_completed(),
            )

            logger.info(
                f"Started audio normalizer with target level: {self.target_level}, max gain: {self.max_gain}"
            )

            # Return a disposable to clean up resources
            def dispose() -> None:
                logger.info("Stopping audio normalizer")
                audio_subscription.dispose()

            return disposable.Disposable(dispose)

        return create(on_subscribe)


if __name__ == "__main__":
    import sys

    from dimos.stream.audio.node_microphone import (
        SounddeviceAudioSource,
    )
    from dimos.stream.audio.node_output import SounddeviceAudioOutput
    from dimos.stream.audio.node_simulated import SimulatedAudioSource
    from dimos.stream.audio.node_volume_monitor import monitor
    from dimos.stream.audio.utils import keepalive

    # Parse command line arguments
    volume_method = "peak"  # Default to peak
    use_mic = False  # Default to microphone input
    target_level = 1  # Default target level

    # Process arguments
    for arg in sys.argv[1:]:
        if arg == "rms":
            volume_method = "rms"
        elif arg == "peak":
            volume_method = "peak"
        elif arg == "mic":
            use_mic = True
        elif arg.startswith("level="):
            try:
                target_level = float(arg.split("=")[1])  # type: ignore[assignment]
            except ValueError:
                print(f"Invalid target level: {arg}")
                sys.exit(1)

    # Create appropriate audio source
    if use_mic:
        audio_source = SounddeviceAudioSource()
        print("Using microphone input")
    else:
        audio_source = SimulatedAudioSource(volume_oscillation=True)
        print("Using simulated audio source")

    # Select volume function
    volume_func = calculate_rms_volume if volume_method == "rms" else calculate_peak_volume

    # Create normalizer
    normalizer = AudioNormalizer(target_level=target_level, volume_func=volume_func)

    # Connect the audio source to the normalizer
    normalizer.consume_audio(audio_source.emit_audio())

    print(f"Using {volume_method} volume method with target level {target_level}")
    SounddeviceAudioOutput().consume_audio(normalizer.emit_audio())

    # Monitor the normalized audio
    monitor(normalizer.emit_audio())
    keepalive()
